"""5 Hz swarm state exchange with acknowledged guidance handoff and map fusion."""
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from robofest_interfaces.msg import MineMap, MineDetection, SwarmState
except ImportError:
    rclpy = None

from .handoff_core import HandoffManager, SwarmPeer, PENDING, ACK, RELEASE, EXPIRED
try:
    from robofest_navigation.map_core import Detection, MineMapStore
except ImportError:
    Detection = None
    MineMapStore = None


class SwarmNode(Node):
    def __init__(self):
        super().__init__('swarm_communication')
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('battery_percent', 100.0)
        self.declare_parameter('handoff_lease_seconds', 5.0)
        self.declare_parameter('handoff_timeout_seconds', 8.0)
        self.task = 'IDLE'
        self.available = True
        self.sequence = 0
        self.peers = {}
        self.local_map = MineMap()
        self.map_store = MineMapStore() if MineMapStore else None
        self.handoff = HandoffManager(
            int(self.get_parameter('drone_id').value),
            float(self.get_parameter('handoff_lease_seconds').value),
            float(self.get_parameter('handoff_timeout_seconds').value),
        )
        self.pub = self.create_publisher(SwarmState, '/swarm/outgoing_state', 10)
        self.command_pub = self.create_publisher(String, '/mission/current_command', 10)
        self.create_subscription(SwarmState, '/swarm/incoming_state', self.incoming_cb, 10)
        self.create_subscription(MineMap, '/navigation/global_mine_map', self.map_cb, 10)
        self.create_subscription(String, '/mission/current_command', self.task_cb, 10)
        self.create_timer(0.2, self.publish_state)

    def task_cb(self, msg):
        if msg.data not in ('YIELD_GUIDANCE', 'HANDOFF_PENDING', 'HANDOFF_ACK'):
            self.task = msg.data

    def map_cb(self, msg):
        self.local_map = msg
        if self.map_store is not None:
            self.map_store.mines = [Detection(m.id, m.x, m.y, m.confidence, m.type, 0.0) for m in msg.mines]

    def incoming_cb(self, msg):
        local_id = int(self.get_parameter('drone_id').value)
        if msg.drone_id == local_id:
            return
        peer = SwarmPeer(msg.drone_id, msg.battery_percent, msg.is_available, msg.current_task,
                         msg.handoff_id, msg.handoff_status, msg.handoff_target_id)
        self.peers[msg.drone_id] = peer
        now = self.get_clock().now().nanoseconds / 1e9
        if self.map_store is not None:
            self.map_store.merge((Detection(m.id, m.x, m.y, m.confidence, m.type) for m in msg.local_map.mines), now)
            self.publish_fused_map()
        current = self.handoff.receive(now, peer, self.available)
        if current.status == ACK and current.handoff_id == msg.handoff_id and current.target_id == local_id:
            self.task = 'ACCEPT_GUIDANCE'
            self.available = False
            self.get_logger().info(f'Acknowledged guidance handoff {current.handoff_id} from drone {msg.drone_id}')
        if self.handoff.apply_ack(peer, now):
            self.task = 'HANDOFF_COMPLETE'
            self.available = False
            self.command_pub.publish(String(data='YIELD_GUIDANCE'))

    def publish_fused_map(self):
        if self.map_store is None:
            return
        msg = MineMap()
        msg.header = self.local_map.header
        msg.mines = []
        for mine in self.map_store.mines:
            out = MineDetection(); out.id = mine.id; out.x = mine.x; out.y = mine.y; out.confidence = mine.confidence; out.type = mine.type; msg.mines.append(out)
        self.local_map = msg

    def publish_state(self):
        now = self.get_clock().now().nanoseconds / 1e9
        battery = float(self.get_parameter('battery_percent').value)
        current = self.handoff.tick(now)
        if current.status == EXPIRED:
            self.get_logger().warning(f'Handoff {current.handoff_id} expired without a valid acknowledgement')
            self.handoff.current.status = ''
        if self.task == 'GUIDE_HUMAN' and battery < 20.0:
            self.handoff.start(now, self.peers.values(), self.task, battery)
        self.sequence += 1
        msg = SwarmState()
        msg.drone_id = int(self.get_parameter('drone_id').value)
        msg.battery_percent = battery
        msg.current_task = self.task
        msg.is_available = self.available
        msg.local_map = self.local_map
        msg.sequence = self.sequence
        msg.handoff_id = self.handoff.current.handoff_id
        msg.handoff_status = self.handoff.current.status
        msg.handoff_target_id = self.handoff.current.target_id
        self.pub.publish(msg)


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run swarm_node')
    rclpy.init(args=args)
    node = SwarmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
