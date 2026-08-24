try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from robofest_interfaces.msg import MineMap, SwarmState
except ImportError:
    rclpy = None
class SwarmNode(Node):
    def __init__(self):
        super().__init__('swarm_communication'); self.declare_parameter('drone_id',1); self.declare_parameter('battery_percent',100.0); self.task='IDLE'; self.available=True; self.map=MineMap()
        self.pub=self.create_publisher(SwarmState,'/swarm/outgoing_state',10); self.task_pub=self.create_publisher(String,'/mission/current_command',10)
        self.create_subscription(SwarmState,'/swarm/incoming_state',self.incoming_cb,10); self.create_subscription(MineMap,'/navigation/global_mine_map',self.map_cb,10); self.create_subscription(String,'/mission/current_command',self.task_cb,10); self.create_timer(0.2,self.publish_state)
    def task_cb(self,msg): self.task=msg.data
    def map_cb(self,msg): self.map=msg
    def incoming_cb(self,msg):
        if msg.drone_id == int(self.get_parameter('drone_id').value): return
        if self.task == 'GUIDE_HUMAN' and float(self.get_parameter('battery_percent').value) < 20.0 and msg.is_available and msg.battery_percent > 50.0:
            out=String(); out.data='YIELD_GUIDANCE'; self.task_pub.publish(out)
    def publish_state(self):
        msg=SwarmState(); msg.drone_id=int(self.get_parameter('drone_id').value); msg.battery_percent=float(self.get_parameter('battery_percent').value); msg.current_task=self.task; msg.is_available=self.available; msg.local_map=self.map; self.pub.publish(msg)
def main(args=None):
    if rclpy is None: raise RuntimeError('ROS 2 is required to run swarm_node')
    rclpy.init(args=args); n=SwarmNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
