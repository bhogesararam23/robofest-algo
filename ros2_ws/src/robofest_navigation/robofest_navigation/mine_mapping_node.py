from time import monotonic
import math

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from robofest_interfaces.msg import MineMap, MineDetection, SwarmState
except ImportError:
    rclpy = None

from .map_core import Detection, MineMapStore


class MineMappingNode(Node):
    def __init__(self):
        super().__init__('mine_mapping')
        self.store = MineMapStore()
        self.pose = (0.0, 0.0, 0.0)
        self.last_odom = 0.0
        self.map_pub = self.create_publisher(MineMap, '/navigation/global_mine_map', 10)
        self.create_subscription(MineMap, '/perception/mine_detections', self.detections_cb, 10)
        self.create_subscription(SwarmState, '/swarm/incoming_state', self.swarm_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(1.0, self.decay_cb)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.last_odom = monotonic()

    def detections_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        for mine in msg.mines:
            self.store.add_relative(Detection(mine.id, mine.x, mine.y, mine.confidence, mine.type), self.pose, now)
        self.publish_map()

    def swarm_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        # Swarm MineMap coordinates are already expressed in the shared odom frame.
        for mine in msg.local_map.mines:
            self.store.add_global(Detection(mine.id, mine.x, mine.y, mine.confidence, mine.type), now)
        self.publish_map()

    def decay_cb(self):
        self.store.decay(self.get_clock().now().nanoseconds / 1e9)
        self.publish_map()

    def publish_map(self):
        msg = MineMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.mines = [self.to_msg(mine) for mine in self.store.mines]
        self.map_pub.publish(msg)

    @staticmethod
    def to_msg(mine):
        msg = MineDetection()
        msg.id = mine.id
        msg.x = mine.x
        msg.y = mine.y
        msg.confidence = mine.confidence
        msg.type = mine.type
        return msg


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run mine_mapping_node')
    rclpy.init(args=args)
    node = MineMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
