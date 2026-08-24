from time import monotonic
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from robofest_interfaces.msg import MineMap, MineDetection
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
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(1.0, self.decay_cb)
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = __import__('math').atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw); self.last_odom = monotonic()
    def detections_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        for mine in msg.mines:
            self.store.add_relative(Detection(mine.id, mine.x, mine.y, mine.confidence, mine.type), self.pose, now)
        self.publish_map()
    def decay_cb(self):
        self.store.decay(self.get_clock().now().nanoseconds / 1e9); self.publish_map()
    def publish_map(self):
        msg = MineMap(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'odom'
        msg.mines = [self.to_msg(m) for m in self.store.mines]; self.map_pub.publish(msg)
    @staticmethod
    def to_msg(m):
        msg = MineDetection(); msg.id = m.id; msg.x = m.x; msg.y = m.y; msg.confidence = m.confidence; msg.type = m.type; return msg

def main(args=None):
    if rclpy is None: raise RuntimeError('ROS 2 is required to run mine_mapping_node')
    rclpy.init(args=args); node = MineMappingNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
