"""Publish a localization heartbeat only when fresh odometry is being received."""
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError:
    rclpy = None


class LocalizationHeartbeatNode(Node):
    def __init__(self):
        super().__init__('localization_heartbeat')
        self.pub = self.create_publisher(String, '/localization/heartbeat', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def odom_cb(self, _msg):
        self.pub.publish(String(data='localization'))


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run localization_heartbeat_node')
    rclpy.init(args=args)
    node = LocalizationHeartbeatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
