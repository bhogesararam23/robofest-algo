"""Adapt the specified lightweight sensor messages to robot_localization inputs."""
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import Range
except ImportError:
    rclpy = None


class SensorAdapterNode(Node):
    def __init__(self):
        super().__init__('localization_sensor_adapter')
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, '/localization/optical_flow_twist', 10)
        self.tof_pub = self.create_publisher(PoseWithCovarianceStamped, '/sensor/tof_pose', 10)
        self.create_subscription(TwistStamped, '/sensor/optical_flow', self.flow_cb, 10)
        self.create_subscription(Range, '/sensor/tof_range', self.tof_cb, 10)

    def flow_cb(self, msg):
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        out.twist.twist = msg.twist
        # Optical-flow XY velocity is trusted; unused covariance entries remain conservative.
        out.twist.covariance[0] = 0.05
        out.twist.covariance[7] = 0.05
        self.twist_pub.publish(out)

    def tof_cb(self, msg):
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = 'base_link'
        out.pose.pose.position.z = msg.range
        out.pose.pose.orientation.w = 1.0
        out.pose.covariance[14] = max(0.001, float(msg.variance) if msg.variance > 0.0 else 0.01)
        self.tof_pub.publish(out)


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run sensor_adapter_node')
    rclpy.init(args=args)
    node = SensorAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
