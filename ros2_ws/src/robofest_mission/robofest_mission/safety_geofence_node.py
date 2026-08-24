"""10 Hz fail-closed geofence and heartbeat watchdog."""
from time import monotonic

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError:
    rclpy = None


class SafetyGeofenceNode(Node):
    def __init__(self):
        super().__init__('safety_geofence')
        self.declare_parameter('arena_width_m', 60.0)
        self.declare_parameter('arena_height_m', 15.0)
        self.declare_parameter('watchdog_timeout_s', 0.5)
        self.x = self.y = 0.0
        self.last_odom = None
        self.required_topics = ('/vision/heartbeat', '/localization/heartbeat', '/drivers/heartbeat')
        self.last_seen = {topic: None for topic in self.required_topics}
        self.command_pub = self.create_publisher(String, '/mission/current_command', 10)
        self.reason_pub = self.create_publisher(String, '/mission/safety_reason', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        for topic in self.required_topics:
            self.create_subscription(String, topic, lambda _msg, t=topic: self.heartbeat_cb(t), 10)
        self.create_timer(0.1, self.check)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.last_odom = monotonic()

    def heartbeat_cb(self, topic):
        self.last_seen[topic] = monotonic()

    def check(self):
        now = monotonic()
        width = float(self.get_parameter('arena_width_m').value)
        height = float(self.get_parameter('arena_height_m').value)
        timeout = float(self.get_parameter('watchdog_timeout_s').value)
        reasons = []
        if not (0.0 <= self.x <= width and 0.0 <= self.y <= height):
            reasons.append(f'geofence breach x={self.x:.3f} y={self.y:.3f}')
        if self.last_odom is None or now - self.last_odom > timeout:
            reasons.append('odometry missing or stale')
        for topic, stamp in self.last_seen.items():
            if stamp is None:
                reasons.append(f'heartbeat never received: {topic}')
            elif now - stamp > timeout:
                reasons.append(f'heartbeat stale: {topic}')
        if reasons:
            self.command_pub.publish(String(data='EMERGENCY_STOP'))
            self.reason_pub.publish(String(data='; '.join(reasons)))


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run safety_geofence_node')
    rclpy.init(args=args)
    node = SafetyGeofenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
