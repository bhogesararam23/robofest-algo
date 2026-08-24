"""50 Hz MAVROS velocity bridge with latched emergency-stop handling."""
from time import monotonic

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, Path
    from std_srvs.srv import Trigger
except ImportError:
    rclpy = None

try:
    from mavros_msgs.srv import CommandBool
except ImportError:
    CommandBool = None


class FCBridgeNode(Node):
    def __init__(self):
        super().__init__('fc_bridge')
        self.declare_parameter('max_velocity_mps', 0.7)
        self.declare_parameter('waypoint_tolerance_m', 0.3)
        self.declare_parameter('takeoff_altitude_m', 1.5)
        self.declare_parameter('emergency_zero_hold_s', 1.0)
        self.declare_parameter('disarm_timeout_s', 5.0)
        self.declare_parameter('disarm_retry_period_s', 0.5)
        self.command = 'HOLD'
        self.pose = (0.0, 0.0, 0.0)
        self.path = []
        self.index = 0
        self.emergency_latched = False
        self.emergency_since = 0.0
        self.disarm_deadline = 0.0
        self.next_disarm_attempt = 0.0
        self.disarm_attempts = 0
        self.disarm_future = None
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.heartbeat = self.create_publisher(String, '/drivers/heartbeat', 10)
        self.status_pub = self.create_publisher(String, '/drivers/emergency_status', 10)
        self.create_subscription(String, '/mission/current_command', self.command_cb, 10)
        self.create_subscription(Path, '/navigation/safe_path', self.path_cb, 10)
        self.create_subscription(String, '/navigation/path_status', self.path_status_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_service(Trigger, '/drivers/reset_emergency', self.reset_emergency_cb)
        self.disarm_client = self.create_client(CommandBool, '/mavros/cmd/arming') if CommandBool else None
        self.create_timer(0.02, self.control_tick)

    def command_cb(self, msg):
        command = msg.data.strip().upper()
        if command == 'EMERGENCY_STOP':
            self.latch_emergency('mission command')
            return
        if self.emergency_latched:
            self.get_logger().warning(f'Ignoring {command} while emergency stop is latched')
            return
        self.command = command
        self.index = 0

    def path_cb(self, msg):
        if self.emergency_latched:
            return
        self.path = list(msg.poses)
        self.index = 0

    def odom_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

    def path_status_cb(self, msg):
        if msg.data == 'NO_SAFE_PATH' and not self.emergency_latched:
            self.command = 'HOLD'
            self.path = []
            self.index = 0
            self.get_logger().error('Planner reported NO_SAFE_PATH; bridge entered HOLD')

    def latch_emergency(self, reason):
        if not self.emergency_latched:
            now = monotonic()
            self.emergency_latched = True
            self.emergency_since = now
            self.disarm_deadline = now + float(self.get_parameter('disarm_timeout_s').value)
            self.next_disarm_attempt = now
            self.disarm_attempts = 0
            self.get_logger().error(f'EMERGENCY_STOP latched: {reason}')
        self.command = 'EMERGENCY_STOP'
        self.path = []
        self.index = 0

    def reset_emergency_cb(self, _request, response):
        if self.emergency_latched and monotonic() < self.disarm_deadline:
            response.success = False
            response.message = 'Cannot reset while emergency disarm window is active'
            return response
        self.emergency_latched = False
        self.command = 'HOLD'
        self.disarm_future = None
        response.success = True
        response.message = 'Emergency latch cleared; vehicle remains in HOLD'
        return response

    def request_disarm(self, now):
        if self.disarm_client is None or self.disarm_future is not None or now < self.next_disarm_attempt or now > self.disarm_deadline:
            return
        self.next_disarm_attempt = now + float(self.get_parameter('disarm_retry_period_s').value)
        if not self.disarm_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warning('MAVROS arming service unavailable; retrying disarm')
            return
        request = CommandBool.Request()
        request.value = False
        self.disarm_attempts += 1
        self.disarm_future = self.disarm_client.call_async(request)
        self.disarm_future.add_done_callback(self.disarm_done_cb)

    def disarm_done_cb(self, future):
        self.disarm_future = None
        try:
            result = future.result()
            if not result.success:
                detail = getattr(result, 'message', f'result={getattr(result, "result", "unknown")}')
                self.get_logger().error(f'MAVROS disarm rejected: {detail}')
        except Exception as exc:
            self.get_logger().error(f'MAVROS disarm call failed: {exc}')

    def control_tick(self):
        now = monotonic()
        out = Twist()
        x, y, z = self.pose
        if self.emergency_latched or self.command == 'EMERGENCY_STOP':
            self.latch_emergency('active command')
            self.request_disarm(now)
        elif self.command in ('SEARCH', 'GUIDE') and self.path:
            target = self.path[min(self.index, len(self.path) - 1)].pose.position
            dx, dy, dz = target.x - x, target.y - y, target.z - z
            distance = (dx * dx + dy * dy + dz * dz) ** 0.5
            if distance < float(self.get_parameter('waypoint_tolerance_m').value) and self.index < len(self.path) - 1:
                self.index += 1
            scale = min(1.0, float(self.get_parameter('max_velocity_mps').value) / max(distance, 0.01))
            out.linear.x, out.linear.y, out.linear.z = dx * scale, dy * scale, dz * scale
        elif self.command == 'TAKEOFF':
            out.linear.z = 0.6 if z < float(self.get_parameter('takeoff_altitude_m').value) else 0.0
        # HOLD, LAND, YIELD_GUIDANCE, and unknown commands intentionally remain zero.
        self.vel_pub.publish(out)
        heartbeat = String(); heartbeat.data = 'fc_bridge'; self.heartbeat.publish(heartbeat)
        if self.emergency_latched:
            status = String(); status.data = f'LATCHED attempts={self.disarm_attempts} deadline={self.disarm_deadline:.3f}'; self.status_pub.publish(status)


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run fc_bridge_node')
    rclpy.init(args=args)
    node = FCBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
