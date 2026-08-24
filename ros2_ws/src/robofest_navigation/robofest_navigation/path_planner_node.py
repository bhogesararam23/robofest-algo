"""Dynamic event-driven and periodic A* replanning with stale-path invalidation."""
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from robofest_interfaces.msg import MineMap
except ImportError:
    rclpy = None

from .planner_core import GridPlanner


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.declare_parameter('goal_x', 59.0)
        self.declare_parameter('goal_y', 7.5)
        self.declare_parameter('replan_period_s', 0.5)
        self.planner = GridPlanner()
        self.pose = (0.5, 7.5)
        self.goal = None
        self.have_map = False
        self.last_path_valid = False
        self.pub = self.create_publisher(Path, '/navigation/safe_path', 10)
        self.status_pub = self.create_publisher(String, '/navigation/path_status', 10)
        self.create_subscription(MineMap, '/navigation/global_mine_map', self.map_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/mission/goal_pose', self.goal_cb, 10)
        self.create_timer(float(self.get_parameter('replan_period_s').value), self.replan)

    def odom_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.replan()

    def map_cb(self, msg):
        # Full rebuild makes additions, moved clusters, and confidence-based removals authoritative.
        self.planner.rebuild((mine.x, mine.y) for mine in msg.mines)
        self.have_map = True
        self.replan()

    def publish_empty_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.poses = []
        self.pub.publish(msg)

    def publish_status(self, status):
        self.status_pub.publish(String(data=status))

    def replan(self):
        if not self.have_map:
            self.publish_status('WAITING_FOR_MAP')
            return
        goal = self.goal or (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
        )
        path = self.planner.plan(self.pose, goal)
        if path is None:
            # Explicitly clear the previous route so the bridge cannot continue following stale geometry.
            self.last_path_valid = False
            self.publish_empty_path()
            self.publish_status('NO_SAFE_PATH')
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.poses = []
        for x, y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub.publish(msg)
        self.last_path_valid = True
        self.publish_status('PATH_VALID')


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run path_planner_node')
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
