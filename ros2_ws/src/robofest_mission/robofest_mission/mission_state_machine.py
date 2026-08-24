"""Action-driven hierarchical mission state machine for RoboFest."""
import asyncio

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry, Path
    from std_srvs.srv import Trigger
    from robofest_interfaces.action import MissionCommand
except ImportError:
    rclpy = None


class MissionStateMachine(Node):
    STATES = ('IDLE', 'TAKEOFF', 'SEARCH', 'PLAN_PATH', 'GUIDE_HUMAN', 'LAND')

    def __init__(self):
        super().__init__('mission_state_machine')
        self.state = 'IDLE'
        self.x = self.y = self.z = 0.0
        self.path_ready = False
        self.path_length = 0
        self.command_pub = self.create_publisher(String, '/mission/current_command', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/navigation/safe_path', self.path_cb, 10)
        self.create_service(Trigger, '/mission/start', self.start_cb)
        self.create_timer(0.1, self.transition_tick)
        self.server = ActionServer(
            self, MissionCommand, '/mission/command',
            execute_callback=self.execute, goal_callback=self.goal, cancel_callback=self.cancel,
        )

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

    def path_cb(self, msg):
        self.path_length = len(msg.poses)
        self.path_ready = self.path_length > 0

    def start_cb(self, _request, response):
        self.set_state('TAKEOFF')
        response.success = True
        response.message = 'Mission started'
        return response

    def transition_tick(self):
        if self.state == 'TAKEOFF' and self.z > 1.5:
            self.set_state('SEARCH')
        elif self.state == 'SEARCH' and self.path_ready:
            self.set_state('PLAN_PATH')
        elif self.state == 'PLAN_PATH':
            self.set_state('GUIDE_HUMAN' if self.path_ready else 'SEARCH')
        elif self.state == 'GUIDE_HUMAN' and self.x >= 58.0 and abs(self.y - 7.5) <= 1.0:
            self.set_state('LAND')
        elif self.state == 'LAND' and self.z < 0.3:
            self.set_state('IDLE')

    def set_state(self, state):
        if state not in self.STATES or state == self.state:
            return
        self.state = state
        self.command_pub.publish(String(data=state if state != 'GUIDE_HUMAN' else 'GUIDE'))

    def goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    async def execute(self, goal_handle):
        command = goal_handle.request.command.upper()
        requested_state = {'TAKEOFF': 'TAKEOFF', 'SEARCH': 'SEARCH', 'GUIDE': 'GUIDE_HUMAN', 'LAND': 'LAND'}.get(command)
        if requested_state is None:
            goal_handle.abort()
            result = MissionCommand.Result()
            result.success = False
            result.final_status = f'Unsupported command: {command}'
            return result
        self.set_state(requested_state)
        feedback = MissionCommand.Feedback()
        feedback.current_state = self.state
        for step in range(1, 6):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MissionCommand.Result()
                result.success = False
                result.final_status = 'Command canceled'
                return result
            await asyncio.sleep(0.1)
            feedback.current_state = self.state
            feedback.progress_percent = step * 20.0
            goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result = MissionCommand.Result()
        result.success = True
        result.final_status = f'{self.state} command accepted'
        return result


def main(args=None):
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run mission_state_machine')
    rclpy.init(args=args)
    node = MissionStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
