from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([FindPackageShare('robofest_description'), 'worlds', 'arena.world'])
    ekf = PathJoinSubstitution([FindPackageShare('robofest_navigation'), 'config', 'ekf.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_sitl', default_value='false'),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world], output='screen',
            condition=IfCondition(LaunchConfiguration('start_gazebo')),
        ),
        ExecuteProcess(
            cmd=['ardupilot', '--model', 'quad'], output='screen',
            condition=IfCondition(LaunchConfiguration('start_sitl')),
        ),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen', parameters=[ekf], remappings=[('/odometry/filtered', '/odom')],
        ),
        Node(package='robofest_navigation', executable='sensor_adapter_node', output='screen'),
        Node(package='robofest_navigation', executable='localization_heartbeat_node', output='screen'),
        Node(package='robofest_perception', executable='fake_vision_node', output='screen'),
        Node(package='robofest_navigation', executable='mine_mapping_node', output='screen'),
        Node(package='robofest_navigation', executable='path_planner_node', output='screen'),
        Node(package='robofest_drivers', executable='fc_bridge_node', output='screen'),
        Node(package='robofest_mission', executable='mission_state_machine', output='screen'),
        Node(package='robofest_mission', executable='swarm_node', output='screen'),
        Node(package='robofest_mission', executable='safety_geofence_node', output='screen'),
    ])
