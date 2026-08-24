from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('robofest_navigation'), 'config', 'ekf.yaml')
    return LaunchDescription([
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen', parameters=[params], remappings=[('/odometry/filtered', '/odom')],
        ),
        Node(
            package='robofest_navigation', executable='sensor_adapter_node',
            name='localization_sensor_adapter', output='screen',
        ),
        Node(
            package='robofest_navigation', executable='localization_heartbeat_node',
            name='localization_heartbeat', output='screen',
        ),
    ])
