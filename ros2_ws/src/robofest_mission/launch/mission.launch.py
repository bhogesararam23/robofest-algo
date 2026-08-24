from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robofest_mission', executable='mission_state_machine', name='mission_state_machine', output='screen'),
        Node(package='robofest_mission', executable='swarm_node', name='swarm_communication', output='screen'),
        Node(package='robofest_mission', executable='safety_geofence_node', name='safety_geofence', output='screen'),
    ])
