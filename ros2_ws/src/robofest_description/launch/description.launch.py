from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    urdf=PathJoinSubstitution([FindPackageShare('robofest_description'),'urdf','drone.urdf.xacro'])
    return LaunchDescription([Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description':Command(['xacro ',urdf])}])])
