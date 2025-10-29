from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='bump_and_go_lidar',
            executable='control_node',
        )
    ])