from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_control',
            executable='teleop_mapper',
            name='teleop_mapper',
            output='screen'
        )
    ])
