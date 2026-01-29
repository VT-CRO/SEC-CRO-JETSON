from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    urdf_path = os.path.join(get_package_share_path('crobot_description'), 'description', 'robot.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ', urdf_path, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Enable ROS2 control if true'
        ),
        robot_state_publisher_node
    ])
