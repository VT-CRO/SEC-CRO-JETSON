import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # # Robot State Publisher - publishes URDF and transforms
    # rsp = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('crobot_description'), 'launch', 'rsp.launch.py'
    #     )]),
    #     launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    # )

    # Controller Manager node
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('crobot_controller'),
        "config",
        "ros2_control.yaml"
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ]
    )

    crobot_behavior = Node(
        package='crobot_behavior',
        executable="bt_launch",
        parameters=[{
                "bt_xml": "/home/ros/ws/src/SEC-NEW-JETSON/src/crobot_behavior/trees/default.xml"
            }]
    )
    return LaunchDescription([
        # rsp,
        crobot_behavior
    ])
