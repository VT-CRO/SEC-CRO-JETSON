import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Robot State Publisher - publishes URDF and transforms
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crobot_description'), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

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

    # Spawn controllers using controller_manager spawner
    # spawn_joint_state_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    #     output='screen'
    # )

    # spawn_ankle_joint_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['ankle_position_controller'],
    #     output='screen'
    # )

    # spawn_wheel_velocity_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['wheel_velocity_controller'],
    #     output='screen'
    # )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crobot_gazebo'), 'launch', 'gazebo.launch.py'
        )])
    )

    # Teleop mapper - converts /cmd_vel to wheel/ankle commands
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('teleop_control'), 'launch', 'teleop_launch.py'
        )])
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",   # important for remote laptop access
            # "use_sim_time": True,  # uncomment if you want it to use sim time
        }],
    )

    return LaunchDescription([
        rsp,
        control_node,
        # spawn_joint_state_broadcaster,
        # spawn_ankle_joint_controller,
        # spawn_wheel_velocity_controller,
        gazebo,
        teleop,
        foxglove_bridge
    ])