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
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crobot_description'), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

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
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['crobot_drive_controller', 'sweeper_position_controller', 'winch_velocity_controller'],
        output='screen'
    )

    delay_controllers_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_controllers],
    ))

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

    # Teleop Twist Keyboard (Only for manual control)
    # now publishes to /crobot_drive_controller/cmd_vel
    # teleop_node = Node(
    #     package="teleop_twist_keyboard",
    #     executable="teleop_twist_keyboard",
    #     name="teleop_twist_keyboard",
    #     output="screen",
    #     prefix="xterm -e",  # open in new terminal window
    #     remappings=[
    #         ("/cmd_vel", "/crobot_drive_controller/cmd_vel")
    #     ]
    # )

    isaac_vslam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('isaac_ros_visual_slam'),
                    'launch',
                    'isaac_ros_visual_slam.launch.py',  # replace with real file
                ])
            ),
            launch_arguments={
                'use_sim_time': 'true',
                # add other args that Isaac launch exposes (if any)
            }.items()
        )

    return LaunchDescription([
        rsp,
        control_node,
        spawn_joint_state_broadcaster,
        delay_controllers_after_joint_state,
        # foxglove_bridge,
        # teleop_node,
        #isaac_vslam,
    ])