import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

def generate_launch_description():

    package_name = 'crobot_bringup'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('crobot_gazebo'), 'launch', 'launch_sim.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'world': os.path.join(
                get_package_share_directory('crobot_gazebo'), 'worlds', 'mining_mayhem.world'
            )
        }.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('crobot_navigation'), 'launch', 'rgbd.launch.py'
        ))
    )

    nav2_dir = get_package_share_directory('nav2_bringup')
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',

            # need to specify a params file otherwise nav2 doesnt work
            'params_file': os.path.join(
                get_package_share_directory('crobot_navigation'), 'config', 'nav2_params.yaml'
            )
        }.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[
            ("cmd_vel_out", "diff_drive_controller/cmd_vel_unstamped")
        ]
    )

    # Add the apriltag_ros node here
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        remappings=[
            ('/image_rect', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ],
        parameters=[{
            'tags_36h11': os.path.join(
                get_package_share_directory('apriltag_ros'),
                'cfg', 'tags_36h11.yaml'
            )
        }]
    )

    crobot_nav_server = Node(
        package='crobot_navigation',
        executable='crobot_navigation_server',
        name='crobot_navigation_server',
        output='screen',
        remappings=[
            ('/odom', '/diff_drive_controller/odom')
        ]
    )

    crobot_bt = Node(
        package='crobot_navigation',
        executable='crobot_navigation_node',
        name='crobot_navigation_node',
        output='screen'
    )

    # Convert URDF to SDF
    xacro_cmd = ExecuteProcess(
        cmd=['xacro', 'src/crobot_description/description/robot.urdf.xacro', '>', '/tmp/robot.urdf'],
        shell=True
    )
    
    sdf_cmd = ExecuteProcess(
        cmd=['gz', 'sdf', '-p', '/tmp/robot.urdf', '>', '/tmp/robot.sdf'],
        shell=True
    )
    
    # Spawn using SDF file
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-file', '/tmp/robot.sdf', '-name', 'crobot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        # slam,
        # nav,
        twist_mux,
        apriltag_node,  # Add apriltag node to launch description
        crobot_nav_server,
        TimerAction(period=5.0, actions=[crobot_bt]),
        xacro_cmd,
        sdf_cmd,
        spawn_entity
    ])
