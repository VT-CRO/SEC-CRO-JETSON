import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #launches the nav2 server
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'  
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': PathJoinSubstitution([
                get_package_share_directory('crobot_behavior'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             get_package_share_directory('nav2_bringup'),
    #             'launch',
    #             'bringup_launch.py'  
    #         ])
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': 'false',
    #         'slam': False,
    #         'map': PathJoinSubstitution([
    #             get_package_share_directory('crobot_gazebo'),
    #             'field',
    #             'map.yaml'
    #         ]),  
    #         'params_file': PathJoinSubstitution([
    #             get_package_share_directory('crobot_behavior'),
    #             'config',
    #             'nav2_params.yaml'
    #         ])
    #     }.items()
    # )

    #launch bringup
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('crobot_bringup'),
                'launch',
                'bringup_testing.launch.py'
            ])
        ])        
    )

    #launches rviz2
    rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ])
    )
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crobot_navigation'),
                'launch',
                'vslam.launch.py'
            ])
        ])
    )

    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crobot_navigation'),
                'launch',
                'nvblox.launch.py'
            ])
        ])
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.05,
            'max_height': 0.1,        # only scan up to 30cm high (your obstacle height)
            'angle_min': -1.5708,     # -90°
            'angle_max': 1.5708,      # 90°
            'angle_increment': 0.0087,
            'scan_time': 0.3333,
            'range_min': 0.30,
            'range_max': 1.0,
            'use_inf': True,
        }], 
        remappings=[
            ('cloud_in', '/camera0/depth/color/points'),  # ← your realsense pointcloud topic
            ('scan', '/scan')
        ]
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

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': PathJoinSubstitution([
                get_package_share_directory('crobot_gazebo'),
                'field',
                'map.yaml'
            ]),  
            'use_sim_time': False
        }]
    )

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    ekf_localization = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[PathJoinSubstitution([
                get_package_share_directory('crobot_behavior'),
                'config',
                'ekf.yaml'
            ])]
        )

    initial_pose = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--times', '10', '/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": 0.541, "y": -0.976, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.6926, "w": 0.7214}}}}'
        ],
        output='screen'
    )

    delay_initial_pose = TimerAction(
        period=20.0,
        actions=[initial_pose]
    )
    
    delayed_vslam = TimerAction(
        period=5.0,      
        actions=[
            LogInfo(msg='Starting VSLAM...'),
            vslam_launch,
        ]
    )

    delayed_ekf = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg='Starting EKF...'),
            ekf_localization,
        ]
    )

    delayed_nvblox = TimerAction(
        period=10.0,     
        actions=[
            LogInfo(msg='Starting NVBLOX...'),
            nvblox_launch,
        ]
    )

    delayed_nav2 = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='Starting NAV2'),
            nav2_launch,
            map_server,
            map_lifecycle,
        ]
    )

    delayed_rviz2 = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='Starting RVIZ2...'),
            rviz2_launch,      
        ]
    )

    delayed_foxglove = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='Starting FOXGLOVE...'),
            foxglove_bridge,      
        ]
    )

    map_odom_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )

    return LaunchDescription([
        bringup_launch,
        # map_server,
        # map_lifecycle,
        delayed_vslam,
        # delayed_ekf,
        # delayed_nvblox,
        delayed_rviz2,
        delayed_nav2,
        delayed_foxglove,
        # delay_initial_pose,
        # pointcloud_to_laserscan,
        # map_odom_publisher,
    ])