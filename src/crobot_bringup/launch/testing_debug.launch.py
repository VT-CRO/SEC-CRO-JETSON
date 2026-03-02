import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # -------------------------------------------------------
    # NAV2 - navigation only (no map server, no AMCL)
    # VSLAM is the sole source of the map frame
    # -------------------------------------------------------
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

    # -------------------------------------------------------
    # Robot bringup (hardware, urdf, etc.)
    # -------------------------------------------------------
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('crobot_bringup'),
                'launch',
                'bringup_testing.launch.py'
            ])
        ])
    )

    # -------------------------------------------------------
    # VSLAM - sole map frame publisher (map -> odom -> base_link)
    # Make sure enable_loop_closure: True in vslam.launch.py
    # -------------------------------------------------------
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crobot_navigation'),
                'launch',
                'vslam.launch.py'
            ])
        ])
    )

    # -------------------------------------------------------
    # NVBLOX - 3D mapping + costmap for nav2
    # Publishes /nvblox_node/static_map and /nvblox_node/dynamic_map
    # Nav2 should be configured to use nvblox costmap plugin
    # -------------------------------------------------------
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crobot_navigation'),
                'launch',
                'nvblox.launch.py'
            ])
        ])
    )

    # -------------------------------------------------------
    # EKF - fuses VSLAM odometry + wheel odometry
    # Make sure nav2_params.yaml has correct ekf config:
    #   odom0: /visual_slam/tracking/odometry
    #   odom1: /odom  (wheel odometry)
    # -------------------------------------------------------
    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            get_package_share_directory('crobot_behavior'),
            'config',
            'nav2_params.yaml'
        ])]
    )

    # -------------------------------------------------------
    # RViz2
    # -------------------------------------------------------
    rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ])
    )

    # -------------------------------------------------------
    # Foxglove bridge for remote monitoring
    # -------------------------------------------------------
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",
        }],
    )

    # -------------------------------------------------------
    # Startup sequence with delays
    # Order matters:
    #   1. Bringup (hardware)
    #   2. VSLAM (establishes map frame first)
    #   3. EKF (needs vslam odometry topic to exist)
    #   4. NVBLOX (needs map frame from vslam)
    #   5. Nav2 (needs costmap from nvblox + map frame from vslam)
    #   6. RViz2 / Foxglove
    # -------------------------------------------------------
    delayed_vslam = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='Starting VSLAM...'),
            vslam_launch,
        ]
    )

    delayed_ekf = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='Starting EKF...'),
            ekf_localization,
        ]
    )

    delayed_nvblox = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='Starting NVBLOX...'),
            nvblox_launch,
        ]
    )

    delayed_nav2 = TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg='Starting NAV2...'),
            nav2_launch,
        ]
    )

    delayed_rviz2 = TimerAction(
        period=22.0,
        actions=[
            LogInfo(msg='Starting RVIZ2...'),
            rviz2_launch,
        ]
    )

    delayed_foxglove = TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg='Starting Foxglove...'),
            foxglove_bridge,
        ]
    )

    return LaunchDescription([
        bringup_launch,
        delayed_vslam,
        delayed_ekf,
        delayed_nvblox,    # re-enabled - needed for nav2 costmap
        delayed_nav2,
        delayed_rviz2,
        delayed_foxglove,
        # Removed:
        #   - map_server       (no static map in Option A)
        #   - map_lifecycle    (no static map in Option A)
        #   - initial_pose     (not needed, vslam initializes its own frame)
        #   - pointcloud_to_laserscan (nvblox handles costmap directly)
    ])