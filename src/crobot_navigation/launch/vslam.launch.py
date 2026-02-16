#!/usr/bin/env python3
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ##################################################
    # RealSense launch arguments
    ##################################################
    enable_infra1 = LaunchConfiguration('enable_infra1')
    enable_infra2 = LaunchConfiguration('enable_infra2')
    enable_color  = LaunchConfiguration('enable_color')
    enable_depth  = LaunchConfiguration('enable_depth')
    enable_gyro   = LaunchConfiguration('enable_gyro')
    enable_accel  = LaunchConfiguration('enable_accel')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    enable_sync   = LaunchConfiguration('enable_sync')
    initial_reset = LaunchConfiguration('initial_reset')
    emitter_enabled = LaunchConfiguration('emitter_enabled')

    ##################################################
    # VSLAM launch arguments
    ##################################################
    tracking_mode = LaunchConfiguration('tracking_mode')  # 0 = stereo, 1 = visual-inertial, 2 = RGBD
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame  = LaunchConfiguration('map_frame')

    ##################################################
    # RealSense driver node (separate process)
    ##################################################
    realsense_camera_node = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
        parameters=[{
            # Streams
            'enable_infra1': enable_infra1,
            'enable_infra2': enable_infra2,
            'enable_color':  enable_color,
            'enable_depth':  enable_depth,

            # IMU
            'enable_gyro':   enable_gyro,
            'enable_accel':  enable_accel,
            'unite_imu_method': unite_imu_method,

            # Time sync / startup
            'enable_sync': enable_sync,
            'initial_reset': initial_reset,

            'emitter_enabled': emitter_enabled,
        }]
    )

    ##################################################
    # Isaac ROS Visual SLAM (Composable node)
    ##################################################
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            # frames / TF publishing for Nav2
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'publish_map_to_odom_tf': True,
            'publish_odom_to_base_tf': True,

            # stereo camera
            'num_cameras': 2,
            'rectified_images': True,

            # 0 = stereo, 1 = visual-inertial, 2 = RGBD
            'tracking_mode': tracking_mode,

            'enable_slam_visualization': True,
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),

            ('visual_slam/image_1', '/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),

            # IMU input (only used if tracking_mode == 1)
            ('visual_slam/imu', '/camera/imu'),

        ],
    )

    container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([
        # --- realSense arguments ---
        DeclareLaunchArgument('enable_infra1', default_value='true'),
        DeclareLaunchArgument('enable_infra2', default_value='true'),
        DeclareLaunchArgument('enable_color',  default_value='false'),
        DeclareLaunchArgument('enable_depth',  default_value='false'),

        DeclareLaunchArgument('enable_gyro',   default_value='true'),
        DeclareLaunchArgument('enable_accel',  default_value='true'),
        DeclareLaunchArgument('unite_imu_method', default_value='copy'),
        DeclareLaunchArgument('enable_sync',   default_value='true'),
        DeclareLaunchArgument('initial_reset', default_value='true'),
        DeclareLaunchArgument('emitter_enabled', default_value='false'),

        # --- VSLAM arguments ---
        DeclareLaunchArgument('tracking_mode', default_value='1'),
        DeclareLaunchArgument('base_frame', default_value='robot/base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame',  default_value='map'),

        # actions
        realsense_camera_node,
        container,
    ])
