import launch
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    realsense_camera_node = Node(
        name='camera0',
        namespace='camera0',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': False,
            'enable_depth': False,
            # 'enable_pointcloud': False,      
            # 'pointcloud.enable': False,
            'depth_module.emitter_enabled': 0,
            # 'depth_module.emitter_on_off': False,
            # '640x360x60'
            'depth_module.infra_profile': '640,360,60',
            'depth_module.profile': '640,360,60',
            # 'rgb_camera.profile': '1920x1080x30',
            'enable_gyro': True,
            'enable_accel': True,
            'gyro_fps': 200,
            'accel_fps': 250,
            'unite_imu_method': 2,
            # 'base_frame_id':'camera0_link',
            'camera_name': 'camera0',
            # 'depth_module.depth_units':0.001,
            # 'depth_module.min_distance':0.1,
            # 'depth_module.max_distance':4.0, # meters
         }],
    )

    # splitter_node = ComposableNode(
    #     namespace='camera0',
    #     name='realsense_splitter_node',
    #     package='realsense_splitter',
    #     plugin='nvblox::RealsenseSplitterNode',
    #     parameters=[{
    #         'input_qos': 'SENSOR_DATA',
    #         'output_qos': 'SENSOR_DATA'
    #     }],
    #     remappings=[
    #         ('input/infra_1', f'/camera0/infra1/image_rect_raw'),
    #         ('input/infra_1_metadata', f'/camera0/infra1/metadata'),
    #         ('input/infra_2', f'/camera0/infra2/image_rect_raw'),
    #         ('input/infra_2_metadata', f'/camera0/infra2/metadata'),
    #         ('input/depth', f'/camera0/depth/image_rect_raw'),
    #         ('input/depth_metadata', f'/camera0/depth/metadata'),
    #         ('input/pointcloud', f'/camera0/depth/color/points'),
    #         ('input/pointcloud_metadata', f'/camera0/depth/metadata'),
    #     ])

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'publish_map_to_odom_tf': True,
            #change this to false if using EKF (ekef publishes this automatically)
            'publish_odom_to_base_tf': False,
            'use_imu': False,
            'enable_loop_closure': True,
            'enable_image_denoising': False,
            'enable_localization_n_mapping': True,
            'rectified_images': True,
            'tracking_mode': 0, # VIO mode (IMU fusion)
            'enable_rectified_pose': True,
            'enable_imu_fusion': False,
            # 'gyro_noise_density': 0.000244,
            # 'gyro_random_walk': 0.000019393,
            # 'accel_noise_density': 0.001862,
            # 'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 19.00,
            'base_frame': 'base_link',
            'imu_frame': 'camera0_gyro_optical_frame',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            # 'feature_detector_threshold': 0.02,
            # 'num_features_threshold': 45,
            # 'harris_k': 0.08,
            'camera_optical_frames': [
                'camera0_infra1_optical_frame',
                'camera0_infra2_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/image_0', 'camera0/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', 'camera0/infra1/camera_info'),
            ('visual_slam/image_1', 'camera0/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', 'camera0/infra2/camera_info'),
            ('visual_slam/imu', 'camera0/imu'),
            ('imu','/camera0/imu'),
            # ('/camera/infra1/image_rect_raw','/camera0/infra1/image_rect_raw'),
            # ('/camera/infra2/image_rect_raw','/camera0/infra2/image_rect_raw'),
            # ('/camera/infra1/camera_info','/camera0/infra1/camera_info'),
            # ('/camera/infra2/camera_info','/camera0/infra2/camera_info'),
        ],
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        # composable_node_descriptions=[visual_slam_node, splitter_node],
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    load_map_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/visual_slam/localize_in_map',
            'isaac_ros_visual_slam_interfaces/srv/LocalizeInMap',
            '"{map_folder_path: \'/ssd/olivia_test_ws/vslam_map\', pose_hint: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"'
        ],
        shell=True
    )

    trigger_map_load = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=visual_slam_launch_container,
            on_start=[load_map_cmd]
        )
    )

    return launch.LaunchDescription([
        visual_slam_launch_container,
        realsense_camera_node,
        trigger_map_load
    ])