from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters= {
        'frame_id': 'base_link',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'use_action_for_goal': True,
        'qos_image': qos,
        'qos_imu': qos,
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySIgma': '0'
    }

    remappings=[
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        ('odom', '/diff_drive_controller/odom')
    ]

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[{'approx_sync': False, 'use_sim_time': use_sim_time, 'qos': qos}],
        remappings=remappings
    )

    # SLAM Mode:
    slam_mapping = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d']
    )
        
    # Localization mode:
    slam_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters,
            {'Mem/IncrementalMemory':'False',
            'Mem/InitWMWithAllNodes':'True'}],
        remappings=remappings
    )

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[parameters],
        remappings=remappings
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'qos',
            default_value='2',
            description='QoS (Quality of service) used for input sensor topics'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Launch in localization mode'
        ),
        slam_mapping,
        slam_localization,
        rgbd_sync,
        # rtabmap_viz
    ])