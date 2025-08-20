import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    package_name = 'apriltag_ros'

    realsense_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={}.items()
    )
    # remappings=[
    #     ('rgb/image', '/camera/image_raw'),
    #     ('rgb/camera_info', '/camera/camera_info'),
    #     ('depth/image', '/camera/depth/image_raw'),
    #     ('odom', '/diff_drive_controller/odom')
    # ]

    apriltag_remappings = [
        ('image_rect', '/camera/camera/color/image_raw'),
        ('camera_info', '/camera/camera/color/camera_info')
    ]

    apriltag_params_file = os.path.join(get_package_share_directory(package_name), 'cfg', 'tags_36h11.yaml')

    apriltags = Node(
        package = 'apriltag_ros',
        executable = 'apriltag_node',
        output = 'screen',
        parameters = [{'approx_sync': False}],
        launch_arguments = {'extra_apriltag_args': '--ros-args --params-file' + apriltag_params_file}.items(),
        remappings = apriltag_remappings
    )

    return LaunchDescription([
        realsense_launch_file,
        apriltags
        # rtabmap_viz
    ])
