from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get the share directory of your package
    my_pkg_share_dir = get_package_share_directory('crobot_gazebo')

    field_path = PathJoinSubstitution([my_pkg_share_dir,'field','field.sdf'])

    world_path = PathJoinSubstitution([my_pkg_share_dir, 'world', 'empty.sdf'])

    map_path = os.path.join(my_pkg_share_dir, 'field','map.yaml')
    
    
    # Or use an empty world from ros_gz_sim package
    # empty_world_path = PathJoinSubstitution([
    #     get_package_share_directory('ros_gz_sim'),
    #     'worlds',
    #     'empty.world'
    # ])

    # Launch New Gazebo (server and GUI)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            # -r is really important for starting gazebo in a running state
            # -v 4 sets the verbosity level to 4 for more detailed output
            'ign_args': [world_path, TextSubstitution(text = ' -r')]
        }.items()
    )

    spawn_entity_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'crobot',
            '-x', '0.53',
            '-y', '-1.10',
            '-z', '0.43'
        ],
        output='screen'
    )

    #The field
    spawn_field_node = Node(
        package = 'ros_ign_gazebo',
        executable='create',
        arguments=[
                '-entity', 'Field', # Name of the spawned model
                '-file', field_path,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0'
            ],
        output='screen'
    )

    # Keep map→odom static since we're not doing localization
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_params = os.path.join(my_pkg_share_dir, 'config', 'ros_gz_bridge.yaml')

    bridge_node = Node(
        package='ros_ign_bridge',          # or ros_ign_bridge if that's what you use
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    # A node for remapping the lidar tf frame to visualise in RVIZ---------------ONLY to be used in debugging plugins--------not a part of

    lidar_tf_fix = Node( 
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'robot/base_link/gpu_lidar'],
    parameters=[{'use_sim_time': True}],
    output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(my_pkg_share_dir, 'config', 'ekf.yaml'),
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity_node,
        spawn_field_node,
        bridge_node,
        # Toggle this for TESTING PLUGINS
        lidar_tf_fix,
        map_to_odom_tf,
        ekf_node
    ])