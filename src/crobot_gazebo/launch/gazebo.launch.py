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
    crobot_description = get_package_share_directory('crobot_description')

    field_path = PathJoinSubstitution([my_pkg_share_dir,'field','field.sdf'])

    world_path = PathJoinSubstitution([my_pkg_share_dir, 'world', 'empty.sdf'])
    
    
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
            'ign_args': [world_path, TextSubstitution(text = ' -r -v 4')]
        }.items()
    )

    # Spawning a URDF model 
    # robot_description_path = crobot_description + "/description/robot.urdf.xacro"
    # with open(robot_description_path, 'r') as file:
    #     robot_description_content = file.read()

    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([get_package_share_directory('crobot_description'), 'launch', 'rsp.launch.py']),
        launch_arguments={
            'urdf_package': 'crobot_description',
            'urdf_package_path': 'description/robot.urdf.xacro'}.items()
    )


    spawn_entity_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'crobot',
            '-x', '0.53',
            '-y', '-3.10',
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

    # spawn controllers using controller_manager spawner
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # spawn_diff_drive = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['diff_drive_controller'],
    #     output='screen'
    # )

    spawn_ankle_joint_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ankle_position_controller'],
        output='screen'
    )

    spawn_wheel_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller'],
        output='screen'
    )

    bridge_params = os.path.join(my_pkg_share_dir,'config','ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )


    return LaunchDescription([
        gazebo_launch,
        description_launch_py,
        spawn_entity_node,
        spawn_field_node,
        spawn_joint_state_broadcaster,
        # i doubt we'll use diff drive because each wheel needs to be independently controlled
        # spawn_diff_drive,
        spawn_ankle_joint_controller,
        spawn_wheel_velocity_controller,
        ros_gz_bridge
    ])