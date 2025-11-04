from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the share directory of your package
    my_pkg_share_dir = get_package_share_directory('gazebo_launch')
    crobot_description = get_package_share_directory('crobot_description')
    
    # Define the path to your world file (if you have a custom one)
    # Example: world_file_path = os.path.join(my_pkg_share_dir, 'worlds', 'my_world.sdf')
    
    # Or use an empty world from ros_gz_sim package
    empty_world_path = PathJoinSubstitution([
        get_package_share_directory('ros_gz_sim'),
        'worlds',
        'empty.world'
    ])

    # Launch New Gazebo (server and GUI)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            # -r is really important for starting gazebo in a running state
            # -v 4 sets the verbosity level to 4 for more detailed output
            'gz_args': ['-r -v 4 empty.sdf']
        }.items()
    )

    # Spawning a URDF model 
    # robot_description_path = crobot_description + "/description/robot.urdf.xacro"
    # with open(robot_description_path, 'r') as file:
    #     robot_description_content = file.read()

    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([get_package_share_directory('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': 'crobot_description',
            'urdf_package_path': 'description/robot.urdf.xacro'}.items()
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'crobot',
            '-z', '0.15'
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

    return LaunchDescription([
        gazebo_launch,
        description_launch_py,
        spawn_entity_node,
        spawn_joint_state_broadcaster,
        # i doubt we'll use diff drive because each wheel needs to be independently controlled
        # spawn_diff_drive,
        spawn_ankle_joint_controller,
        spawn_wheel_velocity_controller,
    ])