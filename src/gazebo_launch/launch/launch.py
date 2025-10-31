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
    
    # Or use an empty world from gazebo_ros
    empty_world_path = PathJoinSubstitution([
        get_package_share_directory('gazebo_ros'),
        'worlds',
        'empty.world'
    ])

    # Launch Gazebo (server and GUI)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [' -r -s -v4 ', empty_world_path] # -r: record, -s: server, -v4: verbose level, empty_world_path: your world file
        }.items()
    )

    # Spawning a URDF model 
    robot_description_path = crobot_description + "/description/robot.urdf.xacro"
    with open(robot_description_path, 'r') as file:
        robot_description_content = file.read()

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'crobot'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     parameters=[{'robot_description': robot_description_content}]
        # ),
        spawn_entity_node,
    ])