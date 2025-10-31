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
            'gz_args': ['empty.sdf']
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
        description_launch_py,
        spawn_entity_node,
    ])