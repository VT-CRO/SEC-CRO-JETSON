import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch.actions import RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

import xacro

def generate_launch_description():

    package_name='crobot_gazebo'

    models_folder=os.path.join(
        get_package_share_directory(package_name), 'models'
    )

    worlds_folder=os.path.join(
        get_package_share_directory(package_name), 'worlds'
    )

    robot_meshes_folder=os.path.join(
        get_package_share_directory('crobot_description'), 'description', 'drivetrain'
    )
    
    print(models_folder)

    world = LaunchConfiguration('world')
    world = os.path.join(worlds_folder, 'mining_mayhem.world')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crobot_description'), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'mining_mayhem.world'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description="World to load"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true' }.items()
            # 'world_name': [PathJoinSubstitution([worlds_folder, LaunchConfiguration('world_file')])],
       
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'crobot',
                                   '-x', '0.5',
                                   '-z', '0.1'
                                  ],
                        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"]
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner]
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml') 
    ros_gz_bridge = Node(
      package="ros_gz_bridge",
      executable="parameter_bridge",
      arguments=[
         '--ros-args',
         '-p',
         f'config_file:={bridge_params}'
      ]
   )
    ros_gz_image_bridge = Node(
      package="ros_gz_image",
      executable="image_bridge",
      arguments=["/camera/image_raw"]
   )
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'world',
        #     default_value='',
        #     description='World file to load'
        # ),
        # SetLaunchConfiguration(name='world_file', 
        #                        value=[LaunchConfiguration('world'), 
        #                               TextSubstitution(text='.sdf')]),
        #SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), ':', models_folder, ':', robot_meshes_folder]),
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge
        # delayed_diff_drive_spawner,
        # joint_broad_spawner
    ])
