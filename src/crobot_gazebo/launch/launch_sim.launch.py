import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch.actions import RegisterEventHandler
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
    
    print(models_folder)

    # world = LaunchConfiguration('world')
    # world = os.path.join(worlds_folder, 'mining_mayhem.world')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]), launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
            # 'world_name': [PathJoinSubstitution([worlds_folder, LaunchConfiguration('world_file')])],
        }.items()
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'crobot'
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

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'world',
        #     default_value='',
        #     description='World file to load'
        # ),
        # SetLaunchConfiguration(name='world_file', 
        #                        value=[LaunchConfiguration('world'), 
        #                               TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), ':', models_folder]),
        rsp,
        gazebo,
        spawn_entity,
        # delayed_diff_drive_spawner,
        # joint_broad_spawner
    ])