import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction,LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #launches the nav2 server
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'  
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': PathJoinSubstitution([
                get_package_share_directory('crobot_gazebo'),
                'field',
                'map.yaml'
            ]),  
            'params_file': PathJoinSubstitution([
                get_package_share_directory('crobot_behavior'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    #launch bringup
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('crobot_bringup'),
                'launch',
                'bringup_testing.launch.py'
            ])
        ])        
    )

    #launches rviz2
    rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ])
    )
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crobot_navigation'),
                'launch',
                'vslam.launch.py'
            ])
        ])
    )

    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crobot_navigation'),
                'launch',
                'nvblox.launch.py'
            ])
        ])
    )
    
    delayed_vslam = TimerAction(
        period=5.0,      
        actions=[
            LogInfo(msg='Starting VSLAM...'),
            vslam_launch,
        ]
    )

    delayed_nvblox = TimerAction(
        period=10.0,     
        actions=[
            LogInfo(msg='Starting NVBLOX...'),
            nvblox_launch,
        ]
    )

    delayed_rviz2 = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='Starting NVBLOX...'),
            rviz2_launch,      
        ]
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",   # important for remote laptop access
            # "use_sim_time": True,  # uncomment if you want it to use sim time
        }],
    )

    return LaunchDescription([
        bringup_launch,
        delayed_vslam,
        delayed_nvblox,
        delayed_rviz2,
        nav2_launch,
        foxglove_bridge,
    ])