from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    LogInfo,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crobot_bringup'),
                'launch',
                'bringup_testing.launch.py'
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

    return LaunchDescription([
        LogInfo(msg='Starting Bringup...'),
        bringup_launch,
        delayed_vslam,
        delayed_nvblox,
    ])
