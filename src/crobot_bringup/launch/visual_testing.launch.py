# main_launch.py
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo
)
from launch.event_handlers import OnProcessExit, OnExecutionComplete
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

    start_vslam_after_bringup = RegisterEventHandler(
        OnExecutionComplete(
            target_action=bringup_launch,
            on_completion=[
                LogInfo(msg='Bringup finished! Starting VSLAM...'),
                vslam_launch,
            ]
        )
    )

    start_nvblox_after_vslam = RegisterEventHandler(
        OnExecutionComplete(
            target_action=vslam_launch,
            on_completion=[
                LogInfo(msg='VSLAM finished! Starting NVBLOX...'),
                nvblox_launch,
            ]
        )
    )

    return LaunchDescription([
        LogInfo(msg='Starting Bringup...'),
        bringup_launch,
        start_vslam_after_bringup,
        start_nvblox_after_vslam
    ])
