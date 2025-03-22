import os
from time import sleep
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
  pkg_nav2_dir = get_package_share_directory('nav2_bringup')

  nav2_launch_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_nav2_dir, 'launch', 'navigation_launch.py')
      ),
      launch_arguments={
          'use_sim_time': 'true'
      }.items()
  )
  
  return LaunchDescription([
    nav2_launch_cmd
  ])