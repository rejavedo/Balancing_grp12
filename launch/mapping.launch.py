# TASK 02: SLAM mapping on the robot — reuses the lab bringup (base, lidar, scan resampler, SLAM Toolbox).
"""Phase A. Drive with balance_keyboard_control, then save with scripts in README §4."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup=os.path.join(get_package_share_directory('balance_bringup'),'launch','balance_slam_bringup.launch.py')
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup))])
