# TASK 02: SLAM Toolbox integration; TASK 10: ROS launch configuration.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share=get_package_share_directory('balancing_search')
    return LaunchDescription([
        DeclareLaunchArgument('params',default_value=os.path.join(share,'config','slam.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')),
            launch_arguments={'slam_params_file':LaunchConfiguration('params'),
                              'use_sim_time':'false'}.items())])
