# TASK 01: Robot-side command guard; TASK 10: launch and parameters.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params',default_value=os.path.join(
            get_package_share_directory('balancing_search'),'config','mission.yaml')),
        Node(package='balancing_search',executable='velocity_guard',output='screen',
             parameters=[LaunchConfiguration('params')])])
