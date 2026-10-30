# TASKS 03, 04, 06, 10: Phase B on the robot — base, lidar, camera, AMCL, Nav2 and the mission nodes.
"""Reuses balance_navigation's AMCL and Nav2 (both tuned for this robot; Nav2 drives /cmd_vel)."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def include(package, name, condition=None, **args):
    path=os.path.join(get_package_share_directory(package),'launch',name)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=args.items(), condition=condition)


def generate_launch_description():
    share=get_package_share_directory('balancing_search')
    arg=LaunchConfiguration
    params=arg('params')
    nodes=[Node(package='balancing_search',executable='target_localizer',output='screen',
                parameters=[params,{'target_class':arg('target_class')}],condition=IfCondition(arg('with_perception'))),
           Node(package='balancing_search',executable='visualizer',output='screen')]
    mission=Node(package='balancing_search',executable='mission',output='screen',
                 parameters=[params,{'auto_start':arg('auto_start')}],condition=IfCondition(arg('with_mission')))
    return LaunchDescription([
        DeclareLaunchArgument('map',description='Saved map YAML from phase A'),
        DeclareLaunchArgument('target_class',default_value='bottle',description='COCO class name of the target'),
        DeclareLaunchArgument('params',default_value=os.path.join(share,'config','mission.yaml')),
        DeclareLaunchArgument('auto_start',default_value='true'),
        DeclareLaunchArgument('with_base',default_value='true',description='false: no chassis driver (no motion possible)'),
        DeclareLaunchArgument('with_camera',default_value='true'),
        DeclareLaunchArgument('with_nav',default_value='true'),
        DeclareLaunchArgument('with_perception',default_value='true'),
        DeclareLaunchArgument('with_mission',default_value='true'),
        include('balance_robot_base','balance_robot_base.launch.py',IfCondition(arg('with_base'))),
        include('balance_usb_cam','usb_cam.launch.py',IfCondition(arg('with_camera')),mode='default'),
        TimerAction(period=2.,actions=[include('lslidar_driver_n10p','lslidar_launch.py')]),
        TimerAction(period=5.,actions=[LogInfo(msg='[mission] map server + AMCL'),
            include('balance_navigation','balance_amcl_localization.launch.py',IfCondition(arg('with_nav')),map=arg('map'))]),
        TimerAction(period=9.,actions=[LogInfo(msg='[mission] Nav2'),
            include('balance_navigation','balance_nav2_core.launch.py',IfCondition(arg('with_nav')))]),
        # YOLO loading saturates the CPU; start it after the Nav2 lifecycle bringup (DDS service replies time out otherwise).
        TimerAction(period=16.,actions=nodes),
        TimerAction(period=18.,actions=[mission]),
    ])
