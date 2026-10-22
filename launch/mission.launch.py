# TASK 03: AMCL localization; TASK 04: Nav2 navigation; TASK 10: integrated launch.
"""ROS 2 Humble only; inherit installed Nav2 defaults and override robot limits.

Hardware drivers, URDF/TF and camera/YOLO are deliberately external: their actual
packages, extrinsics and target are not supplied in the assignment.
"""
import os
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def configure(context):
    if os.environ.get('ROS_DISTRO') != 'humble':
        raise RuntimeError('This configuration targets ROS 2 Humble; port/test before using another distribution')
    share=get_package_share_directory('balancing_search')
    nav=get_package_share_directory('nav2_bringup')
    map_path=os.path.abspath(os.path.expanduser(LaunchConfiguration('map').perform(context)))
    if not os.path.isfile(map_path): raise RuntimeError('Supply the saved map YAML')
    base=LaunchConfiguration('base_frame').perform(context)
    radius=float(LaunchConfiguration('robot_radius').perform(context))
    if radius <= 0: raise RuntimeError('Measure a positive circumscribed robot radius')
    params=LaunchConfiguration('params').perform(context)
    with open(params) as f: project=yaml.safe_load(f)
    limits=project['velocity_guard']['ros__parameters']
    v,w=limits['max_v'],limits['max_w']
    a,alpha=limits['acceleration'],limits['angular_acceleration']
    if min(v,w,a,alpha) <= 0: raise RuntimeError('Velocity/acceleration limits must be positive')
    if project['search_mission']['ros__parameters']['transit_clearance'] < radius:
        raise RuntimeError('Transit clearance cannot be smaller than robot radius')
    with open(os.path.join(nav,'params','nav2_params.yaml')) as f: cfg=yaml.safe_load(f)
    amcl=cfg['amcl']['ros__parameters']
    amcl.update(base_frame_id=base,odom_frame_id='odom',global_frame_id='map',
                scan_topic='/scan',set_initial_pose=False,tf_broadcast=True,
                update_min_d=.05,update_min_a=.05)
    controller=cfg['controller_server']['ros__parameters']
    controller.update(controller_frequency=15.0,min_x_velocity_threshold=.001,
                      min_y_velocity_threshold=.001,min_theta_velocity_threshold=.001,
                      controller_plugins=['FollowPath'],goal_checker_plugins=['general_goal_checker'])
    controller['progress_checker'].update(required_movement_radius=.05,movement_time_allowance=40.0)
    controller['general_goal_checker'].update(xy_goal_tolerance=.12,yaw_goal_tolerance=.12,stateful=True)
    controller['FollowPath'].update(plugin='dwb_core::DWBLocalPlanner',
        min_vel_x=0.0,min_vel_y=0.0,max_vel_x=v,max_vel_y=0.0,max_vel_theta=w,
        min_speed_xy=0.0,max_speed_xy=v,min_speed_theta=0.0,
        acc_lim_x=a,acc_lim_y=0.0,acc_lim_theta=alpha,
        decel_lim_x=-a,decel_lim_y=0.0,decel_lim_theta=-alpha,
        vx_samples=12,vy_samples=1,vtheta_samples=20,sim_time=2.0,
        trans_stopped_velocity=.03,xy_goal_tolerance=.12)
    for name,frame in [('local_costmap','odom'),('global_costmap','map')]:
        cost=cfg[name][name]['ros__parameters']
        cost.update(robot_base_frame=base,global_frame=frame,robot_radius=radius,
                    resolution=.05,track_unknown_space=True,
                    plugins=(['obstacle_layer','inflation_layer'] if name=='local_costmap'
                             else ['static_layer','obstacle_layer','inflation_layer']))
        cost.pop('footprint',None)
        cost['obstacle_layer']={'plugin':'nav2_costmap_2d::ObstacleLayer','enabled':True,
            'observation_sources':'scan','scan':{'topic':'/scan','data_type':'LaserScan',
            'clearing':True,'marking':True,'raytrace_max_range':8.0,'raytrace_min_range':0.0,
            'obstacle_max_range':7.5,'obstacle_min_range':0.0,'max_obstacle_height':2.0}}
        cost['inflation_layer'].update(inflation_radius=max(.55,radius+.2),cost_scaling_factor=3.0)
    cfg['planner_server']['ros__parameters']['GridBased'].update(allow_unknown=False,tolerance=.10)
    cfg['bt_navigator']['ros__parameters'].update(global_frame='map',robot_base_frame=base,
        odom_topic='/odom',default_nav_to_pose_bt_xml=os.path.join(share,'config','search_tree.xml'))
    for section in cfg.values():
        if isinstance(section,dict) and 'ros__parameters' in section:
            section['ros__parameters']['use_sim_time']=False
    for name in ('local_costmap','global_costmap'):
        cfg[name][name]['ros__parameters']['use_sim_time']=False
    # File persists in /tmp for inspection; no mutation of system Nav2 files.
    fd,path=tempfile.mkstemp(prefix='balancing_nav2_',suffix='.yaml')
    with os.fdopen(fd,'w') as f: yaml.safe_dump(cfg,f)
    actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav,'launch','localization_launch.py')),
        launch_arguments={'map':map_path,'params_file':path,'use_sim_time':'false',
                          'autostart':'true','use_composition':'False'}.items())]
    nodes=[('nav2_controller','controller_server'),('nav2_planner','planner_server'),
           ('nav2_bt_navigator','bt_navigator')]
    for package,name in nodes:
        actions.append(Node(package=package,executable=name,name=name,output='screen',
                            parameters=[path],remappings=[('cmd_vel','/nav_cmd_vel')]))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_navigation',output='screen',
        parameters=[{'autostart':True,'node_names':[n for _,n in nodes]}]))
    params=LaunchConfiguration('params').perform(context)
    for exe in ('target_localizer','mission','visualizer'):
        actions.append(Node(package='balancing_search',executable=exe,output='screen',
            parameters=[params,{'base_frame':base}] if exe=='mission' else [params]))
    return actions


def generate_launch_description():
    share=get_package_share_directory('balancing_search')
    return LaunchDescription([
        DeclareLaunchArgument('map',description='Absolute path to saved map YAML'),
        DeclareLaunchArgument('params',default_value=os.path.join(share,'config','mission.yaml')),
        DeclareLaunchArgument('base_frame',default_value='base_link'),
        DeclareLaunchArgument('robot_radius',default_value='0.28',description='MEASURE before running'),
        OpaqueFunction(function=configure)])
