# No-motion check: chassis driver off (nothing can move), static odom->base TF, fake camera without the target.
source /opt/ros/humble/setup.bash; source /home/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=17 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
pkill -f "[b]alance_robot_node"; pkill -f "[u]sb_cam"; pkill -f "[l]slidar"; pkill -f "[s]tatic_transform"; sleep 1
mkdir -p /tmp/nm; cd /tmp/nm; rm -f *.log /home/ros2_ws/balancing_mission_result.json
A=$(python3 -c "import ultralytics,os;print(os.path.dirname(ultralytics.__file__))")/assets/bus.jpg
ros2 run tf2_ros static_transform_publisher --frame-id odom_combined --child-frame-id base_footprint > /dev/null 2>&1 &
ros2 run tf2_ros static_transform_publisher --x 0.002 --z 0.1875 --frame-id base_footprint --child-frame-id laser > /dev/null 2>&1 &
ros2 run tf2_ros static_transform_publisher --x -0.035 --z 0.2655 --frame-id base_footprint --child-frame-id camera_link > /dev/null 2>&1 &
python3 /home/ros2_ws/src/balancing_search/test/fake_camera.py $A 5 > /dev/null 2>&1 &
ros2 topic echo /mission/status std_msgs/msg/String --qos-durability transient_local > status.log 2>&1 &
(sleep 60; ros2 topic info /cmd_vel -v > cmdvel_info.log 2>&1; ros2 topic info /cmd_vel_nav -v >> cmdvel_info.log 2>&1) &
timeout -s INT 420 ros2 launch balancing_search mission.launch.py map:=/home/ros2_ws/maps/arena.yaml target_class:=bottle with_base:=false with_camera:=false > launch.log 2>&1
sleep 3; pkill -f "[f]ake_camera"; pkill -f "[s]tatic_transform"; pkill -f "[t]opic echo"; pkill -f "[l]slidar"
