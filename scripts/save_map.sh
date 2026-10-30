#!/usr/bin/env bash
# Save the SLAM map for phase B. Usage (inside container): save_map.sh [name]   default name: arena
set -e
name=${1:-arena}; dir=/home/ros2_ws/maps; mkdir -p "$dir"
ros2 run nav2_map_server map_saver_cli -f "$dir/$name" --ros-args -p save_map_timeout:=10.0
ls -l "$dir/$name.yaml" "$dir/$name.pgm"
