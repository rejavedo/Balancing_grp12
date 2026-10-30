# balancing_search — autonomous map / search / report / return for the balancing car

ROS 2 Humble Python package for the WheelTec B585 balancing car (Raspberry Pi 5, N10P lidar, USB camera).
Everything runs **on the Pi inside the `ros2_humble` container**; no VM, no `yolo_ros`.
It reuses the lab packages: `balance_bringup` (SLAM), `balance_navigation` (AMCL + Nav2, which drives `/cmd_vel`),
`balance_robot_base`, `lslidar_driver_n10p`, `balance_usb_cam`, and ultralytics with `/home/ros2_ws/weights/yolo26n.pt`.

Mission: Phase A, map with SLAM Toolbox while driving by keyboard, then save the map. Phase B, one launch:
AMCL is initialised at the mapping start (map origin), the robot visits generated viewpoints and turns through
6 headings at each, YOLO finds the target class, the floor-contact point is projected into the map, the
`(x, y)` is printed and saved, then the robot returns to its start and stops.

## 1. Install / update (from a PC with this repo)

```bash
rsync -a --exclude .git --exclude __pycache__ --exclude '*.egg-info' ./ balance@balance17.local:ros2_ws/src/balancing_search/
ssh balance@balance17.local
ros2-start                                     # = docker start + docker exec -it ros2_humble bash
cd /home/ros2_ws && colcon build --symlink-install --packages-select balancing_search && source install/setup.bash
```

Every container terminal: `source /home/ros2_ws/install/setup.bash` (root `.bashrc` already sets `ROS_DOMAIN_ID=17`).
Check the camera is plugged in: `ls /dev/video0`.

## 2. Phase A — mapping (examiner-permitted teleop)

Hold the robot when starting the base (opening the serial port can reset the STM32).
Start mapping **at the assigned start position, facing the way it will face in Phase B**: that pose becomes map `(0, 0, 0)`.

```bash
# terminal 1: base + lidar + scan resampler + SLAM Toolbox
ros2 launch balancing_search mapping.launch.py
# terminal 2: drive slowly round the arena, close the loop
ros2 run balance_robot_base balance_keyboard_control
# terminal 3: save → /home/ros2_ws/maps/arena.yaml + .pgm
/home/ros2_ws/install/balancing_search/lib/balancing_search/save_map.sh arena
```

Drive back to the start, then Ctrl-C terminals 1 and 2.

## 3. Phase B — autonomous mission

Put the robot at the start (same pose as the mapping start), hands off, then:

```bash
ros2 launch balancing_search mission.launch.py map:=/home/ros2_ws/maps/arena.yaml target_class:=bottle
```

`target_class` = exact COCO name (`bottle`, `cup`, `sports ball`, `teddy bear`, `chair`, …).
The launch starts base, camera, lidar, map server + AMCL (5 s), Nav2 (9 s), detector (3 s) and the mission (12 s).
The mission sends the initial pose to AMCL, waits until map, AMCL, Nav2 and perception are ready, and
**starts by itself** (`auto_start`). Watch it:

```bash
ros2 topic echo /mission/status --qos-durability transient_local     # IDLE → SEARCH/NAVIGATING/DWELL … → SUCCESS
ros2 topic echo /target/map_position --qos-durability transient_local
ros2 topic echo /target/diagnostics                                   # per-frame detector result
cat /home/ros2_ws/balancing_mission_result.json                       # final report
```

The mission log prints `TARGET (map, metres): (x, y)` when confirmed and `MISSION SUCCESS — TARGET …` after it
is back within 0.25 m of the start. `TARGET_NOT_FOUND` = searched everything (or 600 s budget) and returned.
`FAULT` = needs an operator (log says why).

| Flag | Default | Use |
|---|---|---|
| `target_class` | `bottle` | target COCO class |
| `auto_start` | `true` | `false` → start with `ros2 service call /mission/start std_srvs/srv/Trigger` |
| `with_base` / `with_camera` / `with_nav` / `with_perception` / `with_mission` | `true` | turn pieces off for bench tests |
| `params` | `config/mission.yaml` | all tunables (start pose, spacing, dwell, timeouts, detector) |

If the start is not the mapping start, set `initial_x/initial_y/initial_yaw` in `config/mission.yaml`
(or use RViz "2D Pose Estimate" before the mission starts, then `auto_start:=false` and call `/mission/start`).

Stop: `ros2 service call /mission/abort std_srvs/srv/Trigger '{}'` (cancels Nav2, sends zero `/cmd_vel`), or Ctrl-C.
Keep a hand near the robot: a network service is not an emergency stop.

## 4. Diagnosing

| Symptom | Check |
|---|---|
| `Waiting to start: Need map, fresh TF/AMCL…` repeats | `ros2 topic echo /amcl_pose --once`; covariance must be < 0.3 m² (xy) / 0.15 rad² (yaw); `ros2 topic hz /scan`; Nav2 lifecycle in launch log |
| `target_class … not in model classes` | spelling of the COCO class |
| `/target/diagnostics` says `no TF` | base not running (`base_footprint → camera_link` comes from `balance_robot_base`) |
| `/target/diagnostics` says `stale image` | camera lag; `ros2 topic hz /image_raw` |
| `Start violates physical transit clearance` | robot too close to a wall at start, or AMCL pose wrong |
| Robot oscillates / goals fail | Nav2 params are the lab ones in `balance_navigation/config/nav2_core_params.yaml` |

## 5. How target coordinates are calculated

For a box `(u_c,v_c,w,h)`, take the visible bottom-centre contact point
`u = u_c`, `v = v_c + h/2`. Reject a contact point clipped by the image border.
Undistort `(u,v)` using K and D to obtain normalized coordinates `(x_n,y_n)`.
The camera optical ray is `d_c = normalize([x_n,y_n,1])`; because the robot's `camera_link` is body-style
(x forward, y left, z up), it is rotated by `OPTICAL_TO_LINK` first (`optical_frame: false`).

Lookup the transform **at the original image timestamp**:
`T_map_camera = [R,t]`. Then `d_m = R d_c` and

```
lambda = (ground_z - t_z) / d_m_z
p_map = t + lambda * d_m
reported position = (p_map_x, p_map_y) metres
```

Require a downward ray, positive lambda, fresh detections and
in-range intersection. Near-horizon rays are rejected because range error grows
rapidly there. Three distinct frames must agree within 0.25 m over eight
seconds (`confirm_*`). These thresholds are configurable; consensus reduces random noise but
does not remove calibration, pitch or bounding-box bias.

The reported point is the estimated **floor contact point**, not an inferred
3-D object centroid. Confirm which convention the examiner expects. Validate
against measured floor coordinates at several ranges and headings.

## 6. Search strategy and limitations

The occupancy grid is eroded by physical transit clearance, treating unknown space
and map boundaries as obstacles. Four-connected flood fill keeps only the component
containing the start. Viewpoints use a larger clearance; the nearest reachable
viewpoint can therefore be selected when the start lacks the larger viewing margin.
An unsafe starting footprint is still rejected; no wall cells are cleared. A greedy nearest-next sampler selects viewing points until
all retained cell centres lie within the configured spacing of a point. At least six
orientation goals at each point provide a 360-degree sweep with stationary dwell.
The actual number increases automatically for the calibrated camera field of view,
with 15% angular overlap (for example, a 60-degree FOV requires eight headings).
Nav2 plans around mapped and live obstacles; no open-loop timed turns are used.

Choose heading increments smaller than the calibrated horizontal field of view
with overlap. Tune spacing to validated detector range and target visibility.
This is a systematic reachable-space search, **not a proof that every possible
object is visible**: occlusion, excluded narrow passages, map errors, failed goals,
range limits and mission timeout can leave unseen regions. Record these limitations
rather than calling a failed/partial run complete coverage.

## 7. Tests

```bash
PYTHONPATH=. python3 test/test_core.py           # 21 geometry / coverage / consensus tests
PYTHONPATH=. python3 test/test_state_machine.py  # 11 mission state-machine tests (ROS stubbed)
```

Bench dry run without the chassis (nothing can move): live lidar, a still image as camera
(`test/fake_camera.py`), static `odom_combined → base_footprint`, real AMCL / Nav2 / mission — see
`docs/VALIDATION.md`.

## 8. Topics and frames

| Interface | Type | Producer → consumer |
|---|---|---|
| `/image_raw` | `sensor_msgs/Image` rgb8 640×480 | usb_cam → target_localizer |
| `/target/observation` | `geometry_msgs/PointStamped` (map) | localizer, every accepted frame |
| `/target/confirmed` | `geometry_msgs/PointStamped` (map) | localizer (3-frame consensus) → mission |
| `/target/diagnostics` | `std_msgs/String` | localizer → operator |
| `/perception/alive` | `std_msgs/Bool` | localizer (every processed frame) → mission |
| `/target/map_position` | `geometry_msgs/PointStamped`, transient-local | mission → examiner/RViz |
| `/mission/status` | `std_msgs/String`, transient-local | mission → operator |
| `/search/viewpoints` | `geometry_msgs/PoseArray` | mission → RViz |
| `/initialpose` | `PoseWithCovarianceStamped` | mission → AMCL (3× at start) |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 (normal); mission sends zeros for 2 s after finish/fault |

Frames: `map → odom_combined` (SLAM in phase A, AMCL in phase B) `→ base_footprint` (base) `→ laser, camera_link` (static, base launch).
`velocity_guard` / `robot_guard.launch.py` are not used in this path (see docs/REVISION.md).
