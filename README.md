# Balancing car: autonomous sandbox search

## Task 1–11 guide

See **[Tasks 1–11: labelled code, explanations and acceptance checks](docs/TASKS_01_TO_11.md)**.
Source comments use `TASK NN` labels. Existing numbered sections below are operating-guide sections, not task numbers.

Clone this repository into `~/ros2_ws/src/balancing_search` (the repository root is the ROS package):

```bash
git clone https://github.com/rejavedo/Balancing_grp12.git ~/ros2_ws/src/balancing_search
```


A ROS 2 Humble Python package for the supplied Wheeltec-style balancing platform,
Raspberry Pi 5, N10 2-D lidar and forward-facing USB camera. Uses the lab's existing
`yolo_ros` installation. Read this guide before enabling motion.

**Status:** source implementation with offline geometry/control tests. ROS 2,
Nav2, the camera, and the physical car are unavailable in the development environment.
No robot demonstration, navigation success rate, detection accuracy or measured
position accuracy is claimed. The project requires the hardware configuration below.
No map or target coordinates have been invented.

**Version 0.2:** Read [the revision notes](docs/REVISION.md) for the start-clearance,
AMCL, scan diagnostics and faster-search changes.

## What is included

| Requirement | Implementation |
|---|---|
| On-site map construction | SLAM Toolbox launch and save-map procedure |
| Localization on saved map | Map server + AMCL; operator sets initial pose before mission |
| Systematic autonomous search | Reachable, obstacle-cleared map coverage points; camera-FOV-aware viewing headings per point |
| Object detection | Existing YOLO pipeline; configurable target class and confidence |
| Target coordinates | Calibrated image ray/floor intersection, timestamped TF to map, multi-frame confirmation |
| Report, then return | Console and latched map point + JSON file; cancel search, navigate to captured start |
| Safe command handling | Robot-side velocity guard, freshness checks, acceleration limits, latched stop |
| Modular ROS system | Mission, localizer, guard, visualizer and scan-audit nodes; three launches, YAML configuration, Nav2 behavior tree |

The package does not replace the STM32 balancing controller or the supplied motor,
lidar, odometry, IMU and robot-description drivers. Keep those running as in the
previous labs. Their actual package/launch names and URDF were not supplied.

## 1. Mandatory configuration

Make these measurements/checks **before** changing the verification flags in
`config/mission.yaml`. They are explicit deployment settings, not automatic tests.

1. Use Ubuntu 22.04 and ROS 2 **Humble** on both machines. Use the lab-tested
   `yolo_ros` version with `yolo_bringup yolov8.launch.py` and
   `yolo_msgs/msg/DetectionArray`. Current upstream YOLO APIs can differ; do not
   upgrade the lab workspace blindly. Record its Git commit in your report.
2. Choose the actual target. Set `target_class` to the exact YOLO `class_name`.
   `bottle` is an example, not an assumption. A custom object absent from the
   model's classes requires a trained detector and validation data. This project
   does not include target-specific weights. There must be one unambiguous target
   of that class in view; multiple matching boxes are rejected.
3. Calibrate the camera **at the same 640x480 image size used for inference**.
   Publish a valid `CameraInfo` with K and distortion coefficients. This localizer
   expects original/raw image coordinates (JPEG decompression preserves them),
   not rectified images with a different K. Supported distortion models are
   `plumb_bob` and `rational_polynomial`.
4. Measure camera extrinsics. Its frame must be an optical frame: x right, y down,
   z forward. Supply `map -> odom -> base_link -> camera_optical_frame` and
   `base_link -> lidar_frame` through the robot's established TF tree. Never add
   a second publisher for an existing TF edge. Measure translation as well as
   orientation; camera height is not the lidar height.
5. Ground-plane localization requires a stationary, floor-standing target, a
   flat floor, and an unobscured bounding-box bottom approximating its contact
   point. `ground_z` is the floor elevation **in the map frame**, which may not
   be zero if the base-frame origin is above the floor. Establish the relationship
   from the robot model and measured camera height. Set `max_range` from validation.
6. **Account for balancing pitch.** The optical frame's orientation at the image
   timestamp must include actual body pitch/roll. If the vendor odometry/TF is
   planar, use its IMU-informed robot-description/TF solution to provide that
   orientation, or restrict acceptance to measured level stationary observations
   after implementing and validating a tilt gate. This package does not guess an
   IMU-to-camera transform. A static camera transform under a planar base frame
   while the body tilts will give biased coordinates. Do not set
   `calibration_verified: true` until this is resolved and measured points agree.
7. Check `/scan`, `/odom`, `/cmd_vel`, the base frame name, and sensor QoS. Set
   `base_frame` consistently in mapping, mission launch and the mission YAML.
   Scan timestamps and camera/TF timestamps must share synchronized clocks.
   Use chrony/NTP and verify host offset is substantially below 0.1 seconds.
8. Measure the circumscribed footprint radius, including wheels and protrusions.
   Pass it as `robot_radius` in the mission launch (0.28 m is an unverified starting
   value). Set `clearance` larger than this radius plus localization margin.
   Set `stop_range` from the lidar origin, accounting for its offset, footprint,
   stopping distance and latency. The supplied 0.38 m and 0.42 m are conservative
   starting settings, not measured dimensions. If they exclude narrow passages,
   adjust only after measurement. Both values must remain physically safe.
9. Confirm stable speeds, acceleration and braking on the real car. Nav2 and guard
   limits start at 0.15 m/s and 0.30 rad/s. Zero `/cmd_vel` means hold position through
   the active balancing controller, **not** switch off the balancing motors.
   Configure and test the vendor controller's own command timeout: if the guard
   process or Pi dies, it can no longer publish zero. Keep a physical stop available.
10. Set `calibration_verified: true` on the VM and `hardware_verified: true` on
    the Pi only after the corresponding checks. Defaults inhibit operation.

The 2-D lidar is used for mapping and collision avoidance. It does not provide
camera depth in this implementation. Objects below its scan plane may be invisible
to navigation; the demonstration environment must not contain undetectable hazards.
If the target is elevated, suspended or its floor contact is hidden, this localization
method is unsuitable: use an aligned depth sensor or a separately validated
camera/lidar association method instead.

## 2. Build on VM and Pi

Unzip the archive. Copy the `balancing_search` folder into `~/ros2_ws/src` on the VM
and the corresponding workspace `/home/ros2_ws/src` inside the Pi lab container.
Do not put the outer archive directory inside the package itself.

In the relevant workspace, with the existing lab `yolo_msgs` available:

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-tf2-ros ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-compressed-image-transport \
  ros-humble-v4l2-camera python3-numpy python3-scipy python3-opencv \
  python3-yaml python3-colcon-common-extensions
# VM workspace; use /home/ros2_ws on the Pi as in the lab.
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select balancing_search
source install/setup.bash
```

The guard itself does not import YOLO. If building a guard-only Pi workspace without
`yolo_msgs`, use `rosdep ... --skip-keys yolo_msgs`; the VM needs built and sourced
`yolo_msgs`. Do not rebuild or alter a working ML environment unnecessarily.

Every terminal on both machines needs its own setup and the same ROS domain:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=YOUR_GROUP_NUMBER
export ROS_LOCALHOST_ONLY=0
```

Replace `YOUR_GROUP_NUMBER` with an integer; do not run it literally. On the Pi use
`source /home/ros2_ws/install/setup.bash`. A Windows download must be copied into
Ubuntu or its VM/shared folder before Linux commands can access it.

## 3. Start the platform and camera

On the Pi, launch the **provided lab base-control, lidar, odometry and robot TF**
launch files. Use your working lab commands; the provided attachments do not give
the owning package of `balance_robot_base.launch.py`, so none is fabricated here.
Verify the base controller subscribes to `/cmd_vel` and publishes odometry and TF.

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Camera terminal on the Pi (replace device and calibration URL):

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r image_raw:=/camera/image_raw -r camera_info:=/camera/camera_info \
  -p video_device:=/dev/video0 -p image_size:='[640,480]' \
  -p camera_frame_id:=camera_optical_frame \
  -p camera_info_url:=file:///home/balance/camera_calibration.yaml
```

The YAML path must exist **inside the container running the driver**. Use your
calibration output, not a fabricated intrinsic matrix. The ROS camera-calibration
package can produce this file using a measured checkerboard.

Second Pi terminal:

```bash
ros2 run image_transport republish raw compressed --ros-args \
  -r in:=/camera/image_raw -r out/compressed:=/camera/image_raw/compressed
```

VM, separate terminals:

```bash
ros2 run image_transport republish compressed raw --ros-args \
  -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_decompressed
```

```bash
ros2 launch yolo_bringup yolov8.launch.py \
  model:=yolov8n.pt device:=cpu \
  input_image_topic:=/camera/image_decompressed \
  use_tracking:=False use_3d:=False use_debug:=True
```

Use the version-specific alternative image_transport syntax in your Lab 6 report
if this invocation is not supported. Confirm:

```bash
ros2 interface show yolo_msgs/msg/DetectionArray
ros2 interface show yolo_msgs/msg/Detection
ros2 interface show yolo_msgs/msg/BoundingBox2D
ros2 topic echo /camera/camera_info --once
ros2 topic echo /yolo/detections --once
ros2 topic hz /yolo/detections
```

Expected box fields: `bbox.center.position.x`, `.y`, `bbox.size.x`, `.y`.
The array must preserve the original image stamp and optical frame ID. Do not
replace its stamp with inference completion time. The reported lab rate is about
1.3 Hz, so four-frame confirmation and 3.5-second viewing pauses are used initially.
Measure the actual end-to-end latency and tune `max_age`/timeouts accordingly.

## 4. Demonstration step 1: map construction

1. Run the platform's normal base/lidar/TF stack. Do not run AMCL simultaneously.
2. On the VM:

```bash
ros2 launch balancing_search mapping.launch.py
```

3. Open RViz with fixed frame `map`. Add Map `/map`, LaserScan `/scan`, and TF.
4. Use examiner-permitted lab teleoperation to traverse the sandbox slowly, expose
   all navigable areas and close a loop. Check walls and free spaces align.
   **The autonomous guard is not running during mapping**; mapping uses the lab's
   approved manual control and stopping procedure.
5. Save in a VM terminal:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/sandbox
```

6. Verify `sandbox.yaml` and its referenced image exist, inspect the map in RViz,
   stop teleoperation, and stop SLAM Toolbox. Preserve both map files together.
   The saved occupancy grid is sufficient for AMCL; a SLAM pose graph is only
   needed if you intend to resume SLAM later.

The target is placed **after** this stage. Do not encode its location in the map
or in the source. Nav2's live obstacle layer handles newly observed obstacles.

## 5. Demonstration step 2: autonomous mission

1. Place the car at the examiner's assigned starting area. Keep the hardware
   drivers, camera and YOLO running; stop every manual `/cmd_vel` publisher.
2. On the Pi, run the guard with your edited parameters:

```bash
ros2 launch balancing_search robot_guard.launch.py \
  params:=/home/ros2_ws/src/balancing_search/config/mission.yaml
```

3. On the VM, start localization, navigation and mission nodes:

```bash
ros2 launch balancing_search mission.launch.py \
  map:=$HOME/maps/sandbox.yaml \
  params:=$HOME/ros2_ws/src/balancing_search/config/mission.yaml \
  base_frame:=base_link robot_radius:=0.28
```

Replace 0.28 with the measured radius. The launch loads Humble's installed Nav2
parameters and applies robot-specific overrides. It starts map server, AMCL,
planner, DWB controller, BT navigator and the custom nodes. The behavior tree
replans at 1 Hz and follows paths; blind backup and aggressive recovery spins
are intentionally absent. Failed goals are recorded and skipped, with two bounded
attempts to return home. The generated Nav2 YAML is left in `/tmp/balancing_nav2_*`
for debugging.

4. In RViz set fixed frame `map` and use **2D Pose Estimate** at the actual starting
   pose and heading. Confirm scan/map alignment and sensible AMCL covariance.
   Initial localization is setup before the autonomous mission; no manual movement
   commands should be issued once the mission starts. If localization cannot be
   established stationary, resolve it under the examiner's permitted setup procedure.
5. Verify `/cmd_vel` has **one publisher**, the robot's `velocity_guard`. Nav2 must
   publish only `/nav_cmd_vel`. No other process may bypass the guard.
6. Start recording in another VM terminal:

```bash
ros2 bag record -o sandbox_trial /map /scan /odom /tf /tf_static \
  /amcl_pose /camera/camera_info /yolo/detections /target/confirmed \
  /target/map_position /mission/status /mission/heartbeat /nav_cmd_vel /cmd_vel
```

7. Start the mission once:

```bash
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

The service rejects missing prerequisites, including a robot guard that is not ready.
Check `/guard/status` for the reason. Stationary AMCL updates are requested every
five seconds, even before start; wait for a fresh pose and retry if necessary. It captures the **current localized
start pose**, computes reachable viewing points and begins autonomy. It does not
assume that the start is map origin `(0,0)`.

8. Monitor the report and state:

```bash
ros2 topic echo /mission/status --qos-durability transient_local
ros2 topic echo /target/map_position --qos-durability transient_local
```

After confirmation, the console prints `TARGET (map, metres): (x.xxx, y.yyy)`;
`/target/map_position` contains the coordinates and `~/balancing_mission_result.json`
on the VM stores them. No example numerical result should be copied into a report
as a measured result. The robot cancels its search goal, waits for termination,
returns to the captured start, verifies position tolerance and disables motion.
`SUCCESS` means a target was reported and a successful return was verified.
`TARGET_NOT_FOUND` means it returned without a detection; check failed goals and
time budget before claiming full coverage. `FAULT` requires operator intervention.

Abort and latch a stop when needed:

```bash
ros2 service call /mission/abort std_srvs/srv/Trigger '{}'
ros2 service call /mission/estop std_srvs/srv/Trigger '{}'
```

`/mission/estop` is serviced on the Pi. A network service cannot replace a physical
stop during a network outage. After an abort, reset the guard latch only while the
mission is disabled, and restart the mission process before another run:

```bash
ros2 service call /mission/reset_estop std_srvs/srv/Trigger '{}'
```

## 6. How target coordinates are calculated

For a box `(u_c,v_c,w,h)`, take the visible bottom-centre contact point
`u = u_c`, `v = v_c + h/2`. Reject a contact point clipped by the image border.
Undistort `(u,v)` using K and D to obtain normalized coordinates `(x_n,y_n)`.
The camera optical ray is `d_c = normalize([x_n,y_n,1])`.

Lookup the transform **at the original image timestamp**:
`T_map_camera = [R,t]`. Then `d_m = R d_c` and

```
lambda = (ground_z - t_z) / d_m_z
p_map = t + lambda * d_m
reported position = (p_map_x, p_map_y) metres
```

Require a downward ray, positive lambda, valid calibration, fresh detections and
in-range intersection. Near-horizon rays are rejected because range error grows
rapidly there. Four distinct source frames must agree within 0.20 m over six
seconds. These thresholds are configurable; consensus reduces random noise but
does not remove calibration, pitch or bounding-box bias.

The reported point is the estimated **floor contact point**, not an inferred
3-D object centroid. Confirm which convention the examiner expects. Validate
against measured floor coordinates at several ranges and headings.

## 7. Search strategy and limitations

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

## 8. Tests and acceptance

Offline tests (no ROS install needed):

```bash
cd ~/ros2_ws/src/balancing_search
PYTHONPATH=. python3 test/test_core.py
PYTHONPATH=. python3 test/test_state_machine.py
python3 -m compileall -q balancing_search launch
```

The ten state-machine tests stub ROS interfaces to check cancellation
ordering, late acceptance after abort, stale inputs and verified return success.
They are unit tests, not action-server integration tests.

The twenty geometry/coverage/scan tests cover optical axes, map rotations/translations, invalid rays,
quaternions, temporal consensus, unknown/disconnected regions, map origin yaw,
invalid starts, diagonal corners and velocity/acceleration limits. These tests
cannot validate ROS launch compatibility, DDS, TF, camera drivers or dynamics.

| On-car test | Pass condition / evidence to record |
|---|---|
| Build and interface check | colcon succeeds; expected message types and only one velocity publisher |
| Mapping | Saved map matches sandbox geometry; scan overlap and loop closure verified |
| Localization | Measured start pose agrees; scan aligns at several poses |
| Camera calibration | Several known floor points meet the examiner's position-error criterion |
| Target detector | Correct class, acceptable false positives and latency at all test ranges |
| Guard network loss | Car stops safely when VM commands/heartbeat disappear |
| Guard process loss | Vendor base timeout stops commands if guard process dies |
| Sensor dropout | Stale scan/perception/TF cannot sustain mission motion |
| New obstacle | Local costmap detects it and Nav2 replans or stops safely |
| Goal cancellation | Target discovery cannot leave an old search goal executing |
| Target absent | Search ends, returns, reports not found without fabricated coordinates |
| Blocked home route | Bounded return attempts then FAULT; never false SUCCESS |
| Full trial | Map, place target, set pose, start, detect/report, return and stop without intervention |

Do not tune only on one target placement. Run several unseen positions, including
near a wall and partially occluded locations, and record target error, completion
time, return error, failures and interventions. Agree acceptable position/return
error thresholds with the examiner; the brief does not specify them.

## 9. Topic and frame contract

| Interface | Type | Producer -> consumer |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | N10 driver -> SLAM/AMCL/Nav2/guard |
| `/odom` | `nav_msgs/Odometry` | Base odometry -> Nav2 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Calibrated camera -> localizer/mission |
| `/yolo/detections` | `yolo_msgs/DetectionArray` | Lab YOLO -> localizer |
| `/target/confirmed` | `geometry_msgs/PointStamped` | Localizer -> mission |
| `/target/map_position` | `geometry_msgs/PointStamped` | Mission -> examiner/log; transient-local |
| `/perception/alive` | `std_msgs/Bool` | Valid timestamp/calibration/TF callback -> mission |
| `/nav_cmd_vel` | `geometry_msgs/Twist` | Nav2 controller -> guard |
| `/mission/heartbeat` | `std_msgs/Bool` | Mission -> guard |
| `/cmd_vel` | `geometry_msgs/Twist` | Guard only -> existing base controller |
| `/search/viewpoints` | `geometry_msgs/PoseArray` | Mission -> RViz; transient-local |
| `/mission/markers` | `visualization_msgs/MarkerArray` | Visualizer -> RViz target and coordinates |
| `/guard/ready`, `/guard/status` | `std_msgs/Bool`, `std_msgs/String` | Guard -> start gate/operator |
| `/mission/status` | `std_msgs/String` | Mission -> operator/log; transient-local |

TF ownership: SLAM publishes `map -> odom` during mapping; AMCL replaces it during
the mission. The existing base stack owns `odom -> base_link` (or its documented
base-footprint/body chain). The robot model owns lidar and camera extrinsics.

## 10. Sources and version notes
- Nav2 Humble source/API: https://api.nav2.org/nav2-humble/html/
- Nav2 Humble default parameters (loaded from installed package at launch):
  https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml
- Lab YOLO upstream: https://github.com/mgonzs13/yolo_ros
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox

The geometry, mission state machine and guard are project code. Use the source in
your submission according to your course AI-use rules; do not claim unperformed
hardware testing or personal authorship contrary to those rules.
