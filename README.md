# balancing_search — map, search, report, return

Run steps only. Everything runs on the Pi inside the `ros2_humble` container.
How it works, limits and test evidence (course docs folder, next to this repo): [system](../docs/systems.md) · [all robot commands](../docs/entry.md) · [mission contract](../docs/contracts/mission.md)

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

It starts everything it needs and **starts the search by itself** after about 30 s, once map, AMCL, Nav2 and the camera are ready. Do not run SLAM, keyboard teleop or a follower at the same time.

| Flag | Default | Use |
|---|---|---|
| `target_class` | `bottle` | exact COCO name (`bottle`, `cup`, `sports ball`, `teddy bear`, `chair`, …) |
| `auto_start` | `true` | `false` → start with `ros2 service call /mission/start std_srvs/srv/Trigger` |
| `with_base` / `with_camera` / `with_nav` / `with_perception` / `with_mission` | `true` | turn pieces off for bench tests (`with_base:=false` = no motion possible) |
| `params` | `config/mission.yaml` | all tunables (start pose, spacing, dwell, timeouts, detector) |
| `nav_params` | `config/nav2.yaml` | Nav2 params (lab ones; global costmap from the saved map only) |

If the start is not the mapping start, set `initial_x/initial_y/initial_yaw` in `config/mission.yaml`
(or use RViz "2D Pose Estimate", launch with `auto_start:=false`, then call `/mission/start`).

Watch:

```bash
ros2 topic echo /mission/status --qos-durability transient_local     # IDLE → SEARCH/NAVIGATING/DWELL … → SUCCESS
ros2 topic echo /target/map_position --qos-durability transient_local
ros2 topic echo /target/diagnostics                                   # per-frame detector result
cat /home/ros2_ws/balancing_mission_result.json                       # final report
```

| Final log line / status | Meaning |
|---|---|
| `TARGET (map, metres): (x, y)` | target confirmed; this is the answer |
| `MISSION SUCCESS — TARGET …` | back within 0.25 m of the start |
| `TARGET_NOT_FOUND` | searched everything (or 600 s budget) and returned |
| `FAULT` | needs an operator; the log says why |

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
| Robot oscillates / goals fail | Nav2 params: `config/nav2.yaml`; planner log `failed to create plan` = goal unreachable in the global costmap |
| Map looks wrong | `ros2 run balancing_search map_viewer` → `http://balance17.local:8080/` (live) or `/saved` |

## 5. Tests

```bash
PYTHONPATH=. python3 test/test_core.py           # geometry / coverage / consensus
PYTHONPATH=. python3 test/test_state_machine.py  # mission state machine (ROS stubbed)
```

Bench dry run without the chassis (nothing can move): [search_mission §4](../docs/contracts/mission.search_mission.md#4-verification) T4.
Arena planning check without the chassis (nothing can move, ~7 min), on the Pi: `docker exec ros2_humble bash /home/ros2_ws/.nomotion.sh`, then `grep -c 'failed to create plan' /tmp/nm/launch.log` (expect 0) inside the container.
