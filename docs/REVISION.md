# Version 0.2: review and fixes

The review correctly identified credible demo blockers. This revision addresses
software issues while keeping unmeasured hardware assumptions explicit.

## Assessment

| Concern | Finding and action |
|---|---|
| Near-wall start | Confirmed. Previously the start had to satisfy the full viewpoint margin. Now physical transit clearance determines reachability; the nearest viewpoint with greater clearance is selected within that connected region. It cannot snap through a wall. |
| AMCL startup | Credible and supported by the Humble AMCL source. Added asynchronous `/request_nomotion_update` calls every five seconds, including before start. Separate x/y and yaw variance gates replace the single 0.10 gate. |
| Slow search | Confirmed algorithmic overhead. Spacing is now 1.2 m; minimum heading count is six, increased automatically for camera FOV; dwell is 3.5 s. |
| Invalid lidar values | Confirmed that zeros/NaNs blocked the original guard. Whether this particular N10 emits them remains unmeasured. Added a scan-audit tool, explicit guard diagnostics and optional bounded small-gap handling. |
| Close wall inhibits guard | Still intentionally true within the configured stop range. Removing this check or snapping a goal cannot make an unsafe start safe. Measure and configure the car before reducing it. |
| Configuration flags | Retained. They represent work that cannot be completed without the target, calibration and hardware. |
| Metadata | The maintainer email remains a clearly identified template field until the group provides its contact. The README GitHub URLs are real upstream source links, not a project repository. No group repository was supplied. |
| Recoveries and tests | No blind backup/recovery spin was added. Hardware tests are still pending. |

## Near-wall start behavior

- `transit_clearance: 0.38` is a separate starting/transit margin, not a measured
  footprint. It must be at least the measured robot radius plus necessary margin.
- `clearance: 0.42` is the larger viewpoint-placement margin.
- `max_seed_distance: 1.0` limits distance to the first reachable viewing point.
- Map resolution contributes a conservative half-cell-diagonal margin.
- Nav2 remains responsible for the actual route from the current start to that
  viewpoint, including the initial pose/footprint collision check.
- `/mission/start` now also requires a fresh `/guard/ready=true`. It will identify
  a blocked guard before accepting a mission that cannot move.

This fixes unnecessary viewpoint-margin rejection. It does **not** guarantee that
any assigned corner is physically traversable. A wall inside `stop_range` still
blocks the guard in every direction. Direction-aware escape from tight starts is
not implemented; use a physically valid assigned pose or tune measured clearances
with the examiner. Never just clear nearby occupied map cells.

## AMCL: freshness and uncertainty are separate

New starting/navigation limits:

- `max_xy_variance: 0.30` square metres for x/y.
- `max_yaw_variance: 0.15` radians squared for yaw.
- `amcl_timeout: 15.0` seconds, checked against receipt time **and source stamp**.
- Fresh map-to-base TF remains required.
- Target reporting retains tighter, separately configurable 0.10 variance limits.

A 0.30 m² variance permits about 0.55 m standard deviation; this is not proof of
accurate localization. Visually verify map/scan alignment before pressing start.
The threshold avoids automatically rejecting a typical initial uncertainty, while
the local scan guard still limits motion. Reduce thresholds if validation allows.
Do not use TF freshness alone as proof of localization quality.

Stationary updates require fresh lidar scans and an active, initialized AMCL.
Repeated updates cannot resolve every symmetric or ambiguous environment and may
make covariance overconfident. A no-motion update does not guarantee convergence.

An automatic relocalization spin was not added: the car would still need adequate
physical clearance and a safe localization/motion policy. The existing search
orientation goals already provide different views once normal navigation starts.

Reference (Humble implementation):
https://api.nav2.org/nav2-humble/html/amcl__node_8cpp_source.html

## Search tuning and timing

| Setting | Previous | Revised |
|---|---:|---:|
| Viewpoint spacing | 0.60 m | 1.20 m |
| Heading count | Fixed 8 | At least 6; increased for calibrated FOV |
| Dwell | 4.0 s | 3.5 s |
| Search budget | 1200 s | 900 s, plus bounded return attempts |
| Individual goal timeout | 90 s | 60 s |
| Linear speed limit | 0.15 m/s | 0.15 m/s |
| Yaw speed limit | 0.30 rad/s | 0.30 rad/s |

Speed limits were not raised without a stability/braking test. Nav2 now reads its
speed and acceleration limits from the same YAML settings as the guard; deploy
the same edited YAML to both computers. Physically validate any increase.

Offline comparison: empty square grids at 0.05 m resolution, start at room centre,
0.42 m viewpoint clearance, 60-degree camera FOV (eight actual headings):

| Synthetic map | Previous points/goals | Revised points/goals | Previous/revised dwell alone |
|---|---:|---:|---:|
| 3 x 3 m | 12 / 96 | 5 / 40 | 384 / 140 s |
| 4 x 4 m | 25 / 200 | 9 / 72 | 800 / 252 s |

These are reproducible software counts, not measured mission durations. Counts
change with starting pose, raster boundaries and obstacles, explaining differences
from other tests. Turning, driving, planning, detection and return add time. The
start log prints actual point/heading counts and dwell-only duration. The 15-minute
budget may stop a partial search; it does not prove a 30-minute overall session
will finish. Mapping time must be measured separately.

A 3 m detection range does not establish visibility behind objects. Test larger
spacing such as 1.5 m on the actual sandbox before using it in the demonstration.
At the lab's roughly 1.3 Hz output, four detections take several seconds; do not
reduce dwell to one second without measuring the effect on confirmation rate.

## Check the real scan

With the normal lidar driver running, motors safely disabled:

```bash
ros2 run balancing_search scan_audit --ros-args \
  -p samples:=30 -p output:=/tmp/scan_audit.json
ros2 topic echo /guard/status
```

The audit writes per-scan counts of finite returns, zeros, NaNs and +infinity,
invalid fraction, longest circular invalid gap, minimum range, frame and stamp age.
It times out after 15 seconds if the stream is missing. This command has **not**
been run on your car; its output is needed before selecting a lidar policy.

Strict defaults (`max_invalid_fraction: 0.0`, `max_invalid_gap_deg: 0.0`) remain
until the actual driver behavior is verified. If testing shows occasional isolated
invalid bins, a possible **trial configuration**, not a validated setting, is:

```yaml
max_invalid_fraction: 0.01
max_invalid_gap_deg: 2.0
```

With those settings only small gaps bounded on both sides by finite ranges are
filled, using the smaller bracket range. Large gaps, excessive invalid fraction,
missing finite brackets, stale scans and close obstacles still stop motion. Zero
and NaN are never converted to infinity. Small-gap interpolation can still miss a
thin obstacle at a depth discontinuity, so validate it at multiple obstacle positions.
If the driver systematically produces wide zero sectors, fix/verify its configuration
or use an appropriate validated sensor-processing design rather than increasing
limits until the car moves. +Infinity is assumed to mean no return; verify that
convention for the installed driver too.

## RViz extras

With fixed frame `map`, add:

1. **PoseArray** `/search/viewpoints` for all generated search locations/headings.
2. **MarkerArray** `/mission/markers` for the confirmed target and `(x, y)` label.

Use transient-local durability where needed. These are diagnostic displays, not
additional control paths. The visualizer starts with the mission launch.

## GitHub and final acceptance

A `.gitignore` now excludes ROS build/install/log output, Python caches, rosbags,
local environment files, calibration filenames and mission output. Review local
files before committing: arbitrary credentials are not automatically detected.

No GitHub push has been performed. GitHub installation was confirmed during this
revision, but its repository tools were not exposed in this running session and
no destination repository was specified. Supply the intended repository URL to
complete that step when repository access is available. The source folder is ready to import. Set the group's
real maintainer name/email in `setup.py` and `package.xml`; they cannot be inferred.

On-car acceptance remains pending. In addition to the README table, test at least:

| Trial | Purpose | Result |
|---|---|---|
| Start near wall, physically clear | Confirm nearest reachable viewpoint and guard readiness | Pending |
| Stationary RViz initial pose | Confirm AMCL updates and start gate | Pending |
| Target in unseen open location | Measure target error, mission and return time | Pending |
| Target near wall | Check contact-point bias and obstacle clearance | Pending |
| Target partly occluded | Check search coverage and false confirmation | Pending |
| Target absent | Verify bounded search, honest outcome and return | Pending |
| Block return route | Verify failure handling, no false success | Pending |
| Interrupt scan/network | Verify watchdog behavior and stable stopping | Pending |

Record ground-truth coordinates before each test, without giving them to the
mission. Record Euclidean target error, return error, elapsed time, interventions
and failure reasons. None of these physical results has been fabricated.
