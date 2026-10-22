# Tasks 1–11: code and explanations

Numbering follows the supplied task checklist image. The assessment brief describes the integrated mission rather than eleven separate programs. Shared files implement several tasks, so the package structure is preserved and comments identify the relevant task sections.

**Status:** uploaded implementation plus external ROS/lab dependencies; hardware acceptance is still unverified. A task label means code coverage, not a claim of completed physical testing.

| Task | Name | Main code |
|---|---|---|
| 1 | [Robot interface](#task-1) | [balancing_search/guard.py](../balancing_search/guard.py) |
| 2 | [SLAM](#task-2) | [launch/mapping.launch.py](../launch/mapping.launch.py) |
| 3 | [Localization](#task-3) | [balancing_search/mission.py](../balancing_search/mission.py) |
| 4 | [Navigation](#task-4) | [launch/mission.launch.py](../launch/mission.launch.py) |
| 5 | [Exploration](#task-5) | [balancing_search/core.py](../balancing_search/core.py) |
| 6 | [Object detection](#task-6) | [balancing_search/localizer.py](../balancing_search/localizer.py) |
| 7 | [Target localization](#task-7) | [balancing_search/localizer.py](../balancing_search/localizer.py) |
| 8 | [Return-to-start](#task-8) | [balancing_search/mission.py](../balancing_search/mission.py) |
| 9 | [Mission orchestration](#task-9) | [balancing_search/mission.py](../balancing_search/mission.py) |
| 10 | [ROS 2 integration](#task-10) | [setup.py](../setup.py) |
| 11 | [System validation](#task-11) | [test/test_core.py](../test/test_core.py) |

<a id="task-1"></a>

## Task 1 — Robot interface

**Code:** [balancing_search/guard.py](../balancing_search/guard.py). **Entry points:** Guard.command(), scan(), tick(); core.limited_twist().

Reads lidar and incoming Nav2 commands, checks freshness and heartbeat, limits velocity/acceleration, and publishes /cmd_vel. Odometry and low-level balancing come from the lab drivers.

**Acceptance / remaining validation:** Verify /scan, /odom and TF, one /cmd_vel publisher, braking and vendor command timeout.

<a id="task-2"></a>

## Task 2 — SLAM

**Code:** [launch/mapping.launch.py](../launch/mapping.launch.py). **Entry points:** generate_launch_description(); config/slam.yaml.

Starts external SLAM Toolbox using project parameters. Map saving is a separate map_saver_cli command described in the operating guide; it is not automated by this launch.

**Acceptance / remaining validation:** Construct and save a fresh sandbox map on site, then inspect scan alignment and map files.

<a id="task-3"></a>

## Task 3 — Localization

**Code:** [balancing_search/mission.py](../balancing_search/mission.py). **Entry points:** Mission.pose(), localized(), refresh_amcl(); launch/mission.launch.py.

Launches external AMCL and map server. Reads map-to-base TF and checks AMCL age and covariance. Requests stationary AMCL updates to support initialization. Initial pose is supplied before autonomy.

**Acceptance / remaining validation:** Compare estimated pose and scan alignment against measured positions and headings.

<a id="task-4"></a>

## Task 4 — Navigation

**Code:** [launch/mission.launch.py](../launch/mission.launch.py). **Entry points:** configure(); Mission.send(); config/search_tree.xml.

Configures external Nav2 planner, DWB controller, costmaps and lifecycle manager. Sends NavigateToPose goals asynchronously. Routes controller output through /nav_cmd_vel to the guard.

**Acceptance / remaining validation:** Test obstacle avoidance, rejected/failed goals, live replanning and cancellation on the car.

<a id="task-5"></a>

## Task 5 — Exploration

**Code:** [balancing_search/core.py](../balancing_search/core.py). **Entry points:** coverage(), heading_count(); Mission.start().

Finds reachable cleared cells in the saved map, selects viewpoints greedily, and adds camera-FOV-aware viewing headings. This is search within an already mapped sandbox, not autonomous frontier SLAM.

**Acceptance / remaining validation:** Inspect /search/viewpoints; test disconnected areas, narrow passages, failed goals and occluded targets.

<a id="task-6"></a>

## Task 6 — Object detection

**Code:** [balancing_search/localizer.py](../balancing_search/localizer.py). **Entry points:** TargetLocalizer.detect(); README camera/YOLO commands.

Consumes detections produced by the external lab yolo_ros pipeline. Filters exact target class and confidence, rejects multiple matching boxes, and checks image age and camera frame. Detector inference and trained weights are not included.

**Acceptance / remaining validation:** Set the actual target class; validate recognition, false positives and inference latency at several distances.

<a id="task-7"></a>

## Task 7 — Target localization

**Code:** [balancing_search/localizer.py](../balancing_search/localizer.py). **Entry points:** TargetLocalizer.detect(); core.rotation(), ground_hit(), Consensus.

Undistorts the bounding-box bottom centre into a camera ray, transforms it at the image timestamp, intersects the measured floor plane and confirms agreement across distinct frames. Mission.on_target() reports map-frame metres.

**Acceptance / remaining validation:** Measure floor-point errors at multiple ranges/headings; verify intrinsics, extrinsics and actual balancing pitch in TF.

<a id="task-8"></a>

## Task 8 — Return-to-start

**Code:** [balancing_search/mission.py](../balancing_search/mission.py). **Entry points:** Mission.start(), return_home(), complete_goal(), finish().

Captures the localized starting pose, sends a return goal after search, checks Nav2 success and positional tolerance, then disables the motion heartbeat. A bounded retry handles some return failures.

**Acceptance / remaining validation:** Verify arrival in the assigned area and stopping; blocked returns must not produce SUCCESS.

<a id="task-9"></a>

## Task 9 — Mission orchestration

**Code:** [balancing_search/mission.py](../balancing_search/mission.py). **Entry points:** Mission.start(), tick(), cancel(), fault(), on_target().

Coordinates startup checks, search goals, dwell, target reporting, cancellation, return and terminal outcomes. Mapping is a separate supervised stage; the state machine coordinates the subsequent autonomous mission.

**Acceptance / remaining validation:** Exercise detection during navigation, late goal acceptance, cancellation timeout, stale perception and localization loss.

<a id="task-10"></a>

## Task 10 — ROS 2 integration

**Code:** [setup.py](../setup.py). **Entry points:** package.xml; launch/; config/; node constructors.

Registers five node executables and installs launch/config files. Connects nodes through typed topics, services, Nav2 actions and TF. Hardware, camera, YOLO and their calibrated transforms must be supplied externally.

**Acceptance / remaining validation:** Run colcon and ROS launch/lifecycle checks with the lab stack; confirm message versions, QoS and TF ownership.

<a id="task-11"></a>

## Task 11 — System validation

**Code:** [test/test_core.py](../test/test_core.py). **Entry points:** test/test_state_machine.py; docs/VALIDATION.md; scan_audit.py.

Provides ROS-independent geometry/control tests, mocked mission callbacks and lidar diagnostics. These checks support development but do not demonstrate physical mission completion.

**Acceptance / remaining validation:** Record full map-search-report-return trials, total time including mapping, target error, return error, failures and interventions.

## Integrated data flow

Lidar feeds SLAM during mapping and AMCL/Nav2 during the mission. The mission generates viewpoints and sends Nav2 goals. Nav2 commands pass through the guard to the existing balancing controller. Camera images feed external YOLO; detections and calibrated TF feed the target localizer. A confirmed map point triggers reporting, cancellation of the search goal and return to the captured start.

## Reading order

Start with the operating guide in [README](../README.md), then `mission.py` for coordination, `core.py` for algorithms, `localizer.py` for perception geometry, and `guard.py` for command handling. See [VALIDATION](VALIDATION.md) for the distinction between automated checks and remaining physical trials.
