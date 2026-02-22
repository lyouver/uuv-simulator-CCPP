# Framework Structure

## 1. Scope
This repository is a UUV simulation and planning stack centered on RexROV in Gazebo.
It integrates:
- world and robot simulation (`uuv_simulator`, `rexrov2`)
- dynamic obstacle perception (`onboard_detector`, workspace sibling package)
- local avoidance trajectory generation (`example/scripts/local_avoidance_planner.py`)
- trajectory tracking control (`uuv_trajectory_control`)
- optional global coverage planning (`underwater_coverage_planning`)

## 2. Workspace Layout
Main repo:
- `example/`: project-specific launch, config, scripts, worlds
- `uuv_simulator/`: UUV simulation/control base
- `rexrov2/`: RexROV model/control assets
- `underwater_coverage_planning/`: global coverage generation tools
- `nps_uw_multibeam_sonar/`: multibeam sonar simulation components
- `navigator_auv/`: additional scripts/tools

Sibling packages used by runtime:
- `../map_manager`
- `../onboard_detector`

## 3. Runtime Pipeline
Core launch:
- `example/src/launch/compact_rexrov_with_trajectory.launch`

Execution chain:
1. Gazebo world starts (`compact_underwater_terrainqqq.world`).
2. RexROV model is spawned and TF is established.
3. Controller stack starts (`rov_pid_controller.launch`).
4. `local_avoidance_planner.py` loads `local_avoidance.yaml`.
5. Planner publishes short-horizon trajectory to:
   - `/<uuv_name>/dp_controller/input_trajectory`
6. DP controller tracks the incoming trajectory.

## 4. Perception and Obstacle Sources
Current local avoidance config:
- `perception_source: map_manager`
- `map_cloud_topic: /onboard_detector/dynamic_point_cloud`
- `detector_velocity_topic: /onboard_detector/velocity_visualizaton`
- `map_use_collision_service: false`

Implication:
- The planner consumes detector/map-style dynamic point cloud and velocity tracks.
- Static collision service (`/dynamic_map/check_pos_collision` or `/occupancy_map/check_pos_collision`) is configured but currently not active because service mode is disabled.

## 5. Planning Layers
Global reference:
- YAML waypoints, default:
  - `example/src/scripts/guiji/trajectory_20260206waypoints.yaml`

Local avoidance:
- `example/scripts/local_avoidance_planner.py`
- Key behavior:
  - predicts obstacle motion and relative collision risk
  - decides avoid vs pass-through
  - produces local trajectory respecting speed/clearance limits
  - publishes planner state diagnostics

Tracking:
- `uuv_trajectory_control` DP controller consumes local trajectory.

## 6. Key Config Files
- `example/src/config/local_avoidance.yaml`
  - source topics, prediction horizon, replan interval, speed limits, clearance
- `onboard_detector/cfg/detector_param.yaml`
  - detection/tracking and target size constraint
- `example/src/worlds/compact_underwater_terrainqqq.world`
  - dynamic obstacle models and motion plugins

## 7. Topic/Service Interfaces
Typical inputs:
- `/rexrov/pose_gt` (odometry/pose)
- `/onboard_detector/dynamic_point_cloud`
- `/onboard_detector/velocity_visualizaton`

Planner output:
- `/<uuv_name>/dp_controller/input_trajectory`
- debug/state topics under `~` namespace of planner node

Optional service (when enabled):
- `/dynamic_map/check_pos_collision`
- `/occupancy_map/check_pos_collision`

## 8. Bring-up Order (Recommended)
1. Main sim + controller + planner:
   - `roslaunch eroas_example compact_rexrov_with_trajectory.launch`
2. Dynamic detector:
   - `roslaunch onboard_detector run_detector.launch show_rviz:=false`
3. Optional map manager (only if using map collision service mode):
   - `roslaunch map_manager dynamic_map.launch`
   - or `roslaunch map_manager occupancy_map.launch`

## 9. Current Operating Mode Snapshot
At the time of writing:
- local planner uses detector-fed dynamic obstacles
- map collision service mode is disabled
- world includes dynamic sphere obstacles for local avoidance tests
- `target_object_size` is configured in detector as `[3.0, 3.0, 3.0]`

## 10. Common Extension Points
- Replace local planner logic inside `local_avoidance_planner.py` while preserving controller interface.
- Switch static collision filtering by enabling `map_use_collision_service`.
- Tune dynamic obstacle realism by editing obstacle models in world file.
- Swap global waypoint file to evaluate different missions.
