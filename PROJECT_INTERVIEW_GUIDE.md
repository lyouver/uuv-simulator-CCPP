# Project Interview Guide

## 1. One-line Summary
Built a RexROV underwater autonomy stack in ROS1/Gazebo with dynamic obstacle perception, local trajectory avoidance, and controller-level tracking.

## 2. 60-second Pitch
This project runs a full UUV simulation loop: Gazebo world, RexROV model, dynamic obstacle detection, local avoidance planning, and closed-loop trajectory tracking.
The planner continuously converts a global waypoint reference into short-horizon safe trajectories and sends them to the DP controller.
The system is practical for testing dynamic avoidance behavior, speed-clearance tradeoffs, and mission-level waypoint execution in underwater environments.

## 3. What Problem It Solves
- Global waypoints alone are not safe in dynamic scenes.
- Pure local reactive rules can be unstable without controller coupling.
- This stack bridges mission path intent and short-horizon collision-aware motion.

## 4. System Architecture to Explain
Use this order in interviews:
1. Simulation and robot model:
   - Gazebo world + RexROV spawn
2. Perception:
   - dynamic point cloud + obstacle velocity estimation
3. Planning:
   - local avoidance planner produces short trajectory
4. Control:
   - DP controller tracks local trajectory in real time
5. Observability:
   - planner mode/diagnostic topics, RViz paths, controller behavior

## 5. Files to Mention Quickly
- Launch entry:
  - `example/src/launch/compact_rexrov_with_trajectory.launch`
- Local planner:
  - `example/scripts/local_avoidance_planner.py`
- Planner config:
  - `example/src/config/local_avoidance.yaml`
- Obstacle world:
  - `example/src/worlds/compact_underwater_terrainqqq.world`
- Detector config:
  - `../onboard_detector/cfg/detector_param.yaml`

## 6. Technical Decisions (Suggested Talking Points)
- Chose short-horizon local trajectory publication instead of direct low-level velocity commands to stay compatible with UUV controller interfaces.
- Separated perception and planning to allow swapping obstacle providers without changing controller stack.
- Kept a configurable switch for map collision service mode versus pure dynamic perception mode.
- Tuned speed and clearance independently to avoid unsafe aggressive motion.

## 7. What You Tuned in Practice
- Obstacle size assumptions (e.g., target size in detector).
- Avoidance clearance radius.
- Replan interval and prediction horizon.
- Avoidance speed upper bound versus nominal cruising speed.
- Dynamic obstacle layout in world for stress tests.

## 8. Results You Can Claim (If Asked)
- System can track a global trajectory while replanning locally under moving obstacles.
- Planner behavior can be shifted between conservative and aggressive via config only.
- The pipeline supports repeatable scenario testing through fixed launch/world/config artifacts.

## 9. Hard Questions and Good Answers
Q: Why not command the controller directly with velocity setpoints?
A: Trajectory interface preserves smoother control behavior and is native to the UUV control stack, reducing integration risk.

Q: How do you prevent unstable replanning oscillation?
A: Use bounded replanning intervals, smoothing, hold windows, and speed limiting under avoidance mode.

Q: How do you validate safety?
A: Scenario-based simulation with controlled obstacle sizes/speeds/positions, plus min-distance and collision observation.

Q: What is the main limitation?
A: Highly coupled tuning across detector noise, planner thresholds, and controller dynamics; robust cross-scenario defaults are nontrivial.

## 10. Demo Script for Interview
1. Launch full stack.
2. Show global path in RViz.
3. Introduce dynamic spheres.
4. Show local planner switching to avoidance mode.
5. Show robot rejoining global path after obstacle pass.
6. Highlight 1-2 config tweaks and behavioral difference.

## 11. Improvement Roadmap
- Add quantitative benchmark script (min distance, clearance violations, completion time).
- Add standardized scenario suite (single obstacle, crossing obstacles, dense field).
- Add planner unit tests for collision-check core functions.
- Add log replay tooling for deterministic regression checks.

## 12. Resume Bullet Templates
- Implemented a ROS-based underwater autonomy stack integrating dynamic obstacle perception, local trajectory avoidance, and DP trajectory tracking in Gazebo.
- Developed and tuned local avoidance planning logic to adapt global waypoint missions to dynamic obstacle scenes while maintaining controller compatibility.
- Built repeatable simulation scenarios and parameterized runtime configs to evaluate safety-speed tradeoffs for autonomous underwater navigation.
