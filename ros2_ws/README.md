# RoboFest ROS 2 workspace

This workspace is the ROS 2 translation of the existing RoboFest firmware algorithms. It separates interfaces, perception, navigation, drivers, mission supervision, robot description, and simulation analysis into independently buildable packages.

## Package map

| Package | Responsibility |
| --- | --- |
| `robofest_interfaces` | `MineDetection`, `MineMap`, `SwarmState`, and `MissionCommand` interfaces. |
| `robofest_perception` | HSV segmentation, circularity filtering, pinhole projection, and fake SITL vision. |
| `robofest_navigation` | Robot-localization EKF configuration, map fusion/decay, and 0.2 m A* planning with 1.0 m inflation. |
| `robofest_drivers` | 50 Hz MAVROS velocity setpoint bridge and command handling. |
| `robofest_mission` | Action-driven mission state machine, 5 Hz swarm state, and 10 Hz safety/geofence watchdog. |
| `robofest_description` | Drone Xacro, 60 m x 15 m Gazebo arena, and simulation launch wiring. |
| `robofest_simulation` | Offline path-clearance validator for recorded/exported path data. |

## Build and test

On a ROS 2 Humble/Iron system, run `cd ros2_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install && . install/setup.bash`. The sandbox used for development does not include ROS 2, so the dependency-free algorithm tests can be run directly with `pytest ros2_ws/src/robofest_navigation/test ros2_ws/src/robofest_simulation/test` and syntax checks with `python3 -m compileall ros2_ws/src`.

The simulation entry point is `ros2 launch robofest_description robofest_simulation.launch.py`; `start_gazebo` defaults to true and `start_sitl` can be enabled when ArduPilot is installed. For a bag export represented as JSON, run `ros2 run robofest_simulation analyze_bag paths.json --mines mines.json --output clearance_report.json`.

## Design notes

ROS 2 cannot publish a bare variable-length array as a topic type, so the requested `/perception/mine_detections` stream uses `robofest_interfaces/MineMap` with the `mines` array. The localization node intentionally delegates fusion to `robot_localization` EKF through `config/ekf.yaml` instead of duplicating a Kalman filter. All safety decisions are fail-safe: a stale odometry/heartbeat stream or an out-of-bounds pose publishes `EMERGENCY_STOP`.
