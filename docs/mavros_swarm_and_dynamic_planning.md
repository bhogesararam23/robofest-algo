# MAVROS Velocity Bridge, Swarm Handoff, and Dynamic Mine Planning

This document describes the hardened behavior implemented in the ROS 2 workspace, including the emergency latch, MAVROS disarm retries, fail-closed watchdog, acknowledged swarm handoff, shared-map fusion, and stale-path invalidation.

## 1. MAVROS velocity-setpoint bridge

The bridge is implemented in `ros2_ws/src/robofest_drivers/robofest_drivers/fc_bridge_node.py` as `FCBridgeNode`.

| Input | Topic | Effect |
|---|---|---|
| High-level command | `/mission/current_command` (`std_msgs/String`) | Replaces the active command and resets the path waypoint index. |
| Safe path | `/navigation/safe_path` (`nav_msgs/Path`) | Replaces the waypoint list and resets traversal to the first waypoint. |
| Pose | `/odom` (`nav_msgs/Odometry`) | Supplies the current XYZ position used for velocity calculation. |
| Output setpoint | `/mavros/setpoint_velocity/cmd_vel` (`geometry_msgs/Twist`) | Publishes the commanded velocity at 50 Hz. |
| Bridge heartbeat | `/drivers/heartbeat` (`std_msgs/String`) | Publishes `fc_bridge` once per 20 ms control cycle. |

For `SEARCH` and `GUIDE`, the bridge selects the current waypoint, computes the vector from the current odometry position to that waypoint, and scales it to a maximum magnitude of approximately 0.7 m/s. Once the distance falls below 0.3 m, it advances to the next waypoint. `TAKEOFF` commands a vertical velocity of 0.6 m/s until the reported altitude reaches 1.5 m. Any command other than `SEARCH`, `GUIDE`, or `TAKEOFF` produces the default zero-valued `Twist`, which behaves as a hold command at the setpoint layer.

### Emergency-stop path

The primary emergency path is:

```text
SafetyGeofenceNode
    -> /mission/current_command: "EMERGENCY_STOP"
    -> FCBridgeNode.command_cb()
    -> FCBridgeNode.control_tick() every 20 ms
    -> zero geometry_msgs/Twist on /mavros/setpoint_velocity/cmd_vel
```

When the active command equals `EMERGENCY_STOP`, the bridge explicitly replaces the outgoing command with an all-zero `Twist`. Because `control_tick()` continues at 50 Hz, the zero command is repeatedly published rather than emitted only once. This avoids leaving a previously commanded velocity latched at the ROS topic layer.

The bridge now latches the emergency state, clears the active path, publishes zero setpoints continuously, and attempts to call the MAVROS `/mavros/cmd/arming` service with `value=false`. Service calls are retried at the configured interval until the disarm deadline. The latch ignores subsequent motion commands and can only be cleared through `/drivers/reset_emergency` after the disarm window has elapsed; clearing returns the bridge to HOLD rather than immediately resuming motion. If `mavros_msgs` is unavailable, the zero-setpoint and latch behavior remains active, while the disarm client is disabled and the node reports that condition through logs/status.

## 2. Geofence and watchdog failsafes

`SafetyGeofenceNode` runs its check callback every 100 ms, equivalent to 10 Hz. It records the latest odometry timestamp and the most recent timestamp for each heartbeat topic. It publishes `EMERGENCY_STOP` if either of the following occurs:

| Failsafe | Current condition |
|---|---|
| Geofence breach | `x < 0`, `x > 60`, `y < 0`, or `y > 15`. Boundary values themselves are accepted. |
| Odometry timeout | More than 0.5 s since the most recent `/odom` message. |
| Heartbeat timeout | More than 0.5 s since a heartbeat already present in the heartbeat dictionary. |

The geofence is expressed in the odometry frame, so it assumes that `/odom` is correctly aligned with the 60 m by 15 m arena coordinates. The command is published repeatedly on every 10 Hz safety tick while the unsafe condition remains true.

The watchdog is now fail-closed. It initializes explicit `seen` state for `/vision/heartbeat`, `/localization/heartbeat`, and `/drivers/heartbeat`; a missing first heartbeat is unsafe, not ignored. The perception node publishes a 10 Hz heartbeat, the localization heartbeat node publishes only when fresh `/odom` arrives, and the bridge publishes its heartbeat at 50 Hz. The safety manager also publishes a diagnostic reason on `/mission/safety_reason` alongside each `EMERGENCY_STOP`.

## 3. Swarm state and guidance handoff

`SwarmNode` publishes a `robofest_interfaces/SwarmState` message every 0.2 s, or 5 Hz, to `/swarm/outgoing_state`. The message contains:

- The configured `drone_id`.
- The configured `battery_percent`.
- The current task string.
- The `is_available` flag.
- The latest local `MineMap` received from `/navigation/global_mine_map`.

Incoming packets arrive on `/swarm/incoming_state`. Packets whose `drone_id` matches the local ID are ignored, preventing self-loop processing. For a different drone, the handoff condition is:

```text
local task == "GUIDE_HUMAN"
AND local battery_percent < 20
AND sender.is_available == true
AND sender.battery_percent > 50
```

The handoff is now an acknowledged protocol. `SwarmState` carries a sequence number, handoff ID, status, and target drone ID. The low-battery guide drone chooses an available peer above 50% battery, publishes `PENDING`, and waits for an `ACK` before publishing `YIELD_GUIDANCE` and marking itself `HANDOFF_COMPLETE`/unavailable. The receiving drone accepts only a request addressed to its own ID, reserves the task as `ACCEPT_GUIDANCE`, and advertises the acknowledgement. Pending handoffs expire after the configured timeout; accepted leases expire after their lease duration. Self-messages are ignored, and no recipient is selected unless the availability and battery rules are satisfied.

Incoming `SwarmState.local_map` detections are merged by the navigation map node in the shared `odom` frame using the same 0.5 m deduplication and confidence fusion rules as local perception. Thus, a peer's mine discovery reaches the same global map and planner pipeline rather than remaining only inside the communication node. `YIELD_GUIDANCE` is explicitly treated as a hold-like command by the bridge, and it cannot override the emergency latch.

## 4. Dynamic mine observations and map fusion

The dynamic planning chain is:

```text
/perception/mine_detections
    -> MineMappingNode
    -> /navigation/global_mine_map
    -> PathPlannerNode
    -> /navigation/safe_path
    -> FCBridgeNode
```

`MineMappingNode` keeps the current drone pose from `/odom`. Each perception detection is transformed from the drone-relative frame into the `odom` frame using planar yaw:

```text
x_global = x_pose + cos(yaw) * x_relative - sin(yaw) * y_relative
y_global = y_pose + sin(yaw) * x_relative + cos(yaw) * y_relative
```

The transformed detection is passed to `MineMapStore`. Existing mines are matched by Euclidean distance with a 0.5 m deduplication radius. A match updates the position using confidence-weighted averaging, increases confidence by `0.1 * incoming_confidence` with a maximum of 1.0, updates the type, and refreshes `last_seen`. A detection outside the radius receives a new ID and is appended to the global map.

The map is published immediately after detection processing. A 1 Hz timer applies aging: after 5 s without being seen, confidence is reduced by 0.1 per timer cycle, and mines below 0.3 confidence are removed. Consequently, a newly detected mine cluster can change the planner's obstacle field within the next map callback, while stale unconfirmed observations eventually disappear. Confirmed or repeatedly observed mines persist because each observation refreshes their timestamp and reinforces confidence.

## 5. Obstacle inflation around dynamic mine clusters

`PathPlannerNode.map_cb()` rebuilds the complete grid whenever a new `/navigation/global_mine_map` message arrives. The grid represents:

| Parameter | Value |
|---|---:|
| Arena width | 60.0 m |
| Arena height | 15.0 m |
| Resolution | 0.2 m/cell |
| Grid dimensions | 300 × 75 cells |
| Mine clearance radius | 1.0 m |
| Lethal cost | 253 |

For each mine, the planner converts its coordinate to a grid cell and marks every cell within a 1.0 m Euclidean radius as lethal (`253`). The implementation checks the cell-center distance rather than using only a square bounding box, so diagonal cells outside the circle remain traversable. Overlapping circles from nearby mines naturally form a connected lethal cluster. A dynamic cluster therefore appears as the union of all current mine-clearance discs; there is no special cluster object required.

The grid is rebuilt from the latest map rather than incrementally adding obstacles. That is important for decay: when a mine is removed because its confidence falls below 0.3, the next rebuild clears its old inflated cells as well. It also means that newly fused swarm or perception observations can enlarge, move, or join an obstacle cluster on the next update.

## 6. A* search and route publication

The planner uses 8-connected A* over all non-lethal cells. Horizontal and vertical moves have unit cost, diagonal moves have cost approximately 1.414, and the heuristic is Euclidean distance to the goal cell. The search immediately fails if the start or goal lies outside the grid or in a lethal cell. During expansion it skips all cells with cost `>= 253`, so the route cannot enter the inflated mine-clearance area.

When the goal is reached, the parent links are reversed into a grid-cell path. The smoothing pass then attempts to connect each retained anchor directly to the farthest later waypoint whose line segment samples only traversable cells. This removes unnecessary stair-step turns while retaining collision checks. The published `nav_msgs/Path` is expressed in the `odom` frame and triggers the velocity bridge to follow the new waypoint sequence.

Because `map_cb()` calls `replan()` on every global-map update, a newly discovered, moved, fused, or removed mine cluster triggers immediate replanning. A 0.5 s timer also replans from the vehicle's current pose, so the route does not remain anchored to an obsolete start cell. If A* fails, the planner publishes an empty `nav_msgs/Path` and `NO_SAFE_PATH` on `/navigation/path_status`; the bridge clears its path and enters HOLD rather than continuing along stale geometry. A successful plan publishes `PATH_VALID` and the newly smoothed route.

## 7. End-to-end interpretation in Gazebo

The simulation launch starts Gazebo conditionally, then starts fake vision, mapping, planning, the bridge, the mission state machine, swarm communication, and safety supervision. In the intended closed loop, Gazebo or SITL supplies odometry, fake or real perception supplies mine detections, map fusion turns those detections into global mine coordinates, and the planner rebuilds the inflated costmap and republishes a safe path. The bridge converts that path into velocity setpoints. Independently, the safety manager can override mission motion by publishing `EMERGENCY_STOP`, which causes the bridge to output zero velocities on its next 20 ms control tick.

The repository now contains the requested hardening implementation. Before hardware flight, the MAVROS service name/type, flight-controller arming policy, heartbeat rates, and handoff lease values should be verified against the target vehicle and network configuration in a hardware-in-the-loop test.
