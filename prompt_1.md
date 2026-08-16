# Prompt 1 — Robofest Minefield Drone Project Bootstrap, Global Rules, Configuration, and Shared Types

Save this file as:

```text
prompt_1.md
```

Use this as the first prompt in your AI coding IDE.

---

## PROMPT 1

You are an expert embedded C++ coding agent working inside an IDE.

Your task is to generate the first foundation of a drone-only onboard software project for the Robofest Gujarat 6.0 Minefield Swarm Drone challenge.

This is Prompt 1 of a multi-prompt build.

For this prompt, generate only the project foundation. Do not generate the full mission logic yet. Do not generate simulation tools, scoring tools, hardware design files, mechanical design files, motor sizing files, battery sizing files, weight calculation files, cloud services, base station software, GPS modules, or remote operator control.

The project must be drone-only, onboard-only, and software-only.

---

## 1. PROJECT PURPOSE

Create the base repository for one autonomous drone that is part of a minimum three-drone swarm.

The drone must help a person-at-risk cross a minefield safely.

The drone must operate without GPS and without external positioning.

The drone must make decisions onboard.

The final software will later include:

- Mission state machine.
- Optical flow and ToF localization.
- Software geofence.
- Mine-marker vision pipeline.
- Mine map with deduplication.
- Safe path planner with 1.0 meter mine clearance.
- Local peer-to-peer swarm communication.
- Hand or voice command recognition.
- Human tracking.
- Visual guidance output.
- Safety manager.
- Flight controller bridge.
- Telemetry logging.

For this first prompt, generate the project scaffold, documentation, configuration files, shared types, and placeholder module files.

---

## 2. OFFICIAL MISSION CONSTRAINTS

Embed these constraints into the project documentation and configuration logic.

Official mission constraints:

- Minimum three drones are required.
- Ideal drone count is 3 to 4 drones.
- Mission time limit is 10 minutes.
- Mission time limit in milliseconds is 600000 ms.
- Field size is 15 meters wide by 60 meters long.
- Start Zone is 1 meter by 15 meters.
- Minefield Zone is 58 meters by 15 meters.
- Exit Zone is 1 meter by 15 meters.
- The safe human path must maintain at least 1.0 meter clearance from every confirmed mine center.
- The drone must not use GPS.
- The drone must not use GNSS.
- The drone must not use any external positioning system.
- The drone must not communicate with cloud servers.
- The drone must not communicate with outside computers.
- The drone must not communicate with base stations.
- The drone must not allow remote operator control.
- All localization, mapping, planning, command recognition, and guidance must run onboard.
- The drone must detect mines.
- The drone must map mines.
- The drone must avoid duplicate mine detections.
- The drone must help generate a safe path.
- The drone must visually mark or guide the safe path.
- The drone must understand hand or voice commands.
- The drone must track the human.
- The drone must re-route if the human leaves the safe path.
- The drone must re-route if a new mine blocks the path.
- The drone must land autonomously when the mission ends.
- The drone must avoid unwanted surface contact.
- Controlled landing is the only allowed surface contact.
- The drone must avoid collision and unsafe proximity.

Derived challenge assumptions:

- Approximately 40 mines may exist.
- About 75 percent may be visible on-ground mines.
- About 25 percent may be buried mines with standardized surface markers.
- Buried mines may be 30 to 50 mm deep.
- Possible obstacles such as poles or trees may exist.
- The drone should support obstacle avoidance if obstacles are detected or preloaded.
- Buried-mine surface marker appearance must be tunable.

Safety notes:

- The software must monitor a kill switch input.
- The physical kill switch should also be able to cut power independently.
- Low battery must trigger landing preparation.
- Critical battery must trigger landing.
- Lost flight controller link must trigger HOLD or LAND.
- Lost localization must trigger HOLD if recoverable, or LAND if unrecoverable.
- Mission timeout must trigger landing.
- Unsafe proximity must trigger corrective action.
- Unwanted surface contact outside controlled landing must be treated as a safety fault.

---

## 3. TARGET SOFTWARE ENVIRONMENT

Generate the project for embedded C++.

Assume the mission computer may be an ESP32-S3.

Assume the flight controller handles low-level flight stabilization.

The mission computer must only send high-level commands to the flight controller.

High-level commands include:

- HOLD.
- TAKEOFF.
- LAND.
- VELOCITY.
- ALTITUDE.
- HEADING.
- ARM.
- DISARM.
- EMERGENCY_STOP.

The flight controller handles:

- Attitude control.
- Rate control.
- Motor mixing.
- Low-level stabilization.
- Low-level flight safety.

Coding rules:

- Use embedded-friendly C++.
- Use `#pragma once` in headers.
- Avoid dynamic memory allocation inside the main control loop.
- Use fixed-size arrays where possible.
- Use strong types and enums.
- Use clear module boundaries.
- Do not use blocking delays.
- The main loop will later run every 20 milliseconds.
- Use constants instead of magic numbers.
- Every constant must be labeled as one of:
  - official
  - derived from uploaded logic
  - tunable implementation default

Do not generate code that depends on:

- GPS.
- GNSS.
- Cloud.
- Internet.
- Outside computers.
- Base stations.
- Remote operators.
- External motion-capture systems.

---

## 4. REPOSITORY STRUCTURE TO GENERATE

Generate this repository structure:

```text
/robofest_drone/
  README.md
  /docs/
    requirements.md
  /config/
    mission_config.h
    thresholds.h
  /src/
    types.h
    main.cpp
    scheduler.h
    scheduler.cpp
    state_machine.h
    state_machine.cpp
    localization.h
    localization.cpp
    geofence.h
    geofence.cpp
    vision_pipeline.h
    vision_pipeline.cpp
    mine_map.h
    mine_map.cpp
    path_planner.h
    path_planner.cpp
    swarm_comm.h
    swarm_comm.cpp
    search_behavior.h
    search_behavior.cpp
    command_layer.h
    command_layer.cpp
    human_tracker.h
    human_tracker.cpp
    marker_controller.h
    marker_controller.cpp
    safety_manager.h
    safety_manager.cpp
    fc_bridge.h
    fc_bridge.cpp
    telemetry.h
    telemetry.cpp
    mission_integration.h
    mission_integration.cpp
  /hal/
    hal_camera.h
    hal_camera.cpp
    hal_optical_flow.h
    hal_optical_flow.cpp
    hal_tof.h
    hal_tof.cpp
    hal_radio.h
    hal_radio.cpp
    hal_gpio.h
    hal_gpio.cpp
    hal_serial.h
    hal_serial.cpp
    hal_storage.h
    hal_storage.cpp
```

For this prompt, generate full meaningful content only for:

```text
README.md
docs/requirements.md
config/mission_config.h
config/thresholds.h
src/types.h
```

For all other files, create the file and add a short placeholder comment.

Example placeholder:

```cpp
// Implemented in a later prompt.
```

Do not skip any file.

---

## 5. README REQUIREMENTS

Generate `README.md`.

The README must include:

- Project title.
- Short project description.
- Statement that this is drone-only onboard software.
- Statement that this is for a Robofest Gujarat 6.0 minefield swarm drone.
- Statement that the drone is part of a minimum three-drone swarm.
- Statement that the software does not use GPS.
- Statement that the software does not use cloud.
- Statement that the software does not use outside computers.
- Statement that the software does not use base stations.
- Statement that the software does not use remote operator control.
- Statement that the mission computer sends high-level commands to the flight controller.
- Statement that the flight controller handles low-level stabilization.
- Statement that the main control loop is non-blocking and will run every 20 milliseconds.
- Folder structure explanation.
- Safety warning.

Include this safety warning near the end of the README:

```text
This software must not be flown without propeller-off bench testing, sensor calibration, geofence validation, failsafe testing, and controlled low-altitude flight tests.
```

---

## 6. REQUIREMENTS DOCUMENT REQUIREMENTS

Generate `docs/requirements.md`.

This document must separate requirements into:

1. Official requirements.
2. Derived requirements.
3. Tunable implementation defaults.

Include the official mission constraints from this prompt.

Include the mission state list:

```text
INIT
CALIBRATE
WAIT_FOR_START
TAKEOFF
FORMATION
SEARCHING
PLANNING
GUIDING
MISSION_COMPLETE
LANDING
DISARMED
HOLD
EMERGENCY
```

Include the mission flow:

```text
INIT -> CALIBRATE -> WAIT_FOR_START -> TAKEOFF -> FORMATION -> SEARCHING
SEARCHING -> PLANNING when enough mine coverage exists
PLANNING -> GUIDING when a valid safe path exists
GUIDING -> SEARCHING if path is invalidated or human leaves corridor
GUIDING -> MISSION_COMPLETE when human reaches Exit Zone
MISSION_COMPLETE -> LANDING
ANY_STATE -> HOLD for pause or recoverable fault
ANY_STATE -> EMERGENCY for kill switch, crash risk, or unrecoverable fault
LANDING -> DISARMED
```

Include drone roles:

```text
SCOUT_LEFT
SCOUT_RIGHT
GUIDE_MARKER
RESERVE
```

Include expected swarm packet types:

```text
HEARTBEAT
ROLE_ASSIGN
CLAIM
YIELD
MINE_UPDATE
PATH_UPDATE
PERSON_UPDATE
HELP_REQUEST
LAND_NOW
```

Include command types:

```text
START
FORWARD
PAUSE
SCAN_LEFT
SCAN_RIGHT
STOP_ABORT
```

Include safety actions:

```text
CONTINUE
HOLD
LAND
EMERGENCY_CUT
```

Include marker patterns:

```text
MARKER_OFF
MARKER_FORWARD
MARKER_STOP
MARKER_LEFT
MARKER_RIGHT
MARKER_SAFE_PATH
MARKER_EMERGENCY
MARKER_MISSION_COMPLETE
```

---

## 7. MISSION CONFIG HEADER REQUIREMENTS

Generate `config/mission_config.h`.

Use `#pragma once`.

Use embedded-friendly C++.

Every constant must have a comment saying one of:

```text
// official
// derived from uploaded logic
// tunable implementation default
```

Include these constants:

```cpp
FIELD_LENGTH_M
FIELD_WIDTH_M
START_ZONE_LENGTH_M
EXIT_ZONE_LENGTH_M
MINEFIELD_LENGTH_M
MINEFIELD_WIDTH_M
MINE_CLEARANCE_RADIUS_M
MISSION_TIME_LIMIT_MS
MINE_COUNT_ESTIMATE
```

Use these default values:

```cpp
FIELD_LENGTH_M = 60.0f
FIELD_WIDTH_M = 15.0f
START_ZONE_LENGTH_M = 1.0f
EXIT_ZONE_LENGTH_M = 1.0f
MINEFIELD_LENGTH_M = 58.0f
MINEFIELD_WIDTH_M = 15.0f
MINE_CLEARANCE_RADIUS_M = 1.0f
MISSION_TIME_LIMIT_MS = 600000UL
MINE_COUNT_ESTIMATE = 40
```

Include zone Y ranges:

```cpp
START_ZONE_Y_MIN = 0.0f
START_ZONE_Y_MAX = 1.0f
MINEFIELD_Y_MIN = 1.0f
MINEFIELD_Y_MAX = 59.0f
EXIT_ZONE_Y_MIN = 59.0f
EXIT_ZONE_Y_MAX = 60.0f
```

Include field coordinate limits:

```cpp
FIELD_X_MIN = 0.0f
FIELD_X_MAX = 15.0f
FIELD_Y_MIN = 0.0f
FIELD_Y_MAX = 60.0f
```

Include takeoff offset constants:

```cpp
TAKEOFF_FIELD_X
TAKEOFF_FIELD_Y
```

Default values:

```cpp
TAKEOFF_FIELD_X = 7.5f
TAKEOFF_FIELD_Y = 0.5f
```

Include software geofence constants:

```cpp
SOFTWARE_GEOFENCE_X_MIN
SOFTWARE_GEOFENCE_X_MAX
SOFTWARE_GEOFENCE_Y_MIN
SOFTWARE_GEOFENCE_Y_MAX
GEOFENCE_WARNING_BAND_M
GEOFENCE_UNCERTAINTY_EXTRA_MARGIN_M
```

Default values:

```cpp
SOFTWARE_GEOFENCE_X_MIN = 0.5f
SOFTWARE_GEOFENCE_X_MAX = 14.5f
SOFTWARE_GEOFENCE_Y_MIN = 0.5f
SOFTWARE_GEOFENCE_Y_MAX = 59.5f
GEOFENCE_WARNING_BAND_M = 0.5f
GEOFENCE_UNCERTAINTY_EXTRA_MARGIN_M = 0.5f
```

Include loop timing constants:

```cpp
MAIN_LOOP_PERIOD_MS = 20UL
VISION_PERIOD_MS = 66UL
TELEMETRY_PERIOD_MS = 1000UL
HEARTBEAT_PERIOD_MS = 250UL
PEER_LOST_TIMEOUT_MS = 2000UL
```

Include drone identity constants:

```cpp
DRONE_ID
DRONE_ROLE
SWARM_PACKET_VERSION
```

Default values:

```cpp
DRONE_ID = 1
DRONE_ROLE = 0
SWARM_PACKET_VERSION = 1
```

Include mission altitude constants:

```cpp
MISSION_ALTITUDE_M
LANDING_ALTITUDE_STEP_M
```

Default values:

```cpp
MISSION_ALTITUDE_M = 2.0f
LANDING_ALTITUDE_STEP_M = 0.2f
```

Include safety constants:

```cpp
BATTERY_LOW_VOLTAGE
BATTERY_CRITICAL_VOLTAGE
FC_LINK_TIMEOUT_MS
CAMERA_STALL_TIMEOUT_MS
RADIO_TIMEOUT_MS
KILL_SWITCH_PIN
UNSAFE_PROXIMITY_DISTANCE_M
SURFACE_CONTACT_ALLOWED_ONLY_LANDING
```

Default values:

```cpp
BATTERY_LOW_VOLTAGE = 14.0f
BATTERY_CRITICAL_VOLTAGE = 13.6f
FC_LINK_TIMEOUT_MS = 500UL
CAMERA_STALL_TIMEOUT_MS = 1000UL
RADIO_TIMEOUT_MS = 2000UL
KILL_SWITCH_PIN = 0
UNSAFE_PROXIMITY_DISTANCE_M = 1.0f
SURFACE_CONTACT_ALLOWED_ONLY_LANDING = true
```

Mark all default values as tunable implementation defaults unless they directly match an official rule.

---

## 8. THRESHOLDS HEADER REQUIREMENTS

Generate `config/thresholds.h`.

Use `#pragma once`.

Use embedded-friendly C++.

Every constant must have a comment saying one of:

```text
// official
// derived from uploaded logic
// tunable implementation default
```

Include localization thresholds:

```cpp
OPTICAL_FLOW_FOCAL_LENGTH_PX
TOF_ALTITUDE_FILTER_ALPHA
FLOW_QUALITY_MIN
DRIFT_UNCERTAINTY_LIMIT_M
LOCALIZATION_RATE_HZ
```

Default values:

```cpp
OPTICAL_FLOW_FOCAL_LENGTH_PX = 400.0f
TOF_ALTITUDE_FILTER_ALPHA = 0.85f
FLOW_QUALITY_MIN = 0.30f
DRIFT_UNCERTAINTY_LIMIT_M = 1.0f
LOCALIZATION_RATE_HZ = 50
```

Include vision thresholds:

```cpp
IMAGE_WIDTH
IMAGE_HEIGHT
H_FOV_DEG
V_FOV_DEG
CIRCULARITY_MIN
BLOB_AREA_MIN_PX
BLOB_AREA_MAX_PX
CONFIDENCE_REPORT_MIN
PERSISTENCE_RADIUS_M
PERSISTENCE_COUNT_MIN
EDGE_REJECT_MARGIN_PX
GLARE_REJECT_ENABLED
```

Default values:

```cpp
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
H_FOV_DEG = 60.0f
V_FOV_DEG = 45.0f
CIRCULARITY_MIN = 0.70f
BLOB_AREA_MIN_PX = 25.0f
BLOB_AREA_MAX_PX = 2500.0f
CONFIDENCE_REPORT_MIN = 45.0f
PERSISTENCE_RADIUS_M = 0.25f
PERSISTENCE_COUNT_MIN = 5
EDGE_REJECT_MARGIN_PX = 8.0f
GLARE_REJECT_ENABLED = true
```

Include mine marker HSV threshold placeholders:

```cpp
ON_GROUND_MINE_HSV_LOW
ON_GROUND_MINE_HSV_HIGH
BURIED_SURFACE_MARKER_HSV_LOW
BURIED_SURFACE_MARKER_HSV_HIGH
```

Use simple placeholder structs or arrays for HSV ranges.

Mark them as tunable implementation defaults.

Include mine map thresholds:

```cpp
SAME_DRONE_DEDUP_RADIUS_M
CROSS_DRONE_DEDUP_RADIUS_M
MINE_CONFIRM_CONFIDENCE_MIN
MINE_STALE_TIMEOUT_MS
MAP_VERSION_START
MINE_FUSION_CONFIDENCE_GAIN
```

Default values:

```cpp
SAME_DRONE_DEDUP_RADIUS_M = 0.30f
CROSS_DRONE_DEDUP_RADIUS_M = 0.50f
MINE_CONFIRM_CONFIDENCE_MIN = 70.0f
MINE_STALE_TIMEOUT_MS = 15000UL
MAP_VERSION_START = 0
MINE_FUSION_CONFIDENCE_GAIN = 0.4f
```

Include path planner thresholds:

```cpp
PATH_GRID_RESOLUTION_M
OBSTACLE_INFLATION_RADIUS_M
HUMAN_CORRIDOR_WIDTH_M
PATH_EXACT_CLEARANCE_STEP_M
PATH_SMOOTHING_DISTANCE_M
MAX_PATH_WAYPOINTS
```

Default values:

```cpp
PATH_GRID_RESOLUTION_M = 0.5f
OBSTACLE_INFLATION_RADIUS_M = 0.5f
HUMAN_CORRIDOR_WIDTH_M = 1.0f
PATH_EXACT_CLEARANCE_STEP_M = 0.25f
PATH_SMOOTHING_DISTANCE_M = 0.75f
MAX_PATH_WAYPOINTS = 24
```

Include command recognition thresholds:

```cpp
START_CONFIDENCE_MIN
FORWARD_CONFIDENCE_MIN
PAUSE_CONFIDENCE_MIN
SCAN_CONFIDENCE_MIN
STOP_CONFIDENCE_MIN
COMMAND_DEBOUNCE_MS
COMMAND_LOCKOUT_MS
COMMAND_HYSTERESIS_FRAMES
```

Default values:

```cpp
START_CONFIDENCE_MIN = 0.70f
FORWARD_CONFIDENCE_MIN = 0.60f
PAUSE_CONFIDENCE_MIN = 0.80f
SCAN_CONFIDENCE_MIN = 0.60f
STOP_CONFIDENCE_MIN = 0.90f
COMMAND_DEBOUNCE_MS = 300UL
COMMAND_LOCKOUT_MS = 1500UL
COMMAND_HYSTERESIS_FRAMES = 3
```

Include human tracking thresholds:

```cpp
HUMAN_TRACK_CONFIDENCE_MIN
HUMAN_OFF_PATH_DISTANCE_M
HUMAN_RECOVERY_TIMEOUT_MS
HUMAN_EXIT_CONFIRM_TIMEOUT_MS
```

Default values:

```cpp
HUMAN_TRACK_CONFIDENCE_MIN = 0.60f
HUMAN_OFF_PATH_DISTANCE_M = 0.75f
HUMAN_RECOVERY_TIMEOUT_MS = 3000UL
HUMAN_EXIT_CONFIRM_TIMEOUT_MS = 2000UL
```

---

## 9. SHARED TYPES HEADER REQUIREMENTS

Generate `src/types.h`.

Use `#pragma once`.

Include `<stdint.h>`.

Define fixed capacity constants:

```cpp
MAX_MINES
MAX_CANDIDATES
MAX_PATH_WAYPOINTS
MAX_SWARM_PEERS
MAX_TELEMETRY_EVENTS
MAX_COMMAND_EVENTS
MAX_COVERAGE_CELLS
SWARM_PAYLOAD_MAX_BYTES
```

Use safe embedded values.

Example:

```cpp
constexpr uint16_t MAX_MINES = 80;
constexpr uint16_t MAX_CANDIDATES = 16;
constexpr uint16_t MAX_PATH_WAYPOINTS = 24;
constexpr uint16_t MAX_SWARM_PEERS = 4;
constexpr uint16_t MAX_TELEMETRY_EVENTS = 64;
constexpr uint16_t MAX_COMMAND_EVENTS = 16;
constexpr uint16_t MAX_COVERAGE_CELLS = 3600;
constexpr uint16_t SWARM_PAYLOAD_MAX_BYTES = 64;
```

Define these enums:

```cpp
DroneState
DroneRole
MineStatus
CommandType
SafetyAction
PacketType
MarkerPattern
LocalizationHealth
GeofenceStatus
VisionMarkerType
FcCommand
CoverageStatus
```

Use these DroneState values:

```cpp
INIT
CALIBRATE
WAIT_FOR_START
TAKEOFF
FORMATION
SEARCHING
PLANNING
GUIDING
MISSION_COMPLETE
LANDING
DISARMED
HOLD
EMERGENCY
```

Use these DroneRole values:

```cpp
SCOUT_LEFT
SCOUT_RIGHT
GUIDE_MARKER
RESERVE
```

Use these MineStatus values:

```cpp
CANDIDATE
CONFIRMED
REJECTED
```

Use these CommandType values:

```cpp
NONE
START
FORWARD
PAUSE
SCAN_LEFT
SCAN_RIGHT
STOP_ABORT
```

Use these SafetyAction values:

```cpp
CONTINUE
HOLD
LAND
EMERGENCY_CUT
```

Use these PacketType values:

```cpp
HEARTBEAT
ROLE_ASSIGN
CLAIM
YIELD
MINE_UPDATE
PATH_UPDATE
PERSON_UPDATE
HELP_REQUEST
LAND_NOW
```

Use these MarkerPattern values:

```cpp
MARKER_OFF
MARKER_FORWARD
MARKER_STOP
MARKER_LEFT
MARKER_RIGHT
MARKER_SAFE_PATH
MARKER_EMERGENCY
MARKER_MISSION_COMPLETE
```

Use these LocalizationHealth values:

```cpp
LOCALIZATION_GOOD
LOCALIZATION_DEGRADED
LOCALIZATION_UNRECOVERABLE
```

Use these GeofenceStatus values:

```cpp
GEOFENCE_INSIDE
GEOFENCE_WARNING
GEOFENCE_NEAR_LIMIT
GEOFENCE_OUTSIDE
```

Use these VisionMarkerType values:

```cpp
UNKNOWN
ON_GROUND_MINE
BURIED_SURFACE_MARKER
```

Use these FcCommand values:

```cpp
HOLD
TAKEOFF
LAND
VELOCITY
ALTITUDE
HEADING
ARM
DISARM
EMERGENCY_STOP
```

Use these CoverageStatus values:

```cpp
UNSCANNED
SCANNED
UNCERTAIN
MINE_CONFIRMED
OBSTACLE_CONFIRMED
```

Define these structs:

```cpp
Vec2
Pose2D
OpticalFlowSample
TofSample
AttitudeSample
VisionCandidate
MineRecord
PathWaypoint
SafePath
SwarmPacket
HumanTrack
TelemetryEvent
```

Required struct fields:

`Vec2`:

```cpp
float x;
float y;
```

`Pose2D`:

```cpp
float local_x;
float local_y;
float field_x;
float field_y;
float yaw_deg;
uint32_t timestamp_ms;
```

`OpticalFlowSample`:

```cpp
bool valid;
float pixel_shift_x;
float pixel_shift_y;
float quality;
uint32_t timestamp_ms;
```

`TofSample`:

```cpp
bool valid;
float altitude_m;
uint32_t timestamp_ms;
```

`AttitudeSample`:

```cpp
bool valid;
bool armed;
float roll_deg;
float pitch_deg;
float yaw_deg;
float altitude_m;
float battery_voltage;
uint32_t timestamp_ms;
```

`VisionCandidate`:

```cpp
float pixel_x;
float pixel_y;
float world_x;
float world_y;
float confidence;
float circularity;
float area;
VisionMarkerType marker_type;
uint32_t timestamp_ms;
```

`MineRecord`:

```cpp
uint16_t mine_id;
float x;
float y;
float confidence;
uint16_t persistence_count;
uint32_t first_seen_time;
uint32_t last_seen_time;
uint8_t source_drone_id;
VisionMarkerType marker_type;
MineStatus status;
uint32_t map_version;
```

`PathWaypoint`:

```cpp
float x;
float y;
```

`SafePath`:

```cpp
bool valid;
uint32_t path_version;
uint32_t created_time;
float corridor_width_m;
uint8_t waypoint_count;
PathWaypoint waypoints[MAX_PATH_WAYPOINTS];
```

`SwarmPacket`:

```cpp
PacketType packet_type;
uint8_t packet_version;
uint8_t sender_drone_id;
uint32_t timestamp_ms;
uint32_t map_version;
uint16_t payload_length;
uint8_t payload[SWARM_PAYLOAD_MAX_BYTES];
```

`HumanTrack`:

```cpp
bool human_detected;
float field_x;
float field_y;
float lateral_deviation_m;
float forward_progress_m;
float tracking_confidence;
bool human_in_exit_zone;
uint32_t timestamp_ms;
```

`TelemetryEvent`:

```cpp
uint32_t timestamp_ms;
uint16_t event_id;
float value;
```

Use default member initializers where useful.

Do not use dynamic containers such as `std::vector` inside these shared types.

---

## 10. PLACEHOLDER FILES

Create all remaining files from the repository structure.

For each placeholder file, add only a short comment.

Example for `.h` files:

```cpp
#pragma once

// Implemented in a later prompt.
```

Example for `.cpp` files:

```cpp
// Implemented in a later prompt.
```

Do not implement full module logic in Prompt 1.

---

## 11. CODING STYLE REQUIREMENTS

Use this coding style:

- Use `.h` and `.cpp` files.
- Use `#pragma once` in headers.
- Use `uint8_t`, `uint16_t`, `uint32_t`, and `int16_t` where appropriate.
- Use `float` for physical measurements.
- Use `constexpr` for compile-time constants.
- Use enums with explicit underlying types where useful.
- Avoid heavy STL use in the main control path.
- Avoid dynamic memory allocation in the main control path.
- Keep function names clear and action-oriented.
- Keep module interfaces simple.
- Do not use GPS.
- Do not use cloud.
- Do not use external positioning.
- Do not use remote control.

---

## 12. OUTPUT REQUIREMENTS

After generating the files, list:

1. All created files.
2. Any assumptions made.
3. Any constants marked as tunable implementation defaults.
4. Any missing hardware details that were replaced with safe defaults.

Do not generate module implementation logic beyond the placeholder comments.

Do not generate tests.

Do not generate simulation code.

Do not generate scoring code.

Do not generate hardware design files.

Do not generate battery sizing, motor sizing, thrust calculation, or weight calculation files.

Generate only the drone-only software foundation.
