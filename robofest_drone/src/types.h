#pragma once

#include <stdint.h>

namespace RobofestDrone {
namespace Types {

// ============================================================================
// FIXED CAPACITY CONSTANTS (EMBEDDED SAFE SIZES)
// ============================================================================

constexpr uint16_t MAX_MINES = 80;
constexpr uint16_t MAX_CANDIDATES = 16;
constexpr uint16_t MAX_PATH_WAYPOINTS = 32;
constexpr uint16_t MAX_SWARM_PEERS = 4;
constexpr uint16_t MAX_TELEMETRY_EVENTS = 64;
constexpr uint16_t MAX_COMMAND_EVENTS = 16;
constexpr uint16_t MAX_COVERAGE_CELLS = 3600;
constexpr uint16_t SWARM_PAYLOAD_MAX_BYTES = 96;


// ============================================================================
// ENUMERATIONS
// ============================================================================

enum class DroneState : uint8_t {
    INIT = 0,
    CALIBRATE,
    WAIT_FOR_START,
    TAKEOFF,
    FORMATION,
    SEARCHING,
    PLANNING,
    GUIDING,
    MISSION_COMPLETE,
    LANDING,
    DISARMED,
    HOLD,
    EMERGENCY
};

enum class DroneRole : uint8_t {
    SCOUT_LEFT = 0,
    SCOUT_RIGHT,
    GUIDE_MARKER,
    RESERVE
};

enum class MineStatus : uint8_t {
    CANDIDATE = 0,
    CONFIRMED,
    REJECTED
};

enum class CommandType : uint8_t {
    NONE = 0,
    START,
    FORWARD,
    PAUSE,
    SCAN_LEFT,
    SCAN_RIGHT,
    STOP_ABORT
};

enum class CommandSource : uint8_t {
    COMMAND_SOURCE_NONE = 0,
    COMMAND_SOURCE_GESTURE,
    COMMAND_SOURCE_VOICE,
    COMMAND_SOURCE_FUSED,
    COMMAND_SOURCE_DEBUG
};

enum class SafetyAction : uint8_t {
    CONTINUE = 0,
    HOLD,
    LAND,
    EMERGENCY_CUT
};

enum class SafetyFault : uint8_t {
    SAFETY_FAULT_NONE = 0,
    SAFETY_FAULT_KILL_SWITCH,
    SAFETY_FAULT_BATTERY_CRITICAL,
    SAFETY_FAULT_BATTERY_LOW,
    SAFETY_FAULT_FC_LINK_LOST,
    SAFETY_FAULT_CAMERA_STALL,
    SAFETY_FAULT_OPTICAL_FLOW_FAILURE,
    SAFETY_FAULT_TOF_FAILURE,
    SAFETY_FAULT_RADIO_TIMEOUT,
    SAFETY_FAULT_SWARM_DEGRADED,
    SAFETY_FAULT_SWARM_CRITICAL,
    SAFETY_FAULT_LOCALIZATION_DEGRADED,
    SAFETY_FAULT_LOCALIZATION_UNRECOVERABLE,
    SAFETY_FAULT_MISSION_TIMEOUT,
    SAFETY_FAULT_GEOFENCE_WARNING,
    SAFETY_FAULT_GEOFENCE_NEAR_LIMIT,
    SAFETY_FAULT_GEOFENCE_OUTSIDE,
    SAFETY_FAULT_COLLISION_RISK,
    SAFETY_FAULT_UNSAFE_PROXIMITY_HUMAN,
    SAFETY_FAULT_UNSAFE_PROXIMITY_PEER,
    SAFETY_FAULT_SURFACE_CONTACT,
    SAFETY_FAULT_WATCHDOG_STALL,
    SAFETY_FAULT_INVALID_INPUT
};

enum class PacketType : uint8_t {
    HEARTBEAT = 0,
    ROLE_ASSIGN,
    CLAIM,
    YIELD,
    MINE_UPDATE,
    PATH_UPDATE,
    PERSON_UPDATE,
    HELP_REQUEST,
    LAND_NOW
};

enum class MarkerPattern : uint8_t {
    MARKER_OFF = 0,
    MARKER_FORWARD,
    MARKER_STOP,
    MARKER_LEFT,
    MARKER_RIGHT,
    MARKER_SAFE_PATH,
    MARKER_EMERGENCY,
    MARKER_MISSION_COMPLETE,
    MARKER_CAUTION,
    MARKER_REJOIN_LEFT,
    MARKER_REJOIN_RIGHT,
    MARKER_LANDING
};

enum class LocalizationHealth : uint8_t {
    LOCALIZATION_GOOD = 0,
    LOCALIZATION_DEGRADED,
    LOCALIZATION_UNRECOVERABLE
};

enum class GeofenceStatus : uint8_t {
    GEOFENCE_INSIDE = 0,
    GEOFENCE_WARNING,
    GEOFENCE_NEAR_LIMIT,
    GEOFENCE_OUTSIDE
};

enum class VisionMarkerType : uint8_t {
    UNKNOWN = 0,
    ON_GROUND_MINE,
    BURIED_SURFACE_MARKER
};

enum class FcCommand : uint8_t {
    HOLD = 0,
    TAKEOFF,
    LAND,
    VELOCITY,
    ALTITUDE,
    HEADING,
    ARM,
    DISARM,
    EMERGENCY_STOP
};

enum class CoverageStatus : uint8_t {
    UNSCANNED = 0,
    SCANNED,
    UNCERTAIN,
    MINE_CONFIRMED,
    OBSTACLE_CONFIRMED
};


// ============================================================================
// CORE DATA STRUCTURES (FIXED ALLOCATION)
// ============================================================================

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    constexpr Vec2() : x(0.0f), y(0.0f) {}
    constexpr Vec2(float _x, float _y) : x(_x), y(_y) {}
};

struct Pose2D {
    float x = 0.0f;
    float y = 0.0f;
    float local_x = 0.0f;
    float local_y = 0.0f;
    float field_x = 0.0f;
    float field_y = 0.0f;
    float yaw_deg = 0.0f;
    uint32_t timestamp_ms = 0;
};

struct OpticalFlowSample {
    bool valid = false;
    float pixel_shift_x = 0.0f;
    float pixel_shift_y = 0.0f;
    float quality = 0.0f;
    uint32_t timestamp_ms = 0;
};

struct TofSample {
    bool valid = false;
    float altitude_m = 0.0f;
    uint32_t timestamp_ms = 0;
};

struct LidarObstacleSample {
    bool valid = false;
    float min_front_distance_m = 12.0f;
    float closest_angle_deg = 0.0f;
    uint16_t point_count = 0;
    bool obstacle_in_warning_zone = false;
    bool obstacle_in_emergency_zone = false;
    uint32_t timestamp_ms = 0;
};

struct AttitudeSample {
    bool valid = false;
    bool armed = false;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float altitude_m = 0.0f;
    float battery_voltage = 0.0f;
    float battery_current = 0.0f;
    bool battery_current_valid = false;
    uint8_t flight_mode = 0;
    uint16_t error_flags = 0;
    uint8_t protocol_version = 1;
    uint32_t timestamp_ms = 0;
};

struct VisionCandidate {
    float pixel_x = 0.0f;
    float pixel_y = 0.0f;
    float world_x = 0.0f;
    float world_y = 0.0f;
    float confidence = 0.0f;
    float circularity = 0.0f;
    float area = 0.0f;
    float normalized_area_score = 0.0f;
    uint16_t persistence_count = 0;
    VisionMarkerType marker_type = VisionMarkerType::UNKNOWN;
    uint32_t frame_id = 0;
    uint32_t timestamp_ms = 0;
};

struct MineRecord {
    uint16_t mine_id = 0;
    float x = 0.0f;
    float y = 0.0f;
    float confidence = 0.0f;
    uint16_t persistence_count = 0;
    uint32_t first_seen_time = 0;
    uint32_t last_seen_time = 0;
    uint8_t source_drone_id = 0;
    VisionMarkerType marker_type = VisionMarkerType::UNKNOWN;
    MineStatus status = MineStatus::CANDIDATE;
    uint32_t map_version = 0;

    bool claimed = false;
    uint8_t claim_owner_id = 0;
    uint32_t claim_time_ms = 0;
    uint32_t claim_expiry_ms = 0;

    uint16_t rejection_reason = 0;
    uint16_t update_count = 0;
};

struct PathWaypoint {
    float x = 0.0f;
    float y = 0.0f;

    constexpr PathWaypoint() : x(0.0f), y(0.0f) {}
    constexpr PathWaypoint(float _x, float _y) : x(_x), y(_y) {}
};

struct SafePath {
    bool valid = false;
    uint32_t path_version = 0;
    uint32_t created_time = 0;
    float corridor_width_m = 0.0f;
    uint8_t waypoint_count = 0;
    PathWaypoint waypoints[MAX_PATH_WAYPOINTS] = {};
};

struct SwarmPacket {
    PacketType packet_type = PacketType::HEARTBEAT;
    uint8_t packet_version = 1;
    uint8_t sender_drone_id = 0;
    uint32_t timestamp_ms = 0;
    uint32_t map_version = 0;
    uint16_t payload_length = 0;
    uint8_t payload[SWARM_PAYLOAD_MAX_BYTES] = {};
};

struct HumanDetectionSample {
    bool valid = false;
    float confidence = 0.0f;
    float pixel_x = 0.0f;
    float pixel_y = 0.0f;
    float pixel_width = 0.0f;
    float pixel_height = 0.0f;
    float distance_m = 0.0f;
    bool distance_valid = false;
    bool field_position_valid = false;
    float field_x = 0.0f;
    float field_y = 0.0f;
    uint32_t timestamp_ms = 0;
};

struct HumanTrack {
    bool human_detected = false;
    float field_x = 0.0f;
    float field_y = 0.0f;
    float velocity_x = 0.0f;
    float velocity_y = 0.0f;
    float lateral_deviation_m = 0.0f;
    float forward_progress_m = 0.0f;
    float tracking_confidence = 0.0f;
    bool human_off_path = false;
    bool human_in_exit_zone = false;
    float distance_to_drone_m = 0.0f;
    uint32_t timestamp_ms = 0;
};

struct CommandSample {
    bool valid = false;
    CommandType command = CommandType::NONE;
    float confidence = 0.0f;
    CommandSource source = CommandSource::COMMAND_SOURCE_NONE;
    uint32_t timestamp_ms = 0;
};

struct CommandEvent {
    CommandType command = CommandType::NONE;
    float confidence = 0.0f;
    CommandSource source = CommandSource::COMMAND_SOURCE_NONE;
    bool accepted = false;
    uint16_t rejection_reason = 0;
    uint32_t timestamp_ms = 0;
};

struct SafetyInputs {
    uint32_t now_ms = 0;

    bool kill_switch_active = false;

    bool battery_present = true;
    float battery_voltage = 12.0f;
    float battery_current = 0.0f;
    bool battery_current_valid = false;

    bool fc_link_healthy = true;
    bool fc_armed = false;
    uint32_t fc_last_rx_ms = 0;

    DroneState drone_state = DroneState::INIT;
    bool mission_timer_running = false;
    uint32_t mission_elapsed_ms = 0;

    LocalizationHealth localization_health = LocalizationHealth::LOCALIZATION_GOOD;
    float drift_uncertainty_m = 0.0f;
    bool localization_update_recent = true;

    float altitude_m = 0.0f;
    float vertical_speed_mps = 0.0f;
    bool altitude_valid = true;
    bool touchdown_allowed = false;
    bool surface_contact_detected = false;

    bool camera_healthy = true;
    uint32_t camera_last_frame_ms = 0;

    bool optical_flow_healthy = true;
    bool tof_healthy = true;

    bool radio_healthy = true;
    uint32_t radio_last_rx_ms = 0;

    bool swarm_healthy = true;
    bool swarm_degraded = false;
    bool swarm_critical = false;
    uint8_t active_peer_count = 0;
    float nearest_peer_distance_m = 100.0f;
    bool nearest_peer_distance_valid = false;

    bool human_detected = false;
    float nearest_human_distance_m = 100.0f;
    bool nearest_human_distance_valid = false;

    GeofenceStatus geofence_status = GeofenceStatus::GEOFENCE_INSIDE;

    bool path_valid = false;
    bool command_stop_requested = false;
};

// Telemetry Severity Levels
constexpr uint8_t TELEMETRY_SEVERITY_DEBUG = 0;
constexpr uint8_t TELEMETRY_SEVERITY_INFO = 1;
constexpr uint8_t TELEMETRY_SEVERITY_WARNING = 2;
constexpr uint8_t TELEMETRY_SEVERITY_CRITICAL = 3;

// Telemetry Module IDs
constexpr uint8_t TELEMETRY_MODULE_SYSTEM = 0;
constexpr uint8_t TELEMETRY_MODULE_STATE_MACHINE = 1;
constexpr uint8_t TELEMETRY_MODULE_LOCALIZATION = 2;
constexpr uint8_t TELEMETRY_MODULE_GEOFENCE = 3;
constexpr uint8_t TELEMETRY_MODULE_VISION = 4;
constexpr uint8_t TELEMETRY_MODULE_MINE_MAP = 5;
constexpr uint8_t TELEMETRY_MODULE_PATH_PLANNER = 6;
constexpr uint8_t TELEMETRY_MODULE_SWARM = 7;
constexpr uint8_t TELEMETRY_MODULE_COMMAND = 8;
constexpr uint8_t TELEMETRY_MODULE_SAFETY = 9;
constexpr uint8_t TELEMETRY_MODULE_FC_BRIDGE = 10;
constexpr uint8_t TELEMETRY_MODULE_HUMAN_TRACKER = 11;
constexpr uint8_t TELEMETRY_MODULE_MARKER = 12;
constexpr uint8_t TELEMETRY_MODULE_SEARCH = 13;
constexpr uint8_t TELEMETRY_MODULE_TELEMETRY = 14;

struct TelemetryEvent {
    uint32_t timestamp_ms = 0;
    uint16_t event_id = 0;
    uint8_t severity = 0;
    uint8_t module_id = 0;
    float value_a = 0.0f;
    float value_b = 0.0f;
    uint16_t context_id = 0;
};

struct MissionSummary {
    bool mission_started = false;
    bool mission_completed = false;
    bool mission_timeout = false;
    bool takeoff_started = false;
    bool takeoff_completed = false;
    bool landing_started = false;
    bool landing_completed = false;
    bool human_exit_confirmed = false;
    bool path_valid_at_completion = false;

    uint32_t mission_start_ms = 0;
    uint32_t mission_end_ms = 0;
    uint32_t mission_elapsed_ms = 0;

    uint16_t state_transition_count = 0;
    uint16_t mine_candidate_count = 0;
    uint16_t mine_confirmed_count = 0;
    uint16_t mine_rejected_count = 0;
    uint16_t claim_sent_count = 0;
    uint16_t claim_received_count = 0;
    uint16_t yield_count = 0;
    uint16_t path_plan_count = 0;
    uint16_t path_invalid_count = 0;
    uint16_t reroute_count = 0;
    uint16_t human_off_path_count = 0;
    uint16_t command_accepted_count = 0;
    uint16_t command_rejected_count = 0;
    uint16_t safety_hold_count = 0;
    uint16_t safety_land_count = 0;
    uint16_t emergency_count = 0;
    uint16_t peer_lost_count = 0;

    float final_drift_uncertainty_m = 0.0f;
    float max_drift_uncertainty_m = 0.0f;
    float min_battery_voltage = 16.8f;
    float final_battery_voltage = 0.0f;
};

struct TelemetryInputs {
    uint32_t now_ms = 0;

    DroneState drone_state = DroneState::INIT;
    SafetyAction safety_action = SafetyAction::CONTINUE;
    LocalizationHealth localization_health = LocalizationHealth::LOCALIZATION_GOOD;
    GeofenceStatus geofence_status = GeofenceStatus::GEOFENCE_INSIDE;

    float battery_voltage = 0.0f;
    float drift_uncertainty_m = 0.0f;
    float altitude_m = 0.0f;

    uint16_t confirmed_mine_count = 0;
    uint16_t candidate_mine_count = 0;

    bool path_valid = false;
    uint32_t path_version = 0;

    bool human_detected = false;
    bool human_off_path = false;
    bool human_in_exit_zone = false;

    bool swarm_healthy = true;
    bool swarm_degraded = false;
    bool swarm_critical = false;
    uint8_t active_peer_count = 0;

    bool storage_healthy = true;
};

} // namespace Types
} // namespace RobofestDrone
