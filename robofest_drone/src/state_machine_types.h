#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"

namespace RobofestDrone {

// ============================================================================
// STATE MACHINE EVENTS
// ============================================================================

enum class StateMachineEvent : uint16_t {
    EVENT_NONE = 0,
    EVENT_SELF_CHECK_PASSED,
    EVENT_SELF_CHECK_FAILED,
    EVENT_CALIBRATION_COMPLETE,
    EVENT_CALIBRATION_FAILED,
    EVENT_START_COMMAND_VALID,
    EVENT_TAKEOFF_REQUESTED,
    EVENT_TAKEOFF_COMPLETE,
    EVENT_TAKEOFF_FAILED,
    EVENT_ALTITUDE_STABLE,
    EVENT_LOCALIZATION_HEALTHY,
    EVENT_LOCALIZATION_DEGRADED,
    EVENT_LOCALIZATION_UNRECOVERABLE,
    EVENT_FORMATION_READY,
    EVENT_FORMATION_TIMEOUT,
    EVENT_ROUTE_POSSIBLE,
    EVENT_ROUTE_NOT_POSSIBLE,
    EVENT_PATH_VALID,
    EVENT_PATH_INVALID,
    EVENT_NEW_MINE_BLOCKS_PATH,
    EVENT_HUMAN_OFF_PATH,
    EVENT_HUMAN_TRACKING_LOST_RECOVERABLE,
    EVENT_HUMAN_TRACKING_LOST_UNRECOVERABLE,
    EVENT_HUMAN_EXIT_REACHED,
    EVENT_MISSION_COMPLETE_CONFIRMED,
    EVENT_PAUSE_COMMAND,
    EVENT_RESUME_COMMAND,
    EVENT_STOP_ABORT_COMMAND,
    EVENT_KILL_SWITCH_ACTIVE,
    EVENT_KILL_SWITCH_CLEARED,
    EVENT_FC_LINK_LOST,
    EVENT_FC_LINK_RECOVERED,
    EVENT_BATTERY_LOW,
    EVENT_BATTERY_CRITICAL,
    EVENT_MISSION_TIMEOUT,
    EVENT_SWARM_PEERS_HEALTHY,
    EVENT_SWARM_DEGRADED,
    EVENT_SWARM_CRITICAL,
    EVENT_TOUCHDOWN_DETECTED,
    EVENT_LANDING_TIMEOUT,
    EVENT_EMERGENCY_STOP_REQUESTED,
    EVENT_DISARM_COMPLETE
};


// ============================================================================
// STATE MACHINE TIMING & THRESHOLD CONSTANTS
// ============================================================================

// tunable implementation default
constexpr uint32_t STATE_MACHINE_CALIBRATION_TIMEOUT_MS = 10000UL;

// tunable implementation default
constexpr uint32_t STATE_MACHINE_START_COMMAND_TIMEOUT_MS = 300000UL;

// tunable implementation default
constexpr uint32_t STATE_MACHINE_TAKEOFF_TIMEOUT_MS = 20000UL;

// tunable implementation default
constexpr uint32_t STATE_MACHINE_FORMATION_TIMEOUT_MS = 30000UL;

// tunable implementation default
constexpr uint32_t STATE_MACHINE_PLANNING_TIMEOUT_MS = 10000UL;

// tunable implementation default
constexpr uint32_t STATE_MACHINE_GUIDING_RECOVERY_TIMEOUT_MS = 5000UL;

// tunable implementation default
constexpr uint32_t STATE_MACHINE_MISSION_COMPLETE_VALIDATION_MS = 2000UL;

// tunable implementation default
constexpr uint32_t STATE_MACHINE_LANDING_TIMEOUT_MS = 30000UL;

// tunable implementation default
constexpr uint32_t STATE_MACHINE_HOLD_RECOVERY_TIMEOUT_MS = 20000UL;

// tunable implementation default
constexpr float STATE_MACHINE_ALTITUDE_NEAR_TARGET_M = 0.25f;

// tunable implementation default
constexpr float STATE_MACHINE_TOUCHDOWN_ALTITUDE_M = 0.15f;


// ============================================================================
// STATE MACHINE INPUTS STRUCTURE
// ============================================================================

struct StateMachineInputs {
    uint32_t now_ms = 0;

    Types::DroneState override_state = Types::DroneState::INIT;
    bool override_enabled = false;

    bool self_check_passed = false;
    bool self_check_failed = false;
    bool calibration_complete = false;
    bool calibration_failed = false;

    Types::CommandType latest_command = Types::CommandType::NONE;
    bool command_valid = false;
    float command_confidence = 0.0f;

    bool start_command_allowed = false;
    bool pause_command_allowed = false;
    bool resume_command_allowed = false;
    bool stop_command_allowed = false;

    bool kill_switch_active = false;
    bool fc_link_healthy = false;
    bool fc_armed = false;

    bool battery_present = false;
    bool battery_low = false;
    bool battery_critical = false;

    Types::LocalizationHealth localization_health = Types::LocalizationHealth::LOCALIZATION_GOOD;
    bool altitude_stable = false;
    float current_altitude_m = 0.0f;
    float target_altitude_m = 0.0f;
    bool takeoff_complete = false;
    bool touchdown_detected = false;

    bool swarm_enabled = false;
    bool peers_healthy = false;
    bool roles_confirmed = false;
    bool swarm_degraded = false;
    bool swarm_critical = false;

    bool search_coverage_sufficient = false;
    bool route_possible = false;
    bool path_valid = false;
    bool path_blocked_by_new_mine = false;
    bool human_off_path = false;
    bool human_tracking_lost_recoverable = false;
    bool human_tracking_lost_unrecoverable = false;
    bool human_in_exit_zone = false;
    bool mission_complete_confirmed = false;

    Types::SafetyAction safety_action = Types::SafetyAction::CONTINUE;
};


// ============================================================================
// STATE MACHINE OUTPUTS STRUCTURE
// ============================================================================

struct StateMachineOutputs {
    Types::DroneState current_state = Types::DroneState::INIT;
    Types::DroneState previous_state = Types::DroneState::INIT;
    Types::DroneState resume_state = Types::DroneState::INIT;

    uint32_t state_entered_ms = 0;
    uint32_t time_in_state_ms = 0;

    bool allow_arm = false;
    bool allow_takeoff = false;
    bool allow_horizontal_flight = false;
    bool allow_search_expansion = false;
    bool allow_guidance = false;
    bool allow_mission_completion = false;

    bool request_hold = false;
    bool request_takeoff = false;
    bool request_land = false;
    bool request_disarm = false;
    bool request_emergency_stop = false;
    bool request_mission_abort = false;

    bool mission_timer_running = false;
    bool mission_time_over = false;
    uint32_t mission_elapsed_ms = 0;

    StateMachineEvent last_event = StateMachineEvent::EVENT_NONE;
    uint16_t telemetry_event_id = TE_STATE_MACHINE_INIT;
    bool telemetry_event_valid = false;
};

} // namespace RobofestDrone
