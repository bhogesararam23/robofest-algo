#include "state_machine.h"
#include <cmath>
#include "../config/mission_config.h"

namespace RobofestDrone {

namespace {
    static StateMachine s_global_state_machine;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

StateMachine::StateMachine() {
    reset();
}

void StateMachine::init() {
    reset();
}

void StateMachine::reset() {
    m_outputs = StateMachineOutputs();
    m_outputs.current_state = Types::DroneState::INIT;
    m_outputs.previous_state = Types::DroneState::INIT;
    m_outputs.resume_state = Types::DroneState::WAIT_FOR_START;
    m_outputs.state_entered_ms = 0;
    m_outputs.time_in_state_ms = 0;
    m_outputs.last_event = StateMachineEvent::EVENT_NONE;
    m_outputs.telemetry_event_id = TE_STATE_MACHINE_INIT;
    m_outputs.telemetry_event_valid = true;

    m_mission_start_time_ms = 0;
    m_mission_timer_active = false;
    m_hold_due_to_pause = false;

    updateOutputFlags();
}


// ============================================================================
// STATUS PREDICATES
// ============================================================================

bool StateMachine::isFlightActive() const {
    switch (m_outputs.current_state) {
        case Types::DroneState::TAKEOFF:
        case Types::DroneState::FORMATION:
        case Types::DroneState::SEARCHING:
        case Types::DroneState::PLANNING:
        case Types::DroneState::GUIDING:
        case Types::DroneState::MISSION_COMPLETE:
        case Types::DroneState::LANDING:
            return true;
        case Types::DroneState::HOLD:
            // In HOLD, flight is active if resume state was an airborne state
            return (m_outputs.resume_state == Types::DroneState::TAKEOFF ||
                    m_outputs.resume_state == Types::DroneState::FORMATION ||
                    m_outputs.resume_state == Types::DroneState::SEARCHING ||
                    m_outputs.resume_state == Types::DroneState::PLANNING ||
                    m_outputs.resume_state == Types::DroneState::GUIDING ||
                    m_outputs.resume_state == Types::DroneState::MISSION_COMPLETE);
        default:
            return false;
    }
}

bool StateMachine::isMissionActive() const {
    switch (m_outputs.current_state) {
        case Types::DroneState::TAKEOFF:
        case Types::DroneState::FORMATION:
        case Types::DroneState::SEARCHING:
        case Types::DroneState::PLANNING:
        case Types::DroneState::GUIDING:
        case Types::DroneState::MISSION_COMPLETE:
        case Types::DroneState::HOLD:
            return true;
        default:
            return false;
    }
}

bool StateMachine::isTerminalState() const {
    return (m_outputs.current_state == Types::DroneState::DISARMED ||
            m_outputs.current_state == Types::DroneState::EMERGENCY);
}


// ============================================================================
// STATE TRANSITION HELPER
// ============================================================================

void StateMachine::setState(Types::DroneState new_state, uint32_t now_ms, StateMachineEvent event) {
    if (m_outputs.current_state == new_state) {
        return;
    }

    // Do not leave terminal states unless explicitly managed
    if (m_outputs.current_state == Types::DroneState::EMERGENCY && new_state != Types::DroneState::INIT) {
        return;
    }

    // Save resume state when entering HOLD from active mission states
    if (new_state == Types::DroneState::HOLD) {
        if (m_outputs.current_state == Types::DroneState::FORMATION ||
            m_outputs.current_state == Types::DroneState::SEARCHING ||
            m_outputs.current_state == Types::DroneState::PLANNING ||
            m_outputs.current_state == Types::DroneState::GUIDING ||
            m_outputs.current_state == Types::DroneState::TAKEOFF) {
            m_outputs.resume_state = m_outputs.current_state;
        }
    }

    m_outputs.previous_state = m_outputs.current_state;
    m_outputs.current_state = new_state;
    m_outputs.state_entered_ms = now_ms;
    m_outputs.time_in_state_ms = 0;
    m_outputs.last_event = event;
    m_outputs.telemetry_event_id = mapStateToTelemetryEvent(new_state);
    m_outputs.telemetry_event_valid = true;

    updateOutputFlags();
}

uint16_t StateMachine::mapStateToTelemetryEvent(Types::DroneState state) const {
    switch (state) {
        case Types::DroneState::INIT:             return TE_STATE_MACHINE_INIT;
        case Types::DroneState::CALIBRATE:        return TE_STATE_MACHINE_CALIBRATE;
        case Types::DroneState::WAIT_FOR_START:   return TE_STATE_MACHINE_WAIT_FOR_START;
        case Types::DroneState::TAKEOFF:          return TE_STATE_MACHINE_TAKEOFF;
        case Types::DroneState::FORMATION:        return TE_STATE_MACHINE_FORMATION;
        case Types::DroneState::SEARCHING:        return TE_STATE_MACHINE_SEARCHING;
        case Types::DroneState::PLANNING:         return TE_STATE_MACHINE_PLANNING;
        case Types::DroneState::GUIDING:          return TE_STATE_MACHINE_GUIDING;
        case Types::DroneState::MISSION_COMPLETE: return TE_STATE_MACHINE_MISSION_COMPLETE;
        case Types::DroneState::LANDING:          return TE_STATE_MACHINE_LANDING;
        case Types::DroneState::DISARMED:         return TE_STATE_MACHINE_DISARMED;
        case Types::DroneState::HOLD:             return TE_STATE_MACHINE_HOLD;
        case Types::DroneState::EMERGENCY:        return TE_STATE_MACHINE_EMERGENCY;
        default:                                  return TE_STATE_MACHINE_INIT;
    }
}


// ============================================================================
// MISSION TIMER
// ============================================================================

void StateMachine::updateMissionTimer(const StateMachineInputs& inputs, uint32_t now_ms) {
    (void)inputs;

    if (m_mission_timer_active) {
        // Unsigned subtraction safely handles timer duration
        m_outputs.mission_elapsed_ms = now_ms - m_mission_start_time_ms;
        m_outputs.mission_timer_running = true;

        if (m_outputs.mission_elapsed_ms >= Config::MISSION_TIME_LIMIT_MS) {
            m_outputs.mission_time_over = true;

            if (m_outputs.current_state != Types::DroneState::LANDING &&
                m_outputs.current_state != Types::DroneState::DISARMED &&
                m_outputs.current_state != Types::DroneState::EMERGENCY) {
                setState(Types::DroneState::LANDING, now_ms, StateMachineEvent::EVENT_MISSION_TIMEOUT);
                m_outputs.telemetry_event_id = TE_MISSION_TIMER_TIMEOUT;
                m_outputs.telemetry_event_valid = true;
            }
        }
    } else {
        m_outputs.mission_timer_running = false;
    }
}


// ============================================================================
// GLOBAL SAFETY OVERRIDES
// ============================================================================

void StateMachine::applySafetyOverrides(const StateMachineInputs& inputs, uint32_t now_ms) {
    // 1. EMERGENCY (Highest Priority)
    if (inputs.kill_switch_active) {
        if (m_outputs.current_state != Types::DroneState::EMERGENCY) {
            setState(Types::DroneState::EMERGENCY, now_ms, StateMachineEvent::EVENT_KILL_SWITCH_ACTIVE);
            m_outputs.telemetry_event_id = TE_KILL_SWITCH_ACTIVE;
            m_outputs.telemetry_event_valid = true;
        }
        return;
    }

    if (inputs.safety_action == Types::SafetyAction::EMERGENCY_CUT) {
        if (m_outputs.current_state != Types::DroneState::EMERGENCY) {
            setState(Types::DroneState::EMERGENCY, now_ms, StateMachineEvent::EVENT_EMERGENCY_STOP_REQUESTED);
            m_outputs.telemetry_event_id = TE_EMERGENCY_ENTERED;
            m_outputs.telemetry_event_valid = true;
        }
        return;
    }

    // 2. LANDING (Critical Priority)
    if (m_outputs.current_state != Types::DroneState::LANDING &&
        m_outputs.current_state != Types::DroneState::DISARMED &&
        m_outputs.current_state != Types::DroneState::EMERGENCY) {

        if (inputs.safety_action == Types::SafetyAction::LAND) {
            setState(Types::DroneState::LANDING, now_ms, StateMachineEvent::EVENT_LANDING_TIMEOUT);
            return;
        }

        if (inputs.battery_critical) {
            setState(Types::DroneState::LANDING, now_ms, StateMachineEvent::EVENT_BATTERY_CRITICAL);
            m_outputs.telemetry_event_id = TE_BATTERY_CRITICAL;
            m_outputs.telemetry_event_valid = true;
            return;
        }

        if (inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_UNRECOVERABLE) {
            setState(Types::DroneState::LANDING, now_ms, StateMachineEvent::EVENT_LOCALIZATION_UNRECOVERABLE);
            m_outputs.telemetry_event_id = TE_SM_LOCALIZATION_UNRECOVERABLE;
            m_outputs.telemetry_event_valid = true;
            return;
        }

        if (inputs.command_valid && inputs.latest_command == Types::CommandType::STOP_ABORT && inputs.stop_command_allowed) {
            if (isFlightActive()) {
                setState(Types::DroneState::LANDING, now_ms, StateMachineEvent::EVENT_STOP_ABORT_COMMAND);
            } else {
                setState(Types::DroneState::DISARMED, now_ms, StateMachineEvent::EVENT_STOP_ABORT_COMMAND);
            }
            m_outputs.telemetry_event_id = TE_STOP_COMMAND_ACCEPTED;
            m_outputs.telemetry_event_valid = true;
            return;
        }
    }

    // 3. HOLD (Recoverable Fault Priority)
    if (m_outputs.current_state != Types::DroneState::HOLD &&
        m_outputs.current_state != Types::DroneState::LANDING &&
        m_outputs.current_state != Types::DroneState::DISARMED &&
        m_outputs.current_state != Types::DroneState::EMERGENCY &&
        m_outputs.current_state != Types::DroneState::INIT &&
        m_outputs.current_state != Types::DroneState::CALIBRATE &&
        m_outputs.current_state != Types::DroneState::WAIT_FOR_START) {

        if (!inputs.fc_link_healthy) {
            setState(Types::DroneState::HOLD, now_ms, StateMachineEvent::EVENT_FC_LINK_LOST);
            m_outputs.telemetry_event_id = TE_SM_FC_LINK_LOST;
            m_outputs.telemetry_event_valid = true;
            return;
        }

        if (inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_DEGRADED) {
            setState(Types::DroneState::HOLD, now_ms, StateMachineEvent::EVENT_LOCALIZATION_DEGRADED);
            m_outputs.telemetry_event_id = TE_SM_LOCALIZATION_DEGRADED;
            m_outputs.telemetry_event_valid = true;
            return;
        }

        if (inputs.command_valid && inputs.latest_command == Types::CommandType::PAUSE && inputs.pause_command_allowed) {
            m_hold_due_to_pause = true;
            setState(Types::DroneState::HOLD, now_ms, StateMachineEvent::EVENT_PAUSE_COMMAND);
            m_outputs.telemetry_event_id = TE_PAUSE_COMMAND_ACCEPTED;
            m_outputs.telemetry_event_valid = true;
            return;
        }
    }
}


// ============================================================================
// MAIN UPDATE DISPATCHER
// ============================================================================

void StateMachine::update(const StateMachineInputs& inputs, uint32_t now_ms) {
    // Check manual override for bench testing
    if (inputs.override_enabled) {
        setState(inputs.override_state, now_ms, StateMachineEvent::EVENT_NONE);
        updateOutputFlags();
        return;
    }

    // Update elapsed time in current state
    m_outputs.time_in_state_ms = now_ms - m_outputs.state_entered_ms;

    // Update mission timer
    updateMissionTimer(inputs, now_ms);

    // Apply safety overrides (EMERGENCY -> LANDING -> HOLD)
    applySafetyOverrides(inputs, now_ms);

    // Execute state-specific transition logic
    switch (m_outputs.current_state) {
        case Types::DroneState::INIT:             handleInit(inputs, now_ms); break;
        case Types::DroneState::CALIBRATE:        handleCalibrate(inputs, now_ms); break;
        case Types::DroneState::WAIT_FOR_START:   handleWaitForStart(inputs, now_ms); break;
        case Types::DroneState::TAKEOFF:          handleTakeoff(inputs, now_ms); break;
        case Types::DroneState::FORMATION:        handleFormation(inputs, now_ms); break;
        case Types::DroneState::SEARCHING:        handleSearching(inputs, now_ms); break;
        case Types::DroneState::PLANNING:         handlePlanning(inputs, now_ms); break;
        case Types::DroneState::GUIDING:          handleGuiding(inputs, now_ms); break;
        case Types::DroneState::MISSION_COMPLETE: handleMissionComplete(inputs, now_ms); break;
        case Types::DroneState::LANDING:          handleLanding(inputs, now_ms); break;
        case Types::DroneState::DISARMED:         handleDisarmed(inputs, now_ms); break;
        case Types::DroneState::HOLD:             handleHold(inputs, now_ms); break;
        case Types::DroneState::EMERGENCY:        handleEmergency(inputs, now_ms); break;
    }

    // Refresh output authority flags
    updateOutputFlags();
}


// ============================================================================
// STATE HANDLERS
// ============================================================================

void StateMachine::handleInit(const StateMachineInputs& inputs, uint32_t now_ms) {
    if (inputs.self_check_failed) {
        setState(Types::DroneState::EMERGENCY, now_ms, StateMachineEvent::EVENT_SELF_CHECK_FAILED);
        return;
    }

    if (inputs.self_check_passed) {
        setState(Types::DroneState::CALIBRATE, now_ms, StateMachineEvent::EVENT_SELF_CHECK_PASSED);
        return;
    }

    if (m_outputs.time_in_state_ms > STATE_MACHINE_CALIBRATION_TIMEOUT_MS) {
        setState(Types::DroneState::EMERGENCY, now_ms, StateMachineEvent::EVENT_SELF_CHECK_FAILED);
    }
}

void StateMachine::handleCalibrate(const StateMachineInputs& inputs, uint32_t now_ms) {
    if (inputs.calibration_failed) {
        setState(Types::DroneState::EMERGENCY, now_ms, StateMachineEvent::EVENT_CALIBRATION_FAILED);
        return;
    }

    if (inputs.calibration_complete) {
        setState(Types::DroneState::WAIT_FOR_START, now_ms, StateMachineEvent::EVENT_CALIBRATION_COMPLETE);
        return;
    }

    if (m_outputs.time_in_state_ms > STATE_MACHINE_CALIBRATION_TIMEOUT_MS) {
        setState(Types::DroneState::EMERGENCY, now_ms, StateMachineEvent::EVENT_CALIBRATION_FAILED);
    }
}

void StateMachine::handleWaitForStart(const StateMachineInputs& inputs, uint32_t now_ms) {
    if (canEnterTakeoff(inputs)) {
        // Valid START command accepted
        m_mission_start_time_ms = now_ms;
        m_mission_timer_active = true;
        setState(Types::DroneState::TAKEOFF, now_ms, StateMachineEvent::EVENT_START_COMMAND_VALID);
        m_outputs.telemetry_event_id = TE_START_COMMAND_ACCEPTED;
        m_outputs.telemetry_event_valid = true;
        return;
    }

    if (inputs.command_valid && inputs.latest_command == Types::CommandType::STOP_ABORT) {
        setState(Types::DroneState::DISARMED, now_ms, StateMachineEvent::EVENT_STOP_ABORT_COMMAND);
    }
}

void StateMachine::handleTakeoff(const StateMachineInputs& inputs, uint32_t now_ms) {
    bool near_target_altitude = (inputs.target_altitude_m > 0.0f) &&
        (std::abs(inputs.current_altitude_m - inputs.target_altitude_m) <= STATE_MACHINE_ALTITUDE_NEAR_TARGET_M);

    if ((inputs.takeoff_complete || near_target_altitude || inputs.altitude_stable) &&
        inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_GOOD) {
        setState(Types::DroneState::FORMATION, now_ms, StateMachineEvent::EVENT_TAKEOFF_COMPLETE);
        return;
    }

    if (m_outputs.time_in_state_ms > STATE_MACHINE_TAKEOFF_TIMEOUT_MS) {
        if (inputs.current_altitude_m > STATE_MACHINE_TOUCHDOWN_ALTITUDE_M) {
            setState(Types::DroneState::LANDING, now_ms, StateMachineEvent::EVENT_TAKEOFF_FAILED);
        } else {
            setState(Types::DroneState::EMERGENCY, now_ms, StateMachineEvent::EVENT_TAKEOFF_FAILED);
        }
    }
}

void StateMachine::handleFormation(const StateMachineInputs& inputs, uint32_t now_ms) {
    if (!inputs.swarm_enabled) {
        setState(Types::DroneState::SEARCHING, now_ms, StateMachineEvent::EVENT_FORMATION_READY);
        return;
    }

    if (inputs.peers_healthy && inputs.roles_confirmed) {
        setState(Types::DroneState::SEARCHING, now_ms, StateMachineEvent::EVENT_FORMATION_READY);
        return;
    }

    if (inputs.swarm_critical) {
        setState(Types::DroneState::HOLD, now_ms, StateMachineEvent::EVENT_SWARM_CRITICAL);
        return;
    }

    if (m_outputs.time_in_state_ms > STATE_MACHINE_FORMATION_TIMEOUT_MS) {
        setState(Types::DroneState::SEARCHING, now_ms, StateMachineEvent::EVENT_FORMATION_TIMEOUT);
    }
}

void StateMachine::handleSearching(const StateMachineInputs& inputs, uint32_t now_ms) {
    if (inputs.route_possible && inputs.search_coverage_sufficient) {
        setState(Types::DroneState::PLANNING, now_ms, StateMachineEvent::EVENT_ROUTE_POSSIBLE);
        return;
    }

    if (inputs.command_valid && inputs.latest_command == Types::CommandType::PAUSE && inputs.pause_command_allowed) {
        m_hold_due_to_pause = true;
        setState(Types::DroneState::HOLD, now_ms, StateMachineEvent::EVENT_PAUSE_COMMAND);
    }
}

void StateMachine::handlePlanning(const StateMachineInputs& inputs, uint32_t now_ms) {
    if (inputs.path_valid) {
        setState(Types::DroneState::GUIDING, now_ms, StateMachineEvent::EVENT_PATH_VALID);
        m_outputs.telemetry_event_id = TE_PATH_VALID_TRANSITION;
        m_outputs.telemetry_event_valid = true;
        return;
    }

    if (inputs.path_blocked_by_new_mine) {
        setState(Types::DroneState::SEARCHING, now_ms, StateMachineEvent::EVENT_NEW_MINE_BLOCKS_PATH);
        m_outputs.telemetry_event_id = TE_PATH_INVALID_TRANSITION;
        m_outputs.telemetry_event_valid = true;
        return;
    }

    if (!inputs.route_possible) {
        setState(Types::DroneState::SEARCHING, now_ms, StateMachineEvent::EVENT_ROUTE_NOT_POSSIBLE);
        return;
    }

    if (m_outputs.time_in_state_ms > STATE_MACHINE_PLANNING_TIMEOUT_MS) {
        setState(Types::DroneState::SEARCHING, now_ms, StateMachineEvent::EVENT_PATH_INVALID);
    }
}

void StateMachine::handleGuiding(const StateMachineInputs& inputs, uint32_t now_ms) {
    if (inputs.human_in_exit_zone && inputs.mission_complete_confirmed) {
        setState(Types::DroneState::MISSION_COMPLETE, now_ms, StateMachineEvent::EVENT_HUMAN_EXIT_REACHED);
        m_outputs.telemetry_event_id = TE_HUMAN_EXIT_REACHED;
        m_outputs.telemetry_event_valid = true;
        return;
    }

    if (inputs.path_blocked_by_new_mine) {
        setState(Types::DroneState::SEARCHING, now_ms, StateMachineEvent::EVENT_NEW_MINE_BLOCKS_PATH);
        return;
    }

    if (inputs.human_off_path) {
        if (inputs.route_possible) {
            setState(Types::DroneState::PLANNING, now_ms, StateMachineEvent::EVENT_HUMAN_OFF_PATH);
        } else {
            setState(Types::DroneState::SEARCHING, now_ms, StateMachineEvent::EVENT_HUMAN_OFF_PATH);
        }
        return;
    }

    if (inputs.human_tracking_lost_recoverable) {
        setState(Types::DroneState::HOLD, now_ms, StateMachineEvent::EVENT_HUMAN_TRACKING_LOST_RECOVERABLE);
        return;
    }

    if (inputs.human_tracking_lost_unrecoverable) {
        setState(Types::DroneState::SEARCHING, now_ms, StateMachineEvent::EVENT_HUMAN_TRACKING_LOST_UNRECOVERABLE);
        return;
    }

    if (!inputs.path_valid) {
        setState(Types::DroneState::SEARCHING, now_ms, StateMachineEvent::EVENT_PATH_INVALID);
    }
}

void StateMachine::handleMissionComplete(const StateMachineInputs& inputs, uint32_t now_ms) {
    m_outputs.allow_mission_completion = true;

    if (m_outputs.time_in_state_ms >= STATE_MACHINE_MISSION_COMPLETE_VALIDATION_MS && inputs.mission_complete_confirmed) {
        setState(Types::DroneState::LANDING, now_ms, StateMachineEvent::EVENT_MISSION_COMPLETE_CONFIRMED);
        return;
    }

    if (!inputs.mission_complete_confirmed && m_outputs.time_in_state_ms < STATE_MACHINE_MISSION_COMPLETE_VALIDATION_MS && inputs.path_valid) {
        setState(Types::DroneState::GUIDING, now_ms, StateMachineEvent::EVENT_PATH_VALID);
    }
}

void StateMachine::handleLanding(const StateMachineInputs& inputs, uint32_t now_ms) {
    m_outputs.request_land = true;

    bool near_ground = (inputs.current_altitude_m <= STATE_MACHINE_TOUCHDOWN_ALTITUDE_M && m_outputs.time_in_state_ms > 2000UL);

    if (inputs.touchdown_detected || near_ground) {
        setState(Types::DroneState::DISARMED, now_ms, StateMachineEvent::EVENT_TOUCHDOWN_DETECTED);
        return;
    }

    if (m_outputs.time_in_state_ms > STATE_MACHINE_LANDING_TIMEOUT_MS) {
        setState(Types::DroneState::DISARMED, now_ms, StateMachineEvent::EVENT_LANDING_TIMEOUT);
    }
}

void StateMachine::handleDisarmed(const StateMachineInputs& inputs, uint32_t now_ms) {
    (void)inputs;
    (void)now_ms;
    m_mission_timer_active = false;
    m_outputs.request_disarm = true;
}

void StateMachine::handleHold(const StateMachineInputs& inputs, uint32_t now_ms) {
    m_outputs.request_hold = true;

    if (m_hold_due_to_pause) {
        if (inputs.command_valid &&
            (inputs.latest_command == Types::CommandType::FORWARD || inputs.latest_command == Types::CommandType::START) &&
            inputs.resume_command_allowed &&
            inputs.safety_action == Types::SafetyAction::CONTINUE &&
            inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_GOOD) {
            m_hold_due_to_pause = false;
            setState(m_outputs.resume_state, now_ms, StateMachineEvent::EVENT_RESUME_COMMAND);
            m_outputs.telemetry_event_id = TE_RESUME_COMMAND_ACCEPTED;
            m_outputs.telemetry_event_valid = true;
            return;
        }
    } else {
        if (inputs.safety_action == Types::SafetyAction::CONTINUE &&
            inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_GOOD &&
            inputs.fc_link_healthy) {
            setState(m_outputs.resume_state, now_ms, StateMachineEvent::EVENT_LOCALIZATION_HEALTHY);
            m_outputs.telemetry_event_id = TE_HOLD_EXITED;
            m_outputs.telemetry_event_valid = true;
            return;
        }
    }

    if (m_outputs.time_in_state_ms > STATE_MACHINE_HOLD_RECOVERY_TIMEOUT_MS) {
        if (isFlightActive()) {
            setState(Types::DroneState::LANDING, now_ms, StateMachineEvent::EVENT_LANDING_TIMEOUT);
        } else {
            setState(Types::DroneState::DISARMED, now_ms, StateMachineEvent::EVENT_DISARM_COMPLETE);
        }
    }
}

void StateMachine::handleEmergency(const StateMachineInputs& inputs, uint32_t now_ms) {
    (void)inputs;
    (void)now_ms;
    m_outputs.request_emergency_stop = true;
    m_outputs.allow_arm = false;
    m_outputs.allow_takeoff = false;
    m_outputs.allow_horizontal_flight = false;
    m_outputs.allow_search_expansion = false;
    m_outputs.allow_guidance = false;
}


// ============================================================================
// TRANSITION GUARDS
// ============================================================================

bool StateMachine::canEnterTakeoff(const StateMachineInputs& inputs) const {
    return (inputs.latest_command == Types::CommandType::START &&
            inputs.command_valid &&
            inputs.start_command_allowed &&
            !inputs.kill_switch_active &&
            !inputs.battery_critical &&
            inputs.fc_link_healthy &&
            inputs.localization_health != Types::LocalizationHealth::LOCALIZATION_UNRECOVERABLE);
}

bool StateMachine::canEnterSearching(const StateMachineInputs& inputs) const {
    return (!inputs.kill_switch_active &&
            !inputs.battery_critical &&
            inputs.localization_health != Types::LocalizationHealth::LOCALIZATION_UNRECOVERABLE);
}

bool StateMachine::canEnterPlanning(const StateMachineInputs& inputs) const {
    return (inputs.route_possible &&
            inputs.localization_health != Types::LocalizationHealth::LOCALIZATION_UNRECOVERABLE);
}

bool StateMachine::canEnterGuiding(const StateMachineInputs& inputs) const {
    return (inputs.path_valid &&
            !inputs.path_blocked_by_new_mine &&
            inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_GOOD);
}

bool StateMachine::canEnterMissionComplete(const StateMachineInputs& inputs) const {
    return (inputs.human_in_exit_zone && inputs.mission_complete_confirmed);
}

bool StateMachine::canEnterLanding(const StateMachineInputs& inputs) const {
    (void)inputs;
    return true;
}

bool StateMachine::shouldEnterHold(const StateMachineInputs& inputs) const {
    return (!inputs.fc_link_healthy ||
            inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_DEGRADED ||
            (inputs.command_valid && inputs.latest_command == Types::CommandType::PAUSE));
}

bool StateMachine::shouldEnterEmergency(const StateMachineInputs& inputs) const {
    return (inputs.kill_switch_active ||
            inputs.safety_action == Types::SafetyAction::EMERGENCY_CUT);
}

bool StateMachine::shouldReturnFromHold(const StateMachineInputs& inputs) const {
    return (inputs.safety_action == Types::SafetyAction::CONTINUE &&
            inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_GOOD &&
            inputs.fc_link_healthy);
}


// ============================================================================
// OUTPUT FLAGS ASSIGNMENT
// ============================================================================

void StateMachine::updateOutputFlags() {
    // Reset all request and authorization flags to safe defaults
    m_outputs.allow_arm = false;
    m_outputs.allow_takeoff = false;
    m_outputs.allow_horizontal_flight = false;
    m_outputs.allow_search_expansion = false;
    m_outputs.allow_guidance = false;
    m_outputs.allow_mission_completion = false;

    m_outputs.request_hold = false;
    m_outputs.request_takeoff = false;
    m_outputs.request_land = false;
    m_outputs.request_disarm = false;
    m_outputs.request_emergency_stop = false;
    m_outputs.request_mission_abort = false;

    switch (m_outputs.current_state) {
        case Types::DroneState::INIT:
            m_outputs.allow_arm = false;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = false;
            break;

        case Types::DroneState::CALIBRATE:
            m_outputs.allow_arm = false;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = false;
            break;

        case Types::DroneState::WAIT_FOR_START:
            m_outputs.allow_arm = true;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = false;
            break;

        case Types::DroneState::TAKEOFF:
            m_outputs.allow_arm = true;
            m_outputs.allow_takeoff = true;
            m_outputs.allow_horizontal_flight = false;
            m_outputs.request_takeoff = true;
            break;

        case Types::DroneState::FORMATION:
            m_outputs.allow_arm = true;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = true;
            m_outputs.allow_search_expansion = false;
            break;

        case Types::DroneState::SEARCHING:
            m_outputs.allow_arm = true;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = true;
            m_outputs.allow_search_expansion = true;
            break;

        case Types::DroneState::PLANNING:
            m_outputs.allow_arm = true;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = true;
            m_outputs.allow_search_expansion = false;
            break;

        case Types::DroneState::GUIDING:
            m_outputs.allow_arm = true;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = true;
            m_outputs.allow_guidance = true;
            break;

        case Types::DroneState::MISSION_COMPLETE:
            m_outputs.allow_arm = true;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = false;
            m_outputs.allow_guidance = true;
            m_outputs.allow_mission_completion = true;
            break;

        case Types::DroneState::LANDING:
            m_outputs.allow_arm = true;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = false;
            m_outputs.request_land = true;
            break;

        case Types::DroneState::DISARMED:
            m_outputs.allow_arm = false;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = false;
            m_outputs.request_disarm = true;
            break;

        case Types::DroneState::HOLD:
            m_outputs.allow_arm = true;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = false;
            m_outputs.request_hold = true;
            break;

        case Types::DroneState::EMERGENCY:
            m_outputs.allow_arm = false;
            m_outputs.allow_takeoff = false;
            m_outputs.allow_horizontal_flight = false;
            m_outputs.allow_search_expansion = false;
            m_outputs.allow_guidance = false;
            m_outputs.request_emergency_stop = true;
            break;
    }
}


// ============================================================================
// MODULE FACADE IMPLEMENTATION
// ============================================================================

void state_machine_init() {
    s_global_state_machine.init();
}

void state_machine_update(uint32_t now_ms) {
    StateMachineInputs inputs;
    inputs.now_ms = now_ms;
    inputs.self_check_passed = true;
    inputs.calibration_complete = true;
    inputs.fc_link_healthy = true;
    inputs.localization_health = Types::LocalizationHealth::LOCALIZATION_GOOD;

    s_global_state_machine.update(inputs, now_ms);
}

Types::DroneState state_machine_get_state() {
    return s_global_state_machine.getState();
}

StateMachineOutputs state_machine_get_outputs() {
    return s_global_state_machine.getOutputs();
}

StateMachine& state_machine_get_instance() {
    return s_global_state_machine;
}

} // namespace RobofestDrone
