#include "telemetry.h"
#include "../hal/hal_system.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static Telemetry s_global_telemetry;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

Telemetry::Telemetry() {
    reset();
}

void Telemetry::init() {
    reset();
    Hal::hal_storage_init();
    storage_healthy_ = Hal::hal_storage_is_healthy();
    if (storage_healthy_) {
        setTelemetryEvent(TE_TELEMETRY_STORAGE_HEALTHY);
    } else {
        setTelemetryEvent(TE_TELEMETRY_STORAGE_UNHEALTHY);
    }
    setTelemetryEvent(TE_TELEMETRY_INITIALIZED);
}

void Telemetry::reset() {
    event_head_ = 0;
    event_count_ = 0;
    buffer_full_ = false;
    std::memset(events_, 0, sizeof(events_));

    summary_ = Types::MissionSummary();

    last_periodic_ms_ = 0;
    last_drift_log_ms_ = 0;
    last_summary_log_ms_ = 0;
    last_storage_retry_ms_ = 0;

    events_this_second_ = 0;
    current_second_ms_ = 0;

    storage_healthy_ = true;
    flush_active_ = false;
    mission_active_ = false;

    previous_state_ = Types::DroneState::INIT;

    last_telemetry_event_id_ = TE_TELEMETRY_INITIALIZED;
    telemetry_event_valid_ = true;
}

void Telemetry::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}


// ============================================================================
// EVENT LOGGING & BUFFER STORAGE
// ============================================================================

void Telemetry::appendEventToBuffer(const Types::TelemetryEvent& event) {
    events_[event_head_] = event;
    event_head_ = (event_head_ + 1) % Config::MAX_TELEMETRY_EVENTS;

    if (event_count_ < Config::MAX_TELEMETRY_EVENTS) {
        event_count_++;
    } else {
        buffer_full_ = true;
    }

    if (event.severity == Types::TELEMETRY_SEVERITY_CRITICAL && Config::TELEMETRY_PERSIST_CRITICAL_EVENTS) {
        if (storage_healthy_) {
            Hal::hal_storage_write_event(event);
        }
    }
}

void Telemetry::logEvent(
    uint16_t event_id,
    uint8_t severity,
    uint8_t module_id,
    float value_a,
    float value_b,
    uint16_t context_id
) {
    uint32_t now = Hal::hal_millis();

    // Reset rate-limit counter every 1000 ms
    if (now - current_second_ms_ >= 1000UL) {
        current_second_ms_ = now;
        events_this_second_ = 0;
    }

    events_this_second_++;

    // Apply rate limiting
    if (events_this_second_ > Config::TELEMETRY_MAX_EVENTS_PER_SECOND) {
        if (severity < Types::TELEMETRY_SEVERITY_WARNING) {
            return; // Drop low-priority events during flood
        }
    }

    Types::TelemetryEvent ev;
    ev.timestamp_ms = now;
    ev.event_id = event_id;
    ev.severity = severity;
    ev.module_id = module_id;
    ev.value_a = value_a;
    ev.value_b = value_b;
    ev.context_id = context_id;

    appendEventToBuffer(ev);
    setTelemetryEvent(event_id);
}

void Telemetry::logInfo(uint16_t event_id, uint8_t module_id, float value_a, float value_b, uint16_t context_id) {
    logEvent(event_id, Types::TELEMETRY_SEVERITY_INFO, module_id, value_a, value_b, context_id);
}

void Telemetry::logWarning(uint16_t event_id, uint8_t module_id, float value_a, float value_b, uint16_t context_id) {
    logEvent(event_id, Types::TELEMETRY_SEVERITY_WARNING, module_id, value_a, value_b, context_id);
}

void Telemetry::logCritical(uint16_t event_id, uint8_t module_id, float value_a, float value_b, uint16_t context_id) {
    logEvent(event_id, Types::TELEMETRY_SEVERITY_CRITICAL, module_id, value_a, value_b, context_id);
}


// ============================================================================
// SPECIALIZED LOGGING METHODS
// ============================================================================

void Telemetry::logStateTransition(Types::DroneState previous_state, Types::DroneState new_state, uint32_t now_ms) {
    (void)now_ms;
    summary_.state_transition_count++;

    if (new_state == Types::DroneState::TAKEOFF) {
        summary_.takeoff_started = true;
    } else if (new_state == Types::DroneState::FORMATION || new_state == Types::DroneState::SEARCHING) {
        summary_.takeoff_completed = true;
    } else if (new_state == Types::DroneState::LANDING) {
        summary_.landing_started = true;
        if (Config::TELEMETRY_FLUSH_BEFORE_LANDING) {
            flushNow();
        }
    } else if (new_state == Types::DroneState::DISARMED && summary_.landing_started) {
        summary_.landing_completed = true;
    } else if (new_state == Types::DroneState::EMERGENCY) {
        summary_.emergency_count++;
        if (Config::TELEMETRY_FLUSH_BEFORE_LANDING) {
            flushNow();
        }
    }

    logInfo(TE_STATE_TRANSITION,
            Types::TELEMETRY_MODULE_STATE_MACHINE,
            static_cast<float>(previous_state),
            static_cast<float>(new_state),
            summary_.state_transition_count);
}

void Telemetry::logMineEvent(uint16_t mine_id, uint8_t status, float confidence, uint32_t now_ms) {
    (void)now_ms;
    if (status == 0) { // Candidate
        summary_.mine_candidate_count++;
        logInfo(TE_MINE_CANDIDATE_ADDED, Types::TELEMETRY_MODULE_MINE_MAP, confidence, 0.0f, mine_id);
    } else if (status == 1) { // Confirmed
        summary_.mine_confirmed_count++;
        logInfo(TE_MINE_CONFIRMED, Types::TELEMETRY_MODULE_MINE_MAP, confidence, 1.0f, mine_id);
    } else { // Rejected / Stale
        summary_.mine_rejected_count++;
        logInfo(TE_MINE_STALE_DECAYED, Types::TELEMETRY_MODULE_MINE_MAP, confidence, 2.0f, mine_id);
    }
}

void Telemetry::logDetectionEvent(float world_x, float world_y, float confidence, uint32_t now_ms) {
    (void)now_ms;
    logInfo(TE_CANDIDATE_EMITTED, Types::TELEMETRY_MODULE_VISION, world_x, world_y, static_cast<uint16_t>(confidence * 100.0f));
}

void Telemetry::logClaimEvent(uint16_t mine_hash, uint8_t owner_drone_id, bool accepted, uint32_t now_ms) {
    (void)now_ms;
    if (accepted) {
        summary_.claim_sent_count++;
    } else {
        summary_.claim_received_count++;
    }
    logInfo(TE_SWARM_CLAIM_SENT, Types::TELEMETRY_MODULE_SWARM, static_cast<float>(owner_drone_id), accepted ? 1.0f : 0.0f, mine_hash);
}

void Telemetry::logYieldEvent(uint16_t mine_hash, uint8_t reason, uint32_t now_ms) {
    (void)now_ms;
    summary_.yield_count++;
    logInfo(TE_SWARM_CLAIM_YIELDED, Types::TELEMETRY_MODULE_SWARM, static_cast<float>(reason), 0.0f, mine_hash);
}

void Telemetry::logPathEvent(uint32_t path_version, bool path_valid, uint8_t reason, uint32_t now_ms) {
    (void)now_ms;
    if (path_valid) {
        summary_.path_plan_count++;
        logInfo(TE_PLANNER_PATH_FOUND, Types::TELEMETRY_MODULE_PATH_PLANNER, static_cast<float>(path_version), 1.0f, reason);
    } else {
        summary_.path_invalid_count++;
        summary_.reroute_count++;
        logWarning(TE_PLANNER_PATH_FAILED, Types::TELEMETRY_MODULE_PATH_PLANNER, static_cast<float>(path_version), 0.0f, reason);
    }
}

void Telemetry::logSafetyEvent(uint16_t fault_code, Types::SafetyAction action, uint32_t now_ms) {
    (void)now_ms;
    if (action == Types::SafetyAction::HOLD) {
        summary_.safety_hold_count++;
        logWarning(TE_SAFETY_ACTION_HOLD, Types::TELEMETRY_MODULE_SAFETY, static_cast<float>(action), 0.0f, fault_code);
    } else if (action == Types::SafetyAction::LAND) {
        summary_.safety_land_count++;
        logCritical(TE_SAFETY_ACTION_LAND, Types::TELEMETRY_MODULE_SAFETY, static_cast<float>(action), 0.0f, fault_code);
    } else if (action == Types::SafetyAction::EMERGENCY_CUT) {
        summary_.emergency_count++;
        logCritical(TE_SAFETY_ACTION_EMERGENCY_CUT, Types::TELEMETRY_MODULE_SAFETY, static_cast<float>(action), 0.0f, fault_code);
    }
}

void Telemetry::logCommandEvent(uint8_t command, float confidence, bool accepted, uint32_t now_ms) {
    (void)now_ms;
    if (accepted) {
        summary_.command_accepted_count++;
        logInfo(TE_COMMAND_START_ACCEPTED + command, Types::TELEMETRY_MODULE_COMMAND, confidence, 1.0f, command);
    } else {
        summary_.command_rejected_count++;
        logWarning(TE_COMMAND_REJECTED_CONFIDENCE, Types::TELEMETRY_MODULE_COMMAND, confidence, 0.0f, command);
    }
}

void Telemetry::logLocalizationEvent(Types::LocalizationHealth health, float drift_uncertainty_m, uint32_t now_ms) {
    (void)now_ms;
    summary_.final_drift_uncertainty_m = drift_uncertainty_m;
    if (drift_uncertainty_m > summary_.max_drift_uncertainty_m) {
        summary_.max_drift_uncertainty_m = drift_uncertainty_m;
    }

    if (health == Types::LocalizationHealth::LOCALIZATION_UNRECOVERABLE) {
        logCritical(TE_LOCALIZATION_UNRECOVERABLE, Types::TELEMETRY_MODULE_LOCALIZATION, drift_uncertainty_m, 0.0f, static_cast<uint16_t>(health));
    } else if (health == Types::LocalizationHealth::LOCALIZATION_DEGRADED) {
        logWarning(TE_LOCALIZATION_DEGRADED, Types::TELEMETRY_MODULE_LOCALIZATION, drift_uncertainty_m, 0.0f, static_cast<uint16_t>(health));
    }
}

void Telemetry::logSwarmEvent(uint8_t peer_id, uint8_t swarm_event_code, uint32_t now_ms) {
    (void)now_ms;
    if (swarm_event_code == 1) { // Peer timed out / lost
        summary_.peer_lost_count++;
        logWarning(TE_SWARM_PEER_TIMEOUT, Types::TELEMETRY_MODULE_SWARM, static_cast<float>(peer_id), 0.0f, swarm_event_code);
    } else if (swarm_event_code == 2) { // Role failover
        logInfo(TE_SWARM_ROLE_FAILOVER, Types::TELEMETRY_MODULE_SWARM, static_cast<float>(peer_id), 0.0f, swarm_event_code);
    }
}

void Telemetry::logHumanEvent(bool detected, bool off_path, bool in_exit_zone, uint32_t now_ms) {
    (void)now_ms;
    if (off_path) {
        summary_.human_off_path_count++;
        logWarning(TE_HUMAN_OFF_PATH_DETECTED, Types::TELEMETRY_MODULE_HUMAN_TRACKER, 1.0f, 0.0f, 0);
    }
    if (in_exit_zone) {
        summary_.human_exit_confirmed = true;
        logInfo(TE_HUMAN_EXIT_ZONE_CONFIRMED, Types::TELEMETRY_MODULE_HUMAN_TRACKER, 1.0f, 1.0f, 0);
    }
    if (detected) {
        logInfo(TE_HUMAN_DETECTED, Types::TELEMETRY_MODULE_HUMAN_TRACKER, 1.0f, 0.0f, 0);
    }
}

void Telemetry::logMarkerEvent(uint8_t marker_pattern, uint32_t now_ms) {
    (void)now_ms;
    logInfo(TE_MARKER_PATTERN_OFF + marker_pattern, Types::TELEMETRY_MODULE_MARKER, static_cast<float>(marker_pattern), 0.0f, 0);
}

void Telemetry::logSearchEvent(float coverage_percent, bool needs_more_scan, uint32_t now_ms) {
    (void)now_ms;
    logInfo(TE_SEARCH_COVERAGE_UPDATED, Types::TELEMETRY_MODULE_SEARCH, coverage_percent, needs_more_scan ? 1.0f : 0.0f, 0);
}


// ============================================================================
// MISSION TIMING & FLUSH
// ============================================================================

void Telemetry::startMission(uint32_t now_ms) {
    summary_.mission_started = true;
    summary_.mission_start_ms = now_ms;
    summary_.mission_end_ms = 0;
    summary_.mission_elapsed_ms = 0;
    mission_active_ = true;
}

void Telemetry::endMission(uint32_t now_ms) {
    summary_.mission_end_ms = now_ms;
    if (summary_.mission_start_ms > 0 && now_ms >= summary_.mission_start_ms) {
        summary_.mission_elapsed_ms = now_ms - summary_.mission_start_ms;
    }
    if (!summary_.mission_timeout && summary_.emergency_count == 0 && summary_.human_exit_confirmed) {
        summary_.mission_completed = true;
    }
    mission_active_ = false;

    // Log compact mission summary event
    logInfo(TE_TELEMETRY_MISSION_SUMMARY,
            Types::TELEMETRY_MODULE_TELEMETRY,
            static_cast<float>(summary_.mission_elapsed_ms) / 1000.0f,
            summary_.final_drift_uncertainty_m,
            summary_.mine_confirmed_count);

    flushNow();
}

void Telemetry::flushNow() {
    if (storage_healthy_) {
        setTelemetryEvent(TE_TELEMETRY_FLUSH_STARTED);
        Hal::hal_storage_flush();
        setTelemetryEvent(TE_TELEMETRY_FLUSH_COMPLETE);
    }
}

bool Telemetry::getLatestEvent(Types::TelemetryEvent& out_event) const {
    if (event_count_ == 0) return false;
    uint16_t latest_idx = (event_head_ == 0) ? (Config::MAX_TELEMETRY_EVENTS - 1) : (event_head_ - 1);
    out_event = events_[latest_idx];
    return true;
}


// ============================================================================
// MAIN UPDATE (50 Hz NON-BLOCKING)
// ============================================================================

void Telemetry::update(const Types::TelemetryInputs& inputs) {
    uint32_t now = inputs.now_ms;

    // 1. Mission Elapsed Time Update
    if (mission_active_ && summary_.mission_start_ms > 0) {
        if (now >= summary_.mission_start_ms) {
            summary_.mission_elapsed_ms = now - summary_.mission_start_ms;
            if (summary_.mission_elapsed_ms >= Config::MISSION_TIME_LIMIT_MS) {
                summary_.mission_timeout = true;
            }
        }
    }

    // 2. Battery Minimum Tracking
    if (inputs.battery_voltage > 0.0f) {
        summary_.final_battery_voltage = inputs.battery_voltage;
        if (inputs.battery_voltage < summary_.min_battery_voltage) {
            summary_.min_battery_voltage = inputs.battery_voltage;
        }
    }

    // 3. State Transition Detection
    if (inputs.drone_state != previous_state_) {
        logStateTransition(previous_state_, inputs.drone_state, now);
        previous_state_ = inputs.drone_state;
    }

    // 4. Periodic Drift Logging (every 2000 ms)
    if (now - last_drift_log_ms_ >= Config::TELEMETRY_DRIFT_LOG_INTERVAL_MS) {
        last_drift_log_ms_ = now;
        logLocalizationEvent(inputs.localization_health, inputs.drift_uncertainty_m, now);
    }

    // 5. Periodic Summary Logging (every 5000 ms)
    if (now - last_summary_log_ms_ >= Config::TELEMETRY_SUMMARY_LOG_INTERVAL_MS) {
        last_summary_log_ms_ = now;
        logInfo(TE_TELEMETRY_PERIODIC_SUMMARY,
                Types::TELEMETRY_MODULE_TELEMETRY,
                static_cast<float>(summary_.mission_elapsed_ms) / 1000.0f,
                inputs.battery_voltage,
                inputs.confirmed_mine_count);
    }

    // 6. Storage Health Check
    storage_healthy_ = inputs.storage_healthy && Hal::hal_storage_is_healthy();
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void telemetry_init() {
    s_global_telemetry.init();
}

void telemetry_update(uint32_t now_ms) {
    Types::TelemetryInputs inputs;
    inputs.now_ms = now_ms;
    s_global_telemetry.update(inputs);
}

void telemetry_log_event(uint16_t event_id, float value) {
    s_global_telemetry.logInfo(event_id, Types::TELEMETRY_MODULE_SYSTEM, value, 0.0f, 0);
}

void telemetry_log_event(
    uint16_t event_id,
    uint8_t severity,
    uint8_t module_id,
    float value_a,
    float value_b,
    uint16_t context_id
) {
    s_global_telemetry.logEvent(event_id, severity, module_id, value_a, value_b, context_id);
}

Telemetry& telemetry_get_instance() {
    return s_global_telemetry;
}

} // namespace RobofestDrone
