#include "safety_manager.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static SafetyManager s_global_safety_manager;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

SafetyManager::SafetyManager() {
    reset();
}

void SafetyManager::init() {
    reset();
}

void SafetyManager::reset() {
    recommended_action_ = Types::SafetyAction::CONTINUE;
    primary_reason_ = 0;
    active_fault_mask_ = 0;

    battery_low_latched_ = false;
    battery_critical_latched_ = false;
    kill_switch_latched_ = false;
    mission_timeout_latched_ = false;
    emergency_latched_ = false;

    battery_low_first_seen_ms_ = 0;
    battery_critical_first_seen_ms_ = 0;
    fc_link_lost_first_seen_ms_ = 0;
    camera_stall_first_seen_ms_ = 0;
    radio_timeout_first_seen_ms_ = 0;
    swarm_critical_first_seen_ms_ = 0;
    localization_degraded_first_seen_ms_ = 0;
    localization_unrecoverable_first_seen_ms_ = 0;
    geofence_outside_first_seen_ms_ = 0;
    collision_risk_first_seen_ms_ = 0;
    unsafe_proximity_first_seen_ms_ = 0;
    surface_contact_first_seen_ms_ = 0;

    last_kill_switch_ms_ = 0;
    last_battery_ok_ms_ = 0;
    last_fc_link_ok_ms_ = 0;
    last_camera_ok_ms_ = 0;
    last_radio_ok_ms_ = 0;
    last_localization_ok_ms_ = 0;

    last_telemetry_event_id_ = TE_SAFETY_INITIALIZED;
    telemetry_event_valid_ = true;
}

void SafetyManager::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}

void SafetyManager::setFault(Types::SafetyFault fault) {
    active_fault_mask_ |= (1UL << static_cast<uint8_t>(fault));
}

void SafetyManager::clearFault(Types::SafetyFault fault) {
    active_fault_mask_ &= ~(1UL << static_cast<uint8_t>(fault));
}

bool SafetyManager::isFaultActive(Types::SafetyFault fault) const {
    return (active_fault_mask_ & (1UL << static_cast<uint8_t>(fault))) != 0;
}


// ============================================================================
// FAULT STATUS QUERIES
// ============================================================================

bool SafetyManager::isCollisionRiskHigh() const {
    return isFaultActive(Types::SafetyFault::SAFETY_FAULT_COLLISION_RISK);
}

bool SafetyManager::isUnsafeProximityActive() const {
    return isFaultActive(Types::SafetyFault::SAFETY_FAULT_UNSAFE_PROXIMITY_HUMAN) ||
           isFaultActive(Types::SafetyFault::SAFETY_FAULT_UNSAFE_PROXIMITY_PEER);
}

bool SafetyManager::isSurfaceContactFault() const {
    return isFaultActive(Types::SafetyFault::SAFETY_FAULT_SURFACE_CONTACT);
}

bool SafetyManager::shouldStopSearchExpansion() const {
    return battery_low_latched_ ||
           isFaultActive(Types::SafetyFault::SAFETY_FAULT_SWARM_DEGRADED) ||
           isFaultActive(Types::SafetyFault::SAFETY_FAULT_LOCALIZATION_DEGRADED) ||
           isFaultActive(Types::SafetyFault::SAFETY_FAULT_RADIO_TIMEOUT);
}

bool SafetyManager::shouldAllowTakeoff() const {
    if (kill_switch_latched_ || emergency_latched_) return false;
    if (battery_critical_latched_) return false;
    if (isFaultActive(Types::SafetyFault::SAFETY_FAULT_FC_LINK_LOST)) return false;
    if (isFaultActive(Types::SafetyFault::SAFETY_FAULT_LOCALIZATION_UNRECOVERABLE)) return false;
    if (mission_timeout_latched_) return false;
    if (isFaultActive(Types::SafetyFault::SAFETY_FAULT_GEOFENCE_OUTSIDE)) return false;
    if (isFaultActive(Types::SafetyFault::SAFETY_FAULT_SURFACE_CONTACT)) return false;
    return true;
}

bool SafetyManager::shouldAllowGuidance() const {
    if (recommended_action_ == Types::SafetyAction::EMERGENCY_CUT ||
        recommended_action_ == Types::SafetyAction::LAND) {
        return false;
    }
    if (isFaultActive(Types::SafetyFault::SAFETY_FAULT_LOCALIZATION_UNRECOVERABLE)) return false;
    if (isFaultActive(Types::SafetyFault::SAFETY_FAULT_COLLISION_RISK)) return false;
    if (isFaultActive(Types::SafetyFault::SAFETY_FAULT_UNSAFE_PROXIMITY_HUMAN)) return false;
    return true;
}

void SafetyManager::acknowledgeEmergency() {
    emergency_latched_ = false;
    setTelemetryEvent(TE_SAFETY_EMERGENCY_ACKNOWLEDGED);
}

void SafetyManager::clearLatchedFaults() {
    if (!kill_switch_latched_) {
        emergency_latched_ = false;
    }
    battery_low_latched_ = false;
    battery_critical_latched_ = false;
    setTelemetryEvent(TE_SAFETY_FAULT_CLEARED);
}


// ============================================================================
// MAIN UPDATE (50 Hz NON-BLOCKING)
// ============================================================================

void SafetyManager::update(const Types::SafetyInputs& inputs) {
    uint32_t now = inputs.now_ms;

    // Track request actions
    bool req_emergency = false;
    bool req_land = false;
    bool req_hold = false;

    // ------------------------------------------------------------------------
    // 1. HARDWARE KILL SWITCH (HIGHEST PRIORITY IMMEDIATE RESPONSE)
    // ------------------------------------------------------------------------
    if (inputs.kill_switch_active) {
        kill_switch_latched_ = true;
        emergency_latched_ = true;
        setFault(Types::SafetyFault::SAFETY_FAULT_KILL_SWITCH);
        req_emergency = true;
        primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_KILL_SWITCH);
        setTelemetryEvent(TE_SAFETY_KILL_SWITCH_ACTIVE);
    } else {
        if (!Config::EMERGENCY_LATCH_UNTIL_RESET) {
            clearFault(Types::SafetyFault::SAFETY_FAULT_KILL_SWITCH);
            kill_switch_latched_ = false;
        }
    }

    if (emergency_latched_) {
        req_emergency = true;
    }

    // ------------------------------------------------------------------------
    // 2. BATTERY MONITORING
    // ------------------------------------------------------------------------
    if (inputs.battery_present && !std::isnan(inputs.battery_voltage)) {
        if (inputs.battery_voltage <= Config::BATTERY_CRITICAL_VOLTAGE) {
            if (battery_critical_first_seen_ms_ == 0) battery_critical_first_seen_ms_ = now;
            if ((now - battery_critical_first_seen_ms_) >= Config::BATTERY_FAULT_DEBOUNCE_MS) {
                battery_critical_latched_ = true;
                setFault(Types::SafetyFault::SAFETY_FAULT_BATTERY_CRITICAL);
                req_land = true;
                primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_BATTERY_CRITICAL);
                setTelemetryEvent(TE_SAFETY_BATTERY_CRITICAL);
            }
        } else {
            battery_critical_first_seen_ms_ = 0;
            if (inputs.battery_voltage > (Config::BATTERY_CRITICAL_VOLTAGE + Config::BATTERY_HYSTERESIS_VOLTAGE)) {
                clearFault(Types::SafetyFault::SAFETY_FAULT_BATTERY_CRITICAL);
                battery_critical_latched_ = false;
            }
        }

        if (inputs.battery_voltage <= Config::BATTERY_LOW_VOLTAGE) {
            if (battery_low_first_seen_ms_ == 0) battery_low_first_seen_ms_ = now;
            if ((now - battery_low_first_seen_ms_) >= Config::BATTERY_FAULT_DEBOUNCE_MS) {
                battery_low_latched_ = true;
                setFault(Types::SafetyFault::SAFETY_FAULT_BATTERY_LOW);
                setTelemetryEvent(TE_SAFETY_BATTERY_LOW);
            }
        } else {
            battery_low_first_seen_ms_ = 0;
            if (inputs.battery_voltage > (Config::BATTERY_LOW_VOLTAGE + Config::BATTERY_HYSTERESIS_VOLTAGE)) {
                clearFault(Types::SafetyFault::SAFETY_FAULT_BATTERY_LOW);
                battery_low_latched_ = false;
            }
        }
    }

    if (battery_critical_latched_) {
        req_land = true;
    }

    // ------------------------------------------------------------------------
    // 3. FLIGHT CONTROLLER LINK WATCHDOG
    // ------------------------------------------------------------------------
    if (!inputs.fc_link_healthy || (now >= inputs.fc_last_rx_ms && (now - inputs.fc_last_rx_ms > Config::FC_LINK_TIMEOUT_MS))) {
        if (fc_link_lost_first_seen_ms_ == 0) fc_link_lost_first_seen_ms_ = now;
        uint32_t elapsed = now - fc_link_lost_first_seen_ms_;

        setFault(Types::SafetyFault::SAFETY_FAULT_FC_LINK_LOST);
        setTelemetryEvent(TE_SAFETY_FC_LINK_LOST);

        if (elapsed >= Config::FC_LINK_LOST_EMERGENCY_MS) {
            req_emergency = true;
            primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_FC_LINK_LOST);
        } else if (elapsed >= Config::FC_LINK_LOST_LAND_MS) {
            req_land = true;
            primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_FC_LINK_LOST);
        } else {
            req_hold = true;
        }
    } else {
        fc_link_lost_first_seen_ms_ = 0;
        clearFault(Types::SafetyFault::SAFETY_FAULT_FC_LINK_LOST);
    }

    // ------------------------------------------------------------------------
    // 4. CAMERA STALL WATCHDOG
    // ------------------------------------------------------------------------
    if (!inputs.camera_healthy || (now >= inputs.camera_last_frame_ms && (now - inputs.camera_last_frame_ms > Config::CAMERA_STALL_TIMEOUT_MS))) {
        if (camera_stall_first_seen_ms_ == 0) camera_stall_first_seen_ms_ = now;
        uint32_t elapsed = now - camera_stall_first_seen_ms_;

        setFault(Types::SafetyFault::SAFETY_FAULT_CAMERA_STALL);
        setTelemetryEvent(TE_SAFETY_CAMERA_STALL);

        if (elapsed >= Config::CAMERA_STALL_LAND_MS) {
            req_land = true;
            primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_CAMERA_STALL);
        } else if (elapsed >= Config::CAMERA_STALL_HOLD_MS) {
            if (inputs.drone_state == Types::DroneState::SEARCHING || inputs.drone_state == Types::DroneState::GUIDING) {
                req_hold = true;
            }
        }
    } else {
        camera_stall_first_seen_ms_ = 0;
        clearFault(Types::SafetyFault::SAFETY_FAULT_CAMERA_STALL);
    }

    // ------------------------------------------------------------------------
    // 5. SENSOR HEALTH (OPTICAL FLOW & TOF)
    // ------------------------------------------------------------------------
    if (!inputs.optical_flow_healthy) {
        setFault(Types::SafetyFault::SAFETY_FAULT_OPTICAL_FLOW_FAILURE);
        setTelemetryEvent(TE_SAFETY_OPTICAL_FLOW_FAILURE);
        if (inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_DEGRADED) {
            req_hold = true;
        }
    } else {
        clearFault(Types::SafetyFault::SAFETY_FAULT_OPTICAL_FLOW_FAILURE);
    }

    if (!inputs.tof_healthy) {
        setFault(Types::SafetyFault::SAFETY_FAULT_TOF_FAILURE);
        setTelemetryEvent(TE_SAFETY_TOF_FAILURE);
        if (!inputs.altitude_valid) {
            req_land = true;
        } else {
            req_hold = true;
        }
    } else {
        clearFault(Types::SafetyFault::SAFETY_FAULT_TOF_FAILURE);
    }

    // ------------------------------------------------------------------------
    // 6. RADIO & SWARM STATUS
    // ------------------------------------------------------------------------
    if (!inputs.radio_healthy || (now >= inputs.radio_last_rx_ms && (now - inputs.radio_last_rx_ms > Config::RADIO_TIMEOUT_MS))) {
        if (radio_timeout_first_seen_ms_ == 0) radio_timeout_first_seen_ms_ = now;
        setFault(Types::SafetyFault::SAFETY_FAULT_RADIO_TIMEOUT);
        setTelemetryEvent(TE_SAFETY_RADIO_TIMEOUT);
    } else {
        radio_timeout_first_seen_ms_ = 0;
        clearFault(Types::SafetyFault::SAFETY_FAULT_RADIO_TIMEOUT);
    }

    if (inputs.swarm_degraded) {
        setFault(Types::SafetyFault::SAFETY_FAULT_SWARM_DEGRADED);
        setTelemetryEvent(TE_SAFETY_SWARM_DEGRADED);
    } else {
        clearFault(Types::SafetyFault::SAFETY_FAULT_SWARM_DEGRADED);
    }

    if (inputs.swarm_critical) {
        if (swarm_critical_first_seen_ms_ == 0) swarm_critical_first_seen_ms_ = now;
        uint32_t elapsed = now - swarm_critical_first_seen_ms_;

        setFault(Types::SafetyFault::SAFETY_FAULT_SWARM_CRITICAL);
        setTelemetryEvent(TE_SAFETY_SWARM_CRITICAL);

        if (elapsed >= Config::SWARM_CRITICAL_HOLD_MS) {
            req_hold = true;
        }
    } else {
        swarm_critical_first_seen_ms_ = 0;
        clearFault(Types::SafetyFault::SAFETY_FAULT_SWARM_CRITICAL);
    }

    // ------------------------------------------------------------------------
    // 7. LOCALIZATION HEALTH & DRIFT UNCERTAINTY
    // ------------------------------------------------------------------------
    if (inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_UNRECOVERABLE ||
        (!std::isnan(inputs.drift_uncertainty_m) && inputs.drift_uncertainty_m >= Config::LOCALIZATION_UNRECOVERABLE_UNCERTAINTY_M)) {
        if (localization_unrecoverable_first_seen_ms_ == 0) localization_unrecoverable_first_seen_ms_ = now;
        if ((now - localization_unrecoverable_first_seen_ms_) >= Config::LOCALIZATION_FAULT_DEBOUNCE_MS) {
            setFault(Types::SafetyFault::SAFETY_FAULT_LOCALIZATION_UNRECOVERABLE);
            req_land = true;
            primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_LOCALIZATION_UNRECOVERABLE);
            setTelemetryEvent(TE_SAFETY_LOCALIZATION_UNRECOVERABLE);
        }
    } else {
        localization_unrecoverable_first_seen_ms_ = 0;
        clearFault(Types::SafetyFault::SAFETY_FAULT_LOCALIZATION_UNRECOVERABLE);
    }

    if (inputs.localization_health == Types::LocalizationHealth::LOCALIZATION_DEGRADED ||
        (!std::isnan(inputs.drift_uncertainty_m) && inputs.drift_uncertainty_m >= Config::LOCALIZATION_DEGRADED_UNCERTAINTY_M)) {
        if (localization_degraded_first_seen_ms_ == 0) localization_degraded_first_seen_ms_ = now;
        if ((now - localization_degraded_first_seen_ms_) >= Config::LOCALIZATION_FAULT_DEBOUNCE_MS) {
            setFault(Types::SafetyFault::SAFETY_FAULT_LOCALIZATION_DEGRADED);
            req_hold = true;
            if (!req_land) primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_LOCALIZATION_DEGRADED);
            setTelemetryEvent(TE_SAFETY_LOCALIZATION_DEGRADED);
        }
    } else {
        localization_degraded_first_seen_ms_ = 0;
        clearFault(Types::SafetyFault::SAFETY_FAULT_LOCALIZATION_DEGRADED);
    }

    // ------------------------------------------------------------------------
    // 8. MISSION TIMEOUT (10 MINUTES STRICT LIMIT)
    // ------------------------------------------------------------------------
    if (inputs.mission_timer_running && inputs.mission_elapsed_ms >= Config::MISSION_TIME_LIMIT_MS) {
        mission_timeout_latched_ = true;
        setFault(Types::SafetyFault::SAFETY_FAULT_MISSION_TIMEOUT);
        req_land = true;
        primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_MISSION_TIMEOUT);
        setTelemetryEvent(TE_SAFETY_MISSION_TIMEOUT);
    }

    if (mission_timeout_latched_) {
        req_land = true;
    }

    // ------------------------------------------------------------------------
    // 9. SOFTWARE GEOFENCE
    // ------------------------------------------------------------------------
    if (inputs.geofence_status == Types::GeofenceStatus::GEOFENCE_WARNING) {
        setFault(Types::SafetyFault::SAFETY_FAULT_GEOFENCE_WARNING);
        setTelemetryEvent(TE_SAFETY_GEOFENCE_WARNING);
        if (Config::GEOFENCE_WARNING_HOLD_ENABLED) req_hold = true;
    } else {
        clearFault(Types::SafetyFault::SAFETY_FAULT_GEOFENCE_WARNING);
    }

    if (inputs.geofence_status == Types::GeofenceStatus::GEOFENCE_NEAR_LIMIT) {
        setFault(Types::SafetyFault::SAFETY_FAULT_GEOFENCE_NEAR_LIMIT);
        setTelemetryEvent(TE_SAFETY_GEOFENCE_NEAR_LIMIT);
        if (inputs.drift_uncertainty_m >= Config::LOCALIZATION_DEGRADED_UNCERTAINTY_M) {
            req_hold = true;
        }
    } else {
        clearFault(Types::SafetyFault::SAFETY_FAULT_GEOFENCE_NEAR_LIMIT);
    }

    if (inputs.geofence_status == Types::GeofenceStatus::GEOFENCE_OUTSIDE) {
        if (geofence_outside_first_seen_ms_ == 0) geofence_outside_first_seen_ms_ = now;
        if ((now - geofence_outside_first_seen_ms_) >= Config::GEOFENCE_OUTSIDE_DEBOUNCE_MS) {
            setFault(Types::SafetyFault::SAFETY_FAULT_GEOFENCE_OUTSIDE);
            setTelemetryEvent(TE_SAFETY_GEOFENCE_OUTSIDE);
            if (Config::GEOFENCE_OUTSIDE_LAND_ENABLED) {
                req_land = true;
                primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_GEOFENCE_OUTSIDE);
            }
        }
    } else {
        geofence_outside_first_seen_ms_ = 0;
        clearFault(Types::SafetyFault::SAFETY_FAULT_GEOFENCE_OUTSIDE);
    }

    // ------------------------------------------------------------------------
    // 10. COLLISION RISK & PROXIMITY
    // ------------------------------------------------------------------------
    if (inputs.nearest_peer_distance_valid && !std::isnan(inputs.nearest_peer_distance_m)) {
        if (inputs.nearest_peer_distance_m <= Config::COLLISION_RISK_PEER_DISTANCE_CRITICAL_M) {
            setFault(Types::SafetyFault::SAFETY_FAULT_COLLISION_RISK);
            req_hold = true;
            setTelemetryEvent(TE_SAFETY_COLLISION_RISK);
        } else if (inputs.nearest_peer_distance_m <= Config::COLLISION_RISK_PEER_DISTANCE_M) {
            if (collision_risk_first_seen_ms_ == 0) collision_risk_first_seen_ms_ = now;
            if ((now - collision_risk_first_seen_ms_) >= Config::PROXIMITY_FAULT_DEBOUNCE_MS) {
                setFault(Types::SafetyFault::SAFETY_FAULT_UNSAFE_PROXIMITY_PEER);
                req_hold = true;
                setTelemetryEvent(TE_SAFETY_UNSAFE_PROXIMITY_PEER);
            }
        } else {
            collision_risk_first_seen_ms_ = 0;
            clearFault(Types::SafetyFault::SAFETY_FAULT_COLLISION_RISK);
            clearFault(Types::SafetyFault::SAFETY_FAULT_UNSAFE_PROXIMITY_PEER);
        }
    }

    if (inputs.nearest_human_distance_valid && !std::isnan(inputs.nearest_human_distance_m)) {
        if (inputs.nearest_human_distance_m <= Config::UNSAFE_PROXIMITY_HUMAN_CRITICAL_M) {
            setFault(Types::SafetyFault::SAFETY_FAULT_UNSAFE_PROXIMITY_HUMAN);
            req_hold = true;
            primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_UNSAFE_PROXIMITY_HUMAN);
            setTelemetryEvent(TE_SAFETY_UNSAFE_PROXIMITY_HUMAN);
        } else if (inputs.nearest_human_distance_m <= Config::UNSAFE_PROXIMITY_HUMAN_DISTANCE_M) {
            if (unsafe_proximity_first_seen_ms_ == 0) unsafe_proximity_first_seen_ms_ = now;
            if ((now - unsafe_proximity_first_seen_ms_) >= Config::PROXIMITY_FAULT_DEBOUNCE_MS) {
                setFault(Types::SafetyFault::SAFETY_FAULT_UNSAFE_PROXIMITY_HUMAN);
                req_hold = true;
                setTelemetryEvent(TE_SAFETY_UNSAFE_PROXIMITY_HUMAN);
            }
        } else {
            unsafe_proximity_first_seen_ms_ = 0;
            clearFault(Types::SafetyFault::SAFETY_FAULT_UNSAFE_PROXIMITY_HUMAN);
        }
    }

    // ------------------------------------------------------------------------
    // 11. UNWANTED SURFACE CONTACT
    // ------------------------------------------------------------------------
    if (inputs.surface_contact_detected && !inputs.touchdown_allowed) {
        if (surface_contact_first_seen_ms_ == 0) surface_contact_first_seen_ms_ = now;
        if ((now - surface_contact_first_seen_ms_) >= Config::SURFACE_CONTACT_DEBOUNCE_MS) {
            setFault(Types::SafetyFault::SAFETY_FAULT_SURFACE_CONTACT);
            setTelemetryEvent(TE_SAFETY_SURFACE_CONTACT);
            if (inputs.fc_armed) {
                req_emergency = true;
                primary_reason_ = static_cast<uint16_t>(Types::SafetyFault::SAFETY_FAULT_SURFACE_CONTACT);
            }
        }
    } else {
        surface_contact_first_seen_ms_ = 0;
        clearFault(Types::SafetyFault::SAFETY_FAULT_SURFACE_CONTACT);
    }

    // ------------------------------------------------------------------------
    // 12. PRIORITY ACTION SELECTION
    // ------------------------------------------------------------------------
    if (req_emergency) {
        recommended_action_ = Types::SafetyAction::EMERGENCY_CUT;
        setTelemetryEvent(TE_SAFETY_ACTION_EMERGENCY_CUT);
    } else if (req_land) {
        recommended_action_ = Types::SafetyAction::LAND;
        setTelemetryEvent(TE_SAFETY_ACTION_LAND);
    } else if (req_hold) {
        recommended_action_ = Types::SafetyAction::HOLD;
        setTelemetryEvent(TE_SAFETY_ACTION_HOLD);
    } else {
        recommended_action_ = Types::SafetyAction::CONTINUE;
        primary_reason_ = 0;
        setTelemetryEvent(TE_SAFETY_ACTION_CONTINUE);
    }
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void safety_manager_init() {
    s_global_safety_manager.init();
}

void safety_manager_update(const Types::SafetyInputs& inputs) {
    s_global_safety_manager.update(inputs);
}

Types::SafetyAction safety_manager_get_recommendation() {
    return s_global_safety_manager.recommendedAction();
}

bool safety_manager_is_safe_to_continue() {
    return s_global_safety_manager.recommendedAction() == Types::SafetyAction::CONTINUE;
}

SafetyManager& safety_manager_get_instance() {
    return s_global_safety_manager;
}

} // namespace RobofestDrone
