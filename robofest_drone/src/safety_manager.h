#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

// ============================================================================
// SAFETY MANAGER CLASS
// ============================================================================

class SafetyManager {
public:
    SafetyManager();

    void init();
    void reset();

    void update(const Types::SafetyInputs& inputs);

    Types::SafetyAction recommendedAction() const { return recommended_action_; }

    uint16_t getPrimaryReason() const { return primary_reason_; }
    uint32_t getActiveFaultMask() const { return active_fault_mask_; }

    bool isFaultActive(Types::SafetyFault fault) const;
    bool isBatteryLow() const { return battery_low_latched_; }
    bool isBatteryCritical() const { return battery_critical_latched_; }
    bool isKillSwitchActive() const { return kill_switch_latched_; }
    bool isMissionTimeOver() const { return mission_timeout_latched_; }
    bool isCollisionRiskHigh() const;
    bool isUnsafeProximityActive() const;
    bool isSurfaceContactFault() const;

    bool shouldStopSearchExpansion() const;
    bool shouldAllowTakeoff() const;
    bool shouldAllowGuidance() const;

    void acknowledgeEmergency();
    void clearLatchedFaults();

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    void setFault(Types::SafetyFault fault);
    void clearFault(Types::SafetyFault fault);
    void setTelemetryEvent(uint16_t event_id);

private:
    Types::SafetyAction recommended_action_ = Types::SafetyAction::CONTINUE;
    uint16_t primary_reason_ = 0;
    uint32_t active_fault_mask_ = 0;

    bool battery_low_latched_ = false;
    bool battery_critical_latched_ = false;
    bool kill_switch_latched_ = false;
    bool mission_timeout_latched_ = false;
    bool emergency_latched_ = false;

    uint32_t battery_low_first_seen_ms_ = 0;
    uint32_t battery_critical_first_seen_ms_ = 0;
    uint32_t fc_link_lost_first_seen_ms_ = 0;
    uint32_t camera_stall_first_seen_ms_ = 0;
    uint32_t radio_timeout_first_seen_ms_ = 0;
    uint32_t swarm_critical_first_seen_ms_ = 0;
    uint32_t localization_degraded_first_seen_ms_ = 0;
    uint32_t localization_unrecoverable_first_seen_ms_ = 0;
    uint32_t geofence_outside_first_seen_ms_ = 0;
    uint32_t collision_risk_first_seen_ms_ = 0;
    uint32_t unsafe_proximity_first_seen_ms_ = 0;
    uint32_t surface_contact_first_seen_ms_ = 0;

    uint32_t last_kill_switch_ms_ = 0;
    uint32_t last_battery_ok_ms_ = 0;
    uint32_t last_fc_link_ok_ms_ = 0;
    uint32_t last_camera_ok_ms_ = 0;
    uint32_t last_radio_ok_ms_ = 0;
    uint32_t last_localization_ok_ms_ = 0;

    uint16_t last_telemetry_event_id_ = TE_SAFETY_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void safety_manager_init();
void safety_manager_update(const Types::SafetyInputs& inputs);
Types::SafetyAction safety_manager_get_recommendation();
inline Types::SafetyAction safety_manager_get_recommended_action() { return safety_manager_get_recommendation(); }
bool safety_manager_is_safe_to_continue();
SafetyManager& safety_manager_get_instance();

} // namespace RobofestDrone
