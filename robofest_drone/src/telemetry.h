#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"
#include "../hal/hal_storage.h"

namespace RobofestDrone {

// ============================================================================
// TELEMETRY & ONBOARD LOGGING CLASS
// ============================================================================

class Telemetry {
public:
    Telemetry();

    void init();
    void reset();

    void update(const Types::TelemetryInputs& inputs);

    void logEvent(uint16_t event_id, uint8_t severity, uint8_t module_id, float value_a, float value_b, uint16_t context_id);
    void logInfo(uint16_t event_id, uint8_t module_id, float value_a = 0.0f, float value_b = 0.0f, uint16_t context_id = 0);
    void logWarning(uint16_t event_id, uint8_t module_id, float value_a = 0.0f, float value_b = 0.0f, uint16_t context_id = 0);
    void logCritical(uint16_t event_id, uint8_t module_id, float value_a = 0.0f, float value_b = 0.0f, uint16_t context_id = 0);

    void logStateTransition(Types::DroneState previous_state, Types::DroneState new_state, uint32_t now_ms);
    void logMineEvent(uint16_t mine_id, uint8_t status, float confidence, uint32_t now_ms);
    void logDetectionEvent(float world_x, float world_y, float confidence, uint32_t now_ms);
    void logClaimEvent(uint16_t mine_hash, uint8_t owner_drone_id, bool accepted, uint32_t now_ms);
    void logYieldEvent(uint16_t mine_hash, uint8_t reason, uint32_t now_ms);
    void logPathEvent(uint32_t path_version, bool path_valid, uint8_t reason, uint32_t now_ms);
    void logSafetyEvent(uint16_t fault_code, Types::SafetyAction action, uint32_t now_ms);
    void logCommandEvent(uint8_t command, float confidence, bool accepted, uint32_t now_ms);
    void logLocalizationEvent(Types::LocalizationHealth health, float drift_uncertainty_m, uint32_t now_ms);
    void logSwarmEvent(uint8_t peer_id, uint8_t swarm_event_code, uint32_t now_ms);
    void logHumanEvent(bool detected, bool off_path, bool in_exit_zone, uint32_t now_ms);
    void logMarkerEvent(uint8_t marker_pattern, uint32_t now_ms);
    void logSearchEvent(float coverage_percent, bool needs_more_scan, uint32_t now_ms);

    bool getLatestEvent(Types::TelemetryEvent& out_event) const;
    uint16_t getEventCount() const { return event_count_; }
    bool isBufferFull() const { return buffer_full_; }

    Types::MissionSummary getMissionSummary() const { return summary_; }

    void startMission(uint32_t now_ms);
    void endMission(uint32_t now_ms);

    void flushNow();
    bool isStorageHealthy() const { return storage_healthy_; }

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    void appendEventToBuffer(const Types::TelemetryEvent& event);
    void setTelemetryEvent(uint16_t event_id);

private:
    Types::TelemetryEvent events_[Config::MAX_TELEMETRY_EVENTS] = {};
    uint16_t event_head_ = 0;
    uint16_t event_count_ = 0;
    bool buffer_full_ = false;

    Types::MissionSummary summary_;

    uint32_t last_periodic_ms_ = 0;
    uint32_t last_drift_log_ms_ = 0;
    uint32_t last_summary_log_ms_ = 0;
    uint32_t last_storage_retry_ms_ = 0;

    uint16_t events_this_second_ = 0;
    uint32_t current_second_ms_ = 0;

    bool storage_healthy_ = true;
    bool flush_active_ = false;
    bool mission_active_ = false;

    Types::DroneState previous_state_ = Types::DroneState::INIT;

    uint16_t last_telemetry_event_id_ = TE_TELEMETRY_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void telemetry_init();
void telemetry_update(uint32_t now_ms);
void telemetry_log_event(uint16_t event_id, float value);
void telemetry_log_event(uint16_t event_id, uint8_t severity, uint8_t module_id, float value_a, float value_b, uint16_t context_id);
Telemetry& telemetry_get_instance();

} // namespace RobofestDrone
