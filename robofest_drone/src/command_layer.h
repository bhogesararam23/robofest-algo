#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"
#include "../hal/hal_command.h"

namespace RobofestDrone {

// ============================================================================
// COMMAND REJECTION REASON CODES
// ============================================================================

constexpr uint16_t COMMAND_ACCEPTED                          = 0;
constexpr uint16_t COMMAND_REJECTED_LOW_CONFIDENCE           = 1;
constexpr uint16_t COMMAND_REJECTED_DEBOUNCE                 = 2;
constexpr uint16_t COMMAND_REJECTED_LOCKOUT                  = 3;
constexpr uint16_t COMMAND_REJECTED_STATE_NOT_ALLOWED        = 4;
constexpr uint16_t COMMAND_REJECTED_SOURCE_DISABLED          = 5;
constexpr uint16_t COMMAND_REJECTED_STALE_SAMPLE             = 6;
constexpr uint16_t COMMAND_REJECTED_SOURCE_CONFLICT          = 7;
constexpr uint16_t COMMAND_REJECTED_INVALID_INPUT            = 8;
constexpr uint16_t COMMAND_REJECTED_DEBUG_DISABLED           = 9;


// ============================================================================
// COMMAND LAYER CLASS
// ============================================================================

class CommandLayer {
public:
    CommandLayer();

    void init();
    void reset();

    void update(uint32_t now_ms);

    Types::CommandType getLatestCommand() const { return accepted_command_; }
    float getLatestConfidence() const { return accepted_confidence_; }
    Types::CommandSource getLatestSource() const { return accepted_source_; }

    bool isCommandValid() const { return command_valid_; }
    bool consumeCommand();
    void clearCommand();

    bool isStartAllowed() const;
    bool isPauseAllowed() const;
    bool isStopAllowed() const;

    void setSystemState(Types::DroneState state) { system_state_ = state; }
    void setAllowStart(bool allowed) { allow_start_ = allowed; }
    void setAllowPause(bool allowed) { allow_pause_ = allowed; }
    void setAllowStop(bool allowed) { allow_stop_ = allowed; }

    void enableGesture(bool enabled);
    void enableVoice(bool enabled);
    void enableDebugCommands(bool enabled) { debug_enabled_ = enabled; }

    uint8_t getStableCount() const { return stable_count_; }
    uint32_t getLastCommandTimeMs() const { return last_command_time_ms_; }
    uint32_t getLastAcceptedTimeMs() const { return accepted_time_ms_; }

    uint8_t getActiveScanDirection() const { return active_scan_direction_; }
    uint32_t getScanCommandExpiryMs() const { return scan_expiry_ms_; }

    bool getLatestCommandEvent(Types::CommandEvent& out) const;
    uint8_t getCommandEventCount() const { return event_count_; }

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    Types::CommandSample fuseInputs(
        const Types::CommandSample& gesture,
        const Types::CommandSample& voice,
        uint32_t now_ms
    );

    float getConfidenceThreshold(Types::CommandType cmd) const;
    bool isCommandPermittedInState(Types::CommandType cmd) const;
    void logEvent(Types::CommandType cmd, float conf, Types::CommandSource src, bool accepted, uint16_t reason, uint32_t ts);
    void setTelemetryEvent(uint16_t event_id);

private:
    Types::CommandType raw_command_ = Types::CommandType::NONE;
    float raw_confidence_ = 0.0f;
    Types::CommandSource raw_source_ = Types::CommandSource::COMMAND_SOURCE_NONE;

    Types::CommandType pending_command_ = Types::CommandType::NONE;
    uint8_t stable_count_ = 0;
    uint32_t pending_first_seen_ms_ = 0;
    uint32_t pending_last_seen_ms_ = 0;

    Types::CommandType accepted_command_ = Types::CommandType::NONE;
    float accepted_confidence_ = 0.0f;
    Types::CommandSource accepted_source_ = Types::CommandSource::COMMAND_SOURCE_NONE;
    uint32_t accepted_time_ms_ = 0;
    bool command_valid_ = false;

    uint32_t lockout_until_ms_ = 0;
    uint32_t last_command_time_ms_ = 0;

    Types::DroneState system_state_ = Types::DroneState::INIT;
    bool allow_start_ = true;
    bool allow_pause_ = true;
    bool allow_stop_ = true;

    bool gesture_enabled_ = Config::GESTURE_ENABLED_DEFAULT;
    bool voice_enabled_ = Config::VOICE_ENABLED_DEFAULT;
    bool debug_enabled_ = Config::DEBUG_COMMANDS_ENABLED_DEFAULT;

    uint8_t active_scan_direction_ = 0; // 0 = none, 1 = left, 2 = right
    uint32_t scan_expiry_ms_ = 0;

    Types::CommandEvent events_[Types::MAX_COMMAND_EVENTS] = {};
    uint8_t event_head_ = 0;
    uint8_t event_count_ = 0;

    uint16_t last_telemetry_event_id_ = TE_COMMAND_LAYER_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void command_layer_init();
void command_layer_update(uint32_t now_ms);
Types::CommandType command_layer_get_command();
inline Types::CommandType command_layer_get_latest_command() { return command_layer_get_command(); }
bool command_layer_consume_command();
CommandLayer& command_layer_get_instance();

} // namespace RobofestDrone
