#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"
#include "../hal/hal_marker.h"

namespace RobofestDrone {

// ============================================================================
// MARKER CONTROLLER CLASS
// ============================================================================

class MarkerController {
public:
    MarkerController();

    void init();
    void reset();

    void update(
        Types::DroneState state,
        const Types::SafePath& path,
        const Types::HumanTrack& human,
        Types::SafetyAction safety_action,
        uint32_t now_ms
    );

    Types::MarkerPattern getActivePattern() const { return active_pattern_; }
    Types::MarkerPattern getPreviousPattern() const { return previous_pattern_; }

    void setOutputEnabled(bool enabled);
    bool isOutputEnabled() const { return output_enabled_; }

    void overridePattern(Types::MarkerPattern pattern, uint32_t duration_ms, uint32_t now_ms);
    void clearOverride();
    bool isOverrideActive() const { return override_active_; }

    void setBrightness(uint8_t brightness_percent);
    uint8_t getBrightness() const { return brightness_percent_; }

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    Types::MarkerPattern evaluatePattern(
        Types::DroneState state,
        const Types::SafePath& path,
        const Types::HumanTrack& human,
        Types::SafetyAction safety_action,
        uint32_t now_ms
    );

    Types::MarkerPattern computeGuidingPattern(
        const Types::SafePath& path,
        const Types::HumanTrack& human
    );

    void setPatternInternal(Types::MarkerPattern pattern, uint32_t now_ms);
    void setTelemetryEvent(uint16_t event_id);

private:
    Types::MarkerPattern active_pattern_ = Types::MarkerPattern::MARKER_OFF;
    Types::MarkerPattern previous_pattern_ = Types::MarkerPattern::MARKER_OFF;
    Types::MarkerPattern override_pattern_ = Types::MarkerPattern::MARKER_OFF;

    bool output_enabled_ = true;
    bool override_active_ = false;
    uint32_t override_start_ms_ = 0;
    uint32_t override_duration_ms_ = 0;

    uint32_t last_update_ms_ = 0;
    uint32_t pattern_start_ms_ = 0;

    uint8_t brightness_percent_ = Config::MARKER_BRIGHTNESS_DEFAULT_PERCENT;

    uint16_t last_telemetry_event_id_ = TE_MARKER_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void marker_controller_init();
void marker_controller_update(
    Types::DroneState state,
    const Types::SafePath& path,
    const Types::HumanTrack& human,
    Types::SafetyAction safety_action,
    uint32_t now_ms
);
Types::MarkerPattern marker_controller_get_pattern();
void marker_controller_set_brightness(uint8_t brightness);
MarkerController& marker_controller_get_instance();

} // namespace RobofestDrone
