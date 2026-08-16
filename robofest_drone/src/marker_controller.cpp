#include "marker_controller.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static MarkerController s_global_marker_controller;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

MarkerController::MarkerController() {
    reset();
}

void MarkerController::init() {
    reset();
    Hal::hal_marker_init();
    setTelemetryEvent(TE_MARKER_INITIALIZED);
}

void MarkerController::reset() {
    active_pattern_ = Types::MarkerPattern::MARKER_OFF;
    previous_pattern_ = Types::MarkerPattern::MARKER_OFF;
    override_pattern_ = Types::MarkerPattern::MARKER_OFF;

    output_enabled_ = Config::MARKER_OUTPUT_ENABLED_DEFAULT;
    override_active_ = false;
    override_start_ms_ = 0;
    override_duration_ms_ = 0;

    last_update_ms_ = 0;
    pattern_start_ms_ = 0;

    brightness_percent_ = Config::MARKER_BRIGHTNESS_DEFAULT_PERCENT;

    last_telemetry_event_id_ = TE_MARKER_INITIALIZED;
    telemetry_event_valid_ = true;
}

void MarkerController::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}

void MarkerController::setOutputEnabled(bool enabled) {
    output_enabled_ = enabled;
    if (enabled) {
        setTelemetryEvent(TE_MARKER_OUTPUT_ENABLED);
    } else {
        setTelemetryEvent(TE_MARKER_OUTPUT_DISABLED);
        Hal::hal_marker_enable(false);
    }
}

void MarkerController::overridePattern(Types::MarkerPattern pattern, uint32_t duration_ms, uint32_t now_ms) {
    override_active_ = true;
    override_pattern_ = pattern;
    override_start_ms_ = now_ms;
    override_duration_ms_ = duration_ms;
    setTelemetryEvent(TE_MARKER_OVERRIDE_STARTED);
}

void MarkerController::clearOverride() {
    override_active_ = false;
    setTelemetryEvent(TE_MARKER_OVERRIDE_EXPIRED);
}

void MarkerController::setBrightness(uint8_t brightness_percent) {
    brightness_percent_ = (brightness_percent > 100) ? 100 : brightness_percent;
    Hal::hal_marker_set_brightness(brightness_percent_);
}


// ============================================================================
// GUIDANCE PATTERN COMPUTATION
// ============================================================================

Types::MarkerPattern MarkerController::computeGuidingPattern(
    const Types::SafePath& path,
    const Types::HumanTrack& human
) {
    if (!path.valid || path.waypoint_count < 2) {
        return Types::MarkerPattern::MARKER_STOP;
    }

    if (!human.human_detected) {
        return Types::MarkerPattern::MARKER_CAUTION;
    }

    if (human.human_in_exit_zone) {
        return Types::MarkerPattern::MARKER_MISSION_COMPLETE;
    }

    // Check if human is off-path -> issue directional rejoin
    if (human.human_off_path) {
        // Find closest segment to determine lateral side
        float min_dist_sq = 1e9f;
        float side_cross = 0.0f;

        for (uint8_t i = 0; i < path.waypoint_count - 1; ++i) {
            float x1 = path.waypoints[i].x;
            float y1 = path.waypoints[i].y;
            float x2 = path.waypoints[i + 1].x;
            float y2 = path.waypoints[i + 1].y;

            float seg_dx = x2 - x1;
            float seg_dy = y2 - y1;
            float seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
            if (seg_len_sq < 1e-4f) continue;

            float t = ((human.field_x - x1) * seg_dx + (human.field_y - y1) * seg_dy) / seg_len_sq;
            t = std::max(0.0f, std::min(1.0f, t));

            float proj_x = x1 + t * seg_dx;
            float proj_y = y1 + t * seg_dy;

            float dx = human.field_x - proj_x;
            float dy = human.field_y - proj_y;
            float dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                // 2D Cross product: seg x (human - p1)
                side_cross = seg_dx * (human.field_y - y1) - seg_dy * (human.field_x - x1);
            }
        }

        // If side_cross > 0, human is to the left of the path forward vector -> steer right to rejoin
        if (side_cross > 0.0f) {
            return Types::MarkerPattern::MARKER_REJOIN_RIGHT;
        } else {
            return Types::MarkerPattern::MARKER_REJOIN_LEFT;
        }
    }

    // Human is safely on path -> evaluate upcoming turn ahead of human
    float progress = human.forward_progress_m;
    float accumulated = 0.0f;

    for (uint8_t i = 0; i < path.waypoint_count - 1; ++i) {
        float x1 = path.waypoints[i].x;
        float y1 = path.waypoints[i].y;
        float x2 = path.waypoints[i + 1].x;
        float y2 = path.waypoints[i + 1].y;

        float seg_dx = x2 - x1;
        float seg_dy = y2 - y1;
        float seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

        if (accumulated + seg_len > progress + 1.0f) {
            // Check turn at waypoint i+1 if within next 2 meters
            if (i + 2 < path.waypoint_count) {
                float x3 = path.waypoints[i + 2].x;
                float y3 = path.waypoints[i + 2].y;

                float next_dx = x3 - x2;
                float next_dy = y3 - y2;

                float turn_cross = seg_dx * next_dy - seg_dy * next_dx;
                if (turn_cross > 0.35f) {
                    return Types::MarkerPattern::MARKER_LEFT;
                } else if (turn_cross < -0.35f) {
                    return Types::MarkerPattern::MARKER_RIGHT;
                }
            }
            return Types::MarkerPattern::MARKER_SAFE_PATH;
        }

        accumulated += seg_len;
    }

    return Types::MarkerPattern::MARKER_FORWARD;
}


// ============================================================================
// PATTERN EVALUATION (PRIORITY-ORDERED)
// ============================================================================

Types::MarkerPattern MarkerController::evaluatePattern(
    Types::DroneState state,
    const Types::SafePath& path,
    const Types::HumanTrack& human,
    Types::SafetyAction safety_action,
    uint32_t now_ms
) {
    // Priority 1: Emergency cut / Emergency state
    if (safety_action == Types::SafetyAction::EMERGENCY_CUT || state == Types::DroneState::EMERGENCY) {
        return Types::MarkerPattern::MARKER_EMERGENCY;
    }

    // Priority 2: Safety action override
    if (safety_action == Types::SafetyAction::LAND) {
        return Types::MarkerPattern::MARKER_LANDING;
    }
    if (safety_action == Types::SafetyAction::HOLD) {
        return Types::MarkerPattern::MARKER_STOP;
    }

    // Priority 3: Manual override
    if (override_active_) {
        if (now_ms >= override_start_ms_ && (now_ms - override_start_ms_) <= override_duration_ms_) {
            return override_pattern_;
        } else {
            override_active_ = false;
            setTelemetryEvent(TE_MARKER_OVERRIDE_EXPIRED);
        }
    }

    // Priority 4: State Machine Guidance
    switch (state) {
        case Types::DroneState::INIT:
        case Types::DroneState::CALIBRATE:
        case Types::DroneState::WAIT_FOR_START:
        case Types::DroneState::DISARMED:
            return Types::MarkerPattern::MARKER_OFF;

        case Types::DroneState::TAKEOFF:
        case Types::DroneState::SEARCHING:
        case Types::DroneState::PLANNING:
        case Types::DroneState::FORMATION:
            return Types::MarkerPattern::MARKER_CAUTION;

        case Types::DroneState::HOLD:
            return Types::MarkerPattern::MARKER_STOP;

        case Types::DroneState::LANDING:
            return Types::MarkerPattern::MARKER_LANDING;

        case Types::DroneState::MISSION_COMPLETE:
            return Types::MarkerPattern::MARKER_MISSION_COMPLETE;

        case Types::DroneState::GUIDING:
            return computeGuidingPattern(path, human);

        case Types::DroneState::EMERGENCY:
        default:
            return Types::MarkerPattern::MARKER_EMERGENCY;
    }
}


// ============================================================================
// MAIN UPDATE (20 Hz NON-BLOCKING)
// ============================================================================

void MarkerController::update(
    Types::DroneState state,
    const Types::SafePath& path,
    const Types::HumanTrack& human,
    Types::SafetyAction safety_action,
    uint32_t now_ms
) {
    last_update_ms_ = now_ms;

    Types::MarkerPattern desired = evaluatePattern(state, path, human, safety_action, now_ms);

    // Apply minimum hold time unless transitioning into EMERGENCY
    if (desired != active_pattern_) {
        if (desired == Types::MarkerPattern::MARKER_EMERGENCY ||
            (now_ms - pattern_start_ms_) >= Config::MARKER_PATTERN_MIN_HOLD_MS) {
            previous_pattern_ = active_pattern_;
            active_pattern_ = desired;
            pattern_start_ms_ = now_ms;

            // Emit pattern telemetry
            switch (active_pattern_) {
                case Types::MarkerPattern::MARKER_OFF: setTelemetryEvent(TE_MARKER_PATTERN_OFF); break;
                case Types::MarkerPattern::MARKER_FORWARD: setTelemetryEvent(TE_MARKER_PATTERN_FORWARD); break;
                case Types::MarkerPattern::MARKER_STOP: setTelemetryEvent(TE_MARKER_PATTERN_STOP); break;
                case Types::MarkerPattern::MARKER_LEFT: setTelemetryEvent(TE_MARKER_PATTERN_LEFT); break;
                case Types::MarkerPattern::MARKER_RIGHT: setTelemetryEvent(TE_MARKER_PATTERN_RIGHT); break;
                case Types::MarkerPattern::MARKER_SAFE_PATH: setTelemetryEvent(TE_MARKER_PATTERN_SAFE_PATH); break;
                case Types::MarkerPattern::MARKER_EMERGENCY: setTelemetryEvent(TE_MARKER_PATTERN_EMERGENCY); break;
                case Types::MarkerPattern::MARKER_MISSION_COMPLETE: setTelemetryEvent(TE_MARKER_PATTERN_MISSION_COMPLETE); break;
                case Types::MarkerPattern::MARKER_CAUTION: setTelemetryEvent(TE_MARKER_PATTERN_CAUTION); break;
                case Types::MarkerPattern::MARKER_REJOIN_LEFT: setTelemetryEvent(TE_MARKER_PATTERN_REJOIN_LEFT); break;
                case Types::MarkerPattern::MARKER_REJOIN_RIGHT: setTelemetryEvent(TE_MARKER_PATTERN_REJOIN_RIGHT); break;
                case Types::MarkerPattern::MARKER_LANDING: setTelemetryEvent(TE_MARKER_PATTERN_LANDING); break;
                default: break;
            }
        }
    }

    // Drive physical HAL outputs
    if (output_enabled_) {
        Hal::hal_marker_set_pattern(active_pattern_);
        Hal::hal_marker_set_brightness(brightness_percent_);
        Hal::hal_marker_enable(true);
    } else {
        Hal::hal_marker_set_pattern(Types::MarkerPattern::MARKER_OFF);
        Hal::hal_marker_enable(false);
    }
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void marker_controller_init() {
    s_global_marker_controller.init();
}

void marker_controller_update(
    Types::DroneState state,
    const Types::SafePath& path,
    const Types::HumanTrack& human,
    Types::SafetyAction safety_action,
    uint32_t now_ms
) {
    s_global_marker_controller.update(state, path, human, safety_action, now_ms);
}

Types::MarkerPattern marker_controller_get_pattern() {
    return s_global_marker_controller.getActivePattern();
}

void marker_controller_set_brightness(uint8_t brightness) {
    s_global_marker_controller.setBrightness(brightness);
}

MarkerController& marker_controller_get_instance() {
    return s_global_marker_controller;
}

} // namespace RobofestDrone
