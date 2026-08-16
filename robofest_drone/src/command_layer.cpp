#include "command_layer.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static CommandLayer s_global_command_layer;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

CommandLayer::CommandLayer() {
    reset();
}

void CommandLayer::init() {
    reset();
    Hal::hal_command_init();
}

void CommandLayer::reset() {
    raw_command_ = Types::CommandType::NONE;
    raw_confidence_ = 0.0f;
    raw_source_ = Types::CommandSource::COMMAND_SOURCE_NONE;

    pending_command_ = Types::CommandType::NONE;
    stable_count_ = 0;
    pending_first_seen_ms_ = 0;
    pending_last_seen_ms_ = 0;

    accepted_command_ = Types::CommandType::NONE;
    accepted_confidence_ = 0.0f;
    accepted_source_ = Types::CommandSource::COMMAND_SOURCE_NONE;
    accepted_time_ms_ = 0;
    command_valid_ = false;

    lockout_until_ms_ = 0;
    last_command_time_ms_ = 0;

    system_state_ = Types::DroneState::INIT;
    allow_start_ = true;
    allow_pause_ = true;
    allow_stop_ = true;

    gesture_enabled_ = Config::GESTURE_ENABLED_DEFAULT;
    voice_enabled_ = Config::VOICE_ENABLED_DEFAULT;
    debug_enabled_ = Config::DEBUG_COMMANDS_ENABLED_DEFAULT;

    active_scan_direction_ = 0;
    scan_expiry_ms_ = 0;

    for (uint8_t i = 0; i < Types::MAX_COMMAND_EVENTS; ++i) {
        events_[i] = Types::CommandEvent();
    }
    event_head_ = 0;
    event_count_ = 0;

    last_telemetry_event_id_ = TE_COMMAND_LAYER_INITIALIZED;
    telemetry_event_valid_ = true;
}

void CommandLayer::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}

void CommandLayer::enableGesture(bool enabled) {
    gesture_enabled_ = enabled;
    Hal::hal_command_enable_gesture(enabled);
    setTelemetryEvent(enabled ? TE_COMMAND_GESTURE_ENABLED : TE_COMMAND_GESTURE_DISABLED);
}

void CommandLayer::enableVoice(bool enabled) {
    voice_enabled_ = enabled;
    Hal::hal_command_enable_voice(enabled);
    setTelemetryEvent(enabled ? TE_COMMAND_VOICE_ENABLED : TE_COMMAND_VOICE_DISABLED);
}


// ============================================================================
// EVENT LOGGING & QUERIES
// ============================================================================

void CommandLayer::logEvent(
    Types::CommandType cmd,
    float conf,
    Types::CommandSource src,
    bool accepted,
    uint16_t reason,
    uint32_t ts
) {
    events_[event_head_].command = cmd;
    events_[event_head_].confidence = conf;
    events_[event_head_].source = src;
    events_[event_head_].accepted = accepted;
    events_[event_head_].rejection_reason = reason;
    events_[event_head_].timestamp_ms = ts;

    event_head_ = (event_head_ + 1) % Types::MAX_COMMAND_EVENTS;
    if (event_count_ < Types::MAX_COMMAND_EVENTS) {
        event_count_++;
    }
}

bool CommandLayer::getLatestCommandEvent(Types::CommandEvent& out) const {
    if (event_count_ == 0) return false;
    uint8_t latest_idx = (event_head_ == 0) ? (Types::MAX_COMMAND_EVENTS - 1) : (event_head_ - 1);
    out = events_[latest_idx];
    return true;
}


// ============================================================================
// PERMISSION CHECKS & ASYMMETRIC THRESHOLDS
// ============================================================================

float CommandLayer::getConfidenceThreshold(Types::CommandType cmd) const {
    switch (cmd) {
        case Types::CommandType::START:      return Config::START_CONFIDENCE_MIN;
        case Types::CommandType::FORWARD:    return Config::FORWARD_CONFIDENCE_MIN;
        case Types::CommandType::PAUSE:      return Config::PAUSE_CONFIDENCE_MIN;
        case Types::CommandType::SCAN_LEFT:  return Config::SCAN_CONFIDENCE_MIN;
        case Types::CommandType::SCAN_RIGHT: return Config::SCAN_CONFIDENCE_MIN;
        case Types::CommandType::STOP_ABORT: return Config::STOP_CONFIDENCE_MIN;
        default:                             return 1.0f;
    }
}

bool CommandLayer::isStartAllowed() const {
    if (!allow_start_) return false;
    if (Config::START_ALLOW_ONLY_BEFORE_TAKEOFF) {
        return (system_state_ == Types::DroneState::INIT ||
                system_state_ == Types::DroneState::CALIBRATE ||
                system_state_ == Types::DroneState::WAIT_FOR_START);
    }
    return true;
}

bool CommandLayer::isPauseAllowed() const {
    if (!allow_pause_) return false;
    if (Config::PAUSE_ALLOW_DURING_FLIGHT_ONLY) {
        return (system_state_ == Types::DroneState::TAKEOFF ||
                system_state_ == Types::DroneState::FORMATION ||
                system_state_ == Types::DroneState::SEARCHING ||
                system_state_ == Types::DroneState::PLANNING ||
                system_state_ == Types::DroneState::GUIDING ||
                system_state_ == Types::DroneState::HOLD);
    }
    return true;
}

bool CommandLayer::isStopAllowed() const {
    if (!allow_stop_) return false;
    return (system_state_ != Types::DroneState::DISARMED);
}

bool CommandLayer::isCommandPermittedInState(Types::CommandType cmd) const {
    switch (cmd) {
        case Types::CommandType::START:
            return isStartAllowed();

        case Types::CommandType::PAUSE:
            return isPauseAllowed();

        case Types::CommandType::STOP_ABORT:
            return isStopAllowed();

        case Types::CommandType::FORWARD:
            return (system_state_ != Types::DroneState::DISARMED &&
                    system_state_ != Types::DroneState::EMERGENCY &&
                    system_state_ != Types::DroneState::MISSION_COMPLETE);

        case Types::CommandType::SCAN_LEFT:
        case Types::CommandType::SCAN_RIGHT:
            return (system_state_ == Types::DroneState::FORMATION ||
                    system_state_ == Types::DroneState::SEARCHING ||
                    system_state_ == Types::DroneState::PLANNING ||
                    system_state_ == Types::DroneState::GUIDING);

        default:
            return false;
    }
}


// ============================================================================
// COMMAND SOURCE FUSION
// ============================================================================

Types::CommandSample CommandLayer::fuseInputs(
    const Types::CommandSample& gesture,
    const Types::CommandSample& voice,
    uint32_t now_ms
) {
    Types::CommandSample result;
    result.valid = false;
    result.command = Types::CommandType::NONE;
    result.confidence = 0.0f;
    result.source = Types::CommandSource::COMMAND_SOURCE_NONE;
    result.timestamp_ms = now_ms;

    bool g_active = gesture_enabled_ && gesture.valid && (gesture.command != Types::CommandType::NONE);
    bool v_active = voice_enabled_ && voice.valid && (voice.command != Types::CommandType::NONE);

    // Stale sample rejection
    if (g_active && (now_ms - gesture.timestamp_ms > Config::COMMAND_STALE_SAMPLE_TIMEOUT_MS)) {
        g_active = false;
        logEvent(gesture.command, gesture.confidence, Types::CommandSource::COMMAND_SOURCE_GESTURE, false, COMMAND_REJECTED_STALE_SAMPLE, now_ms);
        setTelemetryEvent(TE_COMMAND_STALE_SAMPLE);
    }
    if (v_active && (now_ms - voice.timestamp_ms > Config::COMMAND_STALE_SAMPLE_TIMEOUT_MS)) {
        v_active = false;
        logEvent(voice.command, voice.confidence, Types::CommandSource::COMMAND_SOURCE_VOICE, false, COMMAND_REJECTED_STALE_SAMPLE, now_ms);
        setTelemetryEvent(TE_COMMAND_STALE_SAMPLE);
    }

    if (g_active && !v_active) {
        result = gesture;
    } else if (!g_active && v_active) {
        result = voice;
    } else if (g_active && v_active) {
        if (gesture.command == voice.command) {
            // Concordant fusion bonus
            float max_conf = std::max(gesture.confidence, voice.confidence);
            float min_conf = std::min(gesture.confidence, voice.confidence);
            float fused_conf = max_conf + Config::COMMAND_FUSION_BONUS * min_conf;
            if (fused_conf > 1.0f) fused_conf = 1.0f;

            result.valid = true;
            result.command = gesture.command;
            result.confidence = fused_conf;
            result.source = Types::CommandSource::COMMAND_SOURCE_FUSED;
            result.timestamp_ms = now_ms;
        } else {
            // Conflicting inputs
            float diff = std::abs(gesture.confidence - voice.confidence);
            if (diff > Config::COMMAND_SOURCE_CONFLICT_MARGIN) {
                if (gesture.confidence > voice.confidence) {
                    result = gesture;
                    result.confidence *= 0.90f; // Conflict penalty
                } else {
                    result = voice;
                    result.confidence *= 0.90f; // Conflict penalty
                }
            } else {
                // Ambiguous conflict
                result.valid = false;
                result.command = Types::CommandType::NONE;
                result.source = Types::CommandSource::COMMAND_SOURCE_NONE;
                logEvent(gesture.command, gesture.confidence, Types::CommandSource::COMMAND_SOURCE_FUSED, false, COMMAND_REJECTED_SOURCE_CONFLICT, now_ms);
                setTelemetryEvent(TE_COMMAND_SOURCE_CONFLICT);
            }
        }
    }

    return result;
}


// ============================================================================
// MAIN UPDATE (50 Hz NON-BLOCKING)
// ============================================================================

void CommandLayer::update(uint32_t now_ms) {
    // 1. Read hardware samples
    Types::CommandSample g_sample;
    Types::CommandSample v_sample;

    if (gesture_enabled_) {
        Hal::hal_command_read_gesture(g_sample);
    }
    if (voice_enabled_) {
        Hal::hal_command_read_voice(v_sample);
    }

    // 2. Fuse inputs
    Types::CommandSample raw_sample = fuseInputs(g_sample, v_sample, now_ms);
    raw_command_ = raw_sample.command;
    raw_confidence_ = raw_sample.confidence;
    raw_source_ = raw_sample.source;

    // 3. Hysteresis & consecutive frame tracking
    if (raw_sample.valid && raw_sample.command != Types::CommandType::NONE) {
        float req_conf = getConfidenceThreshold(raw_sample.command);
        if (raw_sample.confidence < req_conf) {
            logEvent(raw_sample.command, raw_sample.confidence, raw_sample.source, false, COMMAND_REJECTED_LOW_CONFIDENCE, now_ms);
            setTelemetryEvent(TE_COMMAND_REJECTED_LOW_CONFIDENCE);
            raw_sample.command = Types::CommandType::NONE;
        }
    }

    if (raw_sample.command == pending_command_ && raw_sample.command != Types::CommandType::NONE) {
        stable_count_++;
        pending_last_seen_ms_ = now_ms;
    } else if (raw_sample.command == Types::CommandType::NONE) {
        if ((now_ms - pending_last_seen_ms_) > Config::COMMAND_NONE_GRACE_MS) {
            stable_count_ = 0;
            pending_command_ = Types::CommandType::NONE;
        }
    } else {
        pending_command_ = raw_sample.command;
        stable_count_ = 1;
        pending_first_seen_ms_ = now_ms;
        pending_last_seen_ms_ = now_ms;
    }

    // 4. Debounce and Acceptance Evaluation
    if (pending_command_ != Types::CommandType::NONE && stable_count_ >= Config::COMMAND_HYSTERESIS_FRAMES) {
        if ((now_ms - pending_first_seen_ms_) >= Config::COMMAND_DEBOUNCE_MS) {
            // Check lockout
            if (now_ms < lockout_until_ms_) {
                if (pending_command_ == accepted_command_) {
                    logEvent(pending_command_, raw_confidence_, raw_source_, false, COMMAND_REJECTED_LOCKOUT, now_ms);
                    setTelemetryEvent(TE_COMMAND_REJECTED_LOCKOUT);
                    return;
                }
            }

            // Check permission in current mission state
            if (!isCommandPermittedInState(pending_command_)) {
                logEvent(pending_command_, raw_confidence_, raw_source_, false, COMMAND_REJECTED_STATE_NOT_ALLOWED, now_ms);
                setTelemetryEvent(TE_COMMAND_REJECTED_STATE_NOT_ALLOWED);
                return;
            }

            // Accept command!
            accepted_command_ = pending_command_;
            accepted_confidence_ = raw_confidence_;
            accepted_source_ = raw_source_;
            accepted_time_ms_ = now_ms;
            command_valid_ = true;
            last_command_time_ms_ = now_ms;

            uint32_t lockout_duration = (pending_command_ == Types::CommandType::STOP_ABORT) ?
                                        Config::STOP_COMMAND_LOCKOUT_MS : Config::COMMAND_LOCKOUT_MS;
            lockout_until_ms_ = now_ms + lockout_duration;

            // Handle directional scan commands
            if (pending_command_ == Types::CommandType::SCAN_LEFT) {
                active_scan_direction_ = 1;
                scan_expiry_ms_ = now_ms + Config::SCAN_COMMAND_ACTIVE_MS;
            } else if (pending_command_ == Types::CommandType::SCAN_RIGHT) {
                active_scan_direction_ = 2;
                scan_expiry_ms_ = now_ms + Config::SCAN_COMMAND_ACTIVE_MS;
            }

            logEvent(accepted_command_, accepted_confidence_, accepted_source_, true, COMMAND_ACCEPTED, now_ms);

            switch (accepted_command_) {
                case Types::CommandType::START:      setTelemetryEvent(TE_COMMAND_START_ACCEPTED); break;
                case Types::CommandType::FORWARD:    setTelemetryEvent(TE_COMMAND_FORWARD_ACCEPTED); break;
                case Types::CommandType::PAUSE:      setTelemetryEvent(TE_COMMAND_PAUSE_ACCEPTED); break;
                case Types::CommandType::SCAN_LEFT:  setTelemetryEvent(TE_COMMAND_SCAN_LEFT_ACCEPTED); break;
                case Types::CommandType::SCAN_RIGHT: setTelemetryEvent(TE_COMMAND_SCAN_RIGHT_ACCEPTED); break;
                case Types::CommandType::STOP_ABORT: setTelemetryEvent(TE_COMMAND_STOP_ACCEPTED); break;
                default: break;
            }

            // Reset pending tracking
            pending_command_ = Types::CommandType::NONE;
            stable_count_ = 0;
        }
    }

    // 5. Expiration of unconsumed command
    if (command_valid_ && (now_ms - accepted_time_ms_ > Config::COMMAND_VALID_WINDOW_MS)) {
        command_valid_ = false;
        setTelemetryEvent(TE_COMMAND_VALID_WINDOW_EXPIRED);
    }

    // 6. Expiration of scan direction hint
    if (active_scan_direction_ != 0 && now_ms > scan_expiry_ms_) {
        active_scan_direction_ = 0;
        setTelemetryEvent(TE_COMMAND_SCAN_EXPIRED);
    }
}


// ============================================================================
// CONSUMPTION API
// ============================================================================

bool CommandLayer::consumeCommand() {
    if (command_valid_) {
        command_valid_ = false;
        return true;
    }
    return false;
}

void CommandLayer::clearCommand() {
    command_valid_ = false;
    accepted_command_ = Types::CommandType::NONE;
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void command_layer_init() {
    s_global_command_layer.init();
}

void command_layer_update(uint32_t now_ms) {
    s_global_command_layer.update(now_ms);
}

Types::CommandType command_layer_get_command() {
    return s_global_command_layer.getLatestCommand();
}

bool command_layer_consume_command() {
    return s_global_command_layer.consumeCommand();
}

CommandLayer& command_layer_get_instance() {
    return s_global_command_layer;
}

} // namespace RobofestDrone
