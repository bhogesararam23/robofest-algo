#include "gesture_engine.h"
#include <stddef.h>
#include "../config/thresholds.h"

namespace RobofestDrone {

namespace {
// Normalized-coordinate thresholds (image units, 0..1).

// Stationary hold duration for the START-style command.
constexpr uint32_t HOLD_TRIGGER_MS = 900UL;

// Centroid jitter below this is considered stationary.
constexpr float HOLD_STATIONARY_EPS = 0.06f;

// Lateral sweep: centroid displacement within SWEEP_WINDOW_MS.
constexpr float SWEEP_MIN_DISP = 0.30f;
constexpr uint32_t SWEEP_WINDOW_MS = 450UL;

// Downward push (hand moved toward drone/camera bottom) triggers FORWARD.
constexpr float PUSH_MIN_DY = 0.28f;
constexpr uint32_t PUSH_WINDOW_MS = 400UL;

// Abrupt exit after a sustained hold = stop gesture.
constexpr uint32_t STOP_EXIT_AFTER_MS = 700UL;
constexpr uint32_t STOP_EXIT_WINDOW_MS = 300UL;

constexpr uint32_t COOLDOWN_MS = Config::COMMAND_LOCKOUT_MS;

inline float dist2(float ax, float ay, float bx, float by) {
    const float dx = ax - bx;
    const float dy = ay - by;
    return dx * dx + dy * dy;
}
} // namespace

void GestureEngine::reset() {
    state_ = State::IDLE;
    presence_start_ms_ = 0;
    last_move_ms_ = 0;
    last_emit_ms_ = 0;
    cooldown_end_ms_ = 0;
    anchor_cx_ = 0.0f;
    anchor_cy_ = 0.0f;
    pending_hold_cmd_ = Types::CommandType::NONE;
}

void GestureEngine::emit(
    Types::CommandType cmd,
    Types::CommandSample& out,
    const HandObservation& obs) {
    out.valid = true;
    out.command = cmd;
    out.confidence = 0.85f;
    out.source = Types::CommandSource::COMMAND_SOURCE_GESTURE;
    out.timestamp_ms = obs.timestamp_ms;
    last_emit_ms_ = obs.timestamp_ms;
    cooldown_end_ms_ = obs.timestamp_ms + COOLDOWN_MS;
    state_ = State::COOLDOWN;
    pending_hold_cmd_ = Types::CommandType::NONE;
}

bool GestureEngine::update(const HandObservation& obs, Types::CommandSample& out) {
    out.valid = false;
    out.command = Types::CommandType::NONE;
    out.source = Types::CommandSource::COMMAND_SOURCE_NONE;
    if (!obs.present && state_ == State::IDLE) return false;

    switch (state_) {
        case State::IDLE:
            if (obs.present) {
                state_ = State::TRACKING;
                presence_start_ms_ = obs.timestamp_ms;
                last_move_ms_ = obs.timestamp_ms;
                anchor_cx_ = obs.cx;
                anchor_cy_ = obs.cy;
                pending_hold_cmd_ = Types::CommandType::START;
            }
            break;

        case State::TRACKING: {
            if (!obs.present) {
                // Abrupt exit after sustained hold => stop candidate.
                const uint32_t held =
                    last_move_ms_ - presence_start_ms_;
                if (held >= STOP_EXIT_AFTER_MS &&
                    (obs.timestamp_ms - last_move_ms_) <= STOP_EXIT_WINDOW_MS &&
                    pending_hold_cmd_ != Types::CommandType::NONE) {
                    emit(Types::CommandType::STOP_ABORT, out, obs);
                } else {
                    state_ = State::IDLE;
                }
                break;
            }

            // Fast lateral sweep detection against the tracking anchor.
            const uint32_t since_anchor = obs.timestamp_ms - presence_start_ms_;
            const float ddx = obs.cx - anchor_cx_;
            if (since_anchor <= SWEEP_WINDOW_MS &&
                ddx >= SWEEP_MIN_DISP) {
                emit(Types::CommandType::SCAN_RIGHT, out, obs);
                break;
            }
            if (since_anchor <= SWEEP_WINDOW_MS &&
                ddx <= -SWEEP_MIN_DISP) {
                emit(Types::CommandType::SCAN_LEFT, out, obs);
                break;
            }

            // Downward push.
            const float ddy = obs.cy - anchor_cy_;
            if (since_anchor <= PUSH_WINDOW_MS &&
                ddy >= PUSH_MIN_DY) {
                emit(Types::CommandType::FORWARD, out, obs);
                break;
            }

            // Sustained stationary hold.
            const bool stationary =
                dist2(obs.cx, obs.cy, anchor_cx_, anchor_cy_) <
                HOLD_STATIONARY_EPS * HOLD_STATIONARY_EPS;
            if (stationary) {
                if ((obs.timestamp_ms - presence_start_ms_) >=
                        HOLD_TRIGGER_MS &&
                    pending_hold_cmd_ != Types::CommandType::NONE) {
                    emit(pending_hold_cmd_, out, obs);
                }
            } else {
                // Drifted: re-anchor so slow hand wander doesn't fake sweeps.
                if ((obs.timestamp_ms - last_move_ms_) > 150UL) {
                    anchor_cx_ = obs.cx;
                    anchor_cy_ = obs.cy;
                    presence_start_ms_ = obs.timestamp_ms; // hold timer restarts
                    last_move_ms_ = obs.timestamp_ms;
                }
            }
            break;
        }

        case State::COOLDOWN:
            if (!obs.present ||
                obs.timestamp_ms >= cooldown_end_ms_) {
                if (obs.timestamp_ms >= cooldown_end_ms_) {
                    state_ = State::IDLE;
                }
            }
            break;
    }

    return out.valid;
}

// ============================================================================
// PURE SKIN-BLOB TRACKER
// ============================================================================

bool gesture_skin_blob_centroid(
    const uint8_t* rgb888, uint16_t w, uint16_t h,
    float& out_cx, float& out_cy,
    float& out_area_fraction) {

    out_cx = 0.0f;
    out_cy = 0.0f;
    out_area_fraction = 0.0f;
    if (rgb888 == nullptr || w == 0 || h == 0) return false;

    // Conservative skin chroma gate on raw RGB:
    //   r > g > b ordering with meaningful separation and moderate luma.
    constexpr uint8_t MIN_V = 60;
    constexpr uint8_t MIN_RG_SEP = 12;
    constexpr uint8_t MIN_GB_SEP = 5;

    uint32_t count = 0;
    float sum_x = 0.0f, sum_y = 0.0f;
    for (uint16_t y = 0; y < h; ++y) {
        const uint8_t* row = rgb888 + static_cast<size_t>(y) * w * 3;
        for (uint16_t x = 0; x < w; ++x) {
            const uint8_t r = row[x * 3];
            const uint8_t g = row[x * 3 + 1];
            const uint8_t b = row[x * 3 + 2];
            if (r >= MIN_V &&
                (r - g) >= MIN_RG_SEP &&
                (g - b) >= MIN_GB_SEP) {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }

    if (count == 0) return false;
    const float total = static_cast<float>(w) * h;
    const float frac = static_cast<float>(count) / total;

    // A plausible hand at gesture distance covers a bounded frame share.
    if (frac < 0.004f || frac > 0.45f) return false;

    out_cx = sum_x / static_cast<float>(count) / static_cast<float>(w);
    out_cy = sum_y / static_cast<float>(count) / static_cast<float>(h);
    out_area_fraction = frac;
    return true;
}

} // namespace RobofestDrone
