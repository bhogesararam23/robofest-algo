#pragma once

#include <stdint.h>
#include "types.h"

namespace RobofestDrone {

// ============================================================================
// GESTURE RECOGNITION ENGINE (REQ-DER-103, item 3 - visual command path)
// ----------------------------------------------------------------------------
// Consumes a stream of hand observations (presence flag + normalized
// centroid from a skin-blob tracker) and emits AT MOST ONE command per
// physical gesture via a deterministic state machine:
//
//   IDLE ──present──> TRACKING ──hold >= HOLD_MS stationary──> EMIT(hold cmd)
//   TRACKING ──fast lateral sweep beyond SWEEP threshold──> EMIT(scan cmd)
//   TRACKING ──fast downward push──> EMIT(forward)
//   TRACKING ──abrupt exit after hold──> EMIT(stop candidate)
//   any EMIT ──> COOLDOWN(lockout) ──absent──> IDLE
//
// Debounce guarantees: no repeat emission while the same observation stays
// active, and a hard lockout window between successive commands.
// The engine is pure logic - camera-side blob extraction lives in hal_command.
// ============================================================================

struct HandObservation {
    bool present = false;
    float cx = 0.0f;   // normalized 0..1 image coords
    float cy = 0.0f;
    uint32_t timestamp_ms = 0;
};

class GestureEngine {
public:
    void reset();

    // Feeds one observation; returns true when `out` carries a newly
    // triggered (already debounced) command.
    bool update(const HandObservation& obs, Types::CommandSample& out);

private:
    enum class State : uint8_t {
        IDLE = 0,
        TRACKING,
        COOLDOWN
    };

    void emit(Types::CommandType cmd, Types::CommandSample& out,
              const HandObservation& obs, bool force = false);

    State state_ = State::IDLE;

    uint32_t presence_start_ms_ = 0;  // start of current continuous presence
    uint32_t last_seen_ms_ = 0;       // latest present observation
    uint32_t last_emit_ms_ = 0;
    uint32_t cooldown_end_ms_ = 0;

    float anchor_cx_ = 0.0f;
    float anchor_cy_ = 0.0f;

    Types::CommandType pending_hold_cmd_ = Types::CommandType::NONE;
};

// ============================================================================
// PURE SKIN-BLOB TRACKER
// ----------------------------------------------------------------------------
// Locates the largest skin-tone region in an RGB888 working frame using a
// conservative chroma gate (r>g>b ordering + saturation/value floors). This
// is deliberately narrow: false absences are recoverable, false presences
// create phantom commands. Returns normalized centroid + area fraction.
// ============================================================================
bool gesture_skin_blob_centroid(
    const uint8_t* rgb888, uint16_t w, uint16_t h,
    float& out_cx, float& out_cy,
    float& out_area_fraction);

} // namespace RobofestDrone
