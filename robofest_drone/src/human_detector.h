#pragma once

#include <stdint.h>
#include "types.h"

namespace RobofestDrone {

// ============================================================================
// HUMAN MOTION DETECTOR (REQ-DER-102, item 2 - classical fallback path)
// ----------------------------------------------------------------------------
// Downward-camera person detection without a neural model: a person seen
// from 1.5-3 m altitude is a compact moving region against a mostly static
// ground plane. Frame differencing isolates motion; connected-component
// gating on size/aspect/plausibility rejects shadows, prop vibration and
// lighting flicker. Temporal confirmation (N consecutive hits) suppresses
// single-frame noise.
//
// This module is the ALWAYS-AVAILABLE backend for hal_human and the
// confidence source when the ESP-DL pedestrian model is unavailable or its
// inference budget is exceeded. Pure C++, host-testable, zero allocation
// after init().
// ============================================================================

class HumanMotionDetector {
public:
    HumanMotionDetector();

    // Working resolution (detector downscales internally if larger).
    bool init(uint16_t w, uint16_t h);
    void reset();

    // Consumes one grayscale working frame; emits at most one detection.
    // Returns true when `out` carries a fresh confirmed detection this call.
    bool process(
        const uint8_t* gray,
        Types::HumanDetectionSample& out,
        uint32_t now_ms);

    bool isInitialized() const { return prev_gray_ != nullptr; }
    uint16_t width() const { return w_; }
    uint16_t height() const { return h_; }

private:
    void binarize_diff(const uint8_t* gray);
    bool flood_largest(uint16_t& cx, uint16_t& cy, uint16_t& bw, uint16_t& bh);

    uint16_t w_ = 0;
    uint16_t h_ = 0;
    uint8_t* prev_gray_ = nullptr;   // [w*h]
    uint8_t* background_ = nullptr;  // [w*h] static-scene reference (bg subtraction)
    bool background_valid_ = false;
    uint8_t static_frames_ = 0;      // consecutive non-plausible frames
    uint8_t* diff_mask_ = nullptr;   // [w*h] binary motion mask
    int32_t* queue_x_ = nullptr;     // flood-fill stack/queue
    int32_t* queue_y_ = nullptr;

    uint8_t confirm_hits_ = 0;
    uint32_t last_emit_ms_ = 0;
    float last_cx_ = 0.0f;
    float last_cy_ = 0.0f;
};

// ============================================================================
// PURE TESTABLE CORES
// ============================================================================

// Largest connected component of a binary mask (4-connectivity). Returns the
// pixel count and bbox/centroid; zero count means no component above min_px.
uint32_t human_detector_largest_component(
    const uint8_t* mask, uint16_t w, uint16_t h,
    uint16_t min_pixels,
    float& out_cx, float& out_cy,
    uint16_t& out_bw, uint16_t& out_bh);

// Motion gate: abs(gray - prev) > threshold -> mask=1.
void human_detector_frame_diff(
    const uint8_t* gray, const uint8_t* prev,
    uint16_t n, uint8_t threshold,
    uint8_t* out_mask);

} // namespace RobofestDrone
