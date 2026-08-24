#pragma once

#include <stdint.h>
#include <stddef.h>

namespace RobofestDrone {

// ============================================================================
// IMAGE ENHANCEMENT SUITE (REQ-DER-108/115, items 8 & 15 + dehazing item 5n)
// ----------------------------------------------------------------------------
// Pure, allocation-free, host-testable enhancement stages operating on packed
// RGB888 working buffers at process resolution. The vision pipeline selects
// which stages run based on lighting telemetry; each stage is bounded-time.
// ============================================================================

struct EnhanceDiagnostics {
    float haze_severity = 0.0f;   // 0 = clear .. 1 = heavy haze
    uint16_t shadow_pixels = 0;   // pixels flagged as physical shadows
    bool clahe_applied = false;
    bool dehaze_applied = false;
    bool night_lift_applied = false;
};

// ----------------------------------------------------------------------------
// Gamma lift for night/low-light frames (gamma < 1 brightens midtones).
// ----------------------------------------------------------------------------
void enhance_gamma(uint8_t* rgb888, size_t n_px, float gamma);

// ----------------------------------------------------------------------------
// CLAHE on the V channel: 4x3 tile grid, clip-and-redistribute histogram,
// bilinear blend between tile mappings to avoid seams. Strength in [0..1].
// ----------------------------------------------------------------------------
void enhance_clahe_v(uint8_t* rgb888, uint16_t w, uint16_t h, float strength);

// ----------------------------------------------------------------------------
// Haze severity estimator: combines dark-channel density with global
// contrast collapse. Cheap sampling; deterministic.
// Returns 0 (clear) .. 1 (opaque).
// ----------------------------------------------------------------------------
float enhance_haze_severity(const uint8_t* rgb888, uint16_t w, uint16_t h);

// ----------------------------------------------------------------------------
// Dark-channel-prior dehaze (He et al., simplified fixed-window version).
// strength in [0..1] scales the omega scattering coefficient. Bounded time:
// uses strided min-filter sampling suitable for 160x120 working frames.
// ----------------------------------------------------------------------------
void enhance_dehaze(uint8_t* rgb888, uint16_t w, uint16_t h, float strength);

// ----------------------------------------------------------------------------
// Shadow mask via luminance-ratio + chroma-neutrality test:
// flags pixels significantly darker than their neighborhood while retaining
// similar chromaticity (physical shadows darken all channels roughly evenly,
// unlike dark objects which shift chroma).
// out_flags: one byte per pixel, 1 = shadow-suspect.
// ----------------------------------------------------------------------------
void enhance_shadow_mask(
    const uint8_t* rgb888,
    uint16_t w,
    uint16_t h,
    uint8_t* out_flags);

} // namespace RobofestDrone
