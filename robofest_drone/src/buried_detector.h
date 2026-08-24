#pragma once

#include <stdint.h>
#include "types.h"

namespace RobofestDrone {

// ============================================================================
// BURIED-MINE DETECTOR (REQ-DER-113, item 13 + multispectral hooks item 3n)
// ----------------------------------------------------------------------------
// Detects buried mines from indirect visual cues:
//   1. Soil TEXTURE anomaly - disturbed earth has higher local luminance
//      variance than the surrounding field.
//   2. DEPTH residual - ground returns (lidar/ToF) are plane-fitted; mounds
//      and depressions above the residual threshold indicate subsurface
//      objects.
//   3. SPECTRAL indices - pluggable NDVI / moisture providers. When no
//      multispectral sensor is fitted, RGB-proxy estimators stand in and the
//      index weights simply contribute less discriminative evidence
//      (REQ item 3 adaptation for the XIAO ESP32S3 BOM).
//
// Fused score = texture_term + depth_term + spectral_terms; a candidate is
// emitted to the MineMap when it crosses BURIED_SCORE_EMIT_MIN. All math is
// pure/host-testable; hardware feeds arrive through small structs.
// ============================================================================

struct GroundReturn {
    float x;       // world or body-frame ground coordinates
    float y;
    float height;  // measured height of the soil at that point
};

struct SpectralIndices {
    bool valid = false;
    float ndvi = 0.0f;          // -1..1 vegetation stress proxy
    float moisture = 0.0f;      // 0..1 soil moisture proxy
};

struct BuriedAnomaly {
    bool detected = false;
    float x = 0.0f;
    float y = 0.0f;
    float score = 0.0f;         // 0..1 fused anomaly score
    float texture_ratio = 0.0f; // local variance / global variance
    float depth_residual_m = 0.0f;
    SpectralIndices spectral;
};

class BuriedDetector {
public:
    BuriedDetector();

    void init();
    void reset();

    // Pluggable multispectral provider hook (item 3). Pass nullptr to fall
    // back to RGB-proxy estimation. Signature fills `out` for one ROI.
    typedef void (*SpectralProviderFn)(const uint8_t* rgb888,
                                       uint16_t w, uint16_t h,
                                       uint16_t x0, uint16_t y0,
                                       uint16_t x1, uint16_t y1,
                                       SpectralIndices& out);
    void setSpectralProvider(SpectralProviderFn fn) { spectral_provider_ = fn; }

    // Processes one frame's center ROI plus recent ground returns.
    // rgb888 may be null (depth/spectral-only mode).
    BuriedAnomaly update(
        const uint8_t* rgb888,
        uint16_t w, uint16_t h,
        const GroundReturn* returns,
        uint8_t n_returns,
        uint32_t now_ms
    );

    // Cooldown gate so one disturbed patch doesn't spam candidates.
    bool readyToEmit(uint32_t now_ms) const;
    void markEmitted(uint32_t now_ms);

private:
    bool initialized_ = false;
    uint32_t last_emit_ms_ = 0;
    SpectralProviderFn spectral_provider_ = nullptr;
};

// ============================================================================
// PURE TESTABLE CORES
// ============================================================================

// Mean variance of luminance inside [x0,y0,x1,y1] vs whole-image variance.
float buried_texture_ratio(
    const uint8_t* rgb888, uint16_t w, uint16_t h,
    uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

// Least-squares plane fit z = ax + by + c over ground returns.
// Returns max |residual| in meters; also outputs coefficients when non-null.
float buried_plane_fit_residual(
    const GroundReturn* pts, uint8_t n,
    float* out_a, float* out_b, float* out_c);

// Default RGB-proxy spectral estimator (used when no NIR sensor is present):
// NDVI_proxy ~ (G - R) / (G + R + eps), moisture ~ B / (R+G+B + eps).
void buried_rgb_spectral_proxy(
    const uint8_t* rgb888, uint16_t w, uint16_t h,
    uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
    SpectralIndices& out);

// Fused score assembly (pure).
float buried_fuse_score(float texture_ratio, float depth_residual_m,
                        const SpectralIndices& idx);

} // namespace RobofestDrone
