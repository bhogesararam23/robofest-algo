#pragma once

#include <stdint.h>
#include "vision_geom.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

// ============================================================================
// SHAPE ANALYSIS BEYOND CONVEX HULLS (REQ-DER-107, item 7)
// ----------------------------------------------------------------------------
// The hull-based descriptors in VisionPipeline cannot distinguish a square
// from a rotated octagon nor detect star/concave geometries reliably. This
// module adds:
//   1. Ordered boundary extraction (Moore-neighbor contour trace) that
//      PRESERVES concavities, unlike the unordered BFS border sample set.
//   2. Corner counting via Douglas-Peucker directly on the raw contour.
//   3. Convexity defect counting: how deep and how often the true boundary
//      dives inside its convex hull - the signature of stars/gears/crosses.
//   4. A deterministic decision tree combining circularity, solidity,
//      extent, raw corners and defect count into a ShapeClass label.
// All functions are pure and host-testable; buffers are caller-owned.
// ============================================================================

const char* vision_shape_class_name(ShapeClass cls);

// ----------------------------------------------------------------------------
// Decision tree classifier. Inputs are scale-free descriptor values; the
// thresholds live in config/thresholds.h so field tuning never touches code.
// ----------------------------------------------------------------------------
ShapeClass vision_classify_shape(
    float circularity,     // 4*pi*A/P^2, clamped 0..1
    float solidity,        // A / hull_area, clamped 0..1
    float extent,          // A / bbox_area, clamped 0..1
    float aspect,          // bbox max(w,h)/min(w,h) >= 1
    uint8_t raw_corners,   // DP vertices on the RAW contour
    uint8_t defect_count); // convexity defects above depth floor

// ============================================================================
// LABEL-MASK CONTOUR API (consumed by VisionPipeline::extractBlobs)
// ============================================================================

constexpr uint16_t SHAPE_MAX_CONTOUR_PTS = Config::VISION_CONTOUR_POINTS_MAX;

struct ContourPoint {
    uint16_t x = 0;
    uint16_t y = 0;
};

// Traces the boundary of ONE label in a multi-label mask (pixel == `label`
// is foreground). (min_x,min_y) must be that blob's topmost-leftmost pixel.
// Returns contour length written to out_pts, or 0 on failure.
uint16_t shape_trace_boundary(
    const uint8_t* label_mask, uint16_t w, uint16_t h, uint8_t label,
    uint16_t min_x, uint16_t min_y,
    uint16_t max_x, uint16_t max_y,
    ContourPoint* out_pts, uint16_t cap);

// Corner count on an ordered contour via smoothed turn-angle thresholding:
// a vertex is a corner when the direction change over a small neighborhood
// exceeds angle_threshold_deg. Robust to single-pixel raster staircases.
uint8_t shape_corner_count(
    const ContourPoint* pts, uint16_t n,
    float angle_threshold_deg);

// Maximum concavity depth of the contour inside its convex hull, normalized
// by the contour bbox diagonal (0 = convex .. ~0.5+ = deep star arms).
float shape_concavity_depth(
    const ContourPoint* pts, uint16_t n,
    const float* hull_fx, const float* hull_fy, uint8_t hull_n);

} // namespace RobofestDrone
