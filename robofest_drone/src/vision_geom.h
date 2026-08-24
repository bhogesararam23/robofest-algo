#pragma once

#include <stdint.h>

namespace RobofestDrone {

// ============================================================================
// SHARED VISION GEOMETRY TYPES
// ----------------------------------------------------------------------------
// Minimal dependency-free definitions shared by the vision pipeline, the
// shape-analysis module, and host unit tests (avoids header cycles).
// ============================================================================

struct VisionPoint {
    float x = 0.0f;
    float y = 0.0f;
};

constexpr uint8_t VISION_SHAPE_CLASS_COUNT = 8;

enum class ShapeClass : uint8_t {
    SHAPE_UNKNOWN = 0,
    SHAPE_CIRCLE,
    SHAPE_TRIANGLE,
    SHAPE_QUADRILATERAL,
    SHAPE_PENTAGON,
    SHAPE_HEXAGON_PLUS,
    SHAPE_STAR_CONCAVE,
    SHAPE_BAR
};

} // namespace RobofestDrone
