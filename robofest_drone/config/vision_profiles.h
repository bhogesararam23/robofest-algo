#pragma once

// ============================================================================
// DATA-DRIVEN VISION MARKER PROFILE TABLE (Step 1)
// ----------------------------------------------------------------------------
// Single source of truth for all marker color/shape profiles. Adding a new
// marker color or shape only requires appending one row here - the pipeline
// loops over every enabled entry.
//
// FIRMWARE EXPORT: cv_laptop_protocol.py ([E] key) writes
// vision_profiles_generated.h next to this file with calibrated values. When
// present it overrides the built-in defaults via ROBOFEST_VISION_PROFILE_TABLE_DATA.
//
// HSV scale: H 0-180, S/V 0-255 - identical to OpenCV's default 8-bit range,
// which matches the firmware's inline RGB->HSV conversion.
// ============================================================================

#include <stdint.h>
#include "thresholds.h"

#if defined(__has_include)
#if __has_include("vision_profiles_generated.h")
#include "vision_profiles_generated.h"
#endif
#endif

namespace RobofestDrone {
namespace Config {

struct HsvBandDef {
    uint8_t h_min;
    uint8_t h_max; // < h_min means hue wraparound band (e.g. red: 170..10)
    uint8_t s_min;
    uint8_t s_max;
    uint8_t v_min;
    uint8_t v_max;
};

struct VisionShapeGatesDef {
    float aspect_min;    // normalized bbox aspect ratio max(w,h)/min(w,h), >= 1.0
    float aspect_max;
    float extent_min;    // blob area / bbox area (square ~1.0, circle ~0.785)
    float extent_max;
    float solidity_min;  // blob area / convex hull area (convex ~1.0, star ~0.5)
    uint8_t corners_min; // hull polyline vertex count gate; min==0 disables corner gating
    uint8_t corners_max;
};

struct VisionProfileDef {
    uint8_t marker_type_id; // Types::VisionMarkerType numeric value (wire format)
    bool enabled;
    HsvBandDef primary;     // "sunny" / bright-condition calibration
    bool has_alt_band;
    HsvBandDef alt;         // "overcast" / dim-condition calibration
    float min_area_px;
    float max_area_px;
    float circularity_min;
    float confidence_bias;
    uint16_t expected_marker_area_px;
    VisionShapeGatesDef shape;
};

// --- Shape gate presets (Step 8) ---
static constexpr VisionShapeGatesDef SHAPE_GATES_ANY = {
    1.00f, 10.00f, 0.00f, 1.05f, 0.00f, 0, 255};
static constexpr VisionShapeGatesDef SHAPE_GATES_CIRCLE = {
    1.00f, 1.35f, 0.62f, 1.05f, 0.88f, 0, 255};
static constexpr VisionShapeGatesDef SHAPE_GATES_SQUARE = {
    1.00f, 1.25f, 0.78f, 1.05f, 0.92f, 3, 5};
static constexpr VisionShapeGatesDef SHAPE_GATES_TRIANGLE = {
    1.00f, 2.20f, 0.40f, 0.90f, 0.88f, 2, 4};
static constexpr VisionShapeGatesDef SHAPE_GATES_PENTAGON = {
    1.00f, 1.50f, 0.55f, 1.00f, 0.90f, 4, 6};
static constexpr VisionShapeGatesDef SHAPE_GATES_STAR = {
    1.00f, 1.60f, 0.25f, 0.65f, 0.45f, 7, 12};

#ifndef ROBOFEST_VISION_PROFILE_TABLE_DATA
// ============================================================================
// BUILT-IN DEFAULT PROFILE TABLE (calibrate via cv_laptop_protocol.py, then
// export to replace these seeds). Row order matters: bands are evaluated
// first-match-wins, so narrower bands must come before broader overlapping ones.
// ============================================================================
static constexpr VisionProfileDef VISION_PROFILE_TABLE[] = {

    // 1. On-ground mine: RED circular marker. Hue wraps around 180/0.
    {1, true,
     {170, 10, 100, 255, 100, 255}, true,
     {165, 15, 70, 255, 60, 190},
     Config::BLOB_AREA_MIN_PX, Config::BLOB_AREA_MAX_PX,
     Config::CIRCULARITY_MIN, 0.0f,
     static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX),
     SHAPE_GATES_CIRCLE},

    // 2. Buried surface marker: YELLOW circular marker.
    {2, true,
     {20, 42, 110, 255, 110, 255}, true,
     {18, 45, 85, 255, 75, 210},
     Config::BLOB_AREA_MIN_PX, Config::BLOB_AREA_MAX_PX,
     Config::CIRCULARITY_MIN, 5.0f,
     static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX),
     SHAPE_GATES_CIRCLE},

    // 3. ORANGE marker (between red wrap-end and yellow start).
    {3, true,
     {9, 24, 130, 255, 120, 255}, true,
     {8, 26, 95, 255, 80, 215},
     Config::BLOB_AREA_MIN_PX, Config::BLOB_AREA_MAX_PX,
     Config::CIRCULARITY_MIN, 0.0f,
     static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX),
     SHAPE_GATES_ANY},

    // 4. GREEN marker.
    {4, true,
     {35, 86, 80, 255, 80, 255}, true,
     {33, 88, 60, 255, 55, 220},
     Config::BLOB_AREA_MIN_PX, Config::BLOB_AREA_MAX_PX,
     Config::CIRCULARITY_MIN, 0.0f,
     static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX),
     SHAPE_GATES_ANY},

    // 5. CYAN marker.
    {5, true,
     {87, 99, 90, 255, 90, 255}, true,
     {85, 101, 65, 255, 65, 225},
     Config::BLOB_AREA_MIN_PX, Config::BLOB_AREA_MAX_PX,
     Config::CIRCULARITY_MIN, 0.0f,
     static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX),
     SHAPE_GATES_ANY},

    // 6. BLUE marker (V floor raised: dark blue is easily shadow-confused).
    {6, true,
     {100, 131, 85, 255, 70, 255}, true,
     {98, 133, 60, 255, 50, 230},
     Config::BLOB_AREA_MIN_PX, Config::BLOB_AREA_MAX_PX,
     Config::CIRCULARITY_MIN, 0.0f,
     static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX),
     SHAPE_GATES_ANY},

    // 7. PURPLE marker (before pink: purple owns the 132-159 sliver).
    {7, true,
     {132, 159, 70, 255, 60, 255}, true,
     {130, 161, 50, 255, 45, 235},
     Config::BLOB_AREA_MIN_PX, Config::BLOB_AREA_MAX_PX,
     Config::CIRCULARITY_MIN, 0.0f,
     static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX),
     SHAPE_GATES_ANY},

    // 8. PINK marker.
    {8, true,
     {160, 175, 60, 255, 110, 255}, true,
     {158, 177, 40, 255, 80, 235},
     Config::BLOB_AREA_MIN_PX, Config::BLOB_AREA_MAX_PX,
     Config::CIRCULARITY_MIN, 0.0f,
     static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX),
     SHAPE_GATES_ANY},

    // 9. WHITE marker: any hue, low saturation, high value.
    {9, true,
     {0, 180, 0, 60, 150, 255}, true,
     {0, 180, 0, 80, 120, 235},
     Config::BLOB_AREA_MIN_PX, Config::BLOB_AREA_MAX_PX,
     Config::CIRCULARITY_MIN, 0.0f,
     static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX),
     SHAPE_GATES_ANY},

    // 10. BLACK marker: any hue, low value (watch for shadows during calibration).
    {10, true,
     {0, 180, 0, 140, 0, 55}, true,
     {0, 180, 0, 150, 0, 70},
     Config::BLOB_AREA_MIN_PX, Config::BLOB_AREA_MAX_PX,
     Config::CIRCULARITY_MIN, 0.0f,
     static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX),
     SHAPE_GATES_ANY},
};
#endif // !ROBOFEST_VISION_PROFILE_TABLE_DATA

#ifdef ROBOFEST_VISION_PROFILE_TABLE_DATA
static constexpr VisionProfileDef VISION_PROFILE_TABLE[] =
    ROBOFEST_VISION_PROFILE_TABLE_DATA;
#endif

constexpr uint8_t VISION_PROFILE_COUNT =
    static_cast<uint8_t>(sizeof(VISION_PROFILE_TABLE) / sizeof(VISION_PROFILE_TABLE[0]));

static_assert(VISION_PROFILE_COUNT <= VISION_PROFILE_MAX,
              "VISION_PROFILE_TABLE exceeds VISION_PROFILE_MAX capacity");

} // namespace Config
} // namespace RobofestDrone
