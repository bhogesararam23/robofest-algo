#pragma once

// ============================================================================
// DEFAULT (UNCALIBRATED) CAMERA INTRINSICS - PLACEHOLDER
// ----------------------------------------------------------------------------
// This placeholder keeps the build green before a real calibration session.
// It encodes the same pinhole assumption as the legacy H_FOV/V_FOV constants
// but is marked INVALID so undistort() degrades to an identity pass-through
// and the rest of the pipeline functions unchanged.
//
// REPLACE by running:  python sim/calibrate_camera.py --camera 0 --rows 6 --cols 9
// (REQ-DER-120, item 20)
// ============================================================================

#include <stdint.h>

namespace RobofestDrone {
namespace Config {

constexpr bool   CAM_INTRINSICS_VALID   = false; // invalid until real calibration
constexpr uint16_t CAM_CALIB_WIDTH      = 320u;
constexpr uint16_t CAM_CALIB_HEIGHT     = 240u;

constexpr float  CAM_FX_PX              = 277.1281f;
constexpr float  CAM_FY_PX              = 289.6890f;
constexpr float  CAM_CX_PX              = 160.0000f;
constexpr float  CAM_CY_PX              = 120.0000f;

constexpr float  CAM_K1                 = 0.0f;
constexpr float  CAM_K2                 = 0.0f;
constexpr float  CAM_P1                 = 0.0f;
constexpr float  CAM_P2                 = 0.0f;

} // namespace Config
} // namespace RobofestDrone
