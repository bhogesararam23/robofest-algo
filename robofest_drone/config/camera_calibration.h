#pragma once

// ============================================================================
// Camera Calibration configuration (REQ-DER-120 / item 20)
// ============================================================================
// Values are intended to be generated offline by sim/calibrate_camera.py from a
// standard chessboard pattern.  Until that is done the following defaults are
// deliberately INVALID (valid==false) so that undistort() degrades to an identity
// pass‑through and the rest of the pipeline functions unchanged.
//
// Format: fx, fy (focal lengths in pixels), cx, cy (principal point, pixel offsets),
// k1, k2 (radial distortion), p1, p2 (tangential distortion).
// ============================================================================

#ifndef CAMERA_CALIBRATION_H_INCLUDED
#define CAMERA_CALIBRATION_H_INCLUDED

// --- Host / unit-test defaults (do NOT change these values in production) ---
#if !defined(ARDUINO)
// On the host / test binary we start with identity / invalid calibration.
namespace RobofestDrone {
    namespace Config {
        constexpr bool CAMERA_CALIB_VALID = false;
        constexpr float CAMERA_FX = 0.0f;
        constexpr float CAMERA_FY = 0.0f;
        constexpr float CAMERA_CX = 0.0f;
        constexpr float CAMERA_CY = 0.0f;
        constexpr float CAMERA_K1 = 0.0f;
        constexpr float CAMERA_K2 = 0.0f;
        constexpr float CAMERA_P1 = 0.0f;
        constexpr float CAMERA_P2 = 0.0f;
    }
}
// On the real drone firmware these will be populated at compile‑time from the
// calibrated JSON / binary blob that the ground‑station uploads.  For now use
// the same safe / identity values.
#else
namespace RobofestDrone {
    namespace Config {
        constexpr bool CAMERA_CALIB_VALID = false;
        constexpr float CAMERA_FX = 0.0f;
        constexpr float CAMERA_FY = 0.0f;
        constexpr float CAMERA_CX = 0.0f;
        constexpr float CAMERA_CY = 0.0f;
        constexpr float CAMERA_K1 = 0.0f;
        constexpr float CAMERA_K2 = 0.0f;
        constexpr float CAMERA_P1 = 0.0f;
        constexpr float CAMERA_P2 = 0.0f;
    }
}
#endif

// --- Convenience macro used by undistort to decide whether the map is valid ---
#define CAMERA_CALIB_VALID_() (RobofestDrone::Config::CAMERA_CALIB_VALID)

// ----------------------------------------------------------------------------
// Helper: full-frame resolution; may be overridden per‑platform if needed.
#ifndef CAMERA_DEFAULT_WIDTH
#define CAMERA_DEFAULT_WIDTH 320
#endif
#ifndef CAMERA_DEFAULT_HEIGHT
#define CAMERA_DEFAULT_HEIGHT 240
#endif

// ----------------------------------------------------------------------------
// Flag to enable the undistortion stage in the pipeline (useful for A/B testing).
// ----------------------------------------------------------------------------
#ifndef VISION_UNDISTORT_ENABLED
#define VISION_UNDISTORT_ENABLED 0
#endif

#endif // CAMERA_CALIBRATION_H_INCLUDED