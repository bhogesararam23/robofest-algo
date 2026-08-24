#pragma once

#include <stdint.h>
#include "types.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

// ============================================================================
// THREAT ARBITER (REQ-DER-114 item 14 + moving-target landing item 6n)
// ----------------------------------------------------------------------------
// Pure kinematics shared by the safety layer and the landing controller:
//
//   - Obstacle time-to-collision from relative position/velocity, used to
//     override path following when a dynamic obstacle (human, animal,
//     vehicle) crosses the safety envelope.
//
//   - Constant-velocity Kalman-lite predictor for a moving ground target:
//     projects where the target WILL be so descent aims at the intercept
//     point, plus terminal pixel-error servo law for centered touchdown.
//
// All functions are pure and host-testable; no state, no allocation.
// ============================================================================

struct RelativeObstacle {
    bool valid = false;
    float rel_x = 0.0f;    // obstacle position relative to drone (body frame)
    float rel_y = 0.0f;
    float rel_vx = 0.0f;   // relative velocity (obstacle - drone), m/s
    float rel_vy = 0.0f;
};
// (Struct defined in types.h to avoid duplicate definitions.)

// Time to closest approach / collision along current relative motion.
// Returns -1 when paths diverge (no future collision). Clamped to 30 s.
float threat_time_to_collision(const Types::RelativeObstacle& obs);

// Full evasion verdict: combines TTC with the immediate-radius rule and
// produces an avoidance direction perpendicular to the closing velocity.
struct ThreatVerdict {
    bool evade = false;
    float urgency = 0.0f;      // 0..1 (1 = immediate)
    float avoid_dir_x = 0.0f;  // unit avoidance vector (body frame)
    float avoid_dir_y = 0.0f;
};
ThreatVerdict threat_evaluate(const Types::RelativeObstacle& obs);

// ----------------------------------------------------------------------------
// Moving-target landing (item 6n)
// ----------------------------------------------------------------------------

struct TargetKinematicSample {
    bool valid = false;
    uint32_t timestamp_ms = 0;
    float field_x = 0.0f;
    float field_y = 0.0f;
};


// Constant-velocity target predictor: given two recent samples and a horizon,
// outputs predicted intercept position. Handles stale/invalid samples by
// falling back to the last known position.
struct LandingPredict {
    bool valid = false;
    float intercept_x = 0.0f;
    float intercept_y = 0.0f;
    float target_speed_mps = 0.0f;
};
LandingPredict landing_predict_intercept(
    const Types::TargetKinematicSample& older,
    const Types::TargetKinematicSample& newer,
    float horizon_s);

// Dynamic descent-rate law: faster targets -> shallower, later descent so the
// drone arrives as the target does rather than hovering above a moving point.
float landing_descent_rate_mps(float altitude_m, float target_speed_mps);

// Terminal visual-servo: converts target pixel offset in the down camera into
// body-frame velocity commands for micro-adjustments before touchdown.
// Returns false when already inside tolerance (caller holds descent).
bool landing_servo_velocity(
    float target_pixel_x,
    float target_pixel_y,
    float image_center_x,
    float image_center_y,
    float& out_vx_mps,
    float& out_vy_mps);

} // namespace RobofestDrone
