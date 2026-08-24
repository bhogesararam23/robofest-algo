#include "threat_arbiter.h"
#include "mission_config.h"
#include <cmath>
#include <algorithm>

namespace RobofestDrone {

namespace {
constexpr float TTC_MAX_S = 30.0f;
constexpr float EPS = 1e-6f;
}

// ============================================================================
// OBSTACLE TIME-TO-COLLISION (item 14)
// ============================================================================

float threat_time_to_collision(const Types::RelativeObstacle& obs) {
    if (!obs.valid) return -1.0f;

    const float vx = obs.rel_vx;
    const float vy = obs.rel_vy;
    const float speed2 = vx * vx + vy * vy;
    if (speed2 < EPS) return -1.0f; // static relative geometry: no closing motion

    // Time of closest approach for p(t) = p0 + v*t.
    const float t_cae =
        -(obs.rel_x * vx + obs.rel_y * vy) / speed2;
    if (t_cae <= 0.0f) return -1.0f; // diverging

    // Distance at closest approach.
    const float cx = obs.rel_x + vx * t_cae;
    const float cy = obs.rel_y + vy * t_cae;
    const float miss = std::sqrt(cx * cx + cy * cy);

    if (miss > 1.0f) return -1.0f; // closest pass outside 1 m envelope: no threat

    // Time until inside the immediate radius (linear approximation along the
    // closing range rate), bounded by time-of-closest-approach.
    const float range = std::sqrt(obs.rel_x * obs.rel_x + obs.rel_y * obs.rel_y);
    const float closing = -(obs.rel_x * vx + obs.rel_y * vy) /
                          (range + EPS); // positive when approaching
    if (closing <= EPS) return -1.0f;

    const float range_now =
        (range <= Config::OBSTACLE_IMMEDIATE_RADIUS_M)
            ? 0.0f
            : range - Config::OBSTACLE_IMMEDIATE_RADIUS_M;
    float ttc = range_now / closing;
    if (ttc > TTC_MAX_S) ttc = TTC_MAX_S;
    return ttc;
}

ThreatVerdict threat_evaluate(const Types::RelativeObstacle& obs) {
    ThreatVerdict out;

    if (!obs.valid) return out;

    const float range = std::sqrt(obs.rel_x * obs.rel_x + obs.rel_y * obs.rel_y);

    // Immediate-radius rule dominates.
    if (range <= Config::OBSTACLE_IMMEDIATE_RADIUS_M) {
        out.evade = true;
        out.urgency = 1.0f;
        if (range > EPS) {
            out.avoid_dir_x = obs.rel_x / range;
            out.avoid_dir_y = obs.rel_y / range;
        } else {
            out.avoid_dir_x = 1.0f;
        }
        return out;
    }

    const float ttc = threat_time_to_collision(obs);
    if (ttc >= 0.0f && ttc <= Config::OBSTACLE_TTC_CRITICAL_S) {
        out.evade = true;
        out.urgency = std::min(
            1.0f, Config::OBSTACLE_TTC_CRITICAL_S / (ttc + 0.25f));

        // Avoid perpendicular to closing velocity (left side by convention;
        // caller may mirror based on field bounds).
        const float vlen = std::sqrt(obs.rel_vx * obs.rel_vx +
                                     obs.rel_vy * obs.rel_vy);
        if (vlen > EPS) {
            out.avoid_dir_x = -obs.rel_vy / vlen;
            out.avoid_dir_y = obs.rel_vx / vlen;
        } else {
            const float inv = 1.0f / range;
            out.avoid_dir_x = -obs.rel_x * inv;
            out.avoid_dir_y = -obs.rel_y * inv;
        }
    }

    return out;
}

// ============================================================================
// MOVING-TARGET LANDING (item 6n)
// ============================================================================

LandingPredict landing_predict_intercept(
    const Types::TargetKinematicSample& older,
    const Types::TargetKinematicSample& newer,
    float horizon_s) {

    LandingPredict out;

    if (!newer.valid) return out;

    if (!older.valid || newer.timestamp_ms <= older.timestamp_ms) {
        // No usable history: predict a stationary intercept at last position.
        out.valid = true;
        out.intercept_x = newer.field_x;
        out.intercept_y = newer.field_y;
        out.target_speed_mps = 0.0f;
        return out;
    }

    const float dt_s =
        static_cast<float>(newer.timestamp_ms - older.timestamp_ms) / 1000.0f;
    if (dt_s < 0.05f) {
        out.valid = true;
        out.intercept_x = newer.field_x;
        out.intercept_y = newer.field_y;
        return out;
    }

    const float vx = (newer.field_x - older.field_x) / dt_s;
    const float vy = (newer.field_y - older.field_y) / dt_s;
    const float speed = std::sqrt(vx * vx + vy * vy);

    // Clamp horizon for fast movers so predictions stay physically sane.
    const float h = (speed > 1.5f)
        ? std::min(horizon_s, Config::LANDING_PREDICT_HORIZON_S * 0.6f)
        : horizon_s;

    out.valid = true;
    out.intercept_x = newer.field_x + vx * h;
    out.intercept_y = newer.field_y + vy * h;
    out.target_speed_mps = speed;
    return out;
}

float landing_descent_rate_mps(float altitude_m, float target_speed_mps) {
    // Baseline step descent from mission config, modulated by closure need:
    //   - far above ground -> descend briskly toward predicted intercept
    //   - fast-moving target -> hold altitude longer (shallow approach)
    const float base = Config::LANDING_ALTITUDE_STEP_M;
    const float closure = std::min(1.5f, std::max(0.3f, altitude_m / 2.0f));
    const float speed_penalty = 1.0f / (1.0f + target_speed_mps);
    return base * closure * speed_penalty;
}

bool landing_servo_velocity(
    float target_pixel_x,
    float target_pixel_y,
    float image_center_x,
    float image_center_y,
    float& out_vx_mps,
    float& out_vy_mps) {

    const float err_x = target_pixel_x - image_center_x;
    const float err_y = target_pixel_y - image_center_y;
    const float err_mag = std::sqrt(err_x * err_x + err_y * err_y);

    if (err_mag <= Config::LANDING_SERVO_TOLERANCE_PX) {
        out_vx_mps = 0.0f;
        out_vy_mps = 0.0f;
        return false; // centered: pure vertical descent
    }

    out_vx_mps = err_x * Config::LANDING_SERVO_GAIN_PX_TO_MPS;
    out_vy_mps = err_y * Config::LANDING_SERVO_GAIN_PX_TO_MPS;

    // Cap lateral servo authority to keep descent stable near ground effect.
    const float cap = 0.4f;
    const float mag = std::sqrt(out_vx_mps * out_vx_mps +
                                out_vy_mps * out_vy_mps);
    if (mag > cap) {
        const float scale = cap / mag;
        out_vx_mps *= scale;
        out_vy_mps *= scale;
    }
    return true;
}

} // namespace RobofestDrone
