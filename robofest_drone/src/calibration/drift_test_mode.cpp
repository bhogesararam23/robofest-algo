#include "drift_test_mode.h"
#include "../../hal/hal_system.h"
#include <cmath>
#include <cstdio>

namespace RobofestDrone {

DriftTestMode::DriftTestMode() {
    init();
}

void DriftTestMode::init() {
    test_active_ = false;
    start_time_ms_ = 0;
    last_log_ms_ = 0;
    last_imu_log_ms_ = 0;
    results_ = DriftTestResults();
}

void DriftTestMode::reset() {
    init();
}

bool DriftTestMode::startTest(Localization& loc, uint32_t now_ms) {
    // Reset localization origin to zero
    loc.reset();
    test_active_ = true;
    start_time_ms_ = now_ms;
    last_log_ms_ = now_ms;
    last_imu_log_ms_ = now_ms;

    results_ = DriftTestResults();
    results_.start_x = 0.0f;
    results_.start_y = 0.0f;
    last_pose_ = loc.getPose();

    Hal::hal_log("[DRIFT_TEST] Dead-reckoning test started at origin (0, 0).");
    return true;
}

void DriftTestMode::update(Localization& loc, float altitude_m, const Types::AttitudeSample& attitude, uint32_t now_ms) {
    if (!test_active_) return;

    Types::Pose2D cur_pose = loc.getPose();
    float dx = cur_pose.x - last_pose_.x;
    float dy = cur_pose.y - last_pose_.y;
    float step_dist = std::sqrt(dx * dx + dy * dy);
    results_.total_distance_m += step_dist;
    last_pose_ = cur_pose;

    // Log position every 1 second (1000 ms)
    if (now_ms - last_log_ms_ >= 1000UL) {
        last_log_ms_ = now_ms;
        uint32_t elapsed_s = (now_ms - start_time_ms_) / 1000UL;
        float cur_drift = std::sqrt(cur_pose.x * cur_pose.x + cur_pose.y * cur_pose.y);

        char buf[160];
        std::snprintf(buf, sizeof(buf),
            "[DRIFT_TEST] T:%lus Pose:(%.3f, %.3f)m Alt:%.2fm Yaw:%.1fdeg Drift:%.3fm Dist:%.3fm",
            static_cast<unsigned long>(elapsed_s),
            cur_pose.x, cur_pose.y,
            altitude_m, cur_pose.yaw_deg,
            cur_drift, results_.total_distance_m);
        Hal::hal_log(buf);
    }

    // Periodic IMU bias check logging
    if (now_ms - last_imu_log_ms_ >= 5000UL) {
        last_imu_log_ms_ = now_ms;
        logImuBiasCheck(attitude, now_ms);
    }
}

void DriftTestMode::endTest(Localization& loc, uint32_t now_ms) {
    if (!test_active_) return;

    Types::Pose2D final_pose = loc.getPose();
    results_.final_x = final_pose.x;
    results_.final_y = final_pose.y;
    results_.elapsed_time_ms = now_ms - start_time_ms_;

    // Calculate loop closure Euclidean drift distance
    results_.loop_closure_drift_m = std::sqrt(final_pose.x * final_pose.x + final_pose.y * final_pose.y);
    float elapsed_s = (results_.elapsed_time_ms > 0) ? (results_.elapsed_time_ms / 1000.0f) : 1.0f;
    results_.drift_rate_mps = results_.loop_closure_drift_m / elapsed_s;
    results_.completed = true;
    test_active_ = false;

    char buf[160];
    std::snprintf(buf, sizeof(buf),
        "[DRIFT_TEST] Complete! Duration:%.1fs TotalDist:%.2fm FinalDrift:%.3fm Rate:%.4fm/s",
        elapsed_s,
        results_.total_distance_m,
        results_.loop_closure_drift_m,
        results_.drift_rate_mps);
    Hal::hal_log(buf);
}

void DriftTestMode::logFocalLengthCheck(int16_t pixel_shift_x, int16_t pixel_shift_y, float tof_alt_m, uint32_t dt_us) {
    float dt_s = (dt_us > 0) ? (dt_us / 1000000.0f) : 0.02f;
    float px_speed = std::sqrt(static_cast<float>(pixel_shift_x * pixel_shift_x + pixel_shift_y * pixel_shift_y)) / dt_s;
    char buf[160];
    std::snprintf(buf, sizeof(buf),
        "[FOCAL_CAL] PxShift:(%d, %d) Alt:%.3fm dt:%luus Speed:%.1fpx/s (Expected Focal: %.1fpx)",
        pixel_shift_x, pixel_shift_y, tof_alt_m,
        static_cast<unsigned long>(dt_us),
        px_speed,
        Config::OPTICAL_FLOW_FOCAL_LENGTH_PX);
    Hal::hal_log(buf);
}

void DriftTestMode::logImuBiasCheck(const Types::AttitudeSample& attitude, uint32_t now_ms) {
    (void)now_ms;
    char buf[160];
    std::snprintf(buf, sizeof(buf),
        "[IMU_CAL] Rest Attitude: Roll:%.2fdeg Pitch:%.2fdeg Yaw:%.2fdeg (Armed:%s)",
        attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg,
        attitude.armed ? "YES" : "NO");
    Hal::hal_log(buf);
}

} // namespace RobofestDrone
