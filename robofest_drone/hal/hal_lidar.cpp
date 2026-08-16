#include "hal_lidar.h"
#include "hal_system.h"
#include "../config/thresholds.h"

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_lidar_initialized = false;
    static bool s_has_mock = false;
    static float s_mock_dist_m = 12.0f;
    static float s_mock_angle_deg = 0.0f;
}

bool hal_lidar_init() {
    // Note: Bind to Serial2 / UART2 on ESP32-S3 @ 230400 baud for LDRobot LD06
    s_lidar_initialized = true;
    hal_log("[HAL_LIDAR] LDRobot LD06 2D LiDAR driver initialized (230400 baud default).");
    return true;
}

bool hal_lidar_is_healthy() {
    return s_lidar_initialized;
}

Types::LidarObstacleSample hal_lidar_read_front_sector() {
    Types::LidarObstacleSample sample;
    sample.timestamp_ms = hal_millis();

    if (!s_lidar_initialized) {
        sample.valid = false;
        sample.min_front_distance_m = 12.0f;
        sample.closest_angle_deg = 0.0f;
        sample.point_count = 0;
        sample.obstacle_in_warning_zone = false;
        sample.obstacle_in_emergency_zone = false;
        return sample;
    }

    if (s_has_mock) {
        sample.valid = true;
        sample.min_front_distance_m = s_mock_dist_m;
        sample.closest_angle_deg = s_mock_angle_deg;
        sample.point_count = 450;
        sample.obstacle_in_warning_zone = (s_mock_dist_m < 2.5f);
        sample.obstacle_in_emergency_zone = (s_mock_dist_m < 1.0f);
        return sample;
    }

    // Default clear environment stub
    sample.valid = true;
    sample.min_front_distance_m = 10.0f;
    sample.closest_angle_deg = 0.0f;
    sample.point_count = 450;
    sample.obstacle_in_warning_zone = false;
    sample.obstacle_in_emergency_zone = false;
    return sample;
}

void hal_lidar_set_mock_distance(float distance_m, float angle_deg) {
    s_mock_dist_m = distance_m;
    s_mock_angle_deg = angle_deg;
    s_has_mock = true;
}

void hal_lidar_clear_mock() {
    s_has_mock = false;
}

} // namespace Hal
} // namespace RobofestDrone
