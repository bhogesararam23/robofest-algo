#pragma once

#include <stdint.h>
#include "../src/types.h"

namespace RobofestDrone {
namespace Hal {

// ============================================================================
// LDROBOT LD06 2D LIDAR DRIVER HAL
// ============================================================================
// Target Hardware: LDRobot LD06 360-degree TOF 2D LiDAR Scanner
// - Range: 0.02m to 12.0m
// - Sampling Frequency: 4500 Hz
// - Scan Rate: 5 to 13 Hz (nominal 10 Hz)
// - Interface: UART @ 230400 baud, 8N1
// - Role: Forward & 360-degree obstacle detection and perimeter safety avoidance

bool hal_lidar_init();
bool hal_lidar_is_healthy();
Types::LidarObstacleSample hal_lidar_read_front_sector();

// Set simulated/mock obstacle distance for bench/offline testing
void hal_lidar_set_mock_distance(float distance_m, float angle_deg);
void hal_lidar_clear_mock();

} // namespace Hal
} // namespace RobofestDrone
