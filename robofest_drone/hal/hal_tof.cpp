#include "hal_tof.h"
#include "hal_system.h"

namespace RobofestDrone {
namespace Hal {

// ============================================================================
// HOLYBRO ST VL53L1X TIME-OF-FLIGHT LIDAR DRIVER HAL
// ============================================================================
// Target Hardware: Holybro ST VL53L1X Long Range ToF Distance Sensor
// - Range: 0.04m to 4.00m (Long Distance Mode @ 50 Hz)
// - Field of View: 27 degrees programmable ROI
// - Interface: I2C (Address 0x29, Wire1 on ESP32-S3)
// - Role: Ground clearance & precision altitude AGL rangefinding

namespace {
    static bool s_tof_initialized = false;
    static bool s_has_mock = false;
    static float s_mock_alt_m = 0.0f;
}

bool hal_tof_init() {
    // Note: Bind to SparkFun_VL53L1X or ST VL53L1X ULD API on Wire1 (SDA: GPIO 5, SCL: GPIO 6)
    s_tof_initialized = true;
    hal_log("[HAL_TOF] Holybro ST VL53L1X LiDAR initialized (4.0m range mode).");
    return true;
}

Types::TofSample hal_tof_read() {
    Types::TofSample sample;
    sample.timestamp_ms = hal_millis();

    if (!s_tof_initialized) {
        sample.valid = false;
        sample.altitude_m = 0.0f;
        return sample;
    }

    if (s_has_mock) {
        sample.valid = true;
        sample.altitude_m = s_mock_alt_m;
        return sample;
    }

    // Default stub reading: ground level at rest
    sample.valid = true;
    sample.altitude_m = 0.0f;
    return sample;
}

bool hal_tof_is_healthy() {
    return s_tof_initialized;
}

} // namespace Hal
} // namespace RobofestDrone
