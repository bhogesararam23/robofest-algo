#include "hal_optical_flow.h"
#include "hal_system.h"

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_optical_flow_initialized = false;
}

bool hal_optical_flow_init() {
    // Note: Replace with real SPI/UART register initialization for PMW3901/CX-OF.
    s_optical_flow_initialized = true;
    hal_log("[HAL_OPTICAL_FLOW] Optical flow stub initialized (safe default).");
    return true;
}

Types::OpticalFlowSample hal_optical_flow_read() {
    // Note: Replace with real sensor burst register read on hardware.
    Types::OpticalFlowSample sample;
    sample.valid = false;
    sample.pixel_shift_x = 0.0f;
    sample.pixel_shift_y = 0.0f;
    sample.quality = 0.0f;
    sample.timestamp_ms = hal_millis();
    return sample;
}

bool hal_optical_flow_is_healthy() {
    return s_optical_flow_initialized;
}

} // namespace Hal
} // namespace RobofestDrone
