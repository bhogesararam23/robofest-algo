#pragma once

#include <stdint.h>

namespace RobofestDrone {
namespace Hal {

// ============================================================================
// CAMERA PIXEL FORMATS
// ============================================================================

enum class PixelFormat : uint8_t {
    PIXEL_FORMAT_UNKNOWN = 0,
    PIXEL_FORMAT_GRAY8,
    PIXEL_FORMAT_RGB565,
    PIXEL_FORMAT_RGB888,
    PIXEL_FORMAT_HSV888
};


// ============================================================================
// CAMERA FRAME STRUCTURE
// ============================================================================

struct CameraFrame {
    uint16_t width = 0;
    uint16_t height = 0;
    uint32_t timestamp_ms = 0;
    PixelFormat format = PixelFormat::PIXEL_FORMAT_UNKNOWN;
    uint32_t buffer_size = 0;
    uint8_t* data = nullptr;
};


// ============================================================================
// HAL CAMERA INTERFACE
// ============================================================================

// NOTE: This HAL interface is currently an embedded stub for desktop/compilation tests.
// The physical ESP32-S3 camera driver (e.g. esp_camera via DVP/I2S/DMA for OV2640/OV5640)
// must be connected here for target hardware deployments.

bool hal_camera_init();
bool hal_camera_is_healthy();
bool hal_camera_get_frame(CameraFrame& frame);
bool hal_camera_read_frame();
uint32_t hal_camera_get_last_frame_time_ms();
bool hal_camera_is_stub(); // Runtime marker: true for stub, false for real camera

// Camera exposure and white-balance control
// On real hardware, these write OV5640 sensor registers via SCCB/I2C.
// Locking exposure prevents auto-adjustment from invalidating calibrated HSV values.
void hal_camera_set_auto_exposure(bool enabled);
void hal_camera_set_exposure(int32_t value);
void hal_camera_set_auto_whitebalance(bool enabled);

} // namespace Hal
} // namespace RobofestDrone
