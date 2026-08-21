#include "hal_camera.h"
#include "hal_system.h"
#include <cstdio>

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_camera_initialized = false;
    static bool s_stub_mode = true; // True for stub; real driver should set false
    static uint32_t s_last_frame_ms = 0;
}

// NOTE: This HAL interface is currently an embedded stub for desktop/compilation tests.
// The physical ESP32-S3 camera driver (e.g. esp_camera via DVP/I2S/DMA for OV2640/OV5640)
// must be connected here for target hardware deployments.

bool hal_camera_init() {
    s_camera_initialized = true;
    s_stub_mode = true; // Stub mode by default; real driver should set false
    s_last_frame_ms = hal_millis();
    hal_log("[HAL_CAMERA] Camera stub initialized (safe default).");
    return true;
}

bool hal_camera_read_frame() {
    if (!s_camera_initialized) {
        return false;
    }
    s_last_frame_ms = hal_millis();
    return true;
}

bool hal_camera_get_frame(CameraFrame& frame) {
    if (!s_camera_initialized) {
        frame.width = 0;
        frame.height = 0;
        frame.timestamp_ms = 0;
        frame.format = PixelFormat::PIXEL_FORMAT_UNKNOWN;
        frame.buffer_size = 0;
        frame.data = nullptr;
        return false;
    }

    s_last_frame_ms = hal_millis();
    frame.width = 320;
    frame.height = 240;
    frame.timestamp_ms = s_last_frame_ms;
    frame.format = PixelFormat::PIXEL_FORMAT_RGB565;
    frame.buffer_size = 0;
    frame.data = nullptr; // Stub data; target hardware links esp_camera_fb_t->buf
    return true;
}

bool hal_camera_is_healthy() {
    return s_camera_initialized;
}

bool hal_camera_is_stub() {
    return s_stub_mode;
}

uint32_t hal_camera_get_last_frame_time_ms() {
    return s_last_frame_ms;
}

void hal_camera_set_auto_exposure(bool enabled) {
    // Stub: On real OV5640 hardware, write AEC enable/disable registers via SCCB/I2C.
    // e.g., sensor_set_reg(0x3503, enabled ? 0x00 : 0x07);
    char buf[80];
    std::snprintf(buf, sizeof(buf), "[HAL_CAMERA] Auto-exposure %s (stub)", enabled ? "ENABLED" : "DISABLED");
    hal_log(buf);
}

void hal_camera_set_exposure(int32_t value) {
    // Stub: On real OV5640 hardware, write manual exposure value registers.
    // e.g., sensor_set_reg(0x3500..0x3502, value);
    char buf[80];
    std::snprintf(buf, sizeof(buf), "[HAL_CAMERA] Manual exposure set to %ld (stub)", static_cast<long>(value));
    hal_log(buf);
}

void hal_camera_set_auto_whitebalance(bool enabled) {
    // Stub: On real OV5640 hardware, write AWB enable/disable registers via SCCB/I2C.
    // e.g., sensor_set_reg(0x3406, enabled ? 0x00 : 0x01);
    char buf[80];
    std::snprintf(buf, sizeof(buf), "[HAL_CAMERA] Auto-whitebalance %s (stub)", enabled ? "ENABLED" : "DISABLED");
    hal_log(buf);
}

} // namespace Hal
} // namespace RobofestDrone
