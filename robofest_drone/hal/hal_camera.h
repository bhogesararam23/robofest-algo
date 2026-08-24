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
// CAMERA HAL INTERFACE
// ----------------------------------------------------------------------------
// Real hardware path (REQ-DER-101, item 1): on ESP32-S3 the OV5640 sensor is
// driven by the esp32-camera component (DVP + I2S/DMA into PSRAM frame
// buffers). Host test builds compile a stub that returns synthetic metadata.
// ============================================================================

bool hal_camera_init();
bool hal_camera_is_healthy();
bool hal_camera_get_frame(CameraFrame& frame);
bool hal_camera_read_frame();
uint32_t hal_camera_get_last_frame_time_ms();
bool hal_camera_is_stub(); // Runtime marker: true for stub, false for real camera

// Native sensor resolution/format of the initialized source (item 5).
// Returns false when no camera is initialized.
bool hal_camera_get_native_resolution(uint16_t* w, uint16_t* h, PixelFormat* fmt);

// Pure in-memory pixel format conversion (host-testable).
// dst_fmt: PIXEL_FORMAT_RGB888 or PIXEL_FORMAT_GRAY8 (from RGB565/RGB888 src).
// Returns false on unsupported combination or undersized destination buffer.
bool hal_camera_convert_format(
    const CameraFrame& src,
    PixelFormat dst_fmt,
    uint8_t* dst_buf,
    uint32_t dst_buf_size);

// Camera exposure and white-balance control.
// On real hardware these drive OV5640 AEC/AGC/AWB blocks via esp32-camera.
// Locking exposure prevents auto-adjustment from invalidating calibrated HSV values.
void hal_camera_set_auto_exposure(bool enabled);
void hal_camera_set_exposure(int32_t value);
void hal_camera_set_auto_whitebalance(bool enabled);

// Night / low-light support (REQ-DER-115, item 15): raises AGC ceiling and
// AE level bias; motion blur must be tolerated by caller-side gating.
void hal_camera_set_night_mode(bool enabled);

} // namespace Hal
} // namespace RobofestDrone
