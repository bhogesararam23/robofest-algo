#pragma once

#include <stdint.h>
#include "../hal/hal_camera.h"

namespace RobofestDrone {

// ============================================================================
// FRAME ADAPTER (REQ-DER-105, item 5)
// ----------------------------------------------------------------------------
// Converts any supported camera format/resolution into the CV pipeline's
// working RGB888 buffer at process resolution, preserving aspect ratio via
// edge-replicated letterboxing (never black - black pads would spawn false
// BLACK-marker detections). Records the geometric transform so downstream
// world projection can map process pixels back to native camera pixels.
//
// All functions are pure and host-testable.
// ============================================================================

struct FrameTransform {
    bool valid = false;
    float scale = 1.0f;        // native -> process scale factor
    uint16_t pad_x = 0;        // letterbox padding (process px)
    uint16_t pad_y = 0;
    uint16_t src_w = 0;
    uint16_t src_h = 0;
    uint16_t dst_w = 0;
    uint16_t dst_h = 0;

    // Maps a process-grid point back to native camera pixel coordinates.
    void to_native(float proc_x, float proc_y, float& out_x, float& out_y) const {
        if (!valid || scale <= 0.0f) {
            out_x = proc_x;
            out_y = proc_y;
            return;
        }
        out_x = (proc_x - static_cast<float>(pad_x)) / scale;
        out_y = (proc_y - static_cast<float>(pad_y)) / scale;
    }
};

// Converts src into dst_rgb888 (dst_w*dst_h*3 bytes), bilinear-resized with
// aspect-preserving edge-replicated letterbox. Returns false on unsupported
// input or null buffers. Always produces a fully-written destination buffer
// on success (pads replicated from edge pixels).
bool frame_adapter_convert(
    const Hal::CameraFrame& src,
    uint16_t dst_w,
    uint16_t dst_h,
    bool allow_rescale,
    uint8_t* dst_rgb888,
    FrameTransform& out_tf);

} // namespace RobofestDrone
