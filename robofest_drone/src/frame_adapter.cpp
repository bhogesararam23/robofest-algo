#include "frame_adapter.h"
#include <cstring>

namespace RobofestDrone {

namespace {

inline void fetch_rgb888_or_565(
    const Hal::CameraFrame& src,
    int32_t px, int32_t py,
    uint8_t out[3]) {

    if (src.format == Hal::PixelFormat::PIXEL_FORMAT_RGB565) {
        const uint32_t i = (static_cast<uint32_t>(py) * src.width + px) * 2;
        const uint16_t p = static_cast<uint16_t>(
            src.data[i] | (src.data[i + 1] << 8));
        out[0] = static_cast<uint8_t>(((p >> 11) & 0x1F) << 3);
        out[1] = static_cast<uint8_t>(((p >> 5) & 0x3F) << 2);
        out[2] = static_cast<uint8_t>((p & 0x1F) << 3);
        return;
    }
    const uint32_t i = (static_cast<uint32_t>(py) * src.width + px) * 3;
    out[0] = src.data[i];
    out[1] = src.data[i + 1];
    out[2] = src.data[i + 2];
}

} // namespace

bool frame_adapter_convert(
    const Hal::CameraFrame& src,
    uint16_t dst_w,
    uint16_t dst_h,
    bool allow_rescale,
    uint8_t* dst_rgb888,
    FrameTransform& out_tf) {

    out_tf.valid = false;
    if (!allow_rescale) return false;
    if (dst_rgb888 == nullptr || dst_w < 2 || dst_h < 2) return false;
    if (src.data == nullptr || src.width < 2 || src.height < 2) return false;
    if (src.format != Hal::PixelFormat::PIXEL_FORMAT_RGB565 &&
        src.format != Hal::PixelFormat::PIXEL_FORMAT_RGB888) {
        return false;
    }

    // Letterbox scale: content must FIT both axes => max of the ratios.
    const float sx = static_cast<float>(src.width) / dst_w;
    const float sy = static_cast<float>(src.height) / dst_h;
    const float scale = (sx > sy) ? sx : sy;

    const float content_w = static_cast<float>(src.width) / scale;
    const float content_h = static_cast<float>(src.height) / scale;
    const float off_x_f = (static_cast<float>(dst_w) - content_w) * 0.5f;
    const float off_y_f = (static_cast<float>(dst_h) - content_h) * 0.5f;

    // First pass: bilinear content region.
    for (uint16_t ty = 0; ty < dst_h; ++ty) {
        uint8_t* row = dst_rgb888 + static_cast<size_t>(ty) * dst_w * 3;
        for (uint16_t tx = 0; tx < dst_w; ++tx) {
            const float fx =
                (static_cast<float>(tx) - off_x_f + 0.5f) * scale - 0.5f;
            const float fy =
                (static_cast<float>(ty) - off_y_f + 0.5f) * scale - 0.5f;

            uint8_t* dst = row + static_cast<size_t>(tx) * 3;

            if (fx < 0.0f || fy < 0.0f ||
                fx > static_cast<float>(src.width - 1) ||
                fy > static_cast<float>(src.height - 1)) {
                // Padding pass fills these below; leave zeroed marker.
                dst[0] = dst[1] = dst[2] = 0xFF; // sentinel: unfilled pad
                continue;
            }

            const int32_t x0 = static_cast<int32_t>(fx);
            const int32_t y0 = static_cast<int32_t>(fy);
            const int32_t x1 = (x0 + 1 < src.width) ? x0 + 1 : x0;
            const int32_t y1 = (y0 + 1 < src.height) ? y0 + 1 : y0;
            const float ax = fx - static_cast<float>(x0);
            const float ay = fy - static_cast<float>(y0);

            uint8_t c00[3], c10[3], c01[3], c11[3];
            fetch_rgb888_or_565(src, x0, y0, c00);
            fetch_rgb888_or_565(src, x1, y0, c10);
            fetch_rgb888_or_565(src, x0, y1, c01);
            fetch_rgb888_or_565(src, x1, y1, c11);

            for (int c = 0; c < 3; ++c) {
                const float top = c00[c] + (c10[c] - c00[c]) * ax;
                const float bot = c01[c] + (c11[c] - c01[c]) * ax;
                float v = top + (bot - top) * ay;
                if (v < 0.0f) v = 0.0f;
                if (v > 255.0f) v = 255.0f;
                dst[c] = static_cast<uint8_t>(v + 0.5f);
            }
        }
    }

    // Second pass: replicate nearest content pixel into padding bands so no
    // artificial black border exists (BLACK-marker false-positive source).
    const int32_t cx_lo = static_cast<int32_t>(off_x_f + 0.5f);
    const int32_t cy_lo = static_cast<int32_t>(off_y_f + 0.5f);
    const int32_t cx_hi = static_cast<int32_t>(off_x_f + content_w - 0.5f);
    const int32_t cy_hi = static_cast<int32_t>(off_y_f + content_h - 0.5f);
    if (cx_lo < 0 || cy_lo < 0 ||
        cx_hi >= static_cast<int32_t>(dst_w) ||
        cy_hi >= static_cast<int32_t>(dst_h)) {
        return false; // degenerate geometry guard
    }

    for (uint16_t ty = 0; ty < dst_h; ++ty) {
        uint8_t* row = dst_rgb888 + static_cast<size_t>(ty) * dst_w * 3;
        const int32_t sy_i =
            (ty < static_cast<uint16_t>(cy_lo)) ? cy_lo
            : (ty > static_cast<uint16_t>(cy_hi)) ? cy_hi
            : static_cast<int32_t>(ty);
        for (uint16_t tx = 0; tx < dst_w; ++tx) {
            uint8_t* dst = row + static_cast<size_t>(tx) * 3;
            if (dst[0] == 0xFF && dst[1] == 0xFF && dst[2] == 0xFF) {
                const int32_t sx_i =
                    (tx < static_cast<uint16_t>(cx_lo)) ? cx_lo
                    : (tx > static_cast<uint16_t>(cx_hi)) ? cx_hi
                    : static_cast<int32_t>(tx);
                const uint8_t* src_px =
                    dst_rgb888 + (static_cast<size_t>(sy_i) * dst_w + sx_i) * 3;
                dst[0] = src_px[0];
                dst[1] = src_px[1];
                dst[2] = src_px[2];
            }
        }
    }

    out_tf.valid = true;
    out_tf.scale = scale;
    out_tf.pad_x = static_cast<uint16_t>(off_x_f + 0.5f);
    out_tf.pad_y = static_cast<uint16_t>(off_y_f + 0.5f);
    out_tf.src_w = src.width;
    out_tf.src_h = src.height;
    out_tf.dst_w = dst_w;
    out_tf.dst_h = dst_h;
    return true;
}

} // namespace RobofestDrone
