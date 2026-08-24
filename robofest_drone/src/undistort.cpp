#include "undistort.h"
#include "mem.h"
#include "../config/camera_intrinsics.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

namespace {

struct Calib {
    bool valid;
    float fx, fy, cx, cy, k1, k2, p1, p2;
};

Calib load_calib() {
    return Calib{
        Config::CAM_INTRINSICS_VALID &&
            Config::CAM_FX_PX > 1.0f && Config::CAM_FY_PX > 1.0f,
        Config::CAM_FX_PX, Config::CAM_FY_PX,
        Config::CAM_CX_PX, Config::CAM_CY_PX,
        Config::CAM_K1, Config::CAM_K2,
        Config::CAM_P1, Config::CAM_P2};
}

uint32_t* s_map = nullptr;   // [w*h] packed (src_y<<16)|src_x
uint16_t s_map_w = 0;
uint16_t s_map_h = 0;

// Inverts radial+tangential distortion for one normalized coordinate with
// damped Newton iterations.
void invert_point(const Calib& c, float u, float v, float& ou, float& ov) {
    if (!c.valid) {
        ou = u;
        ov = v;
        return;
    }
    float uu = u, vv = v;
    for (int it = 0; it < 3; ++it) {
        const float r2 = uu * uu + vv * vv;
        const float radial = 1.0f + c.k1 * r2 + c.k2 * r2 * r2;
        const float du = 2.0f * c.p1 * uu * vv + c.p2 * (r2 + 2.0f * uu * uu);
        const float dv = c.p1 * (r2 + 2.0f * vv * vv) + 2.0f * c.p2 * uu * vv;
        const float ru = uu * radial + du - u;
        const float rv = vv * radial + dv - v;
        const float ju = radial + 2.0f * c.k1 * uu * uu +
                         4.0f * c.k2 * r2 * uu * uu + 2.0f * c.p1 * vv + 6.0f * c.p2 * uu;
        const float jv = radial + 2.0f * c.k1 * vv * vv +
                         4.0f * c.k2 * r2 * vv * vv + 6.0f * c.p1 * vv + 2.0f * c.p2 * uu;
        uu -= (ju != 0.0f) ? ru / ju : 0.0f;
        vv -= (jv != 0.0f) ? rv / jv : 0.0f;
    }
    ou = uu;
    ov = vv;
}

} // namespace

bool undistort_is_calibrated() {
    return load_calib().valid;
}

bool undistort_build_map(uint16_t w, uint16_t h) {
    if (w == 0 || h == 0) return false;
    if (s_map != nullptr && s_map_w == w && s_map_h == h) return true;

    robofest_big_free(s_map);
    s_map = static_cast<uint32_t*>(
        robofest_big_alloc(static_cast<size_t>(w) * h * sizeof(uint32_t)));
    if (s_map == nullptr) {
        s_map_w = s_map_h = 0;
        return false;
    }
    s_map_w = w;
    s_map_h = h;

    const Calib c = load_calib();

    if (!c.valid || !Config::VISION_UNDISTORT_ENABLED) {
        // Identity map.
        for (uint16_t y = 0; y < h; ++y) {
            for (uint16_t x = 0; x < w; ++x) {
                s_map[static_cast<size_t>(y) * w + x] =
                    (static_cast<uint32_t>(y) << 16) |
                    static_cast<uint32_t>(x);
            }
        }
        return true;
    }

    for (uint16_t y = 0; y < h; ++y) {
        for (uint16_t x = 0; x < w; ++x) {
            const float un =
                (static_cast<float>(x) - Config::CAM_CX_PX) / Config::CAM_FX_PX;
            const float vn =
                (static_cast<float>(y) - Config::CAM_CY_PX) / Config::CAM_FY_PX;

            float su, sv;
            invert_point(c, un, vn, su, sv);

            float px = su * Config::CAM_FX_PX + Config::CAM_CX_PX;
            float py = sv * Config::CAM_FY_PX + Config::CAM_CY_PX;
            if (px < 0.0f) px = 0.0f;
            if (py < 0.0f) py = 0.0f;
            if (px > static_cast<float>(w) - 1.0f) px = static_cast<float>(w) - 1.0f;
            if (py > static_cast<float>(h) - 1.0f) py = static_cast<float>(h) - 1.0f;

            s_map[static_cast<size_t>(y) * w + x] =
                (static_cast<uint32_t>(py + 0.5f) << 16) |
                static_cast<uint32_t>(px + 0.5f);
        }
    }
    return true;
}

bool undistort_remap_rgb565(
    const uint8_t* src, uint8_t* dst,
    uint16_t w, uint16_t h) {

    if (s_map == nullptr || s_map_w != w || s_map_h != h ||
        src == nullptr || dst == nullptr) {
        return false;
    }

    const size_t n = static_cast<size_t>(w) * h;
    for (size_t i = 0; i < n; ++i) {
        const uint32_t m = s_map[i];
        const uint32_t sx = m & 0xFFFFu;
        const uint32_t sy = m >> 16;
        dst[i * 2] = src[(sy * w + sx) * 2];
        dst[i * 2 + 1] = src[(sy * w + sx) * 2 + 1];
    }
    return true;
}

} // namespace RobofestDrone