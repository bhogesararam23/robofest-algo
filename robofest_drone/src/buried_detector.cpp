#include "buried_detector.h"
#include "telemetry_events.h"
#include "../config/thresholds.h"
#include <cmath>
#include <cstdlib>
#include <algorithm>

namespace RobofestDrone {

namespace {

constexpr uint16_t BURIED_TELEMETRY_STRIDE = 1; // reserved for future batching

inline uint8_t luma_of(const uint8_t* p) {
    return static_cast<uint8_t>((77u * p[0] + 150u * p[1] + 29u * p[2]) >> 8);
}

} // namespace

// ============================================================================
// PURE CORES
// ============================================================================

float buried_texture_ratio(
    const uint8_t* rgb888, uint16_t w, uint16_t h,
    uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

    if (rgb888 == nullptr || w == 0 || h == 0) return 0.0f;
    if (x1 <= x0 || y1 <= y0) return 0.0f;

    // Whole-image mean/variance on a stride grid.
    float g_sum = 0.0f, g_sq = 0.0f;
    uint32_t g_n = 0;
    for (uint32_t y = 0; y < h; y += 3) {
        for (uint32_t x = 0; x < w; x += 3) {
            const size_t i = (y * w + x) * 3;
            const float l = static_cast<float>(luma_of(rgb888 + i));
            g_sum += l;
            g_sq += l * l;
            g_n++;
        }
    }
    if (g_n == 0) return 0.0f;
    const float g_mean = g_sum / g_n;
    const float g_var = g_sq / g_n - g_mean * g_mean;

    // ROI mean/variance at full density.
    float r_sum = 0.0f, r_sq = 0.0f;
    uint32_t r_n = 0;
    for (uint32_t y = y0; y < y1 && y < h; ++y) {
        for (uint32_t x = x0; x < x1 && x < w; ++x) {
            const size_t i = (y * w + x) * 3;
            const float l = static_cast<float>(luma_of(rgb888 + i));
            r_sum += l;
            r_sq += l * l;
            r_n++;
        }
    }
    if (r_n == 0) return 0.0f;
    const float r_mean = r_sum / r_n;
    const float r_var = r_sq / r_n - r_mean * r_mean;

    if (g_var < 0.5f) return (r_var > 2.0f) ? 99.0f : 1.0f;
    return r_var / g_var;
}

float buried_plane_fit_residual(
    const GroundReturn* pts, uint8_t n,
    float* out_a, float* out_b, float* out_c) {

    if (pts == nullptr || n < 4) return 0.0f;

    // Normal equations for least squares z = a*x + b*y + c.
    double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, s1 = static_cast<double>(n);
    double sxz = 0, syz = 0, sz = 0;

    for (uint8_t i = 0; i < n; ++i) {
        const double x = pts[i].x;
        const double y = pts[i].y;
        const double z = pts[i].height;
        sxx += x * x;
        sxy += x * y;
        sx += x;
        syy += y * y;
        sy += y;
        sxz += x * z;
        syz += y * z;
        sz += z;
    }

    // Solve the 3x3 system via Cramer's rule.
    const double m11 = sxx, m12 = sxy, m13 = sx;
    const double m21 = sxy, m22 = syy, m23 = sy;
    const double m31 = sx,  m32 = sy,  m33 = s1;
    const double v1 = sxz, v2 = syz, v3 = sz;

    const double det =
        m11 * (m22 * m33 - m23 * m32) -
        m12 * (m21 * m33 - m23 * m31) +
        m13 * (m21 * m32 - m22 * m31);
    if (std::fabs(det) < 1e-9) return 0.0f;

    const double da = v1 * (m22 * m33 - m23 * m32) -
                      m12 * (v2 * m33 - m23 * v3) +
                      m13 * (v2 * m32 - m22 * v3);
    const double db = m11 * (v2 * m33 - m23 * v3) -
                      v1 * (m21 * m33 - m23 * m31) +
                      m13 * (m21 * v3 - v2 * m31);
    const double dc = m11 * (m22 * v3 - v2 * m32) -
                      m12 * (m21 * v3 - v2 * m31) +
                      v1 * (m21 * m32 - m22 * m31);

    const float a = static_cast<float>(da / det);
    const float b = static_cast<float>(db / det);
    const float c = static_cast<float>(dc / det);

    if (out_a) *out_a = a;
    if (out_b) *out_b = b;
    if (out_c) *out_c = c;

    float max_res = 0.0f;
    for (uint8_t i = 0; i < n; ++i) {
        const float pred = a * pts[i].x + b * pts[i].y + c;
        const float res = std::fabs(pts[i].height - pred);
        if (res > max_res) max_res = res;
    }
    return max_res;
}

void buried_rgb_spectral_proxy(
    const uint8_t* rgb888, uint16_t w, uint16_t h,
    uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
    SpectralIndices& out) {

    out.valid = false;
    out.ndvi = 0.0f;
    out.moisture = 0.0f;
    if (rgb888 == nullptr || w == 0 || h == 0 || x1 <= x0 || y1 <= y0) return;

    double r_sum = 0, g_sum = 0, b_sum = 0;
    uint32_t n = 0;
    for (uint32_t y = y0; y < y1 && y < h; ++y) {
        for (uint32_t x = x0; x < x1 && x < w; ++x) {
            const size_t i = (y * w + x) * 3;
            r_sum += rgb888[i];
            g_sum += rgb888[i + 1];
            b_sum += rgb888[i + 2];
            n++;
        }
    }
    if (n == 0) return;

    const double eps = 1e-3;
    const double r = r_sum / n, g = g_sum / n, b = b_sum / n;
    out.ndvi = static_cast<float>((g - r) / (g + r + eps));
    out.moisture = static_cast<float>(b / (r + g + b + eps));
    out.valid = true;
}

float buried_fuse_score(float texture_ratio, float depth_residual_m,
                        const SpectralIndices& idx) {
    // Texture term: saturates at 3x variance ratio.
    float tex = 0.0f;
    if (texture_ratio >= Config::BURIED_TEXTURE_RATIO_MIN) {
        tex = std::min(1.0f, (texture_ratio - Config::BURIED_TEXTURE_RATIO_MIN) /
                                 (3.0f - Config::BURIED_TEXTURE_RATIO_MIN)) *
              0.45f;
    }

    // Depth term: saturates at 2x residual threshold.
    float depth = 0.0f;
    if (depth_residual_m >= Config::BURIED_PLANE_RESIDUAL_M) {
        depth = std::min(1.0f, depth_residual_m /
                                   (2.0f * Config::BURIED_PLANE_RESIDUAL_M)) *
                0.40f;
    }

    // Spectral terms: disturbed soil tends to break vegetation/moisture
    // uniformity. Signed contributions are folded in as evidence modifiers.
    float spec = 0.0f;
    if (idx.valid) {
        const float ndvi_anomaly =
            std::fabs(idx.ndvi) * Config::BURIED_NDVI_WEIGHT;
        const float moist_anomaly =
            idx.moisture * Config::BURIED_MOISTURE_WEIGHT;
        spec = ndvi_anomaly + moist_anomaly;
        if (spec > 0.15f) spec = 0.15f;
    }

    return std::min(1.0f, tex + depth + spec);
}

// ============================================================================
// CLASS
// ============================================================================

BuriedDetector::BuriedDetector() {
    reset();
}

void BuriedDetector::init() {
    initialized_ = true;
    last_emit_ms_ = 0;
}

void BuriedDetector::reset() {
    initialized_ = false;
    last_emit_ms_ = 0;
    spectral_provider_ = nullptr;
}

bool BuriedDetector::readyToEmit(uint32_t now_ms) const {
    constexpr uint32_t COOLDOWN_MS = 2000UL;
    return (now_ms - last_emit_ms_) >= COOLDOWN_MS;
}

void BuriedDetector::markEmitted(uint32_t now_ms) {
    last_emit_ms_ = now_ms;
}

BuriedAnomaly BuriedDetector::update(
    const uint8_t* rgb888,
    uint16_t w, uint16_t h,
    const GroundReturn* returns,
    uint8_t n_returns,
    uint32_t now_ms) {

    (void)now_ms;
    (void)BURIED_TELEMETRY_STRIDE;
    BuriedAnomaly out;

    if (!initialized_) init();

    // Central ROI covers ~25% of the frame.
    const uint16_t x0 = w / 4;
    const uint16_t x1 = (w * 3) / 4;
    const uint16_t y0 = h / 4;
    const uint16_t y1 = (h * 3) / 4;

    float texture_ratio = 0.0f;
    if (rgb888 != nullptr && Config::BURIED_DETECT_ENABLED) {
        texture_ratio = buried_texture_ratio(rgb888, w, h, x0, y0, x1, y1);
    }

    float depth_res = 0.0f;
    if (n_returns >= 4 && Config::BURIED_DETECT_ENABLED) {
        depth_res = buried_plane_fit_residual(returns, n_returns, nullptr, nullptr, nullptr);
    }

    SpectralIndices idx;
    if (spectral_provider_ != nullptr) {
        spectral_provider_(rgb888, w, h, x0, y0, x1, y1, idx);
    } else if (rgb888 != nullptr) {
        buried_rgb_spectral_proxy(rgb888, w, h, x0, y0, x1, y1, idx);
    }

    const float score = buried_fuse_score(texture_ratio, depth_res, idx);

    out.texture_ratio = texture_ratio;
    out.depth_residual_m = depth_res;
    out.spectral = idx;
    out.score = score;
    out.detected = score >= Config::BURIED_SCORE_EMIT_MIN;
    out.x = 0.0f; // caller projects using its own pose/altitude pipeline
    out.y = 0.0f;
    return out;
}

} // namespace RobofestDrone
