#include "image_enhance.h"
#include "mem.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {

// Scratch capacity cap: enhancements run at process resolution or below.
constexpr size_t ENH_MAX_PX = 320u * 240u;

// PSRAM-backed lazily-allocated scratch (kept out of precious DRAM).
struct EnhanceScratch {
    uint8_t* u8a = nullptr;    // ENH_MAX_PX
    uint8_t* u8b = nullptr;    // ENH_MAX_PX
    uint16_t* u16a = nullptr;  // ENH_MAX_PX
    uint16_t* u16b = nullptr;  // ENH_MAX_PX

    bool ensure() {
        if (u8a == nullptr) u8a = static_cast<uint8_t*>(robofest_big_alloc(ENH_MAX_PX));
        if (u8b == nullptr) u8b = static_cast<uint8_t*>(robofest_big_alloc(ENH_MAX_PX));
        if (u16a == nullptr) u16a = static_cast<uint16_t*>(robofest_big_alloc(ENH_MAX_PX * sizeof(uint16_t)));
        if (u16b == nullptr) u16b = static_cast<uint16_t*>(robofest_big_alloc(ENH_MAX_PX * sizeof(uint16_t)));
        return u8a && u8b && u16a && u16b;
    }
};

EnhanceScratch s_scratch;

constexpr uint16_t CLAHE_TILES_X = 4;
constexpr uint16_t CLAHE_TILES_Y = 3;
constexpr uint16_t HIST_BINS = 32;      // 8-value bins keep RAM tiny
constexpr float CLIP_LIMIT_FRACTION = 0.12f;

inline uint8_t clamp_u8(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return static_cast<uint8_t>(v);
}

inline uint8_t v_of(uint8_t r, uint8_t g, uint8_t b) {
    return std::max(r, std::max(g, b));
}

} // namespace

// ----------------------------------------------------------------------------
// Gamma lift via 256-entry LUT.
// ----------------------------------------------------------------------------
void enhance_gamma(uint8_t* rgb888, size_t n_px, float gamma) {
    if (rgb888 == nullptr || n_px == 0 || gamma <= 0.0f || gamma == 1.0f) return;
    static uint8_t lut[256];
    static bool lut_valid = false;
    static float lut_gamma = 0.0f;
    if (!lut_valid || lut_gamma != gamma) {
        const float inv = 1.0f / gamma;
        for (int i = 0; i < 256; ++i) {
            const float x = static_cast<float>(i) / 255.0f;
            lut[i] = clamp_u8(static_cast<int>(std::pow(x, inv) * 255.0f + 0.5f));
        }
        lut_valid = true;
        lut_gamma = gamma;
    }
    for (size_t i = 0; i < n_px * 3; ++i) {
        rgb888[i] = lut[rgb888[i]];
    }
}

// ----------------------------------------------------------------------------
// CLAHE on V channel with bilinear tile blending.
// ----------------------------------------------------------------------------
void enhance_clahe_v(uint8_t* rgb888, uint16_t w, uint16_t h, float strength) {
    if (rgb888 == nullptr || w < CLAHE_TILES_X || h < CLAHE_TILES_Y) return;
    if (strength <= 0.0f) strength = 0.001f;
    if (strength > 1.0f) strength = 1.0f;

    // 1. Extract V plane + per-tile histograms.
    if (static_cast<size_t>(w) * h > ENH_MAX_PX) return;
    if (!s_scratch.ensure()) return;
    uint8_t* v_plane = s_scratch.u8a;

    static uint32_t hist[CLAHE_TILES_Y][CLAHE_TILES_X][HIST_BINS];
    std::memset(hist, 0, sizeof(hist));

    for (uint16_t y = 0; y < h; ++y) {
        const uint8_t* row = rgb888 + static_cast<size_t>(y) * w * 3;
        const uint16_t ty = std::min<uint16_t>(CLAHE_TILES_Y - 1,
                                               static_cast<uint16_t>(y * CLAHE_TILES_Y / h));
        for (uint16_t x = 0; x < w; ++x) {
            const uint8_t v = v_of(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]);
            v_plane[y * w + x] = v;
            const uint16_t tx = std::min<uint16_t>(CLAHE_TILES_X - 1,
                                                   static_cast<uint16_t>(x * CLAHE_TILES_X / w));
            hist[ty][tx][v >> 3]++;
        }
    }

    // 2. Clip + redistribute each histogram; build 256-entry tile LUTs.
    static uint8_t luts[CLAHE_TILES_Y][CLAHE_TILES_X][256];

    const uint32_t tile_px =
        ((static_cast<uint32_t>(w) + CLAHE_TILES_X - 1) / CLAHE_TILES_X) *
        ((static_cast<uint32_t>(h) + CLAHE_TILES_Y - 1) / CLAHE_TILES_Y);
    // Clip level scales with tile size and requested strength.
    const uint32_t clip_limit = std::max<uint32_t>(
        8u,
        static_cast<uint32_t>(static_cast<float>(tile_px) / HIST_BINS *
                              (0.25f + CLIP_LIMIT_FRACTION) * strength + 0.5f));

    for (uint16_t tyy = 0; tyy < CLAHE_TILES_Y; ++tyy) {
        for (uint16_t txx = 0; txx < CLAHE_TILES_X; ++txx) {
            uint32_t excess = 0;
            for (int b = 0; b < HIST_BINS; ++b) {
                uint32_t c = hist[tyy][txx][b];
                if (c > clip_limit) {
                    excess += c - clip_limit;
                    hist[tyy][txx][b] = clip_limit;
                }
            }
            const uint32_t bonus = excess / HIST_BINS;
            for (int b = 0; b < HIST_BINS; ++b) {
                hist[tyy][txx][b] += bonus;
            }

            // CDF -> LUT (bin centers expanded to 8 values).
            uint32_t acc = 0;
            const uint32_t total = tile_px;
            for (int b = 0; b < HIST_BINS; ++b) {
                acc += hist[tyy][txx][b];
                const float frac = (total > 0)
                    ? static_cast<float>(acc) / static_cast<float>(total) : 0.0f;
                const uint8_t mapped = clamp_u8(
                    static_cast<int>(frac * 255.0f + 0.5f));
                for (int sub = 0; sub < 8; ++sub) {
                    luts[tyy][txx][b * 8 + sub] = mapped;
                }
            }
        }
    }

    // 3. Apply with bilinear blend between the four surrounding tile LUTs.
    const float tile_w_f = static_cast<float>(w) / CLAHE_TILES_X;
    const float tile_h_f = static_cast<float>(h) / CLAHE_TILES_Y;

    for (uint16_t y = 0; y < h; ++y) {
        uint8_t* row = rgb888 + static_cast<size_t>(y) * w * 3;
        const float tf_y = std::max(0.0f, std::min(static_cast<float>(CLAHE_TILES_Y - 1),
                                                   (static_cast<float>(y) + 0.5f) / tile_h_f - 0.5f));
        const int ty0 = static_cast<int>(tf_y);
        const int ty1 = std::min(static_cast<int>(CLAHE_TILES_Y - 1), ty0 + 1);
        const float wy = tf_y - static_cast<float>(ty0);

        for (uint16_t x = 0; x < w; ++x) {
            const float tf_x = std::max(0.0f, std::min(static_cast<float>(CLAHE_TILES_X - 1),
                                                       (static_cast<float>(x) + 0.5f) / tile_w_f - 0.5f));
            const int tx0 = static_cast<int>(tf_x);
            const int tx1 = std::min(static_cast<int>(CLAHE_TILES_X - 1), tx0 + 1);
            const float wx = tf_x - static_cast<float>(tx0);

            const uint8_t v = v_plane[y * w + x];
            const float v00 = luts[ty0][tx0][v];
            const float v10 = luts[ty0][tx1][v];
            const float v01 = luts[ty1][tx0][v];
            const float v11 = luts[ty1][tx1][v];
            const float top = v00 * (1.0f - wx) + v10 * wx;
            const float bot = v01 * (1.0f - wx) + v11 * wx;
            const int new_v = static_cast<int>(top * (1.0f - wy) + bot * wy + 0.5f);

            const int old_v = v;
            if (old_v == new_v || old_v == 0) continue;
            // Scale RGB channels proportionally so hue/sat are preserved.
            const float ratio = static_cast<float>(new_v) / static_cast<float>(old_v);
            row[x * 3 + 0] = clamp_u8(static_cast<int>(row[x * 3 + 0] * ratio + 0.5f));
            row[x * 3 + 1] = clamp_u8(static_cast<int>(row[x * 3 + 1] * ratio + 0.5f));
            row[x * 3 + 2] = clamp_u8(static_cast<int>(row[x * 3 + 2] * ratio + 0.5f));
        }
    }
}

// ----------------------------------------------------------------------------
// Haze severity: dark-channel density + contrast collapse estimate.
// ----------------------------------------------------------------------------
float enhance_haze_severity(const uint8_t* rgb888, uint16_t w, uint16_t h) {
    if (rgb888 == nullptr || w == 0 || h == 0) return 0.0f;

    constexpr int STRIDE = 2;
    constexpr int WIN = 5;   // dark-channel window radius in sampled grid steps

    const int gw = (w + STRIDE - 1) / STRIDE;
    const int gh = (h + STRIDE - 1) / STRIDE;
    if (gw <= 0 || gh <= 0) return 0.0f;

    static uint8_t dark[(ENH_MAX_PX / 2) + 1];
    if (static_cast<size_t>(gw) * gh > sizeof(dark)) return 0.0f;

    uint32_t dark_sum = 0;
    uint8_t v_min_global = 255;
    uint8_t v_max_global = 0;

    for (int gy = 0; gy < gh; ++gy) {
        for (int gx = 0; gx < gw; ++gx) {
            const int cx = std::min(static_cast<int>(w) - 1, gx * STRIDE);
            const int cy = std::min(static_cast<int>(h) - 1, gy * STRIDE);

            uint8_t dmin = 255;
            for (int dy = -WIN; dy <= WIN; dy += 2) {
                const int yy = std::max(0, std::min(static_cast<int>(h) - 1, cy + dy));
                const uint8_t* row = rgb888 + static_cast<size_t>(yy) * w * 3;
                for (int dx = -WIN; dx <= WIN; dx += 2) {
                    const int xx = std::max(0, std::min(static_cast<int>(w) - 1, cx + dx));
                    const uint8_t dm = std::min(row[xx * 3],
                                                std::min(row[xx * 3 + 1], row[xx * 3 + 2]));
                    if (dm < dmin) dmin = dm;
                }
            }
            dark[gy * gw + gx] = dmin;
            dark_sum += dmin;

            const uint8_t v_here = v_of(rgb888[(static_cast<size_t>(cy) * w + cx) * 3],
                                        rgb888[(static_cast<size_t>(cy) * w + cx) * 3 + 1],
                                        rgb888[(static_cast<size_t>(cy) * w + cx) * 3 + 2]);
            if (v_here < v_min_global) v_min_global = v_here;
            if (v_here > v_max_global) v_max_global = v_here;
        }
    }

    const float dark_mean = static_cast<float>(dark_sum) /
                            static_cast<float>(static_cast<size_t>(gw) * gh);
    const float contrast_span = static_cast<float>(v_max_global - v_min_global) / 255.0f;

    // Hazy scenes: bright uniform dark-channel + collapsed contrast.
    const float density = std::min(1.0f, dark_mean / 160.0f);
    const float collapse = 1.0f - std::min(1.0f, contrast_span / 0.75f);
    const float severity = 0.6f * density + 0.4f * collapse;
    return std::max(0.0f, std::min(1.0f, severity));
}

// ----------------------------------------------------------------------------
// Simplified dark-channel-prior dehaze.
// ----------------------------------------------------------------------------
void enhance_dehaze(uint8_t* rgb888, uint16_t w, uint16_t h, float strength) {
    if (rgb888 == nullptr || w == 0 || h == 0) return;
    if (strength <= 0.0f) return;
    if (strength > 1.0f) strength = 1.0f;

    // Atmospheric light: mean of the brightest 1% dark-channel pixels'
    // corresponding input pixels (approximated here by global brightest-V mean).
    uint8_t atm[3] = {220, 220, 220};
    {
        uint32_t best_sum = 0;
        int best_x = -1, best_y = -1;
        for (uint16_t by = 0; by < h; by += 3) {
            for (uint16_t bx = 0; bx < w; bx += 3) {
                const size_t i = (static_cast<size_t>(by) * w + bx) * 3;
                const uint8_t v = v_of(rgb888[i], rgb888[i + 1], rgb888[i + 2]);
                if (static_cast<uint32_t>(v) > best_sum) {
                    best_sum = v;
                    best_x = bx;
                    best_y = by;
                }
            }
        }
        if (best_x >= 0) {
            const size_t i = (static_cast<size_t>(best_y) * w + best_x) * 3;
            atm[0] = rgb888[i];
            atm[1] = rgb888[i + 1];
            atm[2] = rgb888[i + 2];
        }
    }

    const float omega = 0.85f * strength;
    const float t_min = 0.25f;

    for (size_t i = 0; i < static_cast<size_t>(w) * h * 3; i += 3) {
        const uint8_t dm = std::min(rgb888[i], std::min(rgb888[i + 1], rgb888[i + 2]));
        const float raw_t = 1.0f - omega *
            (static_cast<float>(dm) / 255.0f);
        const float t = std::max(t_min, raw_t);
        for (uint8_t ch = 0; ch < 3; ++ch) {
            const float val = (static_cast<float>(rgb888[i + ch]) -
                               static_cast<float>(atm[ch])) / t +
                              static_cast<float>(atm[ch]);
            rgb888[i + ch] = clamp_u8(static_cast<int>(val + 0.5f));
        }
    }
}

// ----------------------------------------------------------------------------
// Shadow mask: darker-than-neighborhood + chroma-neutral.
// ----------------------------------------------------------------------------
void enhance_shadow_mask(const uint8_t* rgb888, uint16_t w, uint16_t h,
                         uint8_t* out_flags) {
    if (rgb888 == nullptr || out_flags == nullptr || w == 0 || h == 0) return;
    if (static_cast<size_t>(w) * h > ENH_MAX_PX) return;
    if (!s_scratch.ensure()) return;

    uint8_t* lum = s_scratch.u8a;
    uint16_t* blur = s_scratch.u16a;
    uint16_t* vert = s_scratch.u16b;

    for (size_t i = 0, p = 0; p < static_cast<size_t>(w) * h; i += 3, ++p) {
        lum[p] = static_cast<uint8_t>((77u * rgb888[i] + 150u * rgb888[i + 1] +
                                       29u * rgb888[i + 2]) >> 8);
    }

    constexpr int R = 7; // neighborhood radius (~15px support at process res)

    // Separable box blur into 'blur' (sum form).
    for (uint16_t y = 0; y < h; ++y) {
        uint32_t acc = 0;
        for (int i = 0; i <= R && i < w; ++i) {
            acc += lum[y * w + i];
        }
        for (uint16_t x = 0; x < w; ++x) {
            const int left = std::max(0, static_cast<int>(x) - R);
            const int right = std::min(static_cast<int>(w) - 1, static_cast<int>(x) + R);
            blur[y * w + x] = static_cast<uint16_t>(acc / (right - left + 1));
            
            if (static_cast<int>(x) - R >= 0) {
                acc -= lum[y * w + (x - R)];
            }
            if (static_cast<int>(x) + R + 1 < w) {
                acc += lum[y * w + (x + R + 1)];
            }
        }
    }
    // Vertical pass writes into the second PSRAM plane (cannot reuse blur
    // in place: column windows read rows written earlier in this pass).
    for (uint16_t x = 0; x < w; ++x) {
        uint32_t acc = 0;
        for (int i = 0; i <= R && i < h; ++i) {
            acc += blur[i * w + x];
        }
        for (uint16_t y = 0; y < h; ++y) {
            const int top = std::max(0, static_cast<int>(y) - R);
            const int bot = std::min(static_cast<int>(h) - 1, static_cast<int>(y) + R);
            vert[y * w + x] = static_cast<uint16_t>(acc / (bot - top + 1));
            
            if (static_cast<int>(y) - R >= 0) {
                acc -= blur[(y - R) * w + x];
            }
            if (static_cast<int>(y) + R + 1 < h) {
                acc += blur[(y + R + 1) * w + x];
            }
        }
    }

    uint16_t shadows = 0;
    for (uint16_t y = 0; y < h; ++y) {
        for (uint16_t x = 0; x < w; ++x) {
            const size_t p = static_cast<size_t>(y) * w + x;
            const uint8_t local_mean = static_cast<uint8_t>(vert[p] >> 0);
            const uint8_t sat_proxy = static_cast<uint8_t>(
                std::abs(static_cast<int>(rgb888[p * 3]) - rgb888[p * 3 + 2]) );

            // Shadow-suspect: much darker than surroundings AND low chroma shift.
            const bool darker = lum[p] < (local_mean * 55u) / 100u;
            const bool neutral = sat_proxy < 40;
            const bool flag = darker && neutral;
            out_flags[p] = flag ? 1u : 0u;
            if (flag && shadows < 0xFFFF) shadows++;
        }
    }
}

} // namespace RobofestDrone
