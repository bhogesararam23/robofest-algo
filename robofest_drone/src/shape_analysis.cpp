#include "shape_analysis.h"
#include <cmath>
#include <cstring>

namespace RobofestDrone {

namespace {

constexpr float M_PI_F = 3.14159265358979323846f;

// 8-neighborhood in clockwise order starting East.
constexpr int8_t kNdx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
constexpr int8_t kNdy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

inline bool fg(const uint8_t* mask, uint16_t w, uint16_t h, int x, int y) {
    if (x < 0 || y < 0 || x >= static_cast<int>(w) || y >= static_cast<int>(h)) {
        return false;
    }
    return mask[static_cast<uint32_t>(y) * w + x] != 0;
}

} // namespace

const char* vision_shape_class_name(ShapeClass cls) {
    switch (cls) {
        case ShapeClass::SHAPE_CIRCLE: return "CIRCLE";
        case ShapeClass::SHAPE_TRIANGLE: return "TRIANGLE";
        case ShapeClass::SHAPE_QUADRILATERAL: return "QUAD";
        case ShapeClass::SHAPE_PENTAGON: return "PENTAGON";
        case ShapeClass::SHAPE_HEXAGON_PLUS: return "HEXAGON+";
        case ShapeClass::SHAPE_STAR_CONCAVE: return "STAR/CONCAVE";
        case ShapeClass::SHAPE_BAR: return "BAR";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// MOORE-NEIGHBOR CONTOUR TRACE
// ============================================================================

uint16_t vision_trace_contour(
    const uint8_t* mask, uint16_t w, uint16_t h,
    uint16_t seed_x, uint16_t seed_y,
    VisionPoint* out_pts, uint16_t cap) {

    if (mask == nullptr || out_pts == nullptr || cap < 4) return 0;
    if (!fg(mask, w, h, seed_x, seed_y)) return 0;

    // Find the start backtrack direction (West of seed), per Moore tracing.
    int bx = static_cast<int>(seed_x) - 1;
    int by = static_cast<int>(seed_y);
    int dir = 6; // search begins from the neighbor after the backtrack slot

    // Single-pixel / line degenerate handling: the loop below with Jacob's
    // criterion covers these; budget still bounds runtime.
    constexpr uint16_t kMaxSteps = 4096;

    uint16_t n = 0;
    int cx = seed_x;
    int cy = seed_y;

    for (uint16_t step = 0; step < kMaxSteps; ++step) {
        bool found = false;
        for (int k = 0; k < 8; ++k) {
            const int probe = (dir + k) & 7;
            const int nx = cx + kNdx[probe];
            const int ny = cy + kNdy[probe];
            if (fg(mask, w, h, nx, ny)) {
                // Record boundary point on direction change into foreground.
                if (n == 0 ||
                    out_pts[n - 1].x != static_cast<float>(cx) ||
                    out_pts[n - 1].y != static_cast<float>(cy)) {
                    if (n >= cap) break;
                    out_pts[n].x = static_cast<float>(cx);
                    out_pts[n].y = static_cast<float>(cy);
                    n++;
                }
                bx = cx - kNdx[(probe + 7) & 7] ; // backtrack neighbor
                by = cy - kNdy[(probe + 7) & 7];
                cx = nx;
                cy = ny;
                // Next search starts rotated relative to entry edge.
                dir = (probe + 6) & 7;
                found = true;
                break;
            }
        }

        if (!found) {
            // Isolated pixel blob.
            if (n == 0) {
                out_pts[0].x = static_cast<float>(seed_x);
                out_pts[0].y = static_cast<float>(seed_y);
                n = 1;
            }
            break;
        }

        // Jacob's stopping criterion: re-entered the seed moving in the same
        // direction as the initial step.
        if (cx == seed_x && cy == seed_y && bx == seed_x - 1 && by == seed_y && n >= 4) {
            break;
        }
        if (n >= cap) break;
    }

    if (n > 2 &&
        out_pts[0].x == out_pts[n - 1].x &&
        out_pts[0].y == out_pts[n - 1].y) {
        n--; // drop duplicated closing vertex; caller treats contour as closed
    }
    return n;
}

// ============================================================================
// CONVEXITY DEFECTS
// ============================================================================

void vision_convexity_defects(
    const VisionPoint* contour, uint16_t contour_n,
    const VisionPoint* hull, uint8_t hull_n,
    float min_depth_px,
    uint8_t& out_defect_count,
    float& out_max_depth) {

    out_defect_count = 0;
    out_max_depth = 0.0f;

    if (contour == nullptr || hull == nullptr || contour_n < 4 || hull_n < 3) {
        return;
    }

    // For each contour vertex compute signed distance to every hull edge and
    // keep the maximum positive inward depth. O(n*h) is bounded: contour <=384
    // points, hull <=~24 vertices for marker-scale shapes.
    bool prev_defect = false;
    for (uint16_t i = 0; i < contour_n; ++i) {
        float best_depth = 0.0f;
        for (uint8_t e = 0; e < hull_n; ++e) {
            const VisionPoint& a = hull[e];
            const VisionPoint& b = hull[(e + 1) % hull_n];
            const float ex = b.x - a.x;
            const float ey = b.y - a.y;
            const float len = std::sqrt(ex * ex + ey * ey);
            if (len < 1e-6f) continue;
            // Inward normal for clockwise-wound hulls (y-down image space):
            // normal points right of travel direction.
            const float nx_ = ey / len;
            const float ny_ = -ex / len;
            const float dx = contour[i].x - a.x;
            const float dy = contour[i].y - a.y;
            const float d = dx * nx_ + dy * ny_;
            if (d > best_depth) best_depth = d;
        }

        if (best_depth > out_max_depth) out_max_depth = best_depth;

        const bool is_defect = best_depth >= min_depth_px;
        // Count rising edges only so one wide concavity = one defect.
        if (is_defect && !prev_defect) {
            if (out_defect_count < 255) out_defect_count++;
        }
        prev_defect = is_defect;
    }
}

// ============================================================================
// DECISION TREE CLASSIFIER
// ============================================================================

ShapeClass vision_classify_shape(
    float circularity,
    float solidity,
    float extent,
    float aspect,
    uint8_t raw_corners,
    uint8_t defect_count) {

    // 1. Concave geometry dominates: deep repeated indentations mean star /
    // gear / cross even when corner counting is noisy at low resolution.
    if (defect_count >= Config::SHAPE_STAR_MIN_DEFECTS &&
        solidity <= Config::SHAPE_STAR_SOLIDITY_MAX) {
        return ShapeClass::SHAPE_STAR_CONCAVE;
    }

    // 2. Elongated bar: few corners, extreme aspect.
    if (raw_corners <= 6 && aspect >= Config::SHAPE_LINE_ASPECT_MIN) {
        return ShapeClass::SHAPE_BAR;
    }

    // 3. Circle: near-perfect circularity and high solidity.
    if (circularity >= Config::SHAPE_CIRCLE_CIRCULARITY_MIN &&
        solidity >= Config::SHAPE_POLY_SOLIDITY_MIN) {
        return ShapeClass::SHAPE_CIRCLE;
    }

    // 4. Regular polygons by raw-corner count. DP on raw contours tends to
    // under-count by one on rasterized corners, hence inclusive bands.
    if (raw_corners >= 2 && raw_corners <= 3) {
        return ShapeClass::SHAPE_TRIANGLE;
    }
    if (raw_corners >= 4 && raw_corners <= 5) {
        return ShapeClass::SHAPE_QUADRILATERAL;
    }
    if (raw_corners == 6) {
        return ShapeClass::SHAPE_PENTAGON;
    }
    if (raw_corners >= 7 && defect_count < Config::SHAPE_STAR_MIN_DEFECTS) {
        return ShapeClass::SHAPE_HEXAGON_PLUS;
    }

    // 5. Fallback: use extent/solidity to at least separate compact convex
    // blobs from irregular noise.
    if (extent >= 0.75f && solidity >= Config::SHAPE_POLY_SOLIDITY_MIN) {
        return ShapeClass::SHAPE_CIRCLE;
    }
    return ShapeClass::SHAPE_UNKNOWN;
}

// ============================================================================
// LABEL-MASK CONTOUR API
// ============================================================================

namespace {

// Shared Moore-trace core parameterized by a pixel predicate so the binary
// and label-mask variants cannot drift apart.
template <typename Pred>
uint16_t trace_core(
    int seed_x, int seed_y,
    Pred is_fg,
    VisionPoint* out_vpts,
    ContourPoint* out_cpts,
    uint16_t cap) {

    auto record = [&](int x, int y, uint16_t idx) {
        if (out_vpts != nullptr) {
            out_vpts[idx].x = static_cast<float>(x);
            out_vpts[idx].y = static_cast<float>(y);
        }
        if (out_cpts != nullptr) {
            out_cpts[idx].x = static_cast<uint16_t>(x);
            out_cpts[idx].y = static_cast<uint16_t>(y);
        }
    };

    constexpr int8_t dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    constexpr int8_t dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    constexpr uint16_t kMaxSteps = 4096;
    uint16_t n = 0;
    int cx = seed_x;
    int cy = seed_y;
    int bx = seed_x - 1; // backtrack: West of seed
    int by = seed_y;
    int dir = 6;

    for (uint16_t step = 0; step < kMaxSteps; ++step) {
        bool found = false;
        for (int k = 0; k < 8; ++k) {
            const int probe = (dir + k) & 7;
            const int nx = cx + dx8[probe];
            const int ny = cy + dy8[probe];
            if (is_fg(nx, ny)) {
                if (n == 0) {
                    record(cx, cy, n++);
                } else if (out_vpts != nullptr &&
                           (out_vpts[n - 1].x != static_cast<float>(cx) ||
                            out_vpts[n - 1].y != static_cast<float>(cy))) {
                    if (n >= cap) break;
                    record(cx, cy, n++);
                } else if (out_cpts != nullptr &&
                           (out_cpts[n - 1].x != static_cast<uint16_t>(cx) ||
                            out_cpts[n - 1].y != static_cast<uint16_t>(cy))) {
                    if (n >= cap) break;
                    record(cx, cy, n++);
                }
                bx = cx - dx8[(probe + 7) & 7];
                by = cy - dy8[(probe + 7) & 7];
                cx = nx;
                cy = ny;
                dir = (probe + 6) & 7;
                found = true;
                break;
            }
        }

        if (!found) {
            if (n == 0) {
                record(seed_x, seed_y, 0);
                n = 1;
            }
            break;
        }

        if (cx == seed_x && cy == seed_y && bx == seed_x - 1 && by == seed_y && n >= 4) {
            break;
        }
        if (n >= cap) break;
    }

    if (n > 2) {
        // Drop duplicated closing vertex when present.
        if (out_cpts != nullptr &&
            out_cpts[0].x == out_cpts[n - 1].x &&
            out_cpts[0].y == out_cpts[n - 1].y) {
            n--;
        } else if (out_vpts != nullptr &&
                   out_vpts[0].x == out_vpts[n - 1].x &&
                   out_vpts[0].y == out_vpts[n - 1].y) {
            n--;
        }
    }
    return n;
}

} // namespace

uint16_t shape_trace_boundary(
    const uint8_t* label_mask, uint16_t w, uint16_t h, uint8_t label,
    uint16_t min_x, uint16_t min_y,
    uint16_t max_x, uint16_t max_y,
    ContourPoint* out_pts, uint16_t cap) {

    (void)max_x;
    (void)max_y;
    if (label_mask == nullptr || out_pts == nullptr || cap < 4 || label == 0) {
        return 0;
    }

    auto pred = [&](int x, int y) -> bool {
        if (x < 0 || y < 0 || x >= static_cast<int>(w) || y >= static_cast<int>(h)) {
            return false;
        }
        return label_mask[static_cast<uint32_t>(y) * w + x] == label;
    };

    if (!pred(min_x, min_y)) return 0;
    return trace_core(min_x, min_y, pred, nullptr, out_pts, cap);
}

uint8_t shape_corner_count(
    const ContourPoint* pts, uint16_t n,
    float angle_threshold_deg) {

    if (pts == nullptr || n < 8) return 0;

    // Neighborhood half-width for direction smoothing: k=2 suppresses single
    // raster staircases while preserving true corners at marker scale.
    constexpr int K = 2;
    const float cos_thresh = std::cos(angle_threshold_deg * 3.14159265358979323846f / 180.0f);

    uint8_t corners = 0;
    bool prev_corner = false;
    for (uint16_t i = 0; i < n; ++i) {
        // Incoming direction averaged over [i-K, i).
        float in_x = 0.0f, in_y = 0.0f;
        for (int k = -K; k <= -1; ++k) {
            const ContourPoint& a = pts[(i + k + n) % n];
            const ContourPoint& b = pts[(i + k + 1 + n) % n];
            in_x += static_cast<float>(b.x - a.x);
            in_y += static_cast<float>(b.y - a.y);
        }
        // Outgoing direction averaged over (i, i+K].
        float out_x = 0.0f, out_y = 0.0f;
        for (int k = 1; k <= K; ++k) {
            const ContourPoint& a = pts[(i + k - 1 + n) % n];
            const ContourPoint& b = pts[(i + k + n) % n];
            out_x += static_cast<float>(b.x - a.x);
            out_y += static_cast<float>(b.y - a.y);
        }

        const float nin = std::sqrt(in_x * in_x + in_y * in_y);
        const float nout = std::sqrt(out_x * out_x + out_y * out_y);
        if (nin < 1e-3f || nout < 1e-3f) {
            prev_corner = false;
            continue;
        }

        const float cosang = (in_x * out_x + in_y * out_y) / (nin * nout);
        const bool is_corner = cosang < cos_thresh; // sharp turn => low cosine

        // Rising-edge count: one corner per convex vertex run.
        if (is_corner && !prev_corner && corners < 255) {
            corners++;
        }
        prev_corner = is_corner;
    }
    return corners;
}

float shape_concavity_depth(
    const ContourPoint* pts, uint16_t n,
    const float* hull_fx, const float* hull_fy, uint8_t hull_n) {

    if (pts == nullptr || hull_fx == nullptr || hull_fy == nullptr ||
        n < 8 || hull_n < 3) {
        return 0.0f;
    }

    // Contour bbox diagonal for normalization.
    float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;
    for (uint16_t i = 0; i < n; ++i) {
        const float x = static_cast<float>(pts[i].x);
        const float y = static_cast<float>(pts[i].y);
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }
    const float diag = std::sqrt((max_x - min_x) * (max_x - min_x) +
                                 (max_y - min_y) * (max_y - min_y));
    if (diag < 1e-3f) return 0.0f;

    // Max inward perpendicular distance from any contour point to any hull edge.
    float max_depth = 0.0f;
    for (uint16_t i = 0; i < n; ++i) {
        const float px = static_cast<float>(pts[i].x);
        const float py = static_cast<float>(pts[i].y);
        for (uint8_t e = 0; e < hull_n; ++e) {
            const float ax = hull_fx[e];
            const float ay = hull_fy[e];
            const float bx = hull_fx[(e + 1) % hull_n];
            const float by = hull_fy[(e + 1) % hull_n];
            const float ex = bx - ax;
            const float ey = by - ay;
            const float len = std::sqrt(ex * ex + ey * ey);
            if (len < 1e-6f) continue;
            // Inward normal (clockwise hull, image coords y-down).
            const float nx_ = ey / len;
            const float ny_ = -ex / len;
            const float d = (px - ax) * nx_ + (py - ay) * ny_;
            if (d > max_depth) max_depth = d;
        }
    }
    return max_depth / diag;
}

} // namespace RobofestDrone
