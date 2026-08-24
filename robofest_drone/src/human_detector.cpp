#include "human_detector.h"
#include "mem.h"
#include "../config/thresholds.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

// ============================================================================
// PURE CORES
// ============================================================================

void human_detector_frame_diff(
    const uint8_t* gray, const uint8_t* prev,
    uint16_t n, uint8_t threshold,
    uint8_t* out_mask) {

    if (gray == nullptr || prev == nullptr || out_mask == nullptr) return;
    for (uint16_t i = 0; i < n; ++i) {
        const int d = static_cast<int>(gray[i]) - static_cast<int>(prev[i]);
        out_mask[i] = (d < 0 ? -d : d) > threshold ? 1 : 0;
    }
}

uint32_t human_detector_largest_component(
    const uint8_t* mask, uint16_t w, uint16_t h,
    uint16_t min_pixels,
    float& out_cx, float& out_cy,
    uint16_t& out_bw, uint16_t& out_bh) {

    out_cx = 0.0f;
    out_cy = 0.0f;
    out_bw = 0;
    out_bh = 0;
    if (mask == nullptr || w == 0 || h == 0) return 0;

    // Flood fill with caller-independent scratch sized to the image.
    static int32_t* qx = nullptr;
    static int32_t* qy = nullptr;
    static uint8_t* visited = nullptr;
    static size_t cap = 0;

    const size_t n = static_cast<size_t>(w) * h;
    if (cap < n) {
        delete[] qx; delete[] qy; delete[] visited;
        qx = new (std::nothrow) int32_t[n]();
        qy = new (std::nothrow) int32_t[n]();
        visited = new (std::nothrow) uint8_t[n]();
        cap = n;
    }
    if (qx == nullptr || qy == nullptr || visited == nullptr) return 0;
    std::memset(visited, 0, n);

    uint32_t best_count = 0;
    float best_cx = 0.0f, best_cy = 0.0f;
    uint16_t best_bw = 0, best_bh = 0;

    for (uint16_t sy = 0; sy < h; ++sy) {
        for (uint16_t sx = 0; sx < w; ++sx) {
            const size_t sidx = static_cast<size_t>(sy) * w + sx;
            if (mask[sidx] == 0 || visited[sidx]) continue;

            // BFS from seed.
            size_t head = 0, tail = 0;
            qx[tail] = sx;
            qy[tail] = sy;
            tail++;
            visited[sidx] = 1;

            uint32_t count = 0;
            uint64_t sum_x = 0, sum_y = 0;
            uint16_t min_x = sx, max_x = sx;
            uint16_t min_y = sy, max_y = sy;

            while (head < tail && tail < n) {
                const int32_t cx = qx[head];
                const int32_t cy = qy[head];
                head++;

                count++;
                sum_x += cx;
                sum_y += cy;
                if (cx < min_x) min_x = cx;
                if (cx > max_x) max_x = cx;
                if (cy < min_y) min_y = cy;
                if (cy > max_y) max_y = cy;

                constexpr int dx4[4] = {1, -1, 0, 0};
                constexpr int dy4[4] = {0, 0, 1, -1};
                for (int d = 0; d < 4; ++d) {
                    const int nx = cx + dx4[d];
                    const int ny = cy + dy4[d];
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    const size_t nidx = static_cast<size_t>(ny) * w + nx;
                    if (mask[nidx] != 0 && !visited[nidx] && tail < n) {
                        visited[nidx] = 1;
                        qx[tail] = nx;
                        qy[tail] = ny;
                        tail++;
                    }
                }
            }

            if (count > best_count && count >= min_pixels) {
                best_count = count;
                best_cx = static_cast<float>(sum_x) / count;
                best_cy = static_cast<float>(sum_y) / count;
                best_bw = static_cast<uint16_t>(max_x - min_x + 1);
                best_bh = static_cast<uint16_t>(max_y - min_y + 1);
            }
        }
    }

    out_cx = best_cx;
    out_cy = best_cy;
    out_bw = best_bw;
    out_bh = best_bh;
    return best_count;
}

// ============================================================================
// DETECTOR CLASS
// ============================================================================

HumanMotionDetector::HumanMotionDetector() = default;

bool HumanMotionDetector::init(uint16_t w, uint16_t h) {
    reset();
    if (w == 0 || h == 0) return false;

    w_ = w;
    h_ = h;
    const size_t n = static_cast<size_t>(w) * h;
    prev_gray_ = static_cast<uint8_t*>(robofest_big_alloc(n));
    background_ = static_cast<uint8_t*>(robofest_big_alloc(n));
    diff_mask_ = static_cast<uint8_t*>(robofest_big_alloc(n));
    queue_x_ = static_cast<int32_t*>(robofest_big_alloc(n * sizeof(int32_t)));
    queue_y_ = static_cast<int32_t*>(robofest_big_alloc(n * sizeof(int32_t)));

    if (!prev_gray_ || !background_ || !diff_mask_ || !queue_x_ || !queue_y_) {
        reset();
        return false;
    }
    std::memset(prev_gray_, 0, n);
    std::memset(background_, 0, n);
    background_valid_ = false;
    static_frames_ = 0;
    return true;
}

void HumanMotionDetector::reset() {
    robofest_big_free(prev_gray_);
    robofest_big_free(background_);
    robofest_big_free(diff_mask_);
    robofest_big_free(queue_x_);
    robofest_big_free(queue_y_);
    prev_gray_ = nullptr;
    background_ = nullptr;
    diff_mask_ = nullptr;
    queue_x_ = nullptr;
    queue_y_ = nullptr;
    background_valid_ = false;
    static_frames_ = 0;
    confirm_hits_ = 0;
    last_emit_ms_ = 0;
}

bool HumanMotionDetector::process(
    const uint8_t* gray,
    Types::HumanDetectionSample& out,
    uint32_t now_ms) {

    out.valid = false;
    if (!isInitialized() || gray == nullptr) return false;

    const uint16_t n = static_cast<uint16_t>(w_) * h_;

    // First frame seeds the background reference (no motion possible yet).
    if (!background_valid_) {
        std::memcpy(background_, gray, n);
        std::memcpy(prev_gray_, gray, n);
        background_valid_ = true;
        return false;
    }

    // Background subtraction: a person differs from the STATIC GROUND for as
    // long as they remain in view - unlike prev-frame differencing, which
    // only lights up thin leading/trailing edge strips of a walk-by.
    human_detector_frame_diff(gray, background_, n,
                              Config::HUMAN_DIFF_THRESHOLD, diff_mask_);
    std::memcpy(prev_gray_, gray, n);

    float cx = 0.0f, cy = 0.0f;
    uint16_t bw = 0, bh = 0;
    const uint32_t px = human_detector_largest_component(
        diff_mask_, w_, h_,
        Config::HUMAN_MOTION_MIN_BLOB_PX, cx, cy, bw, bh);

    bool plausible = px > 0;
    if (plausible) {
        // Size ceiling: motion filling a quarter of the frame is camera
        // shake or lighting change, not a person at mission altitude.
        const float area_ratio =
            static_cast<float>(px) / static_cast<float>(n);
        if (area_ratio > Config::HUMAN_MOTION_MAX_AREA_RATIO) plausible = false;

        // Aspect sanity: overhead person bbox stays roughly compact.
        const float aspect = (bw >= bh)
            ? static_cast<float>(bw) / static_cast<float>(bh)
            : static_cast<float>(bh) / static_cast<float>(bw);
        if (aspect > Config::HUMAN_MOTION_MAX_ASPECT) plausible = false;
    }

    if (!plausible) {
        confirm_hits_ = 0;

        // Static scene: refresh the reference slowly toward current pixels so
        // genuine scene changes (landed drone, new ground texture) stop
        // producing permanent false motion after a while.
        if (++static_frames_ >= 6) {
            std::memcpy(background_, gray, n);
        }
        return false;
    }
    static_frames_ = 0;

    confirm_hits_++;
    if (confirm_hits_ < Config::HUMAN_MOTION_CONFIRM_HITS) return false;

    const bool inside_refractory =
        last_emit_ms_ != 0 &&
        (now_ms - last_emit_ms_) < Config::HUMAN_MOTION_REFRACTORY_MS;

    // Continuation suppression: same blob near the last emission is the SAME
    // person lingering/standing, not a fresh event. Only a genuinely new
    // appearance (different location or after quiet period) re-triggers.
    const float dx = cx - last_cx_;
    const float dy = cy - last_cy_;
    const float near_last =
        std::sqrt(dx * dx + dy * dy) <=
        static_cast<float>((bw > bh ? bw : bh) + 4);
    if (inside_refractory && near_last) {
        return false;
    }

    last_emit_ms_ = now_ms;
    last_cx_ = cx;
    last_cy_ = cy;

    out.valid = true;
    out.confidence =
        std::min(1.0f,
                 0.4f + 0.6f * std::min(1.0f,
                     static_cast<float>(px) /
                     (static_cast<float>(Config::HUMAN_MOTION_MIN_BLOB_PX) * 8)));
    out.pixel_x = cx;
    out.pixel_y = cy;
    out.pixel_width = static_cast<float>(bw);
    out.pixel_height = static_cast<float>(bh);
    out.timestamp_ms = now_ms;
    last_cx_ = cx;
    last_cy_ = cy;
    return true;
}

} // namespace RobofestDrone
