#include "vision_pipeline.h"
#include "mem.h"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include "../hal/hal_system.h"

namespace RobofestDrone {

namespace {
    static VisionPipeline s_global_vision_pipeline;

    constexpr float M_PI_F = 3.14159265358979323846f;
    constexpr float DEG_TO_RAD = M_PI_F / 180.0f;

    // Gray-world blend strength used by exposure normalization (0 = off, 1 = full).
    constexpr float EXPOSURE_GRAY_WORLD_STRENGTH = 0.5f;

    // Large scratch buffers live in PSRAM (see mem.h) to keep internal DRAM
    // free for the radio/WiFi stack. Indexed exactly like the former arrays;
    // allocated once on first use and never freed (process lifetime).
    static uint8_t* s_binary_mask = nullptr;      // [W*H]
    static uint8_t* s_morph_temp = nullptr;       // [W*H]
    static uint8_t* s_label_snapshot = nullptr;   // [W*H]
    static uint16_t* s_bfs_x = nullptr;           // [W*H] BFS queue X
    static uint16_t* s_bfs_y = nullptr;           // [W*H] BFS queue Y

    // Returns false if any buffer could not be allocated (OOM).
    static bool ensure_vision_scratch() {
        constexpr size_t N =
            static_cast<size_t>(Config::VISION_PROCESS_WIDTH) *
            static_cast<size_t>(Config::VISION_PROCESS_HEIGHT);
        if (s_binary_mask == nullptr) {
            s_binary_mask = static_cast<uint8_t*>(robofest_big_alloc(N));
        }
        if (s_morph_temp == nullptr) {
            s_morph_temp = static_cast<uint8_t*>(robofest_big_alloc(N));
        }
        if (s_label_snapshot == nullptr) {
            s_label_snapshot = static_cast<uint8_t*>(robofest_big_alloc(N));
        }
        if (s_bfs_x == nullptr) {
            s_bfs_x = static_cast<uint16_t*>(robofest_big_alloc(N * sizeof(uint16_t)));
        }
        if (s_bfs_y == nullptr) {
            s_bfs_y = static_cast<uint16_t*>(robofest_big_alloc(N * sizeof(uint16_t)));
        }
        return s_binary_mask != nullptr && s_morph_temp != nullptr &&
               s_label_snapshot != nullptr && s_bfs_x != nullptr && s_bfs_y != nullptr;
    }
    static VisionBlob s_extracted_blobs[Config::VISION_MAX_BLOBS];

    // Boundary-pixel collection during BFS (subsampled on overflow) plus the
    // scratch buffers used by hull/corner descriptor computation.
    static uint16_t s_bnd_x[Config::VISION_BOUNDARY_POINTS_MAX];
    static uint16_t s_bnd_y[Config::VISION_BOUNDARY_POINTS_MAX];
    static VisionPoint s_pt_scratch[Config::VISION_HULL_POINTS_MAX];

    static inline float cross_prod(const VisionPoint& o, const VisionPoint& a, const VisionPoint& b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    }

    // Decode one pixel from any supported RGB frame format.
    static inline bool fetch_rgb(const Hal::CameraFrame& f, uint16_t x, uint16_t y,
                                 uint8_t& r, uint8_t& g, uint8_t& b) {
        if (f.data == nullptr || x >= f.width || y >= f.height) return false;
        if (f.format == Hal::PixelFormat::PIXEL_FORMAT_RGB565) {
            uint32_t i = (static_cast<uint32_t>(y) * f.width + x) * 2;
            if (i + 1 >= f.buffer_size) return false;
            uint16_t p = Config::RGB565_LE_BYTE_ORDER
                ? static_cast<uint16_t>(f.data[i] | (f.data[i + 1] << 8))
                : static_cast<uint16_t>(f.data[i + 1] | (f.data[i] << 8));
            r = static_cast<uint8_t>(((p >> 11) & 0x1F) << 3);
            g = static_cast<uint8_t>(((p >> 5) & 0x3F) << 2);
            b = static_cast<uint8_t>((p & 0x1F) << 3);
            return true;
        }
        if (f.format == Hal::PixelFormat::PIXEL_FORMAT_RGB888) {
            uint32_t i = (static_cast<uint32_t>(y) * f.width + x) * 3;
            r = f.data[i];
            g = f.data[i + 1];
            b = f.data[i + 2];
            return true;
        }
        return false;
    }
}

// ============================================================================
// TESTABLE DESCRIPTOR / SCORING HELPERS
// ============================================================================

bool vision_hsv_in_band(
    uint8_t h, uint8_t s, uint8_t v,
    uint8_t h_min, uint8_t h_max,
    uint8_t s_min, uint8_t s_max,
    uint8_t v_min, uint8_t v_max)
{
    bool h_ok = (h_min <= h_max)
        ? (h >= h_min && h <= h_max)
        : (h >= h_min || h <= h_max);
    return h_ok && s >= s_min && s <= s_max && v >= v_min && v <= v_max;
}

float vision_range_score(float value, float lo, float hi)
{
    if (hi < lo) return 1.0f; // degenerate gate -> neutral
    if (value >= lo && value <= hi) return 1.0f;
    float span = hi - lo;
    float margin = span * Config::CONF_GATE_SOFT_MARGIN_RATIO;
    if (margin < 0.05f) margin = 0.05f;
    float dist = (value < lo) ? (lo - value) : (value - hi);
    float score = 1.0f - dist / margin;
    if (score < 0.0f) score = 0.0f;
    return score;
}

float vision_floor_score(float value, float floor_value)
{
    if (value >= floor_value) return 1.0f;
    float margin = floor_value * Config::CONF_GATE_SOFT_MARGIN_RATIO;
    if (margin < 0.05f) margin = 0.05f;
    float score = 1.0f - (floor_value - value) / margin;
    if (score < 0.0f) score = 0.0f;
    return score;
}

float vision_corner_score(uint8_t corners, uint8_t cmin, uint8_t cmax)
{
    if (cmin == 0 || cmax == 0 || cmin > cmax) return 1.0f; // gating disabled
    if (corners >= cmin && corners <= cmax) return 1.0f;
    float dist = (corners < cmin) ? static_cast<float>(cmin - corners)
                                  : static_cast<float>(corners - cmax);
    float score = 1.0f - dist / 3.0f; // fully mismatched three corners away
    if (score < 0.0f) score = 0.0f;
    return score;
}

float vision_shape_match(
    float aspect, float extent, float solidity, uint8_t corners,
    const VisionMarkerProfile& profile)
{
    float aspect_score = vision_range_score(aspect, profile.aspect_min, profile.aspect_max);
    float extent_score = vision_range_score(extent, profile.extent_min, profile.extent_max);
    float solidity_score = vision_floor_score(solidity, profile.solidity_min);
    float corner_score = vision_corner_score(corners, profile.corners_min, profile.corners_max);
    return 0.15f * aspect_score +
           0.35f * extent_score +
           0.30f * solidity_score +
           0.20f * corner_score;
}

float vision_blob_confidence(const VisionBlob& blob, const VisionMarkerProfile& profile)
{
    float circ_ratio = blob.circularity / Config::CONF_CIRC_PERFECT_AT;
    if (circ_ratio > 1.0f) circ_ratio = 1.0f;
    float circ_term = Config::CONF_WEIGHT_CIRCULARITY * circ_ratio;

    float area_ratio = blob.area /
        static_cast<float>(profile.expected_marker_area_px > 0 ? profile.expected_marker_area_px : 1);
    if (area_ratio > 1.0f) area_ratio = 1.0f;
    float area_term = Config::CONF_WEIGHT_AREA * area_ratio;

    float shape = vision_shape_match(
        blob.aspect_ratio, blob.extent, blob.solidity, blob.corner_count, profile);

    float confidence = circ_term + Config::CONF_WEIGHT_SHAPE_MATCH * shape +
                       area_term + profile.confidence_bias;
    if (confidence > 100.0f) confidence = 100.0f;
    if (confidence < 0.0f) confidence = 0.0f;
    return confidence;
}

float vision_polygon_area(const VisionPoint* pts, uint8_t n)
{
    if (pts == nullptr || n < 3) return 0.0f;
    float sum = 0.0f;
    for (uint8_t i = 0; i < n; ++i) {
        const VisionPoint& a = pts[i];
        const VisionPoint& b = pts[(i + 1) % n];
        sum += a.x * b.y - b.x * a.y;
    }
    return std::fabs(sum) * 0.5f;
}

uint8_t vision_convex_hull(const VisionPoint* pts, uint8_t n, VisionPoint* out, uint8_t out_cap)
{
    static VisionPoint sorted[Config::VISION_BOUNDARY_POINTS_MAX];
    static VisionPoint build[2 * Config::VISION_BOUNDARY_POINTS_MAX];

    if (pts == nullptr || out == nullptr || n < 3 || out_cap < 3) return 0;

    std::memcpy(sorted, pts, n * sizeof(VisionPoint));
    std::sort(sorted, sorted + n, [](const VisionPoint& a, const VisionPoint& b) {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    });

    int k = 0;
    for (int i = 0; i < n; ++i) {
        while (k >= 2 && cross_prod(build[k - 2], build[k - 1], sorted[i]) <= 0.0f) k--;
        build[k++] = sorted[i];
    }
    int lower_bound_idx = k + 1;
    for (int i = n - 2; i >= 0; --i) {
        while (k >= lower_bound_idx && cross_prod(build[k - 2], build[k - 1], sorted[i]) <= 0.0f) k--;
        build[k++] = sorted[i];
    }

    int hull_n = k - 1; // last entry duplicates the first
    if (hull_n < 3) return 0;
    if (hull_n > out_cap) hull_n = out_cap;
    std::memcpy(out, build, hull_n * sizeof(VisionPoint));
    return static_cast<uint8_t>(hull_n);
}

uint8_t vision_poly_corner_count(const VisionPoint* poly, uint8_t n, float epsilon)
{
    static bool kept[Config::VISION_HULL_POINTS_MAX];
    struct Seg { int16_t i; int16_t j; };
    static Seg stack[Config::VISION_HULL_POINTS_MAX + 2];

    if (poly == nullptr || n < 3) return n;

    uint8_t m = n;
    if (m > Config::VISION_HULL_POINTS_MAX) m = Config::VISION_HULL_POINTS_MAX;

    std::memset(kept, 0, sizeof(bool) * m);

    uint8_t anchor_b = m / 2;
    kept[0] = true;
    if (anchor_b < m) kept[anchor_b] = true;

    int sp = 0;
    stack[sp++] = Seg{0, static_cast<int16_t>(anchor_b)};
    stack[sp++] = Seg{static_cast<int16_t>(anchor_b), static_cast<int16_t>(m)};

    while (sp > 0) {
        Seg s = stack[--sp];
        int seg_len = s.j - s.i;
        if (seg_len < 2) continue;

        const VisionPoint& A = poly[s.i % m];
        const VisionPoint& B = poly[s.j % m];
        float dx = B.x - A.x;
        float dy = B.y - A.y;
        float norm = std::sqrt(dx * dx + dy * dy);

        float d_max = -1.0f;
        int k_best = -1;
        for (int k = s.i + 1; k < s.j; ++k) {
            const VisionPoint& P = poly[k % m];
            float d;
            if (norm < 1e-6f) {
                float ex = P.x - A.x;
                float ey = P.y - A.y;
                d = std::sqrt(ex * ex + ey * ey);
            } else {
                d = std::fabs(dy * (P.x - A.x) - dx * (P.y - A.y)) / norm;
            }
            if (d > d_max) {
                d_max = d;
                k_best = k;
            }
        }

        if (d_max > epsilon && k_best > 0 && sp < Config::VISION_HULL_POINTS_MAX) {
            kept[k_best % m] = true;
            stack[sp++] = Seg{s.i, static_cast<int16_t>(k_best)};
            stack[sp++] = Seg{static_cast<int16_t>(k_best), s.j};
        }
    }

    uint8_t count = 0;
    for (uint8_t i = 0; i < m; ++i) {
        if (kept[i]) count++;
    }
    return count;
}


// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

VisionPipeline::VisionPipeline() {
    reset();
}

void VisionPipeline::init() {
    reset();
    Hal::hal_camera_init();
    camera_healthy_ = Hal::hal_camera_is_healthy();

    if (!ensure_vision_scratch()) {
        Hal::hal_log("[VISION][ERROR] Scratch buffer allocation failed (OOM).");
        setTelemetryEvent(TE_VISION_MASK_EMPTY);
        return;
    }

    // Lock camera exposure and white-balance to prevent auto-adjustment from
    // invalidating calibrated HSV values mid-flight.
    Hal::hal_camera_set_auto_exposure(Config::CAMERA_AUTO_EXPOSURE_ENABLED);
    if (Config::CAMERA_MANUAL_EXPOSURE_VALUE != 0) {
        Hal::hal_camera_set_exposure(Config::CAMERA_MANUAL_EXPOSURE_VALUE);
    }
    Hal::hal_camera_set_auto_whitebalance(Config::CAMERA_AUTO_WHITEBALANCE_ENABLED);

    if (Hal::hal_camera_is_stub()) {
        setTelemetryEvent(TE_VISION_MASK_EMPTY); // Distinct event: using stub camera, no real data
        Hal::hal_log("[VISION] Using stub camera - no real image data being processed.");
    }

    Types::TelemetryEvent evt;
    evt.timestamp_ms = Hal::hal_millis();
    evt.event_id = TE_VISION_PROFILE_TABLE_LOADED;
    evt.severity = Types::TELEMETRY_SEVERITY_INFO;
    evt.module_id = Types::TELEMETRY_MODULE_VISION;
    evt.value_a = static_cast<float>(profile_count_);
    Hal::hal_log("[VISION] Profile table initialized.");
    (void)evt;
}

void VisionPipeline::reset() {
    initProfiles();
    active_profile_type_ = Types::VisionMarkerType::ON_GROUND_MINE;
    scan_all_profiles_ = true;

    candidate_count_ = 0;
    for (uint8_t i = 0; i < Config::VISION_MAX_CANDIDATES; ++i) {
        candidates_[i] = Types::VisionCandidate();
    }

    for (uint8_t i = 0; i < Config::VISION_MAX_PERSISTENCE_TRACKS; ++i) {
        tracks_[i] = VisionPersistenceTrack();
    }
    next_track_id_ = 1;

    lighting_mode_ = VisionLightingMode::SUNNY_PRIMARY;
    last_lighting_switch_ms_ = 0;
    last_exposure_log_ms_ = 0;
    frame_mean_v_ = 0.0f;
    exposure_gain_ = 1.0f;
    gain_r_ = 1.0f;
    gain_g_ = 1.0f;
    gain_b_ = 1.0f;
    labels_present_bits_ = 0;
    for (uint8_t i = 0; i < Config::VISION_PROFILE_MAX; ++i) {
        label_pixel_counts_[i] = 0;
    }

    h_fov_deg_ = Config::H_FOV_DEG;
    v_fov_deg_ = Config::V_FOV_DEG;
    image_center_x_ = Config::IMAGE_CENTER_X;
    image_center_y_ = Config::IMAGE_CENTER_Y;

    attitude_compensation_enabled_ = Config::VISION_ATTITUDE_COMPENSATION_ENABLED;
    downscale_enabled_ = Config::VISION_DOWNSCALE_ENABLED;

    camera_healthy_ = false;
    last_frame_time_ms_ = 0;
    last_process_time_ms_ = 0;
    processing_duration_us_ = 0;
    frame_count_ = 0;
    frame_timeout_count_ = 0;
    dropped_frame_count_ = 0;

    last_telemetry_event_id_ = TE_VISION_INITIALIZED;
    telemetry_event_valid_ = true;
}

void VisionPipeline::initProfiles() {
    profile_count_ = 0;
    uint8_t table_rows = Config::VISION_PROFILE_COUNT;
    if (table_rows > Config::VISION_PROFILE_MAX) table_rows = Config::VISION_PROFILE_MAX;

    for (uint8_t i = 0; i < table_rows; ++i) {
        const Config::VisionProfileDef& row = Config::VISION_PROFILE_TABLE[i];
        VisionMarkerProfile& p = profiles_[profile_count_];

        p.profile_type = static_cast<Types::VisionMarkerType>(row.marker_type_id);
        p.enabled = row.enabled;

        p.h_min = row.primary.h_min;
        p.h_max = row.primary.h_max;
        p.s_min = row.primary.s_min;
        p.s_max = row.primary.s_max;
        p.v_min = row.primary.v_min;
        p.v_max = row.primary.v_max;

        p.has_alt_band = row.has_alt_band;
        p.alt_h_min = row.alt.h_min;
        p.alt_h_max = row.alt.h_max;
        p.alt_s_min = row.alt.s_min;
        p.alt_s_max = row.alt.s_max;
        p.alt_v_min = row.alt.v_min;
        p.alt_v_max = row.alt.v_max;

        p.min_area_px = row.min_area_px;
        p.max_area_px = row.max_area_px;
        p.circularity_min = row.circularity_min;
        p.confidence_bias = row.confidence_bias;
        p.expected_marker_area_px = row.expected_marker_area_px;

        p.aspect_min = row.shape.aspect_min;
        p.aspect_max = row.shape.aspect_max;
        p.extent_min = row.shape.extent_min;
        p.extent_max = row.shape.extent_max;
        p.solidity_min = row.shape.solidity_min;
        p.corners_min = row.shape.corners_min;
        p.corners_max = row.shape.corners_max;

        profile_count_++;
    }
}


// ============================================================================
// PROFILE ACCESS & CALIBRATION SETTERS
// ============================================================================

void VisionPipeline::setActiveProfile(Types::VisionMarkerType profile) {
    active_profile_type_ = profile;
    scan_all_profiles_ = false;
    for (uint8_t i = 0; i < profile_count_; ++i) {
        profiles_[i].enabled = (profiles_[i].profile_type == profile);
    }
}

void VisionPipeline::setAllProfilesEnabled(bool enabled) {
    scan_all_profiles_ = enabled;
    for (uint8_t i = 0; i < profile_count_; ++i) {
        profiles_[i].enabled = enabled;
    }
    if (!enabled) {
        for (uint8_t i = 0; i < profile_count_; ++i) {
            if (profiles_[i].profile_type == active_profile_type_) {
                profiles_[i].enabled = true;
            }
        }
    }
}

void VisionPipeline::setProfileEnabled(Types::VisionMarkerType type, bool enabled) {
    for (uint8_t i = 0; i < profile_count_; ++i) {
        if (profiles_[i].profile_type == type) {
            profiles_[i].enabled = enabled;
        }
    }
}

VisionMarkerProfile* VisionPipeline::getProfileByType(Types::VisionMarkerType type) {
    for (uint8_t i = 0; i < profile_count_; ++i) {
        if (profiles_[i].profile_type == type) return &profiles_[i];
    }
    return nullptr;
}

const VisionMarkerProfile* VisionPipeline::getProfileByType(Types::VisionMarkerType type) const {
    for (uint8_t i = 0; i < profile_count_; ++i) {
        if (profiles_[i].profile_type == type) return &profiles_[i];
    }
    return nullptr;
}

VisionMarkerProfile* VisionPipeline::getProfileByIndex(uint8_t index) {
    if (index >= profile_count_) return nullptr;
    return &profiles_[index];
}

const VisionMarkerProfile* VisionPipeline::getProfileByIndex(uint8_t index) const {
    if (index >= profile_count_) return nullptr;
    return &profiles_[index];
}

void VisionPipeline::restoreProfileDefaults() {
    initProfiles();
    scan_all_profiles_ = true;
}

void VisionPipeline::setCameraCalibration(
    float h_fov_deg,
    float v_fov_deg,
    float image_center_x,
    float image_center_y
) {
    if (h_fov_deg > 10.0f && v_fov_deg > 10.0f) {
        h_fov_deg_ = h_fov_deg;
        v_fov_deg_ = v_fov_deg;
        image_center_x_ = image_center_x;
        image_center_y_ = image_center_y;
    }
}

void VisionPipeline::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}

bool VisionPipeline::isFrameFresh(uint32_t now_ms) const {
    return (now_ms - last_frame_time_ms_) <= Config::CAMERA_STALL_TIMEOUT_MS;
}

Types::VisionCandidate VisionPipeline::getCandidate(uint8_t index) const {
    if (!camera_healthy_) {
        return Types::VisionCandidate();
    }
    if (index < candidate_count_) {
        return candidates_[index];
    }
    return Types::VisionCandidate();
}


// ============================================================================
// EXPOSURE METRIC & LIGHTING MODE SELECTION (STEPS 5-6)
// ============================================================================

void VisionPipeline::measureExposureAndLighting(const Hal::CameraFrame& frame, uint32_t now_ms) {
    frame_mean_v_ = 0.0f;
    exposure_gain_ = 1.0f;
    gain_r_ = 1.0f;
    gain_g_ = 1.0f;
    gain_b_ = 1.0f;

    if (frame.data == nullptr || frame.width == 0 || frame.height == 0 ||
        (frame.format != Hal::PixelFormat::PIXEL_FORMAT_RGB565 &&
         frame.format != Hal::PixelFormat::PIXEL_FORMAT_RGB888)) {
        return;
    }

    const uint16_t step = 4;
    uint32_t sum_r = 0, sum_g = 0, sum_b = 0, sum_v = 0;
    uint32_t samples = 0;

    for (uint16_t y = 0; y < frame.height; y += step) {
        for (uint16_t x = 0; x < frame.width; x += step) {
            uint8_t r = 0, g = 0, b = 0;
            if (!fetch_rgb(frame, x, y, r, g, b)) continue;
            uint8_t v = std::max(r, std::max(g, b));
            sum_r += r; sum_g += g; sum_b += b; sum_v += v;
            samples++;
        }
    }
    if (samples == 0) return;

    float mean_r = static_cast<float>(sum_r) / samples;
    float mean_g = static_cast<float>(sum_g) / samples;
    float mean_b = static_cast<float>(sum_b) / samples;
    float mean_v = static_cast<float>(sum_v) / samples;
    frame_mean_v_ = mean_v;

    if (Config::VISION_EXPOSURE_NORM_ENABLED && mean_v > 10.0f) {
        float base = Config::VISION_EXPOSURE_TARGET_MEAN_V / mean_v;
        if (base < Config::VISION_EXPOSURE_GAIN_MIN) base = Config::VISION_EXPOSURE_GAIN_MIN;
        if (base > Config::VISION_EXPOSURE_GAIN_MAX) base = Config::VISION_EXPOSURE_GAIN_MAX;

        float channel_avg = (mean_r + mean_g + mean_b) / 3.0f;
        auto gray_world_gain = [&](float mean_c) -> float {
            if (mean_c < 5.0f) return base;
            float g = base * (1.0f + EXPOSURE_GRAY_WORLD_STRENGTH * (channel_avg / mean_c - 1.0f));
            if (g < Config::VISION_EXPOSURE_GAIN_MIN) g = Config::VISION_EXPOSURE_GAIN_MIN;
            if (g > Config::VISION_EXPOSURE_GAIN_MAX) g = Config::VISION_EXPOSURE_GAIN_MAX;
            return g;
        };

        gain_r_ = gray_world_gain(mean_r);
        gain_g_ = gray_world_gain(mean_g);
        gain_b_ = gray_world_gain(mean_b);
        exposure_gain_ = base;
    }

    if (Config::VISION_LIGHTING_ADAPTIVE_ENABLED &&
        (now_ms - last_lighting_switch_ms_) >= Config::VISION_LIGHTING_HYSTERESIS_MS) {
        VisionLightingMode desired = (mean_v >= Config::VISION_LIGHTING_V_SUNNY_MIN)
            ? VisionLightingMode::SUNNY_PRIMARY
            : VisionLightingMode::OVERCAST_ALT;
        if (desired != lighting_mode_) {
            lighting_mode_ = desired;
            last_lighting_switch_ms_ = now_ms;
            setTelemetryEvent(TE_VISION_LIGHTING_MODE_CHANGED);
        } else {
            last_lighting_switch_ms_ = now_ms;
        }
    }

    if ((now_ms - last_exposure_log_ms_) >= Config::VISION_LIGHTING_HYSTERESIS_MS ||
        last_exposure_log_ms_ == 0) {
        last_exposure_log_ms_ = now_ms;
        Types::TelemetryEvent evt;
        evt.timestamp_ms = now_ms;
        evt.event_id = TE_VISION_EXPOSURE_METRIC;
        evt.severity = Types::TELEMETRY_SEVERITY_DEBUG;
        evt.module_id = Types::TELEMETRY_MODULE_VISION;
        evt.value_a = frame_mean_v_;
        evt.value_b = exposure_gain_;
        evt.context_id = static_cast<uint16_t>(lighting_mode_);
        (void)evt;
    }
}


// ============================================================================
// MULTI-COLOR HSV SEGMENTATION (SINGLE PASS LABEL MAP)
// ============================================================================

void VisionPipeline::segmentMultiLabelMask(const Hal::CameraFrame& frame) {
    constexpr size_t SCRATCH_N =
        static_cast<size_t>(Config::VISION_PROCESS_WIDTH) *
        static_cast<size_t>(Config::VISION_PROCESS_HEIGHT);
    std::memset(s_binary_mask, 0, SCRATCH_N);
    labels_present_bits_ = 0;
    for (uint8_t i = 0; i < Config::VISION_PROFILE_MAX; ++i) {
        label_pixel_counts_[i] = 0;
    }

    if (frame.data == nullptr || frame.width == 0 || frame.height == 0) {
        setTelemetryEvent(TE_VISION_MASK_EMPTY);
        return;
    }

    // Check pixel format once upfront - bail with distinct telemetry if unsupported,
    // rather than discovering it one pixel at a time and producing an empty mask
    // indistinguishable from "no markers visible."
    if (frame.format != Hal::PixelFormat::PIXEL_FORMAT_RGB565 &&
        frame.format != Hal::PixelFormat::PIXEL_FORMAT_RGB888) {
        setTelemetryEvent(TE_VISION_UNSUPPORTED_PIXEL_FORMAT);
        return;
    }

    uint16_t step_x = frame.width / Config::VISION_PROCESS_WIDTH;
    uint16_t step_y = frame.height / Config::VISION_PROCESS_HEIGHT;
    if (step_x == 0) step_x = 1;
    if (step_y == 0) step_y = 1;

    for (uint16_t py = 0; py < Config::VISION_PROCESS_HEIGHT; ++py) {
        uint16_t src_y = py * step_y;
        if (src_y >= frame.height) break;

        for (uint16_t px = 0; px < Config::VISION_PROCESS_WIDTH; ++px) {
            uint16_t src_x = px * step_x;
            if (src_x >= frame.width) break;

            uint8_t r = 0, g = 0, b = 0;
            if (!fetch_rgb(frame, src_x, src_y, r, g, b)) continue;

            // Optional 3x3 box blur: average the pixel with its neighbors to smooth
            // single-pixel sensor noise before HSV conversion. Only applies when
            // blur is enabled AND the pixel is not at the frame boundary.
            if (Config::VISION_BLUR_ENABLED &&
                src_x > 0 && src_y > 0 &&
                src_x < (frame.width - 1) && src_y < (frame.height - 1)) {
                uint32_t sum_r = r, sum_g = g, sum_b = b;
                uint8_t count = 1;
                // Sample 4 cardinal neighbors (cheaper than full 3x3, good enough for noise)
                static const int16_t ndx[4] = {-1, 1, 0, 0};
                static const int16_t ndy[4] = {0, 0, -1, 1};
                for (int n = 0; n < 4; ++n) {
                    uint8_t nr = 0, ng = 0, nb = 0;
                    if (!fetch_rgb(frame,
                                   static_cast<uint16_t>(src_x + ndx[n]),
                                   static_cast<uint16_t>(src_y + ndy[n]),
                                   nr, ng, nb)) {
                        continue;
                    }
                    sum_r += nr;
                    sum_g += ng;
                    sum_b += nb;
                    count++;
                }
                r = static_cast<uint8_t>(sum_r / count);
                g = static_cast<uint8_t>(sum_g / count);
                b = static_cast<uint8_t>(sum_b / count);
            }

            // Exposure normalization gains (Step 5) applied after blur.
            if (gain_r_ != 1.0f) {
                uint32_t rr = static_cast<uint32_t>(r * gain_r_ + 0.5f);
                if (rr > 255) rr = 255;
                r = static_cast<uint8_t>(rr);
            }
            if (gain_g_ != 1.0f) {
                uint32_t gg = static_cast<uint32_t>(g * gain_g_ + 0.5f);
                if (gg > 255) gg = 255;
                g = static_cast<uint8_t>(gg);
            }
            if (gain_b_ != 1.0f) {
                uint32_t bb = static_cast<uint32_t>(b * gain_b_ + 0.5f);
                if (bb > 255) bb = 255;
                b = static_cast<uint8_t>(bb);
            }

            // Fast inline RGB to HSV conversion
            uint8_t c_max = std::max(r, std::max(g, b));
            uint8_t c_min = std::min(r, std::min(g, b));
            uint8_t delta = c_max - c_min;

            uint8_t v = c_max;
            uint8_t s = (c_max == 0) ? 0 : static_cast<uint8_t>((255UL * delta) / c_max);
            uint8_t h = 0;

            if (delta > 0) {
                int16_t h_calc = 0;
                if (c_max == r) {
                    h_calc = 30 * (static_cast<int16_t>(g) - static_cast<int16_t>(b)) / delta;
                } else if (c_max == g) {
                    h_calc = 60 + 30 * (static_cast<int16_t>(b) - static_cast<int16_t>(r)) / delta;
                } else {
                    h_calc = 120 + 30 * (static_cast<int16_t>(r) - static_cast<int16_t>(g)) / delta;
                }
                if (h_calc < 0) h_calc += 180;
                h = static_cast<uint8_t>(h_calc);
            }

            // First-match-wins classification against every enabled profile band.
            uint32_t idx = static_cast<uint32_t>(py) * Config::VISION_PROCESS_WIDTH + px;
            for (uint8_t pi = 0; pi < profile_count_; ++pi) {
                const VisionMarkerProfile& p = profiles_[pi];
                if (!p.enabled) continue;

                bool use_alt = (lighting_mode_ == VisionLightingMode::OVERCAST_ALT) && p.has_alt_band;
                uint8_t bh_min = use_alt ? p.alt_h_min : p.h_min;
                uint8_t bh_max = use_alt ? p.alt_h_max : p.h_max;
                uint8_t bs_min = use_alt ? p.alt_s_min : p.s_min;
                uint8_t bs_max = use_alt ? p.alt_s_max : p.s_max;
                uint8_t bv_min = use_alt ? p.alt_v_min : p.v_min;
                uint8_t bv_max = use_alt ? p.alt_v_max : p.v_max;

                if (vision_hsv_in_band(h, s, v, bh_min, bh_max, bs_min, bs_max, bv_min, bv_max)) {
                    s_binary_mask[idx] = static_cast<uint8_t>(pi + 1);
                    labels_present_bits_ |= (1UL << pi);
                    label_pixel_counts_[pi]++;
                    break;
                }
            }
        }
    }
}


// ============================================================================
// PER-LABEL MORPHOLOGICAL NOISE CLEANUP (ERODE + DILATE ON EACH LABEL)
// ============================================================================

void VisionPipeline::applyMorphologyCleanup() {
    if (!Config::VISION_MORPHOLOGY_ENABLED) {
        return;
    }

    const uint16_t W = Config::VISION_PROCESS_WIDTH;
    const uint16_t H = Config::VISION_PROCESS_HEIGHT;
    const uint8_t R = Config::VISION_MORPHOLOGY_RADIUS;

    std::memcpy(s_label_snapshot, s_binary_mask, W * H);
    std::memset(s_binary_mask, 0, W * H);

    for (uint8_t pi = 0; pi < profile_count_; ++pi) {
        if (!(labels_present_bits_ & (1UL << pi))) continue;
        if (label_pixel_counts_[pi] < Config::VISION_MORPH_MIN_LABEL_PX) continue;

        const uint8_t label = static_cast<uint8_t>(pi + 1);

        // Build binary view of this label
        for (uint32_t idx = 0; idx < static_cast<uint32_t>(W) * H; ++idx) {
            s_morph_temp[idx] = (s_label_snapshot[idx] == label) ? 1 : 0;
        }

        // Pass 1: Erosion — pixel stays ON only if ALL neighbors within radius are ON.
        for (uint16_t y = 0; y < H; ++y) {
            for (uint16_t x = 0; x < W; ++x) {
                uint32_t idx = y * W + x;
                if (s_morph_temp[idx] == 0) {
                    s_binary_mask[idx] = 0;
                    continue;
                }
                bool all_set = true;
                for (int16_t dy = -R; dy <= R && all_set; ++dy) {
                    for (int16_t dx = -R; dx <= R && all_set; ++dx) {
                        int16_t nx = static_cast<int16_t>(x) + dx;
                        int16_t ny = static_cast<int16_t>(y) + dy;
                        if (nx < 0 || nx >= W || ny < 0 || ny >= H) {
                            all_set = false;
                        } else if (s_morph_temp[ny * W + nx] == 0) {
                            all_set = false;
                        }
                    }
                }
                s_binary_mask[idx] = all_set ? 1 : 0;
            }
        }

        // Pass 2: Dilation — pixel turns ON if ANY neighbor within radius is ON.
        std::memcpy(s_morph_temp, s_binary_mask, W * H);
        for (uint16_t y = 0; y < H; ++y) {
            for (uint16_t x = 0; x < W; ++x) {
                uint32_t idx = y * W + x;
                if (s_morph_temp[idx] != 0) continue;
                bool any_set = false;
                for (int16_t dy = -R; dy <= R && !any_set; ++dy) {
                    for (int16_t dx = -R; dx <= R && !any_set; ++dx) {
                        int16_t nx = static_cast<int16_t>(x) + dx;
                        int16_t ny = static_cast<int16_t>(y) + dy;
                        if (nx >= 0 && nx < W && ny >= 0 && ny < H) {
                            if (s_morph_temp[ny * W + nx] != 0) {
                                any_set = true;
                            }
                        }
                    }
                }
                if (any_set) {
                    s_binary_mask[idx] = 1;
                }
            }
        }

        // Write back with OR so previously processed labels are preserved.
        for (uint32_t idx = 0; idx < static_cast<uint32_t>(W) * H; ++idx) {
            if (s_binary_mask[idx] == 1) {
                s_binary_mask[idx] = label;
            }
        }
    }
}


// ============================================================================
// CONNECTED COMPONENT EXTRACTION WITH SHAPE DESCRIPTORS (Steps 7-9)
// ============================================================================

uint8_t VisionPipeline::extractBlobs() {
    uint8_t blob_count = 0;
    const uint16_t W = Config::VISION_PROCESS_WIDTH;
    const uint16_t H = Config::VISION_PROCESS_HEIGHT;

    // Reuse the morphology snapshot buffer as the "claimed" map for this scan.
    std::memset(s_label_snapshot, 0, W * H);

    float scale_x = static_cast<float>(Config::IMAGE_WIDTH) / static_cast<float>(W);
    float scale_y = static_cast<float>(Config::IMAGE_HEIGHT) / static_cast<float>(H);

    for (uint16_t y = 0; y < H; ++y) {
        for (uint16_t x = 0; x < W; ++x) {
            uint32_t idx = static_cast<uint32_t>(y) * W + x;
            uint8_t label = s_binary_mask[idx];

            if (label == 0 || s_label_snapshot[idx] != 0) continue;
            if (label > profile_count_) continue;

            const VisionMarkerProfile& profile =
                profiles_[label - 1];

            uint16_t q_head = 0;
            uint16_t q_tail = 0;
            s_bfs_x[q_tail] = x;
            s_bfs_y[q_tail] = y;
            q_tail++;
            s_label_snapshot[idx] = 1;

            uint32_t area = 0;
            uint32_t sum_x = 0;
            uint32_t sum_y = 0;
            uint16_t min_x = x, max_x = x;
            uint16_t min_y = y, max_y = y;
            uint32_t perimeter = 0;
            uint16_t bnd_count = 0;
            uint32_t bnd_total = 0;

            while (q_head < q_tail && q_tail < W * H) {
                uint16_t cx = s_bfs_x[q_head];
                uint16_t cy = s_bfs_y[q_head];
                q_head++;

                area++;
                sum_x += cx;
                sum_y += cy;

                if (cx < min_x) min_x = cx;
                if (cx > max_x) max_x = cx;
                if (cy < min_y) min_y = cy;
                if (cy > max_y) max_y = cy;

                // Check 4-connectivity neighbors
                static const int dx[4] = {0, 1, 0, -1};
                static const int dy[4] = {-1, 0, 1, 0};
                bool is_border_pixel = false;

                for (int d = 0; d < 4; ++d) {
                    int nx = static_cast<int>(cx) + dx[d];
                    int ny = static_cast<int>(cy) + dy[d];

                    if (nx < 0 || nx >= W || ny < 0 || ny >= H) {
                        is_border_pixel = true;
                    } else {
                        uint32_t n_idx = static_cast<uint32_t>(ny) * W + nx;
                        if (s_binary_mask[n_idx] == 0) {
                            is_border_pixel = true;
                        } else if (s_binary_mask[n_idx] != label) {
                            is_border_pixel = true; // adjacent other-color region
                        } else if (s_label_snapshot[n_idx] == 0 &&
                                   q_tail < W * H) {
                            s_label_snapshot[n_idx] = 1;
                            s_bfs_x[q_tail] = static_cast<uint16_t>(nx);
                            s_bfs_y[q_tail] = static_cast<uint16_t>(ny);
                            q_tail++;
                        }
                    }
                }

                if (is_border_pixel) {
                    perimeter++;
                    bnd_total++;
                    if (bnd_count < Config::VISION_BOUNDARY_POINTS_MAX) {
                        s_bnd_x[bnd_count] = cx;
                        s_bnd_y[bnd_count] = cy;
                        bnd_count++;
                    } else {
                        // Overwrite round-robin to keep spatial spread when overflowing
                        s_bnd_x[bnd_total % Config::VISION_BOUNDARY_POINTS_MAX] = cx;
                        s_bnd_y[bnd_total % Config::VISION_BOUNDARY_POINTS_MAX] = cy;
                    }
                }
            }

            if (q_tail >= W * H) {
                setTelemetryEvent(TE_VISION_BLOB_EXCEEDED_SCAN_BUFFER);
                continue;
            }

            if (blob_count >= Config::VISION_MAX_BLOBS) {
                return blob_count;
            }

            float full_area = static_cast<float>(area) * (scale_x * scale_y);
            float full_perimeter = static_cast<float>(perimeter) *
                std::sqrt((scale_x * scale_x + scale_y * scale_y) * 0.5f);

            if (full_perimeter < 1.0f) full_perimeter = 1.0f;

            // Circularity metric: 4 * PI * Area / (Perimeter^2)
            float circularity = (4.0f * M_PI_F * full_area) / (full_perimeter * full_perimeter);
            if (circularity > 1.0f) circularity = 1.0f;
            if (circularity < 0.0f) circularity = 0.0f;

            // Glare rejection before general area filter (unchanged thresholds)
            if (Config::GLARE_REJECT_ENABLED && full_area > Config::GLARE_AREA_MAX_PX) {
                setTelemetryEvent(TE_VISION_BLOB_REJECTED_GLARE);
                continue;
            }

            if (full_area < profile.min_area_px || full_area > profile.max_area_px) {
                setTelemetryEvent(TE_VISION_BLOB_REJECTED_AREA);
                continue;
            }

            if (circularity < profile.circularity_min) {
                setTelemetryEvent(TE_VISION_BLOB_REJECTED_CIRCULARITY);
                continue;
            }

            float full_min_x = static_cast<float>(min_x) * scale_x;
            float full_max_x = static_cast<float>(max_x) * scale_x;
            float full_min_y = static_cast<float>(min_y) * scale_y;
            float full_max_y = static_cast<float>(max_y) * scale_y;

            if (full_min_x <= Config::EDGE_REJECT_MARGIN_PX ||
                full_max_x >= (Config::IMAGE_WIDTH - Config::EDGE_REJECT_MARGIN_PX) ||
                full_min_y <= Config::EDGE_REJECT_MARGIN_PX ||
                full_max_y >= (Config::IMAGE_HEIGHT - Config::EDGE_REJECT_MARGIN_PX)) {
                setTelemetryEvent(TE_VISION_BLOB_REJECTED_EDGE);
                continue;
            }

            // ---- Shape descriptors (Step 7): aspect + extent are free; solidity
            // and corners come from the convex hull over boundary samples. ----
            float bbox_w = static_cast<float>(max_x - min_x + 1);
            float bbox_h = static_cast<float>(max_y - min_y + 1);
            float aspect = (bbox_w >= bbox_h) ? (bbox_w / bbox_h) : (bbox_h / bbox_w);
            float extent = static_cast<float>(area) / (bbox_w * bbox_h);
            float solidity = 1.0f;
            uint8_t corners = 0;

            uint8_t n_pts = 0;
            if (bnd_count > 0) {
                uint16_t stride = 1;
                if (bnd_count > Config::VISION_HULL_POINTS_MAX) {
                    stride = static_cast<uint16_t>(bnd_count / Config::VISION_HULL_POINTS_MAX) + 1;
                }
                for (uint16_t i = 0; i < bnd_count && n_pts < Config::VISION_HULL_POINTS_MAX; i += stride) {
                    s_pt_scratch[n_pts].x = static_cast<float>(s_bnd_x[i]);
                    s_pt_scratch[n_pts].y = static_cast<float>(s_bnd_y[i]);
                    n_pts++;
                }
            }

            VisionPoint hull[Config::VISION_HULL_POINTS_MAX];
            uint8_t hull_n = vision_convex_hull(s_pt_scratch, n_pts, hull, Config::VISION_HULL_POINTS_MAX);
            if (hull_n >= 3) {
                float hull_area = vision_polygon_area(hull, hull_n);
                if (hull_area > 0.5f) {
                    solidity = static_cast<float>(area) / hull_area;
                    if (solidity > 1.0f) solidity = 1.0f;
                    if (solidity < 0.0f) solidity = 0.0f;
                }
                if (Config::VISION_CORNER_DETECT_ENABLED &&
                    profile.corners_min > 0 && profile.corners_max > 0) {
                    float bbox_diag = std::sqrt(bbox_w * bbox_w + bbox_h * bbox_h);
                    float epsilon = Config::SHAPE_CORNER_EPSILON_RATIO * bbox_diag;
                    corners = vision_poly_corner_count(hull, hull_n, epsilon);
                }
            }

            float shape_match_val = vision_shape_match(
                aspect, extent, solidity, corners, profile);
            if (shape_match_val < 0.30f) {
                setTelemetryEvent(TE_VISION_BLOB_REJECTED_SHAPE);
            }

            VisionBlob& blob = s_extracted_blobs[blob_count];
            blob.centroid_x = (static_cast<float>(sum_x) / static_cast<float>(area)) * scale_x;
            blob.centroid_y = (static_cast<float>(sum_y) / static_cast<float>(area)) * scale_y;
            blob.area = full_area;
            blob.perimeter = full_perimeter;
            blob.circularity = circularity;
            blob.x_min = static_cast<uint16_t>(full_min_x);
            blob.x_max = static_cast<uint16_t>(full_max_x);
            blob.y_min = static_cast<uint16_t>(full_min_y);
            blob.y_max = static_cast<uint16_t>(full_max_y);
            blob.aspect_ratio = aspect;
            blob.extent = extent;
            blob.solidity = solidity;
            blob.corner_count = corners;
            blob.profile_type = profile.profile_type;
            blob.valid = true;
            blob_count++;
        }
    }

    return blob_count;
}


// ============================================================================
// WORLD PROJECTION (GROUND PLANE RAY INTERSECTION)
// ============================================================================

void VisionPipeline::projectToWorld(
    float pixel_x,
    float pixel_y,
    float altitude_m,
    const Types::Pose2D& drone_pose,
    const Types::AttitudeSample& attitude,
    float& out_world_x,
    float& out_world_y,
    bool& out_valid
) {
    out_valid = true;  // assume valid unless we detect an issue

    if (altitude_m < Config::MIN_PROJECTION_ALTITUDE_M) {
        setTelemetryEvent(TE_VISION_ALTITUDE_TOO_LOW);
        out_valid = false;
        out_world_x = drone_pose.field_x;
        out_world_y = drone_pose.field_y;
        return;
    }

    // Normalized camera coordinate space
    float u = (pixel_x - image_center_x_) / (Config::IMAGE_WIDTH * 0.5f);
    float v = (pixel_y - image_center_y_) / (Config::IMAGE_HEIGHT * 0.5f);

    float tan_h_half = std::tan((h_fov_deg_ * 0.5f) * DEG_TO_RAD);
    float tan_v_half = std::tan((v_fov_deg_ * 0.5f) * DEG_TO_RAD);

    float ray_body_x = u * tan_h_half;
    float ray_body_y = v * tan_v_half;
    float ray_body_z = 1.0f; // Downward along optical axis

    if (attitude_compensation_enabled_ && attitude.valid) {
        if (std::abs(attitude.roll_deg) > 30.0f || std::abs(attitude.pitch_deg) > 30.0f) {
            setTelemetryEvent(TE_VISION_ATTITUDE_INVALID);
            out_valid = false;
            out_world_x = drone_pose.field_x;
            out_world_y = drone_pose.field_y;
            return;
        }

        if (std::abs(attitude.roll_deg) > 15.0f || std::abs(attitude.pitch_deg) > 15.0f) {
            setTelemetryEvent(TE_VISION_PROJECTION_DEGRADED);
        }

        // Apply Roll (phi), Pitch (theta), and Heading Yaw (psi) rotation
        float phi   = attitude.roll_deg * DEG_TO_RAD;
        float theta = attitude.pitch_deg * DEG_TO_RAD;
        float psi   = drone_pose.yaw_deg * DEG_TO_RAD;

        float cos_phi = std::cos(phi);
        float sin_phi = std::sin(phi);
        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);
        float cos_psi = std::cos(psi);
        float sin_psi = std::sin(psi);

        // Rotation matrix: R = R_z(psi) * R_y(theta) * R_x(phi)
        float r_z = -sin_theta * ray_body_x + sin_phi * cos_theta * ray_body_y + cos_phi * cos_theta * ray_body_z;
        if (r_z <= 0.05f) r_z = 0.05f; // Protect against grazing ground ray

        float r_x = (cos_psi * cos_theta) * ray_body_x +
                    (cos_psi * sin_theta * sin_phi - sin_psi * cos_phi) * ray_body_y +
                    (cos_psi * sin_theta * cos_phi + sin_psi * sin_phi) * ray_body_z;

        float r_y = (sin_psi * cos_theta) * ray_body_x +
                    (sin_psi * sin_theta * sin_phi + cos_psi * cos_phi) * ray_body_y +
                    (sin_psi * sin_theta * cos_phi - cos_psi * sin_phi) * ray_body_z;

        float scale = altitude_m / r_z;
        out_world_x = drone_pose.field_x + (r_x * scale);
        out_world_y = drone_pose.field_y + (r_y * scale);
    } else {
        // Fallback flat projection
        float yaw_rad = drone_pose.yaw_deg * DEG_TO_RAD;
        float cos_yaw = std::cos(yaw_rad);
        float sin_yaw = std::sin(yaw_rad);

        float offset_x = altitude_m * ray_body_x;
        float offset_y = altitude_m * ray_body_y;

        out_world_x = drone_pose.field_x + (offset_x * cos_yaw - offset_y * sin_yaw);
        out_world_y = drone_pose.field_y + (offset_x * sin_yaw + offset_y * cos_yaw);
    }
}


// ============================================================================
// TEMPORAL PERSISTENCE & CANDIDATE REPORTING
// ============================================================================

void VisionPipeline::pruneStaleTracks(uint32_t now_ms) {
    // Prune expired tracks on a wall-clock basis, regardless of camera state.
    for (uint8_t t = 0; t < Config::VISION_MAX_PERSISTENCE_TRACKS; ++t) {
        if (tracks_[t].active) {
            if ((now_ms - tracks_[t].last_seen_ms) > Config::PERSISTENCE_TIMEOUT_MS) {
                tracks_[t].active = false;
                setTelemetryEvent(TE_VISION_TRACK_EXPIRED);
            }
        }
    }
}

void VisionPipeline::updatePersistenceTracks(
    const VisionBlob* blobs,
    uint8_t blob_count,
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
) {
    // 1. Fuse new blob detections into persistence tracks
    for (uint8_t b = 0; b < blob_count; ++b) {
        if (!blobs[b].valid) continue;

        const VisionMarkerProfile* profile = getProfileByType(blobs[b].profile_type);
        if (profile == nullptr || !profile->enabled) continue;

        float world_x = 0.0f;
        float world_y = 0.0f;
        bool proj_valid = false;
        projectToWorld(blobs[b].centroid_x, blobs[b].centroid_y, fused_altitude_m, drone_pose, attitude, world_x, world_y, proj_valid);

        if (!proj_valid) {
            continue;
        }

        // Confidence calculation (Step 8 formula)
        float normalized_area_score = std::min(
            Config::CONF_WEIGHT_AREA,
            Config::CONF_WEIGHT_AREA * (blobs[b].area /
                static_cast<float>(profile->expected_marker_area_px > 0 ? profile->expected_marker_area_px : 1)));

        float confidence = vision_blob_confidence(blobs[b], *profile);

        if (confidence < Config::CONFIDENCE_REPORT_MIN) {
            continue;
        }

        // Check distance match with existing persistence tracks of the SAME marker
        // type. Skip tracks already claimed this frame so no track is double-claimed
        // by multiple blobs in the same frame.
        int match_idx = -1;
        float min_dist_sq = Config::PERSISTENCE_RADIUS_M * Config::PERSISTENCE_RADIUS_M;

        for (uint8_t t = 0; t < Config::VISION_MAX_PERSISTENCE_TRACKS; ++t) {
            if (tracks_[t].active && tracks_[t].marker_type == blobs[b].profile_type &&
                tracks_[t].frame_id != frame_count_) {
                float dx = tracks_[t].world_x - world_x;
                float dy = tracks_[t].world_y - world_y;
                float dist_sq = dx * dx + dy * dy;

                if (dist_sq <= min_dist_sq) {
                    min_dist_sq = dist_sq;
                    match_idx = t;
                }
            }
        }

        if (match_idx >= 0) {
            // Fuse existing track
            tracks_[match_idx].world_x = 0.7f * tracks_[match_idx].world_x + 0.3f * world_x;
            tracks_[match_idx].world_y = 0.7f * tracks_[match_idx].world_y + 0.3f * world_y;
            tracks_[match_idx].pixel_x = blobs[b].centroid_x;
            tracks_[match_idx].pixel_y = blobs[b].centroid_y;
            tracks_[match_idx].average_confidence = 0.6f * tracks_[match_idx].average_confidence + 0.4f * confidence;
            tracks_[match_idx].circularity = 0.6f * tracks_[match_idx].circularity + 0.4f * blobs[b].circularity;
            tracks_[match_idx].area = 0.6f * tracks_[match_idx].area + 0.4f * blobs[b].area;
            tracks_[match_idx].normalized_area_score = normalized_area_score;
            tracks_[match_idx].extent = 0.6f * tracks_[match_idx].extent + 0.4f * blobs[b].extent;
            tracks_[match_idx].solidity = 0.6f * tracks_[match_idx].solidity + 0.4f * blobs[b].solidity;
            tracks_[match_idx].corner_count = blobs[b].corner_count;
            tracks_[match_idx].persistence_count++;
            tracks_[match_idx].last_seen_ms = now_ms;
            tracks_[match_idx].frame_id = frame_count_;
            setTelemetryEvent(TE_VISION_TRACK_FUSED);
        } else {
            // Allocate new track
            for (uint8_t t = 0; t < Config::VISION_MAX_PERSISTENCE_TRACKS; ++t) {
                if (!tracks_[t].active) {
                    tracks_[t].track_id = next_track_id_++;
                    tracks_[t].world_x = world_x;
                    tracks_[t].world_y = world_y;
                    tracks_[t].pixel_x = blobs[b].centroid_x;
                    tracks_[t].pixel_y = blobs[b].centroid_y;
                    tracks_[t].average_confidence = confidence;
                    tracks_[t].circularity = blobs[b].circularity;
                    tracks_[t].area = blobs[b].area;
                    tracks_[t].normalized_area_score = normalized_area_score;
                    tracks_[t].extent = blobs[b].extent;
                    tracks_[t].solidity = blobs[b].solidity;
                    tracks_[t].corner_count = blobs[b].corner_count;
                    tracks_[t].persistence_count = 1;
                    tracks_[t].last_seen_ms = now_ms;
                    tracks_[t].marker_type = blobs[b].profile_type;
                    tracks_[t].frame_id = frame_count_;
                    tracks_[t].active = true;
                    setTelemetryEvent(TE_VISION_TRACK_CREATED);
                    break;
                }
            }
        }
    }

    // 2. Populate stable candidates exceeding persistence threshold
    candidate_count_ = 0;
    for (uint8_t t = 0; t < Config::VISION_MAX_PERSISTENCE_TRACKS; ++t) {
        if (tracks_[t].active && tracks_[t].persistence_count >= Config::PERSISTENCE_COUNT_MIN) {
            if (candidate_count_ < Config::VISION_MAX_CANDIDATES) {
                candidates_[candidate_count_].pixel_x = tracks_[t].pixel_x;
                candidates_[candidate_count_].pixel_y = tracks_[t].pixel_y;
                candidates_[candidate_count_].world_x = tracks_[t].world_x;
                candidates_[candidate_count_].world_y = tracks_[t].world_y;
                candidates_[candidate_count_].confidence = tracks_[t].average_confidence;
                candidates_[candidate_count_].circularity = tracks_[t].circularity;
                candidates_[candidate_count_].area = tracks_[t].area;
                candidates_[candidate_count_].normalized_area_score = tracks_[t].normalized_area_score;
                candidates_[candidate_count_].extent = tracks_[t].extent;
                candidates_[candidate_count_].solidity = tracks_[t].solidity;
                candidates_[candidate_count_].corner_count = tracks_[t].corner_count;
                candidates_[candidate_count_].persistence_count = tracks_[t].persistence_count;
                candidates_[candidate_count_].marker_type = tracks_[t].marker_type;
                candidates_[candidate_count_].frame_id = tracks_[t].frame_id;
                candidates_[candidate_count_].timestamp_ms = tracks_[t].last_seen_ms;
                candidate_count_++;
            }
        }
    }

    if (candidate_count_ > 0) {
        setTelemetryEvent(TE_VISION_CANDIDATE_REPORTED);
    }
}


// ============================================================================
// MAIN PIPELINE UPDATE DISPATCHER
// ============================================================================

void VisionPipeline::update(
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
) {
    uint32_t start_us = Hal::hal_micros();
    last_process_time_ms_ = now_ms;

    // Defensive: buffers may be missing if init() ran under OOM conditions.
    if (!ensure_vision_scratch()) {
        setTelemetryEvent(TE_VISION_MASK_EMPTY);
        return;
    }

    // Prune stale tracks on a wall-clock basis, even when no frame is retrieved.
    pruneStaleTracks(now_ms);

    Hal::CameraFrame frame;
    if (!Hal::hal_camera_get_frame(frame)) {
        frame_timeout_count_++;
        if ((now_ms - last_frame_time_ms_) > Config::CAMERA_STALL_TIMEOUT_MS) {
            camera_healthy_ = false;
            setTelemetryEvent(TE_VISION_FRAME_TIMEOUT);
        }
        // Safety net: refuse to return candidates when camera is unhealthy
        // so callers can't act on frozen, stale data.
        return;
    }

    last_frame_time_ms_ = now_ms;
    camera_healthy_ = true;
    frame_count_++;

    // Execute vision pipeline stages (all enabled profiles simultaneously)
    measureExposureAndLighting(frame, now_ms);
    segmentMultiLabelMask(frame);
    applyMorphologyCleanup();
    uint8_t blob_count = extractBlobs();
    updatePersistenceTracks(s_extracted_blobs, blob_count, drone_pose, fused_altitude_m, attitude, now_ms);

    uint32_t end_us = Hal::hal_micros();
    processing_duration_us_ = end_us - start_us;

    if (processing_duration_us_ > 20000UL) { // Over 20ms
        setTelemetryEvent(TE_VISION_PROCESSING_SLOW);
    }
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void vision_pipeline_init() {
    s_global_vision_pipeline.init();
}

void vision_pipeline_update(
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    uint32_t now_ms
) {
    Types::AttitudeSample attitude;
    attitude.valid = false;
    s_global_vision_pipeline.update(drone_pose, fused_altitude_m, attitude, now_ms);
}

void vision_pipeline_update_full(
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
) {
    s_global_vision_pipeline.update(drone_pose, fused_altitude_m, attitude, now_ms);
}

uint8_t vision_pipeline_get_candidates(Types::VisionCandidate* out_candidates, uint8_t max_count) {
    uint8_t count = s_global_vision_pipeline.getCandidateCount();
    if (count > max_count) count = max_count;
    for (uint8_t i = 0; i < count; ++i) {
        out_candidates[i] = s_global_vision_pipeline.getCandidate(i);
    }
    return count;
}

VisionPipeline& vision_pipeline_get_instance() {
    return s_global_vision_pipeline;
}

} // namespace RobofestDrone

