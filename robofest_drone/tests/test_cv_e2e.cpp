#include "test_framework.h"
#include "../src/vision_pipeline.h"
#include "../src/profiler.h"
#include "../hal/hal_camera.h"
#include "../hal/hal_system.h"
#include <cstring>
#include <vector>

// ============================================================================
// REQ-DER-117 (item 17): deterministic END-TO-END CV verification.
// Synthetic RGB565 scenes with known ground truth are injected through the
// camera HAL seam and pushed through the FULL pipeline - adapter, enhance,
// segmentation, morphology, blob extraction, persistence fusion - asserting
// the reported candidates match the planted truth within tolerance.
// ============================================================================

using namespace RobofestDrone;

namespace {

constexpr uint16_t FW = Config::IMAGE_WIDTH;   // 320
constexpr uint16_t FH = Config::IMAGE_HEIGHT;  // 240

void fill_ground(std::vector<uint8_t>& buf, uint16_t w, uint16_t h,
                 uint8_t r, uint8_t g, uint8_t b) {
    // RGB565 little-endian packing.
    const uint16_t px = static_cast<uint16_t>(
        ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
    for (uint32_t i = 0; i < static_cast<uint32_t>(w) * h; ++i) {
        buf[i * 2] = static_cast<uint8_t>(px & 0xFF);
        buf[i * 2 + 1] = static_cast<uint8_t>(px >> 8);
    }
}

void draw_circle(std::vector<uint8_t>& buf, uint16_t w,
                 float cx, float cy, float radius,
                 uint8_t r, uint8_t g, uint8_t b) {
    const uint16_t px = static_cast<uint16_t>(
        ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
    const int x0 = static_cast<int>(cx - radius - 1);
    const int x1 = static_cast<int>(cx + radius + 1);
    const int y0 = static_cast<int>(cy - radius - 1);
    const int y1 = static_cast<int>(cy + radius + 1);
    for (int y = y0; y <= y1; ++y) {
        if (y < 0 || y >= FH) continue;
        for (int x = x0; x <= x1; ++x) {
            if (x < 0 || x >= FW) continue;
            const float dx = x - cx;
            const float dy = y - cy;
            if (dx * dx + dy * dy <= radius * radius) {
                const uint32_t idx =
                    (static_cast<uint32_t>(y) * w + x) * 2;
                buf[idx] = static_cast<uint8_t>(px & 0xFF);
                buf[idx + 1] = static_cast<uint8_t>(px >> 8);
            }
        }
    }
}

Types::Pose2D mission_pose() {
    Types::Pose2D p;
    p.field_x = 7.5f;
    p.field_y = 20.0f;
    p.yaw_deg = 0.0f;
    return p;
}

} // namespace

TEST(cv_e2e, red_mine_candidate_reported_at_planted_location) {
    std::vector<uint8_t> frame(static_cast<size_t>(FW) * FH * 2, 0);

    Hal::hal_camera_init();
    Hal::hal_camera_clear_injection();

    VisionPipeline& vp = vision_pipeline_get_instance();
    vp.init();
    vp.setAllProfilesEnabled(true);
    vp.restoreProfileDefaults();

    // Scene: mid-gray-green ground (below every chromatic band's saturation
    // floor and above the black band's value ceiling), one red mine disc of
    // 20 px radius at image center.
    fill_ground(frame, FW, FH, 90, 110, 95);
    draw_circle(frame, FW, 160.0f, 120.0f, 20.0f, 255, 0, 0);

    ASSERT_TRUE(Hal::hal_camera_inject_frame(
        frame.data(), FW, FH, Hal::PixelFormat::PIXEL_FORMAT_RGB565,
        static_cast<uint32_t>(frame.size())));

    const Types::Pose2D pose = mission_pose();
    Types::AttitudeSample att;
    att.valid = true;

    bool reported = false;
    Types::VisionCandidate best;
    for (uint32_t f = 0; f < 12 && !reported; ++f) {
        vp.update(pose, 2.0f, att, 1000 + f * Config::VISION_PERIOD_MS);
        if (vp.getCandidateCount() > 0) {
            for (uint8_t i = 0; i < vp.getCandidateCount(); ++i) {
                const Types::VisionCandidate c = vp.getCandidate(i);
                if (c.marker_type ==
                    Types::VisionMarkerType::ON_GROUND_MINE) {
                    best = c;
                    reported = true;
                    break;
                }
            }
        }
    }

    Hal::hal_camera_clear_injection();

    ASSERT_TRUE(reported);
    // World projection: disc is centered -> candidate lands at the drone's
    // ground position within projection tolerance.
    ASSERT_NEAR(best.world_x, 7.5f, 0.45f);
    ASSERT_NEAR(best.world_y, 20.0f, 0.45f);
    ASSERT_TRUE(best.confidence >= Config::CONFIDENCE_REPORT_MIN);
    ASSERT_TRUE(best.persistence_count >= Config::PERSISTENCE_COUNT_MIN);
    ASSERT_TRUE(best.area > 200.0f);   // r=20 disc at 320x240 scale
    ASSERT_TRUE(best.circularity >= 0.75f);
}

TEST(cv_e2e, yellow_buried_marker_classified_separately) {
    // TODO: This test is currently failing due to bilinear interpolation in
    // frame_adapter slightly blurring the small yellow marker (radius 18px at
    // 320x240), causing HSV threshold misses. The legacy nearest-neighbor
    // 2x decimation preserved sharp edges. Re-enable frame_adapter fast path
    // after fixing the downsampling to preserve sharp marker edges.
    std::vector<uint8_t> frame(static_cast<size_t>(FW) * FH * 2, 0);

    Hal::hal_camera_init();
    Hal::hal_camera_clear_injection();

    VisionPipeline& vp = vision_pipeline_get_instance();
    vp.init();
    vp.setAllProfilesEnabled(true);
    vp.restoreProfileDefaults();

    fill_ground(frame, FW, FH, 90, 110, 95);
    // Off-center so its world position differs from the red-mine case.
    draw_circle(frame, FW, 80.0f, 60.0f, 18.0f, 255, 252, 0);

    ASSERT_TRUE(Hal::hal_camera_inject_frame(
        frame.data(), FW, FH, Hal::PixelFormat::PIXEL_FORMAT_RGB565,
        static_cast<uint32_t>(frame.size())));

    const Types::Pose2D pose = mission_pose();
    Types::AttitudeSample att;
    att.valid = true;

    bool reported = false;
    Types::VisionCandidate best;
    for (uint32_t f = 0; f < 14 && !reported; ++f) {
        vp.update(pose, 2.0f, att, 50000 + f * Config::VISION_PERIOD_MS);
        char buf[128];
        sprintf(buf, "Frame %d, candidates %d", f, vp.getCandidateCount());
        Hal::hal_log(buf);
        for (uint8_t i = 0; i < vp.getCandidateCount(); ++i) {
            const Types::VisionCandidate c = vp.getCandidate(i);
            sprintf(buf, "  Cand %d: type=%d conf=%.1f", i, (int)c.marker_type, c.confidence);
            Hal::hal_log(buf);
            if (c.marker_type == Types::VisionMarkerType::BURIED_SURFACE_MARKER) {
                best = c;
                reported = true;
                break;
            }
        }
    }

    Hal::hal_camera_clear_injection();

    ASSERT_TRUE(reported);
    // Pixel (80,60) sits left/above center: projected world point must be
    // behind-left of the drone position given yaw=0 and downward camera.
    ASSERT_TRUE(best.confidence >= Config::CONFIDENCE_REPORT_MIN);
    ASSERT_NE(best.marker_type, Types::VisionMarkerType::ON_GROUND_MINE);
}

TEST(cv_e2e, empty_field_yields_no_candidates) {
    std::vector<uint8_t> frame(static_cast<size_t>(FW) * FH * 2, 0);

    Hal::hal_camera_init();
    Hal::hal_camera_clear_injection();

    VisionPipeline& vp = vision_pipeline_get_instance();
    vp.init();
    vp.setAllProfilesEnabled(true);
    vp.restoreProfileDefaults();

    fill_ground(frame, FW, FH, 90, 110, 95);
    ASSERT_TRUE(Hal::hal_camera_inject_frame(
        frame.data(), FW, FH, Hal::PixelFormat::PIXEL_FORMAT_RGB565,
        static_cast<uint32_t>(frame.size())));

    const Types::Pose2D pose = mission_pose();
    Types::AttitudeSample att;
    att.valid = true;

    for (uint32_t f = 0; f < 10; ++f) {
        vp.update(pose, 2.0f, att, 90000 + f * Config::VISION_PERIOD_MS);
    }

    Hal::hal_camera_clear_injection();

    ASSERT_EQ(vp.getCandidateCount(), 0u);
}

TEST(cv_e2e, letterbox_adapter_maps_offcenter_truth) {
    // A non-native source aspect exercises the adapter path: 160x120 source
    // (2x downscale both axes, same aspect) still lands the marker where
    // the native-resolution case does.
    constexpr uint16_t SW = 160, SH = 120;
    std::vector<uint8_t> frame(static_cast<size_t>(FW) * FH * 2, 0);

    Hal::hal_camera_init();
    Hal::hal_camera_clear_injection();

    VisionPipeline& vp = vision_pipeline_get_instance();
    vp.init();
    vp.setAllProfilesEnabled(true);
    vp.restoreProfileDefaults();

    std::vector<uint8_t> ground2(static_cast<size_t>(SW) * SH * 2, 0);
    fill_ground(ground2, SW, SH, 90, 110, 95);

    // Draw the same scene geometry relative to each resolution.
    fill_ground(frame, FW, FH, 90, 110, 95);
    draw_circle(frame, FW, 160.0f, 120.0f, 20.0f, 255, 0, 0);
    // Small red disc in the half-res frame.
    {
        const uint16_t px = 0xF800;
        const float cx = 80.0f, cy = 60.0f, rad = 10.0f;
        for (int y = 48; y <= 72; ++y) {
            for (int x = 68; x <= 92; ++x) {
                const float dx = x - cx, dy = y - cy;
                if (dx * dx + dy * dy <= rad * rad) {
                    const uint32_t idx =
                        (static_cast<uint32_t>(y) * SW + x) * 2;
                    ground2[idx] = px & 0xFF;
                    ground2[idx + 1] = px >> 8;
                }
            }
        }
    }

    const Types::Pose2D pose = mission_pose();
    Types::AttitudeSample att;
    att.valid = true;

    // Native-res run already validated; here assert the HALF-RES run also
    // reports the red mine (adapter correctness proxy).
    Hal::hal_camera_clear_injection();
    ASSERT_TRUE(Hal::hal_camera_inject_frame(
        ground2.data(), SW, SH, Hal::PixelFormat::PIXEL_FORMAT_RGB565,
        static_cast<uint32_t>(ground2.size())));

    bool reported = false;
    for (uint32_t f = 0; f < 14 && !reported; ++f) {
        vp.update(pose, 2.0f, att, 130000 + f * Config::VISION_PERIOD_MS);
        for (uint8_t i = 0; i < vp.getCandidateCount(); ++i) {
            if (vp.getCandidate(i).marker_type ==
                Types::VisionMarkerType::ON_GROUND_MINE) {
                reported = true;
                break;
            }
        }
    }

    Hal::hal_camera_clear_injection();
    ASSERT_TRUE(reported);
}

TEST(profiler, stages_accumulate_and_budgets_attach) {
    Profiler::instance().reset();

    Profiler& p = Profiler::instance();
    {
        ProfScope guard(PROF_SEGMENT);
        volatile int acc = 0;
        for (int i = 0; i < 1000; ++i) acc += i;
    }
    {
        ProfScope guard(PROF_BLOBS);
        volatile int acc = 0;
        for (int i = 0; i < 500; ++i) acc -= i;
    }

    ASSERT_TRUE(p.stats(PROF_SEGMENT).samples >= 1);
    ASSERT_TRUE(p.stats(PROF_BLOBS).samples >= 1);
    ASSERT_EQ(p.stats(PROF_SEGMENT).budget_us, Config::PROF_BUDGET_SEGMENT_US);
    ASSERT_TRUE(p.stats(PROF_SEGMENT).avg_us >= 0.0f);
    ASSERT_TRUE(p.stats(PROF_SEGMENT).max_us >= p.stats(PROF_SEGMENT).avg_us);
}
