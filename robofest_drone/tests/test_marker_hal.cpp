#include "test_framework.h"
#include "../hal/hal_marker.h"
#include <cmath>

// ============================================================================
// REQ-DER-104 (item 4): visual guidance marker output.
// Verifies the pure pattern->RGB render table and guidance-vector mapping
// that drives the WS2812B RMT output on target hardware.
// ============================================================================

using RobofestDrone::Types::MarkerPattern;
namespace Hal = RobofestDrone::Hal;

static void rgb_at(MarkerPattern p, uint8_t idx, uint8_t phase,
                   uint8_t gidx, uint8_t out[3]) {
    Hal::marker_render_led(p, idx, phase, gidx, out);
}

TEST(marker_hal, off_pattern_is_black_everywhere) {
    for (uint8_t idx = 0; idx < Hal::MARKER_LED_COUNT; ++idx) {
        for (int ph = 0; ph <= 255; ph += 51) {
            uint8_t rgb[3];
            rgb_at(MarkerPattern::MARKER_OFF, idx, static_cast<uint8_t>(ph), 8, rgb);
            ASSERT_EQ(rgb[0], 0);
            ASSERT_EQ(rgb[1], 0);
            ASSERT_EQ(rgb[2], 0);
        }
    }
}

TEST(marker_hal, stop_is_red_dominant_and_never_fully_dark) {
    for (int ph = 0; ph <= 255; ph += 15) {
        uint8_t rgb[3];
        rgb_at(MarkerPattern::MARKER_STOP, 5, static_cast<uint8_t>(ph), 8, rgb);
        ASSERT_TRUE(rgb[0] >= 100);           // red channel always strong
        ASSERT_EQ(rgb[1], 0);
        ASSERT_EQ(rgb[2], 0);
    }
}

TEST(marker_hal, emergency_strobe_alternates_two_distinct_states) {
    bool saw_white_hot = false;
    bool saw_red_hot = false;
    for (int ph_i = 0; ph_i <= 255; ++ph_i) {
        uint8_t rgb[3];
        rgb_at(MarkerPattern::MARKER_EMERGENCY, 3, static_cast<uint8_t>(ph_i), 8, rgb);
        if (rgb[0] > 240 && rgb[1] > 240 && rgb[2] > 240) saw_white_hot = true;
        if (rgb[0] > 240 && rgb[1] < 80 && rgb[2] < 40) saw_red_hot = true;
    }
    ASSERT_TRUE(saw_white_hot);
    ASSERT_TRUE(saw_red_hot);
}

TEST(marker_hal, safe_path_green_corridor) {
    for (int ph = 0; ph <= 255; ph += 25) {
        uint8_t rgb[3];
        rgb_at(MarkerPattern::MARKER_SAFE_PATH, 9, static_cast<uint8_t>(ph), 8, rgb);
        ASSERT_TRUE(rgb[1] > 100);            // green dominant
        ASSERT_TRUE(rgb[0] == 0);
        ASSERT_TRUE(rgb[2] <= 30);
    }
}

TEST(marker_hal, directional_patterns_activate_correct_side) {
    // LEFT sweep with guidance window at low index: left side lit more than right.
    int left_sum = 0;
    int right_sum = 0;
    for (int ph = 0; ph <= 255; ph += 7) {
        for (uint8_t idx = 0; idx < 4; ++idx) {
            uint8_t rgb[3];
            rgb_at(MarkerPattern::MARKER_LEFT, idx, static_cast<uint8_t>(ph), 2, rgb);
            left_sum += rgb[0];
        }
        for (uint8_t idx = 12; idx < 16; ++idx) {
            uint8_t rgb[3];
            rgb_at(MarkerPattern::MARKER_LEFT, idx, static_cast<uint8_t>(ph), 2, rgb);
            right_sum += rgb[0];
        }
    }
    ASSERT_TRUE(left_sum > right_sum * 2);
    ASSERT_TRUE(left_sum > 5000);
    left_sum = 0;
    right_sum = 0;
    for (int ph = 0; ph <= 255; ph += 7) {
        for (uint8_t idx = 0; idx < 4; ++idx) {
            uint8_t rgb[3];
            rgb_at(MarkerPattern::MARKER_RIGHT, idx, static_cast<uint8_t>(ph), 13, rgb);
            left_sum += rgb[0];
        }
        for (uint8_t idx = 12; idx < 16; ++idx) {
            uint8_t rgb[3];
            rgb_at(MarkerPattern::MARKER_RIGHT, idx, static_cast<uint8_t>(ph), 13, rgb);
            right_sum += rgb[0];
        }
    }
    ASSERT_TRUE(right_sum > left_sum * 2);
    ASSERT_TRUE(right_sum > 5000);
}

TEST(marker_hal, guidance_index_mapping_and_clamping) {
    ASSERT_EQ(Hal::hal_marker_guidance_index(0.0f), 8u);
    ASSERT_EQ(Hal::hal_marker_guidance_index(-90.0f), 0u);
    ASSERT_EQ(Hal::hal_marker_guidance_index(90.0f),
              static_cast<uint8_t>(Hal::MARKER_LED_COUNT - 1));
    // Extreme inputs clamp, not wrap.
    ASSERT_EQ(Hal::hal_marker_guidance_index(-500.0f), 0u);
    ASSERT_EQ(Hal::hal_marker_guidance_index(500.0f),
              static_cast<uint8_t>(Hal::MARKER_LED_COUNT - 1));
    // Monotonicity sample points.
    ASSERT_TRUE(Hal::hal_marker_guidance_index(-45.0f) <
                Hal::hal_marker_guidance_index(45.0f));
}
