#include "test_framework.h"
#include "../config/vision_profiles.h"

// ============================================================================
// REQ-DER-1xx (item 19): the generated/committed vision profile table must be
// structurally valid and semantically equivalent to the built-in seed table.
// ============================================================================

using RobofestDrone::Config::VISION_PROFILE_TABLE;
using RobofestDrone::Config::VISION_PROFILE_COUNT;

TEST(config_profiles, table_present_and_sized) {
    ASSERT_TRUE(VISION_PROFILE_COUNT >= 8);
    ASSERT_TRUE(VISION_PROFILE_COUNT <= RobofestDrone::Config::VISION_PROFILE_MAX);
}

TEST(config_profiles, ids_unique_and_in_wire_range) {
    for (uint8_t i = 0; i < VISION_PROFILE_COUNT; ++i) {
        for (uint8_t j = i + 1; j < VISION_PROFILE_COUNT; ++j) {
            ASSERT_TRUE(VISION_PROFILE_TABLE[i].marker_type_id !=
                        VISION_PROFILE_TABLE[j].marker_type_id);
        }
        ASSERT_TRUE(VISION_PROFILE_TABLE[i].marker_type_id >= 1);
        ASSERT_TRUE(VISION_PROFILE_TABLE[i].marker_type_id <= 15);
    }
}

TEST(config_profiles, hsv_bands_within_device_scale) {
    // H 0-180 (OpenCV scale), S/V 0-255. Wraparound bands (h_min > h_max) are legal.
    for (uint8_t i = 0; i < VISION_PROFILE_COUNT; ++i) {
        const auto& p = VISION_PROFILE_TABLE[i];
        const RobofestDrone::Config::HsvBandDef* bands[2] = {&p.primary, &p.alt};
        const int band_count = p.has_alt_band ? 2 : 1;
        for (int b = 0; b < band_count; ++b) {
            const auto& band = *bands[b];
            ASSERT_TRUE(band.h_min <= 180);
            ASSERT_TRUE(band.h_max <= 180);
            ASSERT_TRUE(band.s_min <= band.s_max);
            ASSERT_TRUE(band.v_min <= band.v_max);
            ASSERT_TRUE(band.v_max <= 255);
        }
    }
}

TEST(config_profiles, area_gates_sane) {
    for (uint8_t i = 0; i < VISION_PROFILE_COUNT; ++i) {
        const auto& p = VISION_PROFILE_TABLE[i];
        ASSERT_TRUE(p.min_area_px > 0.0f);
        ASSERT_TRUE(p.max_area_px > p.min_area_px);
        ASSERT_TRUE(p.circularity_min >= 0.0f && p.circularity_min <= 1.0f);
        ASSERT_TRUE(p.expected_marker_area_px > 0);
    }
}

TEST(config_profiles, shape_gates_ordered) {
    for (uint8_t i = 0; i < VISION_PROFILE_COUNT; ++i) {
        const auto& g = VISION_PROFILE_TABLE[i].shape;
        ASSERT_TRUE(g.aspect_min <= g.aspect_max);
        ASSERT_TRUE(g.extent_min <= g.extent_max);
        ASSERT_TRUE(g.corners_min <= g.corners_max);
        ASSERT_TRUE(g.solidity_min <= 1.05f);
    }
}

TEST(config_profiles, red_mine_row_matches_builtin_seed) {
    // Guard against accidental drift between generated header and built-ins:
    // row 1 is the RED mine marker and drives mission-critical detection.
    const auto& r = VISION_PROFILE_TABLE[0];
    ASSERT_EQ(r.marker_type_id, 1u);
    ASSERT_FLOAT_EQ(r.primary.h_min, 170.0f);
    ASSERT_FLOAT_EQ(r.primary.h_max, 10.0f); // wraparound
    ASSERT_FLOAT_EQ(r.primary.s_min, 100.0f);
    ASSERT_FLOAT_EQ(r.alt.v_min, 60.0f);
    ASSERT_FLOAT_EQ(r.confidence_bias, 0.0f);
}

TEST(config_profiles, yellow_buried_bias_preserved) {
    bool found = false;
    for (uint8_t i = 0; i < VISION_PROFILE_COUNT; ++i) {
        if (VISION_PROFILE_TABLE[i].marker_type_id == 2) {
            ASSERT_FLOAT_EQ(VISION_PROFILE_TABLE[i].confidence_bias, 5.0f);
            found = true;
        }
    }
    ASSERT_TRUE(found);
}
