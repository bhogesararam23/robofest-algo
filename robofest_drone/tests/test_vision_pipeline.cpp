#include "test_framework.h"
#include "../src/vision_pipeline.h"
#include "../src/shape_analysis.h"

using namespace RobofestDrone;

namespace {

VisionMarkerProfile make_circle_profile() {
    VisionMarkerProfile p;
    p.profile_type = Types::VisionMarkerType::ON_GROUND_MINE;
    p.enabled = true;
    p.h_min = 170;
    p.h_max = 10; // wraparound band
    p.s_min = 100;
    p.s_max = 255;
    p.v_min = 100;
    p.v_max = 255;
    p.min_area_px = 25.0f;
    p.max_area_px = 2500.0f;
    p.circularity_min = 0.70f;
    p.confidence_bias = 0.0f;
    p.expected_marker_area_px = 400;
    p.aspect_min = 1.0f;
    p.aspect_max = 1.35f;
    p.extent_min = 0.62f;
    p.extent_max = 1.05f;
    p.solidity_min = 0.88f;
    p.corners_min = 0;
    p.corners_max = 255;
    return p;
}

void fill_square(std::vector<VisionPoint>& pts, float x0, float y0, float size, float step) {
    for (float x = x0; x <= x0 + size; x += step) {
        pts.push_back({x, y0});
        pts.push_back({x, y0 + size});
    }
    for (float y = y0; y <= y0 + size; y += step) {
        pts.push_back({x0, y});
        pts.push_back({x0 + size, y});
    }
}

} // namespace

// ============================================================================
// HSV BAND INCLUSION (HUE WRAPAROUND - Step 3)
// ============================================================================

TEST(VisionBandTest, NonWrappingBandInclusiveBounds) {
    ASSERT_TRUE(vision_hsv_in_band(5, 150, 200, 0, 15, 100, 255, 100, 255));
    ASSERT_TRUE(vision_hsv_in_band(0, 100, 100, 0, 15, 100, 255, 100, 255));
    ASSERT_TRUE(vision_hsv_in_band(15, 100, 100, 0, 15, 100, 255, 100, 255));
    ASSERT_FALSE(vision_hsv_in_band(16, 150, 200, 0, 15, 100, 255, 100, 255));
    ASSERT_FALSE(vision_hsv_in_band(5, 99, 200, 0, 15, 100, 255, 100, 255));
    ASSERT_FALSE(vision_hsv_in_band(5, 150, 99, 0, 15, 100, 255, 100, 255));
}

TEST(VisionBandTest, WraparoundRedBandMatchesBothSides) {
    // Red wraps: h in [170..180] U [0..10]
    ASSERT_TRUE(vision_hsv_in_band(175, 150, 200, 170, 10, 100, 255, 100, 255));
    ASSERT_TRUE(vision_hsv_in_band(179, 150, 200, 170, 10, 100, 255, 100, 255));
    ASSERT_TRUE(vision_hsv_in_band(0, 150, 200, 170, 10, 100, 255, 100, 255));
    ASSERT_TRUE(vision_hsv_in_band(10, 150, 200, 170, 10, 100, 255, 100, 255));
    ASSERT_FALSE(vision_hsv_in_band(11, 150, 200, 170, 10, 100, 255, 100, 255));
    ASSERT_FALSE(vision_hsv_in_band(169, 150, 200, 170, 10, 100, 255, 100, 255));
}

TEST(VisionProfileTableTest, BuiltInTableValidAndRedWraps) {
    ASSERT_TRUE(Config::VISION_PROFILE_COUNT > 0);
    ASSERT_TRUE(Config::VISION_PROFILE_COUNT <= Config::VISION_PROFILE_MAX);

    for (uint8_t i = 0; i < Config::VISION_PROFILE_COUNT; ++i) {
        const Config::VisionProfileDef& row = Config::VISION_PROFILE_TABLE[i];
        ASSERT_TRUE(row.marker_type_id != 0); // no UNKNOWN rows
        if (row.marker_type_id == 1) {
            // Red family must be configured to wrap across the hue seam.
            ASSERT_TRUE(row.primary.h_min > row.primary.h_max);
            ASSERT_TRUE(row.alt.h_min > row.alt.h_max);
        }
        ASSERT_TRUE(row.shape.extent_max >= row.shape.extent_min);
    }
}

// ============================================================================
// SOFT SCORING FUNCTIONS (Step 8)
// ============================================================================

TEST(VisionScoringTest, RangeScorePerfectInsideFalloffOutside) {
    ASSERT_FLOAT_EQ(vision_range_score(0.5f, 0.4f, 0.9f), 1.0f);
    // One margin outside the gate decays to zero
    float span = 0.5f;
    float margin = span * Config::CONF_GATE_SOFT_MARGIN_RATIO;
    ASSERT_NEAR(vision_range_score(0.9f + margin, 0.4f, 0.9f), 0.0f, 0.01f);
    // Halfway into the margin scores ~0.5
    ASSERT_NEAR(vision_range_score(0.9f + margin * 0.5f, 0.4f, 0.9f), 0.5f, 0.02f);
    // Degenerate gate is neutral
    ASSERT_FLOAT_EQ(vision_range_score(3.0f, 5.0f, 1.0f), 1.0f);
}

TEST(VisionScoringTest, FloorScoreDecaysBelowFloor) {
    ASSERT_FLOAT_EQ(vision_floor_score(0.95f, 0.88f), 1.0f);
    float margin = 0.88f * Config::CONF_GATE_SOFT_MARGIN_RATIO;
    ASSERT_NEAR(vision_floor_score(0.88f - margin, 0.88f), 0.0f, 0.01f);
}

TEST(VisionScoringTest, CornerScoreNeutralWhenDisabled) {
    // corners_min == 0 disables corner gating entirely (circle profiles)
    ASSERT_FLOAT_EQ(vision_corner_score(0, 0, 255), 1.0f);
    ASSERT_FLOAT_EQ(vision_corner_score(37, 0, 255), 1.0f);
    ASSERT_FLOAT_EQ(vision_corner_score(4, 3, 5), 1.0f);
    ASSERT_NEAR(vision_corner_score(6, 3, 5), 1.0f - 1.0f / 3.0f, 0.01f);
    ASSERT_FLOAT_EQ(vision_corner_score(9, 3, 5), 0.0f);
}

TEST(VisionScoringTest, ShapeMatchPenalizesWrongShape) {
    VisionMarkerProfile square_profile = make_circle_profile();
    square_profile.aspect_min = 1.0f;
    square_profile.aspect_max = 1.25f;
    square_profile.extent_min = 0.78f;
    square_profile.extent_max = 1.05f;
    square_profile.solidity_min = 0.92f;
    square_profile.corners_min = 3;
    square_profile.corners_max = 5;

    float good = vision_shape_match(1.0f, 0.95f, 0.97f, 4, square_profile);
    float wrong_shape = vision_shape_match(2.2f, 0.45f, 0.55f, 12, square_profile);
    ASSERT_NEAR(good, 1.0f, 0.02f);
    ASSERT_NEAR(wrong_shape, 0.0f, 0.02f);
    ASSERT_TRUE(wrong_shape < good * 0.5f);
}

// ============================================================================
// GEOMETRY: HULL / AREA / CORNERS (Steps 7 & 9)
// ============================================================================

TEST(VisionGeometryTest, PolygonAreaSquareAndTriangle) {
    std::vector<VisionPoint> square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    ASSERT_NEAR(vision_polygon_area(square.data(), 4), 100.0f, 0.01f);

    std::vector<VisionPoint> tri = {{0, 0}, {10, 0}, {0, 10}};
    ASSERT_NEAR(vision_polygon_area(tri.data(), 3), 50.0f, 0.01f);
}

TEST(VisionGeometryTest, ConvexHullCountAndArea) {
    std::vector<VisionPoint> cloud;
    fill_square(cloud, 0.0f, 0.0f, 20.0f, 2.0f);

    std::vector<VisionPoint> hull(Config::VISION_HULL_POINTS_MAX);
    uint8_t n = vision_convex_hull(cloud.data(),
                                   static_cast<uint8_t>(cloud.size()),
                                   hull.data(), Config::VISION_HULL_POINTS_MAX);
    ASSERT_TRUE(n >= 4 && n <= 8); // axis-aligned square hull (collinear dropped)
    ASSERT_NEAR(vision_polygon_area(hull.data(), n), 400.0f, 4.0f);
}

TEST(VisionGeometryTest, CornerCountDistinguishesShapes) {
    auto count_corners = [](std::vector<VisionPoint>& poly) {
        return vision_poly_corner_count(poly.data(),
                                        static_cast<uint8_t>(poly.size()), 0.6f);
    };

    std::vector<VisionPoint> square = {{0, 0}, {20, 0}, {20, 20}, {0, 20}};
    uint8_t sq_corners = count_corners(square);
    ASSERT_TRUE(sq_corners == 4);

    std::vector<VisionPoint> triangle = {{0, 0}, {20, 0}, {10, 18}};
    uint8_t tri_corners = count_corners(triangle);
    ASSERT_TRUE(tri_corners == 3);

    // Pentagon with collinear-ish midpoints on each edge to exercise DP pruning
    std::vector<VisionPoint> pentagon = {
        {10, 0}, {19.5f, 6.9f}, {16, 18}, {4, 18}, {0.5f, 6.9f}};
    uint8_t pent_corners = count_corners(pentagon);
    ASSERT_TRUE(pent_corners >= 4 && pent_corners <= 6);
    ASSERT_TRUE(sq_corners != tri_corners);
}

// ============================================================================
// CONFIDENCE FORMULA (Step 8)
// ============================================================================

TEST(VisionConfidenceTest, CircleBlobScoresHighOnCircleProfile) {
    VisionMarkerProfile prof = make_circle_profile();

    VisionBlob blob;
    blob.area = 400.0f;
    blob.circularity = 0.85f;
    blob.aspect_ratio = 1.05f;
    blob.extent = 0.785f;
    blob.solidity = 0.97f;

    float conf = vision_blob_confidence(blob, prof);
    // circ_term 24 + shape ~1.0*36 + area_term 40 = ~100
    ASSERT_NEAR(conf, 100.0f, 3.0f);
}

TEST(VisionConfidenceTest, WrongShapeLowersConfidenceNotHardRejected) {
    VisionMarkerProfile circle_prof = make_circle_profile();

    VisionBlob elongated;
    elongated.area = 400.0f;
    elongated.circularity = 0.85f;
    elongated.aspect_ratio = 4.0f;
    elongated.extent = 0.55f;
    elongated.solidity = 0.65f;

    float bad_conf = vision_blob_confidence(elongated, circle_prof);
    ASSERT_TRUE(bad_conf < 80.0f);
    ASSERT_TRUE(bad_conf > 24.0f); // soft penalty, not zeroed

    VisionBlob good;
    good.area = 400.0f;
    good.circularity = 0.85f;
    good.aspect_ratio = 1.0f;
    good.extent = 0.79f;
    good.solidity = 0.98f;
    float good_conf = vision_blob_confidence(good, circle_prof);
    ASSERT_TRUE(bad_conf + 10.0f < good_conf);
}

TEST(VisionConfidenceTest, BiasAndClampingApplied) {
    VisionMarkerProfile prof = make_circle_profile();
    prof.confidence_bias = 7.0f;

    VisionBlob weak;
    weak.area = 50.0f;
    weak.circularity = 0.72f;
    weak.aspect_ratio = 1.1f;
    weak.extent = 0.70f;
    weak.solidity = 0.90f;

    float base = vision_blob_confidence(weak, make_circle_profile());
    float biased = vision_blob_confidence(weak, prof);
    ASSERT_NEAR(biased, base + 7.0f, 0.01f);

    VisionBlob huge;
    huge.area = 100000.0f;
    huge.circularity = 1.0f;
    huge.aspect_ratio = 1.0f;
    huge.extent = 0.9f;
    huge.solidity = 1.0f;
    float clamped = vision_blob_confidence(huge, prof);
    ASSERT_TRUE(clamped <= 100.0f);
}

// ============================================================================
// PIPELINE PROFILE TABLE ACCESS (Step 1 wiring)
// ============================================================================

TEST(VisionPipelineAccessTest, TableLoadedWithUniqueTypes) {
    VisionPipeline pipe;
    ASSERT_TRUE(pipe.getProfileCount() > 0);

    for (uint8_t i = 0; i < pipe.getProfileCount(); ++i) {
        const VisionMarkerProfile* pi = pipe.getProfileByIndex(i);
        ASSERT_TRUE(pi != nullptr);
        for (uint8_t j = static_cast<uint8_t>(i + 1); j < pipe.getProfileCount(); ++j) {
            const VisionMarkerProfile* pj = pipe.getProfileByIndex(j);
            ASSERT_TRUE(pi->profile_type != pj->profile_type);
        }
    }

    const VisionMarkerProfile* red = pipe.getProfileByType(Types::VisionMarkerType::ON_GROUND_MINE);
    ASSERT_TRUE(red != nullptr);
    ASSERT_TRUE(red->h_min > red->h_max); // wraparound preserved at runtime

    const VisionMarkerProfile* missing =
        pipe.getProfileByType(static_cast<Types::VisionMarkerType>(250));
    ASSERT_TRUE(missing == nullptr);
    ASSERT_TRUE(pipe.getProfileByIndex(pipe.getProfileCount()) == nullptr);
}

TEST(VisionPipelineAccessTest, FocusModeEnablesOnlySelectedProfile) {
    VisionPipeline pipe;
    Types::VisionMarkerType target = pipe.getProfileByIndex(0)->profile_type;

    pipe.setActiveProfile(target);
    int enabled = 0;
    for (uint8_t i = 0; i < pipe.getProfileCount(); ++i) {
        if (pipe.getProfileByIndex(i)->enabled) enabled++;
    }
    ASSERT_EQ(enabled, 1);

    pipe.setAllProfilesEnabled(true);
    enabled = 0;
    for (uint8_t i = 0; i < pipe.getProfileCount(); ++i) {
        if (pipe.getProfileByIndex(i)->enabled) enabled++;
    }
    ASSERT_EQ(enabled, pipe.getProfileCount());

    pipe.setAllProfilesEnabled(false);
    enabled = 0;
    for (uint8_t i = 0; i < pipe.getProfileCount(); ++i) {
        if (pipe.getProfileByIndex(i)->enabled) enabled++;
    }
    ASSERT_EQ(enabled, 1); // active focus profile stays enabled

    Types::VisionMarkerType other = pipe.getProfileByIndex(
        pipe.getProfileCount() > 1 ? 1 : 0)->profile_type;
    pipe.setProfileEnabled(other, true);
    bool other_on = false;
    for (uint8_t i = 0; i < pipe.getProfileCount(); ++i) {
        if (pipe.getProfileByIndex(i)->profile_type == other &&
            pipe.getProfileByIndex(i)->enabled) {
            other_on = true;
        }
    }
    ASSERT_TRUE(other_on);

    pipe.restoreProfileDefaults();
    enabled = 0;
    for (uint8_t i = 0; i < pipe.getProfileCount(); ++i) {
        if (pipe.getProfileByIndex(i)->enabled) enabled++;
    }
    ASSERT_EQ(enabled, pipe.getProfileCount());
}

// ============================================================================
// SHAPE CLASSIFICATION BANDS (Issue 2 fix: off-by-one corrected)
// ============================================================================

TEST(ShapeClassificationTest, BandsMatchTrueCornerCounts) {
    // Each corner count must map to the shape matching its actual name.
    // Previous code had bands shifted by one (5 corners => QUAD, 6 => PENTAGON).
    ASSERT_EQ(vision_classify_shape(0.9f, 0.95f, 0.85f, 1.0f, 3, 0),
              ShapeClass::SHAPE_TRIANGLE);
    ASSERT_EQ(vision_classify_shape(0.85f, 0.95f, 0.80f, 1.1f, 4, 0),
              ShapeClass::SHAPE_QUADRILATERAL);
    ASSERT_EQ(vision_classify_shape(0.82f, 0.93f, 0.78f, 1.0f, 5, 0),
              ShapeClass::SHAPE_PENTAGON);
    ASSERT_EQ(vision_classify_shape(0.80f, 0.92f, 0.75f, 1.0f, 6, 0),
              ShapeClass::SHAPE_HEXAGON_PLUS);
    ASSERT_EQ(vision_classify_shape(0.78f, 0.90f, 0.72f, 1.0f, 7, 0),
              ShapeClass::SHAPE_HEXAGON_PLUS);
    ASSERT_EQ(vision_classify_shape(0.76f, 0.88f, 0.70f, 1.0f, 8, 0),
              ShapeClass::SHAPE_HEXAGON_PLUS);
}

TEST(ShapeClassificationTest, CircleStillDetected) {
    // High circularity + high solidity => circle, regardless of corner count.
    ASSERT_EQ(vision_classify_shape(0.95f, 0.95f, 0.90f, 1.05f, 12, 0),
              ShapeClass::SHAPE_CIRCLE);
}

TEST(ShapeClassificationTest, StarDetectedByDefectsAndLowSolidity) {
    // Many defects + low solidity => star/concave.
    ASSERT_EQ(vision_classify_shape(0.60f, 0.50f, 0.65f, 1.0f, 8, 5),
              ShapeClass::SHAPE_STAR_CONCAVE);
}

TEST(ShapeClassificationTest, BarDetectedByHighAspect) {
    ASSERT_EQ(vision_classify_shape(0.70f, 0.90f, 0.60f, 5.0f, 4, 0),
              ShapeClass::SHAPE_BAR);
}
