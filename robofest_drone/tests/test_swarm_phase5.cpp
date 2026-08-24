#include "test_framework.h"
#include "../src/mine_map.h"
#include "../src/threat_arbiter.h"
#include "../src/buried_detector.h"

// ============================================================================
// Phase 5 tests: cross-drone fusion (item 11), classification consensus
// (item 12), obstacle TTC (item 14), moving-target landing (item 6n),
// buried-mine detection cores (item 13).
// ============================================================================

using namespace RobofestDrone;

// ---------------------------------------------------------------- item 11 --

TEST(vision_fusion, obs_weight_prefers_close_confident_observers) {
    const float close = MineMap::vision_obs_weight(80.0f, 1.0f);
    const float far = MineMap::vision_obs_weight(80.0f, 8.0f);
    ASSERT_TRUE(close > far);
    ASSERT_TRUE(close > 0.0f && close <= 1.0f);
    // Zero confidence or negative distance carry no weight.
    ASSERT_FLOAT_EQ(MineMap::vision_obs_weight(0.0f, 1.0f), 0.0f);
    ASSERT_FLOAT_EQ(MineMap::vision_obs_weight(50.0f, -2.0f), 0.0f);
}

TEST(vision_fusion, observation_fuses_into_map_and_records_vote) {
    MineMap map;
    map.init();
    map.setSelfDroneId(1);

    Types::VisionObsPayload obs;
    obs.x = 5.0f;
    obs.y = 20.0f;
    obs.confidence = 70.0f;
    obs.observer_distance_m = 2.0f;
    obs.marker_type = Types::VisionMarkerType::ON_GROUND_MINE;

    map.addVisionObservation(obs, 2, 1000);

    Types::MineRecord rec;
    uint16_t n = map.getCandidateMines(&rec, 1);
    ASSERT_TRUE(n == 1);
    ASSERT_NEAR(rec.x, 5.0f, 0.01f);
    ASSERT_EQ(rec.obs_n, 1u);
}

// ---------------------------------------------------------------- item 12 --

TEST(marker_consensus, unanimous_votes_win) {
    Types::VisionMarkerType types[3] = {
        Types::VisionMarkerType::ON_GROUND_MINE,
        Types::VisionMarkerType::ON_GROUND_MINE,
        Types::VisionMarkerType::ON_GROUND_MINE};
    float weights[3] = {0.8f, 0.9f, 0.7f};

    Types::MarkerConsensus out;
    ASSERT_TRUE(MineMap::resolve_votes(types, weights, 3, out));
    ASSERT_TRUE(out.winning_type == Types::VisionMarkerType::ON_GROUND_MINE);
    ASSERT_TRUE(!out.ambiguous);
    ASSERT_NEAR(out.weighted_agreement, 1.0f, 0.001f);
}

TEST(marker_consensus, split_votes_flagged_ambiguous) {
    Types::VisionMarkerType types[2] = {
        Types::VisionMarkerType::MARKER_BLUE,
        Types::VisionMarkerType::MARKER_PURPLE};
    float weights[2] = {0.5f, 0.5f};

    Types::MarkerConsensus out;
    ASSERT_TRUE(MineMap::resolve_votes(types, weights, 2, out));
    ASSERT_TRUE(out.ambiguous);
    ASSERT_NEAR(out.weighted_agreement, 0.5f, 0.001f);
}

TEST(marker_consensus, low_total_weight_is_ambiguous_even_if_unanimous) {
    Types::VisionMarkerType types[2] = {
        Types::VisionMarkerType::MARKER_GREEN,
        Types::VisionMarkerType::MARKER_GREEN};
    float weights[2] = {0.2f, 0.2f}; // total 0.4 < MIN_WEIGHT

    Types::MarkerConsensus out;
    ASSERT_TRUE(MineMap::resolve_votes(types, weights, 2, out));
    ASSERT_TRUE(out.winning_type == Types::VisionMarkerType::MARKER_GREEN);
    ASSERT_TRUE(out.ambiguous);
}

// ---------------------------------------------------------------- item 14 --

TEST(obstacle_ttc, approaching_obstacle_yields_positive_ttc) {
    Types::RelativeObstacle obs;
    obs.valid = true;
    obs.rel_x = 4.0f;   // ahead
    obs.rel_y = 0.0f;
    obs.rel_vx = -2.0f; // closing at 2 m/s
    obs.rel_vy = 0.0f;

    const float ttc = threat_time_to_collision(obs);
    ASSERT_TRUE(ttc > 0.0f);
    ASSERT_NEAR(ttc, 1.7f, 0.15f); // (4.0-0.6)/2.0

    ThreatVerdict v = threat_evaluate(obs);
    ASSERT_TRUE(v.evade);
    ASSERT_TRUE(v.urgency > 0.0f);
}

TEST(obstacle_ttc, diverging_or_static_geometry_is_not_threat) {
    Types::RelativeObstacle diverging;
    diverging.valid = true;
    diverging.rel_x = 3.0f;
    diverging.rel_vx = +1.0f; // moving away
    ASSERT_TRUE(threat_time_to_collision(diverging) < 0.0f);

    Types::RelativeObstacle static_side;
    static_side.valid = true;
    static_side.rel_x = 0.0f;
    static_side.rel_y = 5.0f; // stationary pole beside us
    ASSERT_TRUE(threat_time_to_collision(static_side) < 0.0f);
    ASSERT_TRUE(!threat_evaluate(static_side).evade);
}

TEST(obstacle_ttc, immediate_radius_forces_full_urgency) {
    Types::RelativeObstacle obs;
    obs.valid = true;
    obs.rel_x = 0.4f;
    obs.rel_y = 0.1f; // inside 0.6 m bubble
    ThreatVerdict v = threat_evaluate(obs);
    ASSERT_TRUE(v.evade);
    ASSERT_FLOAT_EQ(v.urgency, 1.0f);
}

// ----------------------------------------------------------- item 6n ------

TEST(landing_predict, constant_velocity_extrapolation) {
    // Fast mover (>1.5 m/s): horizon is clamped to 0.6 * default.
    Types::TargetKinematicSample older;
    older.valid = true;
    older.timestamp_ms = 0;
    older.field_x = 10.0f;
    older.field_y = 30.0f;

    Types::TargetKinematicSample newer;
    newer.valid = true;
    newer.timestamp_ms = 500;      // dt = 0.5 s
    newer.field_x = 11.0f;         // vx = +2 m/s
    newer.field_y = 29.0f;         // vy = -2 m/s

    LandingPredict p = landing_predict_intercept(older, newer, 1.0f);
    ASSERT_TRUE(p.valid);
    ASSERT_NEAR(p.target_speed_mps, 2.83f, 0.05f);

    const float h_fast = Config::LANDING_PREDICT_HORIZON_S * 0.6f; // clamped
    ASSERT_NEAR(p.intercept_x, 11.0f + 2.0f * h_fast, 0.05f);
    ASSERT_NEAR(p.intercept_y, 29.0f - 2.0f * h_fast, 0.05f);

    // Slow mover (<=1.5 m/s): full horizon applies.
    newer.timestamp_ms = 1000;     // dt = 1 s
    newer.field_x = 10.5f;         // vx = +0.5 m/s
    newer.field_y = 30.0f;         // vy = 0

    p = landing_predict_intercept(older, newer, 1.0f);
    ASSERT_TRUE(p.valid);
    ASSERT_NEAR(p.target_speed_mps, 0.5f, 0.02f);
    ASSERT_NEAR(p.intercept_x, 10.5f + 0.5f * 1.0f, 0.05f); // full horizon
}

TEST(landing_predict, invalid_history_falls_back_to_position) {
    Types::TargetKinematicSample bad;
    bad.valid = false;
    Types::TargetKinematicSample now;
    now.valid = true;
    now.timestamp_ms = 1000;
    now.field_x = 7.0f;
    now.field_y = 8.0f;

    LandingPredict p = landing_predict_intercept(bad, now, 1.0f);
    ASSERT_TRUE(p.valid);
    ASSERT_NEAR(p.intercept_x, 7.0f, 0.001f);
    ASSERT_NEAR(p.intercept_y, 8.0f, 0.001f);
    ASSERT_FLOAT_EQ(p.target_speed_mps, 0.0f);
}

TEST(landing_servo, error_drives_velocity_and_tolerance_releases) {
    float vx = 0, vy = 0;
    // 40 px right of center -> positive x command.
    ASSERT_TRUE(landing_servo_velocity(200, 120, 160, 120, vx, vy));
    ASSERT_TRUE(vx > 0.0f);
    ASSERT_NEAR(vy, 0.0f, 0.0001f);

    // Within tolerance -> centered, no lateral command.
    bool needs_fix = landing_servo_velocity(165, 121, 160, 120, vx, vy);
    ASSERT_TRUE(!needs_fix);
    ASSERT_FLOAT_EQ(vx, 0.0f);
}

// ---------------------------------------------------------------- item 13 --

TEST(buried_detector, plane_fit_residual_detects_mound) {
    GroundReturn flat[5] = {
        {0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {2, 2, 0}, {1, 1, 0}};
    ASSERT_NEAR(buried_plane_fit_residual(flat, 5, nullptr, nullptr, nullptr),
                0.0f, 0.005f);

    GroundReturn mound[5] = {
        {0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {2, 2, 0}, {1, 1, 0.08f}};
    ASSERT_TRUE(buried_plane_fit_residual(mound, 5, nullptr, nullptr, nullptr) >
                Config::BURIED_PLANE_RESIDUAL_M);
}

TEST(buried_detector, fuse_score_requires_combined_evidence) {
    SpectralIndices none;
    none.valid = false;

    // Weak texture alone stays below emit threshold.
    const float weak = buried_fuse_score(
        Config::BURIED_TEXTURE_RATIO_MIN + 0.1f, 0.0f, none);
    ASSERT_TRUE(weak < Config::BURIED_SCORE_EMIT_MIN);

    // Strong texture + depth breach crosses it.
    const float strong =
        buried_fuse_score(2.9f, 0.06f, none);
    ASSERT_TRUE(strong >= Config::BURIED_SCORE_EMIT_MIN);
}
