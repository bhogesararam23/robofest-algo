#include "test_framework.h"
#include "../src/mine_map.h"
#include "../src/types.h"
#include "../config/thresholds.h"

using namespace RobofestDrone;

TEST(MineDedupTest, MergeNearbyDetections) {
    MineMap map;
    map.init();
    map.setSelfDroneId(1);

    // First detection at (5.0, 5.0)
    map.addDetection(5.0f, 5.0f, 60.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1000UL);
    ASSERT_EQ(map.getMineCount(), 1);

    // Second detection within 0.30m radius at (5.1, 5.1) -> ~0.14m distance
    map.addDetection(5.1f, 5.1f, 70.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1100UL);

    // Should merge into 1 single candidate mine record
    ASSERT_EQ(map.getMineCount(), 1);
}

TEST(MineDedupTest, SeparateDistantDetections) {
    MineMap map;
    map.init();
    map.setSelfDroneId(1);

    // Mine A at (5.0, 5.0)
    map.addDetection(5.0f, 5.0f, 60.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1000UL);

    // Mine B at (10.0, 10.0) -> far away
    map.addDetection(10.0f, 10.0f, 60.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1100UL);

    // Must yield 2 distinct mine candidates
    ASSERT_EQ(map.getMineCount(), 2);
}

TEST(MineDedupTest, ConfidenceFusionIncreasesScore) {
    MineMap map;
    map.init();
    map.setSelfDroneId(1);

    map.addDetection(5.0f, 5.0f, 50.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1000UL);
    Types::MineRecord record_initial;
    ASSERT_TRUE(map.getMineById(1, record_initial));
    float conf_1 = record_initial.confidence;

    // Second detection with higher confidence
    map.addDetection(5.02f, 5.01f, 80.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1100UL);
    Types::MineRecord record_updated;
    ASSERT_TRUE(map.getMineById(1, record_updated));
    float conf_2 = record_updated.confidence;

    // Fused confidence must strictly increase
    ASSERT_TRUE(conf_2 > conf_1);
}

TEST(MineDedupTest, StaleDecayReducesConfidence) {
    MineMap map;
    map.init();
    map.setSelfDroneId(1);

    // Add candidate at T=1000ms
    map.addDetection(5.0f, 5.0f, 50.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1000UL);
    Types::MineRecord rec;
    ASSERT_TRUE(map.getMineById(1, rec));
    float initial_conf = rec.confidence;

    // Advance time by 30 seconds with multiple periodic decay ticks
    for (uint32_t t = 2000UL; t <= 35000UL; t += 1000UL) {
        map.update(t);
    }

    Types::MineRecord decayed_rec;
    if (map.getMineById(1, decayed_rec)) {
        ASSERT_TRUE(decayed_rec.confidence < initial_conf);
    } else {
        // Alternatively decayed and pruned from map
        ASSERT_EQ(map.getMineCount(), 0);
    }
}
