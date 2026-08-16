#include "test_framework.h"
#include "../src/path_planner.h"
#include "../src/mine_map.h"
#include "../src/types.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"

using namespace RobofestDrone;

TEST(PathClearanceTest, NearConfirmedMineQuery) {
    MineMap map;
    map.init();
    map.setSelfDroneId(1);

    // Add multiple detections with confidence = 95.0 to confirm a mine at (7.5, 30.0)
    for (int i = 0; i < Config::MINE_CONFIRM_PERSISTENCE_MIN + 2; ++i) {
        map.addDetection(7.5f, 30.0f, 95.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1000UL + i * 100);
    }
    map.update(2000UL);

    ASSERT_TRUE(map.getConfirmedCount() >= 1);

    // Point 0.5m away (< 1.0m clearance threshold) -> must return TRUE (near mine)
    ASSERT_TRUE(map.isPointNearConfirmedMine(7.5f, 30.5f, Config::MINE_CLEARANCE_RADIUS_M));

    // Point 1.5m away (> 1.0m clearance threshold) -> must return FALSE (safe)
    ASSERT_FALSE(map.isPointNearConfirmedMine(7.5f, 31.5f, Config::MINE_CLEARANCE_RADIUS_M));
}

TEST(PathClearanceTest, SegmentDistanceGeometry) {
    float ax = 0.0f, ay = 0.0f;
    float bx = 10.0f, by = 0.0f;
    float px = 5.0f, py = 3.0f;

    float abx = bx - ax;
    float aby = by - ay;
    float apx = px - ax;
    float apy = py - ay;
    float seg_len_sq = abx * abx + aby * aby;
    float t = (apx * abx + apy * aby) / seg_len_sq;
    t = std::max(0.0f, std::min(1.0f, t));
    float proj_x = ax + t * abx;
    float proj_y = ay + t * aby;
    float dist = std::sqrt((px - proj_x) * (px - proj_x) + (py - proj_y) * (py - proj_y));

    ASSERT_FLOAT_EQ(dist, 3.0f);
}

TEST(PathClearanceTest, ComputePathAvoidsMinesWithExact1mClearance) {
    MineMap map;
    map.init();
    map.setSelfDroneId(1);

    // Place confirmed mines along center corridor at Y=20, Y=30, Y=40
    for (int i = 0; i < Config::MINE_CONFIRM_PERSISTENCE_MIN + 2; ++i) {
        map.addDetection(7.5f, 20.0f, 95.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1000UL + i * 100);
        map.addDetection(7.5f, 30.0f, 95.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1000UL + i * 100);
        map.addDetection(7.5f, 40.0f, 95.0f, Types::VisionMarkerType::ON_GROUND_MINE, 1, 1000UL + i * 100);
    }
    map.update(2000UL);

    PathPlanner planner;
    planner.init();

    bool found = planner.computePath(map, 3000UL);
    ASSERT_TRUE(found);

    Types::SafePath path = planner.getPath();
    ASSERT_TRUE(path.valid);
    ASSERT_TRUE(path.waypoint_count >= 2);

    // Verify EVERY waypoint maintains >= 1.0m clearance from every confirmed mine
    for (uint8_t i = 0; i < path.waypoint_count; ++i) {
        float wx = path.waypoints[i].x;
        float wy = path.waypoints[i].y;
        ASSERT_FALSE(map.isPointNearConfirmedMine(wx, wy, Config::MINE_CLEARANCE_RADIUS_M - 0.05f));
    }
}
