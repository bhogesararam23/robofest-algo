#include "test_framework.h"
#include "../src/geofence.h"
#include "../src/types.h"
#include "../config/mission_config.h"

using namespace RobofestDrone;

TEST(GeofenceTest, InsideBounds) {
    Geofence gf;
    gf.init();

    Types::Pose2D pose;
    pose.x = 7.5f;
    pose.y = 30.0f;
    pose.field_x = 7.5f;
    pose.field_y = 30.0f;
    pose.yaw_deg = 0.0f;

    gf.update(pose, 0.0f, 1000UL);
    gf.update(pose, 0.0f, 1300UL);

    ASSERT_TRUE(gf.isInside());
    ASSERT_EQ(gf.getStatus(), Types::GeofenceStatus::GEOFENCE_INSIDE);
}

TEST(GeofenceTest, WarningBand) {
    Geofence gf;
    gf.init();

    Types::Pose2D pose;
    // Approaching right boundary: software max is 14.5m, warning band is 0.5m -> 14.15m gives dist 0.35m
    pose.x = 14.15f;
    pose.y = 30.0f;
    pose.field_x = 14.15f;
    pose.field_y = 30.0f;
    pose.yaw_deg = 0.0f;

    gf.update(pose, 0.0f, 1000UL);
    gf.update(pose, 0.0f, 1300UL);

    ASSERT_EQ(gf.getStatus(), Types::GeofenceStatus::GEOFENCE_WARNING);
    Types::Vec2 corr = gf.correctionVectorToCenter();
    // Correction should push left (-X direction)
    ASSERT_TRUE(corr.x < -0.01f);
}

TEST(GeofenceTest, OutsideBounds) {
    Geofence gf;
    gf.init();

    Types::Pose2D pose;
    pose.x = 15.5f; // Past physical arena limit (15.0m)
    pose.y = 30.0f;
    pose.field_x = 15.5f;
    pose.field_y = 30.0f;
    pose.yaw_deg = 0.0f;

    gf.update(pose, 0.0f, 1000UL);
    gf.update(pose, 0.0f, 1300UL);

    ASSERT_FALSE(gf.isInside());
    ASSERT_EQ(gf.getStatus(), Types::GeofenceStatus::GEOFENCE_OUTSIDE);
}

TEST(GeofenceTest, DriftMarginShrinkage) {
    Geofence gf;
    gf.init();

    Types::Pose2D pose;
    pose.x = 13.4f;
    pose.y = 30.0f;
    pose.field_x = 13.4f;
    pose.field_y = 30.0f;
    pose.yaw_deg = 0.0f;

    // With zero drift (effective max is 14.5m): dist is 1.1m -> INSIDE
    gf.update(pose, 0.0f, 1000UL);
    gf.update(pose, 0.0f, 1300UL);
    ASSERT_EQ(gf.getStatus(), Types::GeofenceStatus::GEOFENCE_INSIDE);

    // With 0.8m drift uncertainty (effective max shrinks to 13.7m): dist becomes 0.30m -> WARNING band
    gf.update(pose, 0.8f, 2000UL);
    gf.update(pose, 0.8f, 2300UL);
    ASSERT_TRUE(gf.getStatus() == Types::GeofenceStatus::GEOFENCE_WARNING ||
                gf.getStatus() == Types::GeofenceStatus::GEOFENCE_NEAR_LIMIT);
}
