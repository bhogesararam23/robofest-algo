#include "test_framework.h"
#include <cmath>

TEST(LocalizationMathTest, OpticalFlowVelocityFormula) {
    // Optical flow velocity formula:
    // v_body_mps = (pixel_shift / focal_length_px) * altitude_m / dt_s
    float focal_length_px = 350.0f;
    float altitude_m = 2.0f;
    float dt_s = 0.020f; // 20ms = 50Hz

    // Suppose pixel shift is 35 pixels in 20ms
    float pixel_shift_x = 35.0f;
    float expected_vx = (pixel_shift_x / focal_length_px) * altitude_m / dt_s;
    // Calculation: (35 / 350) * 2.0 / 0.020 = 0.1 * 2.0 / 0.02 = 0.2 / 0.02 = 10.0 m/s

    ASSERT_FLOAT_EQ(expected_vx, 10.0f);

    // Suppose pixel shift is 1.4 pixels in 20ms at 2.0m altitude
    pixel_shift_x = 1.4f;
    expected_vx = (pixel_shift_x / focal_length_px) * altitude_m / dt_s;
    // Calculation: (1.4 / 350) * 2.0 / 0.020 = 0.004 * 2.0 / 0.02 = 0.008 / 0.02 = 0.40 m/s (cruise speed)

    ASSERT_FLOAT_EQ(expected_vx, 0.40f);
}

TEST(LocalizationMathTest, BodyToFieldFrameRotation) {
    // Test 2D rotation matrix:
    // [ vx_field ] = [ cos(yaw)  -sin(yaw) ] [ vx_body ]
    // [ vy_field ] = [ sin(yaw)   cos(yaw) ] [ vy_body ]

    float vx_body = 0.0f;
    float vy_body = 1.0f; // Moving forward along body +Y

    // 1. Heading = 0 deg (facing +Y in arena coordinates)
    float yaw_rad = 0.0f;
    float vx_field = vx_body * std::cos(yaw_rad) - vy_body * std::sin(yaw_rad);
    float vy_field = vx_body * std::sin(yaw_rad) + vy_body * std::cos(yaw_rad);

    ASSERT_NEAR(vx_field, 0.0f, 0.001f);
    ASSERT_NEAR(vy_field, 1.0f, 0.001f);

    // 2. Heading = +90 deg (facing +X)
    yaw_rad = 90.0f * 3.14159265358979323846f / 180.0f;
    vx_field = vx_body * std::cos(yaw_rad) - vy_body * std::sin(yaw_rad);
    vy_field = vx_body * std::sin(yaw_rad) + vy_body * std::cos(yaw_rad);

    ASSERT_NEAR(vx_field, -1.0f, 0.001f);
    ASSERT_NEAR(vy_field, 0.0f, 0.001f);
}
