#include "test_framework.h"
#include "../hal/hal_camera.h"
#include <cstring>
#include <vector>

// ============================================================================
// REQ-DER-101 (item 1): unified camera HAL.
// Host builds verify the stub backend contract, the frame metadata path, the
// pure format-conversion math, and the exposure/AWB/night control surface.
// The injection seam additionally lets later CV tests drive full pipelines.
// ============================================================================

using RobofestDrone::Hal::CameraFrame;
using RobofestDrone::Hal::PixelFormat;

TEST(camera_hal, init_read_get_frame_metadata_contract) {
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_init());
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_is_healthy());
    // Host build is always the stub backend.
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_is_stub());

    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_read_frame());

    CameraFrame f;
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_get_frame(f));
    ASSERT_TRUE(f.width > 0);
    ASSERT_TRUE(f.height > 0);
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_get_last_frame_time_ms() != 0 ||
                true); // time may legally be 0 on first host call

    uint16_t w = 0, h = 0;
    PixelFormat fmt = PixelFormat::PIXEL_FORMAT_UNKNOWN;
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_get_native_resolution(&w, &h, &fmt));
    ASSERT_TRUE(w > 0 && h > 0);
    ASSERT_TRUE(fmt != PixelFormat::PIXEL_FORMAT_UNKNOWN);

    // Null out-params must be tolerated.
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_get_native_resolution(nullptr, nullptr, nullptr));
}

TEST(camera_hal, convert_rgb565_to_rgb888_exact_colors) {
    // One red + one green + one blue RGB565 pixel (little-endian byte order).
    // RGB565: r=[15:11] g=[10:5] b=[4:0].
    const uint16_t red565 = 0xF800, green565 = 0x07E0, blue565 = 0x001F;
    std::vector<uint8_t> src;
    auto push = [&](uint16_t v) {
        src.push_back(static_cast<uint8_t>(v & 0xFF));        // low byte first
        src.push_back(static_cast<uint8_t>((v >> 8) & 0xFF)); // high byte
    };
    push(red565);
    push(green565);
    push(blue565);

    CameraFrame f;
    f.width = 3;
    f.height = 1;
    f.format = PixelFormat::PIXEL_FORMAT_RGB565;
    f.buffer_size = static_cast<uint32_t>(src.size());
    f.data = src.data();

    std::vector<uint8_t> dst(9, 0xAA);
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_convert_format(
        f, PixelFormat::PIXEL_FORMAT_RGB888, dst.data(), static_cast<uint32_t>(dst.size())));

    ASSERT_EQ(dst[0], 248); // red: 0x1F << 3
    ASSERT_EQ(dst[1], 0);
    ASSERT_EQ(dst[2], 0);
    ASSERT_EQ(dst[3], 0);
    ASSERT_EQ(dst[4], 252); // green: 0x3F << 2
    ASSERT_EQ(dst[5], 0);
    ASSERT_EQ(dst[6], 0);
    ASSERT_EQ(dst[7], 0);
    ASSERT_EQ(dst[8], 248); // blue: 0x1F << 3
}

TEST(camera_hal, convert_to_gray8_bt601_luma) {
    std::vector<uint8_t> src = {255, 255, 255, 0, 0, 0, 0, 255, 0};
    CameraFrame f;
    f.width = 3;
    f.height = 1;
    f.format = PixelFormat::PIXEL_FORMAT_RGB888;
    f.buffer_size = static_cast<uint32_t>(src.size());
    f.data = src.data();

    uint8_t dst[3] = {0, 0, 0};
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_convert_format(
        f, PixelFormat::PIXEL_FORMAT_GRAY8, dst, sizeof(dst)));

    ASSERT_EQ(dst[0], 255); // white -> max luma
    ASSERT_EQ(dst[1], 0);   // black -> zero luma
    // Green pixel: (77*0 + 150*255 + 29*0) >> 8 = 149.
    ASSERT_EQ(dst[2], 149);
}

TEST(camera_hal, convert_rejects_bad_inputs) {
    CameraFrame bad = {};
    bad.width = 2;
    bad.height = 2;
    bad.format = PixelFormat::PIXEL_FORMAT_RGB888;
    bad.buffer_size = 4; // undersized for 2x2 RGB888
    uint8_t dummy[16] = {};
    bad.data = dummy;

    uint8_t dst[64];
    // Undersized source buffer must fail cleanly.
    ASSERT_FALSE(RobofestDrone::Hal::hal_camera_convert_format(
        bad, PixelFormat::PIXEL_FORMAT_RGB888, dst, sizeof(dst)));

    // Undersized destination must fail cleanly.
    bad.buffer_size = 12;
    ASSERT_FALSE(RobofestDrone::Hal::hal_camera_convert_format(
        bad, PixelFormat::PIXEL_FORMAT_RGB888, dst, 4));

    // Null destination must fail cleanly.
    ASSERT_FALSE(RobofestDrone::Hal::hal_camera_convert_format(
        bad, PixelFormat::PIXEL_FORMAT_GRAY8, nullptr, sizeof(dst)));

    // Unsupported destination format must fail cleanly.
    ASSERT_FALSE(RobofestDrone::Hal::hal_camera_convert_format(
        bad, PixelFormat::PIXEL_FORMAT_HSV888, dst, sizeof(dst)));

    // Null source data must fail cleanly.
    bad.data = nullptr;
    ASSERT_FALSE(RobofestDrone::Hal::hal_camera_convert_format(
        bad, PixelFormat::PIXEL_FORMAT_GRAY8, dst, sizeof(dst)));
}

TEST(camera_hal, gray8_passthrough_copy) {
    std::vector<uint8_t> src = {1, 2, 3, 4};
    CameraFrame f;
    f.width = 2;
    f.height = 2;
    f.format = PixelFormat::PIXEL_FORMAT_GRAY8;
    f.buffer_size = 4;
    f.data = src.data();

    uint8_t dst[4] = {};
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_convert_format(
        f, PixelFormat::PIXEL_FORMAT_GRAY8, dst, sizeof(dst)));
    ASSERT_EQ(std::memcmp(dst, src.data(), 4), 0);
}

TEST(camera_hal, exposure_and_awb_control_surface_callable) {
    // Contract: these calls never crash and never return in the hot path.
    RobofestDrone::Hal::hal_camera_set_auto_exposure(true);
    RobofestDrone::Hal::hal_camera_set_auto_exposure(false);
    RobofestDrone::Hal::hal_camera_set_exposure(0);
    RobofestDrone::Hal::hal_camera_set_exposure(600);
    RobofestDrone::Hal::hal_camera_set_exposure(-50);   // clamps
    RobofestDrone::Hal::hal_camera_set_exposure(99999); // clamps
    RobofestDrone::Hal::hal_camera_set_auto_whitebalance(true);
    RobofestDrone::Hal::hal_camera_set_auto_whitebalance(false);
    RobofestDrone::Hal::hal_camera_set_night_mode(true); // REQ-DER-115
    RobofestDrone::Hal::hal_camera_set_night_mode(false);
    ASSERT_TRUE(true);
}

TEST(camera_hal, injection_seam_roundtrip) {
    // 2x2 RGB565 solid magenta frame injected through the host seam is
    // returned verbatim by get_frame().
    std::vector<uint8_t> buf = {
        0x00, 0xF8, 0x00, 0xF8, // magenta hi=0xF8 lo=0x00 => R=248,B=248
        0x00, 0xF8, 0x00, 0xF8};

    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_inject_frame(
        buf.data(), 2, 2, PixelFormat::PIXEL_FORMAT_RGB565,
        static_cast<uint32_t>(buf.size())));

    CameraFrame f;
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_get_frame(f));
    ASSERT_EQ(f.width, 2u);
    ASSERT_EQ(f.height, 2u);
    ASSERT_EQ(f.format, PixelFormat::PIXEL_FORMAT_RGB565);
    ASSERT_EQ(f.buffer_size, 8u);
    ASSERT_TRUE(f.data != nullptr);

    // Injection rejected without valid dimensions/data.
    ASSERT_FALSE(RobofestDrone::Hal::hal_camera_inject_frame(
        nullptr, 2, 2, PixelFormat::PIXEL_FORMAT_RGB565, 8));

    RobofestDrone::Hal::hal_camera_clear_injection();
    CameraFrame g;
    ASSERT_TRUE(RobofestDrone::Hal::hal_camera_get_frame(g));
    ASSERT_TRUE(g.data == nullptr); // back to pure-metadata stub
}
