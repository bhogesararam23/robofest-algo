#include "hal_camera.h"
#include "hal_system.h"
#include <cstdio>
#include <cstring>

#if defined(ARDUINO)
#include <esp_camera.h>
#endif

namespace RobofestDrone {
namespace Hal {

#if defined(ARDUINO)

// ============================================================================
// REAL HARDWARE BACKEND - OV5640 via esp32-camera (Seeed XIAO ESP32-S3 Sense)
// ============================================================================

namespace {

// XIAO ESP32S3 Sense expansion board OV5640 DVP pin map (CameraWebServer model
// CAMERA_MODEL_XIAO_ESP32S3_SENSE).
constexpr int PWDN_GPIO_NUM  = -1;
constexpr int RESET_GPIO_NUM = -1;
constexpr int XCLK_GPIO_NUM  = 10;
constexpr int SIOD_GPIO_NUM  = 40;
constexpr int SIOC_GPIO_NUM  = 39;
constexpr int Y9_GPIO_NUM    = 48;
constexpr int Y8_GPIO_NUM    = 11;
constexpr int Y7_GPIO_NUM    = 12;
constexpr int Y6_GPIO_NUM    = 14;
constexpr int Y5_GPIO_NUM    = 16;
constexpr int Y4_GPIO_NUM    = 18;
constexpr int Y3_GPIO_NUM    = 17;
constexpr int Y2_GPIO_NUM    = 15;
constexpr int VSYNC_GPIO_NUM = 38;
constexpr int HREF_GPIO_NUM  = 47;
constexpr int PCLK_GPIO_NUM  = 13;

bool s_camera_initialized = false;
bool s_camera_ready = false;      // driver up and producing frames
bool s_night_mode = false;
uint32_t s_last_frame_ms = 0;
camera_fb_t* s_current_fb = nullptr;

void release_current_fb() {
    if (s_current_fb != nullptr) {
        esp_camera_fb_return(s_current_fb);
        s_current_fb = nullptr;
    }
}

PixelFormat fb_pixel_format(pixformat_t fmt) {
    switch (fmt) {
        case PIXFORMAT_RGB565: return PixelFormat::PIXEL_FORMAT_RGB565;
        case PIXFORMAT_RGB888: return PixelFormat::PIXEL_FORMAT_RGB888;
        case PIXFORMAT_GRAYSCALE: return PixelFormat::PIXEL_FORMAT_GRAY8;
        default: return PixelFormat::PIXEL_FORMAT_UNKNOWN;
    }
}

} // namespace

bool hal_camera_init() {
    if (s_camera_initialized) {
        return s_camera_ready;
    }
    s_camera_initialized = true;

    camera_config_t cfg = {};
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_pwdn     = PWDN_GPIO_NUM;
    cfg.pin_reset    = RESET_GPIO_NUM;
    cfg.pin_xclk     = XCLK_GPIO_NUM;
    cfg.pin_sscb_sda = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl = SIOC_GPIO_NUM;
    cfg.pin_sccb_sda = SIOD_GPIO_NUM;
    cfg.pin_sccb_scl = SIOC_GPIO_NUM;
    cfg.pin_d7       = Y9_GPIO_NUM;
    cfg.pin_d6       = Y8_GPIO_NUM;
    cfg.pin_d5       = Y7_GPIO_NUM;
    cfg.pin_d4       = Y6_GPIO_NUM;
    cfg.pin_d3       = Y5_GPIO_NUM;
    cfg.pin_d2       = Y4_GPIO_NUM;
    cfg.pin_d1       = Y3_GPIO_NUM;
    cfg.pin_d0       = Y2_GPIO_NUM;
    cfg.pin_vsync    = VSYNC_GPIO_NUM;
    cfg.pin_href     = HREF_GPIO_NUM;
    cfg.pin_pclk     = PCLK_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;

    // Working format: QVGA RGB565 keeps the DMA buffers in PSRAM budget while
    // the vision pipeline downsamples internally to its process resolution.
    cfg.pixel_format = PIXFORMAT_RGB565;
    cfg.frame_size   = FRAMESIZE_QVGA;
    cfg.jpeg_quality = 12; // unused for RGB565, kept for safety
    cfg.fb_count     = 2;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;

    release_current_fb();
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        s_camera_ready = false;
        char buf[96];
        std::snprintf(buf, sizeof(buf),
                      "[HAL_CAMERA] esp_camera_init FAILED err=0x%x - falling back to stub semantics",
                      static_cast<unsigned>(err));
        hal_log(buf);
        return false;
    }

    // Initial exposure discipline: match the calibrated-HSV contract.
    sensor_t* s = esp_camera_sensor_get();
    if (s != nullptr) {
        s->set_exposure_ctrl(s, 1);          // AEC on until mission locks it
        s->set_wb_mode(s, 0);                // auto white balance
        s->set_awb_gain(s, 1);
        s->set_gain_ctrl(s, 0);              // AGC off for deterministic gain
        s->set_hmirror(s, 0);
        s->set_vflip(s, 1);                  // downward-mount orientation
    }

    s_camera_ready = true;
    hal_log("[HAL_CAMERA] OV5640 initialized (QVGA RGB565, PSRAM fb).");
    return true;
}

static bool acquire_latest_frame_locked() {
    if (!s_camera_ready) return false;
    release_current_fb();
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb == nullptr) {
        return false;
    }
    s_current_fb = fb;
    s_last_frame_ms = hal_millis();
    return true;
}

bool hal_camera_read_frame() {
    return acquire_latest_frame_locked();
}

bool hal_camera_get_frame(CameraFrame& frame) {
    frame.width = 0;
    frame.height = 0;
    frame.timestamp_ms = 0;
    frame.format = PixelFormat::PIXEL_FORMAT_UNKNOWN;
    frame.buffer_size = 0;
    frame.data = nullptr;

    if (!s_camera_initialized || !acquire_latest_frame_locked()) {
        return false;
    }

    frame.width = s_current_fb->width;
    frame.height = s_current_fb->height;
    frame.timestamp_ms = s_last_frame_ms;
    frame.format = fb_pixel_format(s_current_fb->format);
    frame.buffer_size = s_current_fb->len;
    frame.data = s_current_fb->buf;
    return true;
}

bool hal_camera_is_healthy() {
    return s_camera_ready;
}

bool hal_camera_is_stub() {
    return !s_camera_ready;
}

uint32_t hal_camera_get_last_frame_time_ms() {
    return s_last_frame_ms;
}

bool hal_camera_get_native_resolution(uint16_t* w, uint16_t* h, PixelFormat* fmt) {
    if (!s_camera_ready) return false;
    sensor_t* s = esp_camera_sensor_get();
    if (s == nullptr) return false;
    if (w) *w = 320;
    if (h) *h = 240;
    if (fmt) *fmt = PixelFormat::PIXEL_FORMAT_RGB565;
    (void)s;
    return true;
}

void hal_camera_set_auto_exposure(bool enabled) {
    if (!s_camera_ready) return;
    sensor_t* s = esp_camera_sensor_get();
    if (s == nullptr) return;
    s->set_exposure_ctrl(s, enabled ? 1 : 0);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "[HAL_CAMERA] AEC %s", enabled ? "ENABLED" : "LOCKED");
    hal_log(buf);
}

void hal_camera_set_exposure(int32_t value) {
    if (!s_camera_ready) return;
    sensor_t* s = esp_camera_sensor_get();
    if (s == nullptr) return;
    if (value < 0) value = 0;
    if (value > 1200) value = 1200; // OV5640 AEC register range used by driver
    s->set_exposure_ctrl(s, 0);     // manual mode
    s->set_aec_value(s, static_cast<int>(value));
    char buf[72];
    std::snprintf(buf, sizeof(buf), "[HAL_CAMERA] Manual exposure set to %ld",
                  static_cast<long>(value));
    hal_log(buf);
}

void hal_camera_set_auto_whitebalance(bool enabled) {
    if (!s_camera_ready) return;
    sensor_t* s = esp_camera_sensor_get();
    if (s == nullptr) return;
    if (enabled) {
        s->set_wb_mode(s, 0);   // continuous auto WB
        s->set_awb_gain(s, 1);
    } else {
        // Lock to the sunny preset for deterministic calibrated colors.
        s->set_wb_mode(s, 1);
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "[HAL_CAMERA] AWB %s", enabled ? "ENABLED" : "LOCKED");
    hal_log(buf);
}

void hal_camera_set_night_mode(bool enabled) {
    s_night_mode = enabled;
    if (!s_camera_ready) return;
    sensor_t* s = esp_camera_sensor_get();
    if (s == nullptr) return;
    if (enabled) {
        s->set_gainceiling(s, GAINCEILING_64X);
        s->set_gain_ctrl(s, 1);
        s->set_ae_level(s, 2);
        hal_log("[HAL_CAMERA] NIGHT MODE engaged (AGC x64 ceiling, AE bias +2)");
    } else {
        s->set_gainceiling(s, GAINCEILING_8X);
        s->set_gain_ctrl(s, 0);
        s->set_ae_level(s, 0);
        hal_log("[HAL_CAMERA] Night mode disabled.");
    }
}

#else // !ARDUINO

// ============================================================================
// HOST STUB BACKEND (unit-test / desktop verification)
// ============================================================================

namespace {
    bool s_camera_initialized = false;
    bool s_stub_mode = true;
    bool s_night_mode = false;
    uint16_t s_stub_w = 320;
    uint16_t s_stub_h = 240;
    uint32_t s_last_frame_ms = 0;
}

bool hal_camera_init() {
    s_camera_initialized = true;
    s_stub_mode = true;
    s_last_frame_ms = hal_millis();
    hal_log("[HAL_CAMERA] Camera stub initialized (safe default).");
    return true;
}

bool hal_camera_read_frame() {
    if (!s_camera_initialized) return false;
    s_last_frame_ms = hal_millis();
    return true;
}

bool hal_camera_get_frame(CameraFrame& frame) {
    if (!s_camera_initialized) {
        frame.width = 0;
        frame.height = 0;
        frame.timestamp_ms = 0;
        frame.format = PixelFormat::PIXEL_FORMAT_UNKNOWN;
        frame.buffer_size = 0;
        frame.data = nullptr;
        return false;
    }
    s_last_frame_ms = hal_millis();
    frame.width = s_stub_w;
    frame.height = s_stub_h;
    frame.timestamp_ms = s_last_frame_ms;
    frame.format = PixelFormat::PIXEL_FORMAT_RGB565;
    frame.buffer_size = 0;
    frame.data = nullptr; // Stub carries no pixels by design
    return true;
}

bool hal_camera_is_healthy() { return s_camera_initialized; }
bool hal_camera_is_stub() { return s_stub_mode; }
uint32_t hal_camera_get_last_frame_time_ms() { return s_last_frame_ms; }

bool hal_camera_get_native_resolution(uint16_t* w, uint16_t* h, PixelFormat* fmt) {
    if (!s_camera_initialized) return false;
    if (w) *w = s_stub_w;
    if (h) *h = s_stub_h;
    if (fmt) *fmt = PixelFormat::PIXEL_FORMAT_RGB565;
    return true;
}

void hal_camera_set_auto_exposure(bool enabled) {
    char buf[80];
    std::snprintf(buf, sizeof(buf), "[HAL_CAMERA] Auto-exposure %s (stub)",
                  enabled ? "ENABLED" : "DISABLED");
    hal_log(buf);
}

void hal_camera_set_exposure(int32_t value) {
    char buf[80];
    std::snprintf(buf, sizeof(buf), "[HAL_CAMERA] Manual exposure set to %ld (stub)",
                  static_cast<long>(value));
    hal_log(buf);
}

void hal_camera_set_auto_whitebalance(bool enabled) {
    char buf[80];
    std::snprintf(buf, sizeof(buf), "[HAL_CAMERA] Auto-whitebalance %s (stub)",
                  enabled ? "ENABLED" : "DISABLED");
    hal_log(buf);
}

void hal_camera_set_night_mode(bool enabled) {
    s_night_mode = enabled;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "[HAL_CAMERA] Night mode %s (stub)",
                  enabled ? "ON" : "OFF");
    hal_log(buf);
}

#endif // ARDUINO

// ============================================================================
// PURE FORMAT CONVERSION (shared by both backends, host-testable)
// ============================================================================

namespace {

inline void rgb565_to_rgb888(const uint8_t* p, uint8_t& r, uint8_t& g, uint8_t& b,
                             bool little_endian) {
    uint16_t v = little_endian
        ? static_cast<uint16_t>(p[0] | (p[1] << 8))
        : static_cast<uint16_t>(p[1] | (p[0] << 8));
    r = static_cast<uint8_t>(((v >> 11) & 0x1F) << 3);
    g = static_cast<uint8_t>(((v >> 5) & 0x3F) << 2);
    b = static_cast<uint8_t>((v & 0x1F) << 3);
}

inline uint8_t rgb_to_gray(uint8_t r, uint8_t g, uint8_t b) {
    // ITU-R BT.601 luma in integer fixed point (sum of weights = 256).
    return static_cast<uint8_t>((77u * r + 150u * g + 29u * b) >> 8);
}

} // namespace

bool hal_camera_convert_format(
    const CameraFrame& src,
    PixelFormat dst_fmt,
    uint8_t* dst_buf,
    uint32_t dst_buf_size) {

    if (src.data == nullptr || dst_buf == nullptr ||
        src.width == 0 || src.height == 0) {
        return false;
    }

    const size_t n_px = static_cast<size_t>(src.width) * src.height;

    size_t needed;
    if (dst_fmt == PixelFormat::PIXEL_FORMAT_RGB888) needed = n_px * 3;
    else if (dst_fmt == PixelFormat::PIXEL_FORMAT_GRAY8) needed = n_px;
    else return false;

    if (dst_buf_size < needed) return false;

    const bool le =
#ifdef ARDUINO
        true; // Xtensa is little-endian
#else
        true; // host targets here are little-endian (x86/MinGW)
#endif
    (void)le;

    if (src.format == PixelFormat::PIXEL_FORMAT_RGB565) {
        if (src.buffer_size < n_px * 2) return false;
        if (dst_fmt == PixelFormat::PIXEL_FORMAT_RGB888) {
            for (size_t i = 0; i < n_px; ++i) {
                uint8_t r, g, b;
                rgb565_to_rgb888(src.data + i * 2, r, g, b, true);
                dst_buf[i * 3 + 0] = r;
                dst_buf[i * 3 + 1] = g;
                dst_buf[i * 3 + 2] = b;
            }
        } else { // GRAY8
            for (size_t i = 0; i < n_px; ++i) {
                uint8_t r, g, b;
                rgb565_to_rgb888(src.data + i * 2, r, g, b, true);
                dst_buf[i] = rgb_to_gray(r, g, b);
            }
        }
        return true;
    }

    if (src.format == PixelFormat::PIXEL_FORMAT_RGB888) {
        if (src.buffer_size < n_px * 3) return false;
        if (dst_fmt == PixelFormat::PIXEL_FORMAT_RGB888) {
            std::memcpy(dst_buf, src.data, n_px * 3);
            return true;
        }
        for (size_t i = 0; i < n_px; ++i) {
            dst_buf[i] = rgb_to_gray(src.data[i * 3], src.data[i * 3 + 1], src.data[i * 3 + 2]);
        }
        return true;
    }

    if (src.format == PixelFormat::PIXEL_FORMAT_GRAY8 && dst_fmt == PixelFormat::PIXEL_FORMAT_GRAY8) {
        if (src.buffer_size < n_px) return false;
        std::memcpy(dst_buf, src.data, n_px);
        return true;
    }

    return false;
}

} // namespace Hal
} // namespace RobofestDrone
