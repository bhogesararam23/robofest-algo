#include "hal_human.h"
#include "hal_system.h"
#include "../src/mem.h"
#include "../config/thresholds.h"
#include <stddef.h>
#include "hal_camera.h"
#include "human_detector.h"

#if defined(ARDUINO) && __has_include(<esp_dl_model.h>)
#define ROBOFEST_HAS_ESP_DL 1
#include <esp_dl_model.h>
#endif

namespace RobofestDrone {
namespace Hal {

namespace {
    bool s_human_initialized = false;
    Types::HumanDetectionSample s_mock_detection;
    bool s_mock_available = false;

    // Classical motion backend (always compiled; host-testable).
    HumanMotionDetector s_motion;
    uint8_t* s_gray_work = nullptr;   // [gw*gh]
    uint16_t s_gw = 0;
    uint16_t s_gh = 0;

#ifdef ROBOFEST_HAS_ESP_DL
    // ESP-DL pedestrian model handles (item 2 primary path).
    void* s_ped_model = nullptr;
    uint8_t s_model_divider = 0;
    Types::HumanDetectionSample s_model_last;
#endif

    void release_gray_work() {
        robofest_big_free(s_gray_work);
        s_gray_work = nullptr;
        s_gw = 0;
        s_gh = 0;
    }

    // Downsamples the latest camera frame to a grayscale working buffer.
    bool grab_gray_frame() {
        Hal::CameraFrame f;
        if (!hal_camera_get_frame(f)) return false;
        if (f.data == nullptr || f.width == 0 || f.height == 0) return false;
        if (f.format != PixelFormat::PIXEL_FORMAT_RGB565 &&
            f.format != PixelFormat::PIXEL_FORMAT_RGB888) {
            return false;
        }

        constexpr uint16_t GW = Config::VISION_PROCESS_WIDTH / 2;
        constexpr uint16_t GH = Config::VISION_PROCESS_HEIGHT / 2;
        const size_t need = static_cast<size_t>(GW) * GH;
        if (s_gray_work == nullptr || s_gw != GW || s_gh != GH) {
            release_gray_work();
            s_gray_work = static_cast<uint8_t*>(robofest_big_alloc(need));
            if (s_gray_work == nullptr) return false;
            s_gw = GW;
            s_gh = GH;
        }

        const uint16_t step_x = f.width / GW;
        const uint16_t step_y = f.height / GH;
        if (step_x == 0 || step_y == 0) return false;

        for (uint16_t y = 0; y < GH; ++y) {
            for (uint16_t x = 0; x < GW; ++x) {
                const uint32_t sx = static_cast<uint32_t>(x) * step_x;
                const uint32_t sy = static_cast<uint32_t>(y) * step_y;
                uint8_t r, g, b;
                if (f.format == PixelFormat::PIXEL_FORMAT_RGB565) {
                    const uint32_t i = (sy * f.width + sx) * 2;
                    if (i + 1 >= f.buffer_size) return false;
                    const uint16_t p =
                        static_cast<uint16_t>(f.data[i] | (f.data[i + 1] << 8));
                    r = static_cast<uint8_t>(((p >> 11) & 0x1F) << 3);
                    g = static_cast<uint8_t>(((p >> 5) & 0x3F) << 2);
                    b = static_cast<uint8_t>((p & 0x1F) << 3);
                } else {
                    const uint32_t i = (sy * f.width + sx) * 3;
                    r = f.data[i];
                    g = f.data[i + 1];
                    b = f.data[i + 2];
                }
                s_gray_work[static_cast<size_t>(y) * GW + x] =
                    static_cast<uint8_t>((77u * r + 150u * g + 29u * b) >> 8);
            }
        }
        return true;
    }
}

bool hal_human_init() {
    s_mock_available = false;
    s_human_initialized = true;

    if (!s_motion.init(Config::VISION_PROCESS_WIDTH / 2,
                       Config::VISION_PROCESS_HEIGHT / 2)) {
        s_human_initialized = false;
        hal_log("[HAL_HUMAN] Motion detector init FAILED (OOM).");
        return false;
    }

#ifdef ROBOFEST_HAS_ESP_DL
    // Primary path: ESP-DL pedestrian detection at QVGA. Failure is non-fatal
    // - the classical detector below remains fully functional.
    s_ped_model = dl::model::PedestrianDetect::create(
        Config::VISION_PROCESS_WIDTH, Config::VISION_PROCESS_HEIGHT);
    if (s_ped_model == nullptr) {
        hal_log("[HAL_HUMAN] ESP-DL pedestrian model unavailable - classical fallback active.");
    } else {
        hal_log("[HAL_HUMAN] ESP-DL pedestrian model loaded.");
    }
#else
    hal_log("[HAL_HUMAN] Human detection: classical motion backend initialized.");
#endif
    return s_human_initialized;
}

bool hal_human_is_healthy() {
    return s_human_initialized && s_motion.isInitialized();
}

bool hal_human_read_detection(Types::HumanDetectionSample& out_sample) {
    out_sample = Types::HumanDetectionSample();
    if (!s_human_initialized) return false;

    if (s_mock_available) {
        out_sample = s_mock_detection;
        s_mock_available = false;
        return true;
    }

    const uint32_t now = hal_millis();

#ifdef ROBOFEST_HAS_ESP_DL
    // Neural path on a divider so inference never starves the 50 Hz loop.
    if (s_ped_model != nullptr &&
        (++s_model_divider % Config::HUMAN_MODEL_RUN_DIVIDER) == 0 &&
        grab_gray_frame()) {
        // ESP-DL expects RGB888 in its own tensor layout; the gray working
        // frame is expanded once per run.
        static uint8_t rgb_qvga[Config::VISION_PROCESS_WIDTH *
                                Config::VISION_PROCESS_HEIGHT * 3];
        constexpr uint16_t N_PX =
            Config::VISION_PROCESS_WIDTH * Config::VISION_PROCESS_HEIGHT;
        for (uint16_t i = 0; i < N_PX; ++i) {
            rgb_qvga[i * 3] = rgb_qvga[i * 3 + 1] = rgb_qvga[i * 3 + 2] =
                s_gray_work[(i / 4)];
        }
        auto& det = *static_cast<dl::model::PedestrianDetect*>(s_ped_model);
        det.run(rgb_qvga);
        auto results = det.get_results();
        for (auto& r : results) {
            const float conf = r.second;
            if (conf >= Config::HUMAN_MODEL_CONFIDENCE_MIN) {
                s_model_last.valid = true;
                s_model_last.confidence = conf;
                s_model_last.pixel_width = static_cast<float>(r.first.box[2] - r.first.box[0]);
                s_model_last.pixel_height = static_cast<float>(r.first.box[3] - r.first.box[1]);
                s_model_last.pixel_x = static_cast<float>(r.first.box[0] + r.first.box[2]) * 0.5f;
                s_model_last.pixel_y = static_cast<float>(r.first.box[1] + r.first.box[3]) * 0.5f;
                s_model_last.timestamp_ms = now;
                break;
            }
        }
    }
    if (s_model_last.valid &&
        (now - s_model_last.timestamp_ms) <= Config::MINE_DETECTION_MAX_AGE_MS) {
        out_sample = s_model_last;
        return true;
    }
#endif

    // Classical path every call (cheap frame differencing).
    if (!grab_gray_frame()) return false;
    Types::HumanDetectionSample sample;
    if (s_motion.process(s_gray_work, sample, now)) {
        out_sample = sample;
        return true;
    }
    return false;
}

void hal_human_set_mock_detection(const Types::HumanDetectionSample& sample) {
    s_mock_detection = sample;
    s_mock_available = true;
}

} // namespace Hal
} // namespace RobofestDrone