#include "hal_command.h"
#include "hal_system.h"
#include "hal_camera.h"
#include "gesture_engine.h"
#include "../src/mem.h"

#if defined(ARDUINO) && __has_include(<ESP_SR.h>)
#define ROBOFEST_HAS_ESP_SR 1
#include <ESP_SR.h>
#endif

namespace RobofestDrone {
namespace Hal {

namespace {
    bool s_cmd_initialized = false;
    bool s_gesture_enabled = true;
    bool s_voice_enabled = false;

    Types::CommandSample s_mock_sample;
    bool s_has_mock = false;

    GestureEngine s_gesture;
    uint8_t* s_hand_rgb = nullptr;    // [HW*HH*3] downsampled hand-view frame
    constexpr uint16_t HW = 64;
    constexpr uint16_t HH = 48;

#if defined(ROBOFEST_HAS_ESP_SR)
    // Voice keyword-spotting front-end (ESP-SR MultiNet on ESP32-S3 PDM mic).
    // Phrase table order maps directly onto CommandType ids.
    constexpr const char* kVoicePhrases[] = {
        "START MISSION",   // START
        "FORWARD",         // FORWARD
        "HOLD POSITION",   // PAUSE
        "SCAN LEFT",       // SCAN_LEFT
        "SCAN RIGHT",      // SCAN_RIGHT
        "STOP ABORT",      // STOP_ABORT
    };
    constexpr uint8_t kVoicePhraseCount =
        static_cast<uint8_t>(sizeof(kVoicePhrases) / sizeof(kVoicePhrases[0]));

    volatile Types::CommandSample s_voice_latest;
    volatile bool s_voice_fresh = false;

    void sr_event_handler(void* arg, int32_t event, int phrase_id) {
        (void)arg;
        (void)event;
        if (phrase_id < 0 || phrase_id >= kVoicePhraseCount) return;
        Types::CommandSample s;
        s.valid = true;
        s.command = static_cast<Types::CommandType>(
            static_cast<uint8_t>(Types::CommandType::START) + phrase_id);
        s.confidence = 0.80f;
        s.source = Types::CommandSource::COMMAND_SOURCE_VOICE;
        s.timestamp_ms = hal_millis();
        s_voice_latest = s;
        s_voice_fresh = true;
    }
#endif

    bool grab_hand_frame() {
        CameraFrame f;
        if (!hal_camera_get_frame(f)) return false;
        if (f.data == nullptr || f.width < HW || f.height < HH) return false;
        if (f.format != PixelFormat::PIXEL_FORMAT_RGB565 &&
            f.format != PixelFormat::PIXEL_FORMAT_RGB888) {
            return false;
        }
        if (s_hand_rgb == nullptr) {
            s_hand_rgb = static_cast<uint8_t*>(
                robofest_big_alloc(static_cast<size_t>(HW) * HH * 3));
            if (s_hand_rgb == nullptr) return false;
        }

        const uint16_t step_x = f.width / HW;
        const uint16_t step_y = f.height / HH;
        for (uint16_t y = 0; y < HH; ++y) {
            for (uint16_t x = 0; x < HW; ++x) {
                const uint32_t sx = static_cast<uint32_t>(x) * step_x;
                const uint32_t sy = static_cast<uint32_t>(y) * step_y;
                uint8_t* dst = s_hand_rgb + (static_cast<size_t>(y) * HW + x) * 3;
                if (f.format == PixelFormat::PIXEL_FORMAT_RGB565) {
                    const uint32_t i = (sy * f.width + sx) * 2;
                    if (i + 1 >= f.buffer_size) return false;
                    const uint16_t p =
                        static_cast<uint16_t>(f.data[i] | (f.data[i + 1] << 8));
                    dst[0] = static_cast<uint8_t>(((p >> 11) & 0x1F) << 3);
                    dst[1] = static_cast<uint8_t>(((p >> 5) & 0x3F) << 2);
                    dst[2] = static_cast<uint8_t>((p & 0x1F) << 3);
                } else {
                    const uint32_t i = (sy * f.width + sx) * 3;
                    dst[0] = f.data[i];
                    dst[1] = f.data[i + 1];
                    dst[2] = f.data[i + 2];
                }
            }
        }
        return true;
    }
}

bool hal_command_init() {
    s_has_mock = false;
    s_cmd_initialized = true;
    s_gesture.reset();

#if defined(ROBOFEST_HAS_ESP_SR)
    if (s_voice_enabled) {
        for (uint8_t i = 0; i < kVoicePhraseCount; ++i) {
            ESP_SR.addPhrase(kVoicePhrases[i]);
        }
        ESP_SR.onEvent(sr_event_handler, nullptr);
        if (!ESP_SR.begin()) {
            hal_log("[HAL_COMMAND] ESP-SR voice KWS failed to start - voice disabled.");
            s_voice_enabled = false;
        } else {
            hal_log("[HAL_COMMAND] ESP-SR voice KWS active with command dictionary.");
        }
    }
#else
    hal_log("[HAL_COMMAND] Gesture engine initialized; voice KWS requires ESP-SR build.");
#endif
    return s_cmd_initialized;
}

bool hal_command_is_healthy() {
    return s_cmd_initialized;
}

bool hal_command_read_gesture(Types::CommandSample& sample) {
    sample = Types::CommandSample();
    sample.command = Types::CommandType::NONE;
    if (!s_cmd_initialized || !s_gesture_enabled) return false;

    if (s_has_mock) {
        sample = s_mock_sample;
        s_has_mock = false;
        return sample.valid;
    }

    if (!grab_hand_frame()) return false;

    float cx, cy, frac;
    HandObservation obs;
    obs.present = gesture_skin_blob_centroid(s_hand_rgb, HW, HH, cx, cy, frac);
    if (obs.present) {
        obs.cx = cx;
        obs.cy = cy;
        obs.timestamp_ms = hal_millis();
    } else {
        obs.timestamp_ms = hal_millis();
    }

    return s_gesture.update(obs, sample);
}

bool hal_command_read_voice(Types::CommandSample& sample) {
    sample = Types::CommandSample();
    sample.command = Types::CommandType::NONE;
    if (!s_cmd_initialized || !s_voice_enabled) return false;

    if (s_has_mock) {
        sample = s_mock_sample;
        s_has_mock = false;
        return sample.valid;
    }

#if defined(ROBOFEST_HAS_ESP_SR)
    if (s_voice_fresh) {
        s_voice_fresh = false;
        sample = s_voice_latest;
        return sample.valid;
    }
#endif
    return false;
}

void hal_command_enable_gesture(bool enabled) {
    s_gesture_enabled = enabled;
}

void hal_command_enable_voice(bool enabled) {
    s_voice_enabled = enabled;
}

void hal_command_set_mock_gesture(const Types::CommandSample& sample) {
    s_mock_sample = sample;
    s_has_mock = true;
}

void hal_command_clear_mock() {
    s_has_mock = false;
}

} // namespace Hal
} // namespace RobofestDrone