#include "hal_marker.h"
#include "hal_system.h"
#include "../config/thresholds.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>

#if defined(ARDUINO)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/rmt.h>
#include <soc/rmt_reg.h>
#endif

namespace RobofestDrone {
namespace Hal {

// ============================================================================
// PURE RENDER MATH - pattern table + guidance->LED-index translation
// (shared by both backends; unit-tested on host)
// ============================================================================

namespace {

struct Rgb { uint8_t r, g, b; };

constexpr uint8_t PHASE_STEPS = 255;

inline uint8_t scale_channel(uint8_t c, uint8_t brightness_percent) {
    return static_cast<uint8_t>((static_cast<uint16_t>(c) * brightness_percent) / 100u);
}

// Triangle wave 0..1..0 over one phase period.
inline float pulse(uint8_t phase) {
    const float p = static_cast<float>(phase) / static_cast<float>(PHASE_STEPS);
    return (p <= 0.5f) ? (p * 2.0f) : ((1.0f - p) * 2.0f);
}

// Hard blink: 50% duty.
inline bool blink_on(uint8_t phase) {
    return phase < (PHASE_STEPS / 2);
}

// Comet position for chase animations: sweeps 0..COUNT-1 then wraps.
inline int comet_head(uint8_t phase, uint8_t count, bool reverse) {
    const float p = static_cast<float>(phase) / static_cast<float>(PHASE_STEPS);
    int head = static_cast<int>(p * count * 2.0f) % count;
    if (reverse) head = (count - 1) - head;
    return head;
}

Rgb render_pattern_led(Types::MarkerPattern pattern, uint8_t idx, uint8_t phase,
                       uint8_t guidance_idx) {
    constexpr uint8_t N = MARKER_LED_COUNT;
    switch (pattern) {
        case Types::MarkerPattern::MARKER_OFF:
            return {0, 0, 0};

        case Types::MarkerPattern::MARKER_FORWARD: {
            // Green comet sweeping forward with a steering window bias.
            int head = comet_head(phase, N, false);
            head = std::min(N - 1, std::max(0, head +
                (static_cast<int>(guidance_idx) - (N / 2)) / 4));
            if (idx == head || idx == ((head + N - 1) % N)) return {0, 220, 40};
            return {0, 25, 5};
        }

        case Types::MarkerPattern::MARKER_STOP: {
            // Solid red with slow breathing intensity.
            const float b = 0.55f + 0.45f * pulse(phase);
            return {static_cast<uint8_t>(230 * b), 0, 0};
        }

        case Types::MarkerPattern::MARKER_LEFT:
        case Types::MarkerPattern::MARKER_REJOIN_LEFT: {
            // Amber/magenta sweep toward the left half of the strip.
            const int head = comet_head(phase, N, true);
            const bool active = (idx <= guidance_idx) && (std::abs(static_cast<int>(idx) - head) <= 3);
            if (pattern == Types::MarkerPattern::MARKER_LEFT)
                return active ? Rgb{240, 140, 0} : Rgb{30, 18, 0};
            return active ? Rgb{200, 0, 160} : Rgb{26, 0, 20};
        }

        case Types::MarkerPattern::MARKER_RIGHT:
        case Types::MarkerPattern::MARKER_REJOIN_RIGHT: {
            const int head = comet_head(phase, N, false);
            const bool active = (idx >= guidance_idx) && (std::abs(static_cast<int>(idx) - head) <= 3);
            if (pattern == Types::MarkerPattern::MARKER_RIGHT)
                return active ? Rgb{240, 140, 0} : Rgb{30, 18, 0};
            return active ? Rgb{200, 0, 160} : Rgb{26, 0, 20};
        }

        case Types::MarkerPattern::MARKER_SAFE_PATH: {
            // Continuous green corridor: gentle full-strip breathing.
            const float b = 0.65f + 0.35f * pulse(phase);
            return {0, static_cast<uint8_t>(200 * b), 20};
        }

        case Types::MarkerPattern::MARKER_EMERGENCY:
            // Rapid red/white strobe (highest urgency).
            return blink_on(phase) ? Rgb{255, 40, 0} : Rgb{255, 255, 255};

        case Types::MarkerPattern::MARKER_MISSION_COMPLETE: {
            // Alternating green/cyan celebration blink.
            const bool odd = (idx & 1u) != 0u;
            const bool on = blink_on(phase);
            if (on) return odd ? Rgb{0, 220, 120} : Rgb{0, 120, 220};
            return {0, 10, 10};
        }

        case Types::MarkerPattern::MARKER_CAUTION:
            return blink_on(phase) ? Rgb{220, 130, 0} : Rgb{25, 15, 0};

        case Types::MarkerPattern::MARKER_LANDING: {
            // Blue descending comet from both strip ends toward the middle.
            int head = comet_head(phase, N / 2, false);
            const int mirror_lo = head;
            const int mirror_hi = (N - 1) - head;
            if (idx == mirror_lo || idx == mirror_hi) return {0, 60, 240};
            return {0, 6, 30};
        }

        default:
            return {0, 0, 0};
    }
}

} // namespace

uint8_t hal_marker_guidance_index(float heading_error_deg) {
    // Linear map across the strip: -90 deg -> LED 0, 0 -> center, +90 -> last.
    constexpr uint8_t N = MARKER_LED_COUNT;
    float clamped = std::max(-90.0f, std::min(90.0f, heading_error_deg));
    float idx_f = ((clamped + 90.0f) / 180.0f) * static_cast<float>(N - 1);
    int idx = static_cast<int>(idx_f + 0.5f);
    return static_cast<uint8_t>(std::max(0, std::min(static_cast<int>(N - 1), idx)));
}

void marker_render_led(Types::MarkerPattern pattern, uint8_t led_index, uint8_t phase,
                       uint8_t guidance_idx, uint8_t out_rgb[3]) {
    if (out_rgb == nullptr) return;
    Rgb c = render_pattern_led(pattern, led_index, phase, guidance_idx);
    out_rgb[0] = c.r;
    out_rgb[1] = c.g;
    out_rgb[2] = c.b;
}

// ============================================================================
// SHARED STATE (written by control thread, read by render loop)
// ============================================================================

namespace {

struct MarkerState {
    Types::MarkerPattern pattern = Types::MarkerPattern::MARKER_OFF;
    uint8_t brightness = Config::MARKER_BRIGHTNESS_DEFAULT_PERCENT;
    bool enabled = true;
    float guidance_deg = 0.0f;
};

volatile MarkerState s_state;

#if defined(ARDUINO)
portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t s_render_task_handle = nullptr;
bool s_rmt_ready = false;
bool s_hw_ok = false;
#endif

bool s_marker_initialized = false;

#if defined(ARDUINO)

// ---------------------------------------------------------------------------
// WS2812B timing at RMT source clock 80 MHz, divider 2 -> 25 ns per tick.
// ---------------------------------------------------------------------------
constexpr uint16_t WS_T0H_TICKS = 14;   // 350 ns
constexpr uint16_t WS_T0L_TICKS = 32;   // 800 ns
constexpr uint16_t WS_T1H_TICKS = 28;   // 700 ns
constexpr uint16_t WS_T1L_TICKS = 24;   // 600 ns
constexpr uint16_t WS_RESET_CYCLES = 2500; // ~62.5 us low

void encode_led_frame(const uint8_t* rgb, rmt_item32_t* out) {
    // WS2812B wants GRB byte order.
    const uint8_t grb[3] = {rgb[1], rgb[0], rgb[2]};
    for (uint8_t byte_i = 0; byte_i < 3; ++byte_i) {
        for (uint8_t bit_i = 0; bit_i < 8; ++bit_i) {
            const bool one = (grb[byte_i] >> (7 - bit_i)) & 0x1u;
            rmt_item32_t item;
            item.level0 = 1;
            item.duration0 = one ? WS_T1H_TICKS : WS_T0H_TICKS;
            item.level1 = 0;
            item.duration1 = one ? WS_T1L_TICKS : WS_T0L_TICKS;
            *out++ = item;
        }
    }
}

void marker_render_task(void* arg) {
    static uint8_t frame_rgb[MARKER_LED_COUNT * 3];
    static rmt_item32_t rmt_items[MARKER_LED_COUNT * 24];

    uint8_t local_phase = 0;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(Config::MARKER_GUIDANCE_UPDATE_MS);

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        // Snapshot shared state under a very short critical section.
        Types::MarkerPattern pattern;
        uint8_t brightness;
        bool enabled;
        float guidance_deg;
        taskENTER_CRITICAL(&s_state_mux);
        pattern = s_state.pattern;
        brightness = s_state.brightness;
        enabled = s_state.enabled;
        guidance_deg = s_state.guidance_deg;
        taskEXIT_CRITICAL(&s_state_mux);

        const uint8_t gidx = hal_marker_guidance_index(guidance_deg);

        for (uint8_t i = 0; i < MARKER_LED_COUNT; ++i) {
            uint8_t rgb[3] = {0, 0, 0};
            if (enabled && pattern != Types::MarkerPattern::MARKER_OFF) {
                marker_render_led(pattern, i, local_phase, gidx, rgb);
                for (uint8_t ch = 0; ch < 3; ++ch) {
                    rgb[ch] = scale_channel(rgb[ch], brightness);
                }
            }
            frame_rgb[i * 3 + 0] = rgb[0];
            frame_rgb[i * 3 + 1] = rgb[1];
            frame_rgb[i * 3 + 2] = rgb[2];
        }

        if (s_rmt_ready) {
            for (uint8_t i = 0; i < MARKER_LED_COUNT; ++i) {
                encode_led_frame(frame_rgb + i * 3, rmt_items + i * 24);
            }
            rmt_write_items(RMT_CHANNEL_0, rmt_items,
                            static_cast<size_t>(MARKER_LED_COUNT) * 24, true);
            // Reset latch between frames keeps colors stable across strips.
            rmt_item32_t reset_item = {};
            reset_item.level0 = 0;
            reset_item.duration0 = WS_RESET_CYCLES;
            reset_item.level1 = 0;
            reset_item.duration1 = WS_RESET_CYCLES;
            rmt_write_items(RMT_CHANNEL_0, &reset_item, 1, true);
        }

        local_phase += 5u; // ~51 phase-steps/s at 20 Hz -> smooth animations
    }
}

#endif // ARDUINO

} // namespace

// ============================================================================
// HAL API
// ============================================================================

bool hal_marker_init() {
#if defined(ARDUINO)
    if (s_marker_initialized) {
        return s_hw_ok;
    }
    s_marker_initialized = true;

    rmt_config_t cfg = {};
    cfg.channel = RMT_CHANNEL_0;
    cfg.gpio_num = static_cast<gpio_num_t>(MARKER_LED_DATA_PIN);
    cfg.rmt_mode = RMT_MODE_TX;
    cfg.clk_div = 2; // 80 MHz / 2 -> 12.5 ns * 2 = 25 ns resolution
    cfg.mem_block_num = 1;
    cfg.tx_config.loop_en = false;
    cfg.tx_config.carrier_en = false;
    cfg.tx_config.idle_output_en = true;
    cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    esp_err_t err = rmt_config(&cfg);
    if (err == ESP_OK) {
        err = rmt_driver_install(cfg.channel, 0, 0);
    }
    s_rmt_ready = (err == ESP_OK);
    if (!s_rmt_ready) {
        hal_log("[HAL_MARKER] RMT init failed - LEDs disabled, patterns still tracked.");
    } else {
        char buf[80];
        std::snprintf(buf, sizeof(buf),
                      "[HAL_MARKER] WS2812B RMT output ready (GPIO %d, %u LEDs).",
                      MARKER_LED_DATA_PIN, static_cast<unsigned>(MARKER_LED_COUNT));
        hal_log(buf);
    }

    if (xTaskCreatePinnedToCore(marker_render_task, "marker_led", 2048,
                                nullptr, 3, &s_render_task_handle,
                                APP_CPU_NUM) != pdPASS) {
        s_render_task_handle = nullptr;
        hal_log("[HAL_MARKER][WARN] Render task spawn failed.");
    }

    s_hw_ok = true;
    return s_hw_ok;
#else
    if (s_marker_initialized) return true;
    s_marker_initialized = true;
    hal_log("[HAL_MARKER] Visual guidance marker HAL stub initialized (safe default).");
    return true;
#endif
}

bool hal_marker_is_healthy() {
#if defined(ARDUINO)
    return s_marker_initialized && s_hw_ok;
#else
    return s_marker_initialized;
#endif
}

bool hal_marker_is_stub() {
#if defined(ARDUINO)
    return !s_rmt_ready;
#else
    return true;
#endif
}

void hal_marker_set_pattern(Types::MarkerPattern pattern) {
#if defined(ARDUINO)
    taskENTER_CRITICAL(&s_state_mux);
    s_state.pattern = pattern;
    taskEXIT_CRITICAL(&s_state_mux);
#else
    s_state.pattern = pattern;
#endif
}

void hal_marker_set_brightness(uint8_t brightness_percent) {
    if (brightness_percent > 100) brightness_percent = 100;
#if defined(ARDUINO)
    taskENTER_CRITICAL(&s_state_mux);
    s_state.brightness = brightness_percent;
    taskEXIT_CRITICAL(&s_state_mux);
#else
    s_state.brightness = brightness_percent;
#endif
}

void hal_marker_enable(bool enabled) {
#if defined(ARDUINO)
    taskENTER_CRITICAL(&s_state_mux);
    s_state.enabled = enabled;
    if (!enabled) s_state.pattern = Types::MarkerPattern::MARKER_OFF;
    taskEXIT_CRITICAL(&s_state_mux);
#else
    s_state.enabled = enabled;
    if (!enabled) s_state.pattern = Types::MarkerPattern::MARKER_OFF;
#endif
}

Types::MarkerPattern hal_marker_get_pattern() {
    return s_state.pattern;
}

void hal_marker_set_guidance_deg(float heading_error_deg) {
#if defined(ARDUINO)
    taskENTER_CRITICAL(&s_state_mux);
    s_state.guidance_deg = heading_error_deg;
    taskEXIT_CRITICAL(&s_state_mux);
#else
    s_state.guidance_deg = heading_error_deg;
#endif
}

} // namespace Hal
} // namespace RobofestDrone
