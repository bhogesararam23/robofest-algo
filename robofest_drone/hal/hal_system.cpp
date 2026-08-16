#include "hal_system.h"
#include <cstdio>
#include <chrono>

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_system_initialized = false;
    static auto s_start_time = std::chrono::steady_clock::now();
}

void hal_system_init() {
    // Note: Replace with real ESP32-S3 esp_timer / FreeRTOS initialization on target hardware.
    s_start_time = std::chrono::steady_clock::now();
    s_system_initialized = true;
    hal_log("[HAL_SYSTEM] System timers and non-blocking logger initialized.");
}

uint32_t hal_millis() {
    // Note: Replace with real ESP32-S3 millis() / (esp_timer_get_time() / 1000ULL) on target hardware.
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - s_start_time).count();
    return static_cast<uint32_t>(elapsed_ms);
}

uint32_t hal_micros() {
    // Note: Replace with real ESP32-S3 micros() / esp_timer_get_time() on target hardware.
    auto now = std::chrono::steady_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - s_start_time).count();
    return static_cast<uint32_t>(elapsed_us);
}

void hal_log(const char* message) {
    if (message != nullptr) {
        std::printf("%s\n", message);
    }
}

void hal_log_value(const char* message, float value) {
    if (message != nullptr) {
        std::printf("%s: %.4f\n", message, static_cast<double>(value));
    }
}

} // namespace Hal
} // namespace RobofestDrone
