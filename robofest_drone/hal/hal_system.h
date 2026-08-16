#pragma once

#include <stdint.h>

namespace RobofestDrone {
namespace Hal {

// Initializes system timers, clocks, and platform logging.
// Must be replaced with real ESP32-S3 time and logging drivers on target hardware.
void hal_system_init();

// Returns milliseconds elapsed since system boot.
uint32_t hal_millis();

// Returns microseconds elapsed since system boot.
uint32_t hal_micros();

// Non-blocking log message output over UART/Console.
void hal_log(const char* message);

// Non-blocking log output with float value over UART/Console.
void hal_log_value(const char* message, float value);

} // namespace Hal
} // namespace RobofestDrone
