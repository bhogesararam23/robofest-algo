#include "hal_gpio.h"
#include "hal_system.h"
#include "../config/mission_config.h"

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_gpio_initialized = false;
    static bool s_kill_switch_state = false;
    static Types::MarkerPattern s_current_marker_pattern = Types::MarkerPattern::MARKER_OFF;
}

bool hal_gpio_init() {
    // Note: Replace with real ESP32 gpio_config() for KILL_SWITCH_PIN and marker outputs.
    s_gpio_initialized = true;
    s_kill_switch_state = false;
    s_current_marker_pattern = Types::MarkerPattern::MARKER_OFF;
    hal_log("[HAL_GPIO] GPIO and kill switch input initialized (safe default).");
    return true;
}

bool hal_kill_switch_active() {
    // Note: Replace with real gpio_get_level(Config::KILL_SWITCH_PIN).
    // Physical kill switch also independently isolates battery/ESC power hardware rail.
    return s_kill_switch_state;
}

void hal_marker_set(Types::MarkerPattern pattern) {
    s_current_marker_pattern = pattern;
    // Note: Replace with real laser projector / GPIO LED pattern modulation.
}

bool hal_gpio_is_healthy() {
    return s_gpio_initialized;
}

} // namespace Hal
} // namespace RobofestDrone
