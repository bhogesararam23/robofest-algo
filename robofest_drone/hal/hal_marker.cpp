#include "hal_marker.h"
#include "hal_system.h"

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_marker_initialized = false;
    static bool s_marker_enabled = true;
    static uint8_t s_brightness_percent = 80;
    static Types::MarkerPattern s_active_pattern = Types::MarkerPattern::MARKER_OFF;
}

bool hal_marker_init() {
    // Note: Replace with real ESP32 RMT/PWM LED driver or laser projector GPIO routing.
    s_marker_initialized = true;
    s_marker_enabled = true;
    s_brightness_percent = 80;
    s_active_pattern = Types::MarkerPattern::MARKER_OFF;
    hal_log("[HAL_MARKER] Visual guidance marker HAL stub initialized (safe default).");
    return true;
}

bool hal_marker_is_healthy() {
    return s_marker_initialized;
}

void hal_marker_set_pattern(Types::MarkerPattern pattern) {
    s_active_pattern = pattern;
}

void hal_marker_set_brightness(uint8_t brightness_percent) {
    s_brightness_percent = (brightness_percent > 100) ? 100 : brightness_percent;
}

void hal_marker_enable(bool enabled) {
    s_marker_enabled = enabled;
    if (!enabled) {
        s_active_pattern = Types::MarkerPattern::MARKER_OFF;
    }
}

Types::MarkerPattern hal_marker_get_pattern() {
    return s_active_pattern;
}

} // namespace Hal
} // namespace RobofestDrone
