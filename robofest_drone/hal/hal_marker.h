#pragma once

#include <stdint.h>
#include "../src/types.h"

namespace RobofestDrone {
namespace Hal {

// Initializes visual guidance marker outputs (directional LEDs, projected light cues).
bool hal_marker_init();

// Returns true if marker output hardware driver is healthy.
bool hal_marker_is_healthy();

// Sets the active visual guidance pattern on physical marker hardware.
void hal_marker_set_pattern(Types::MarkerPattern pattern);

// Sets the brightness percentage (0 - 100%) for marker LED arrays.
void hal_marker_set_brightness(uint8_t brightness_percent);

// Enables or disables the physical marker output.
void hal_marker_enable(bool enabled);

// Returns the current pattern set on the hardware.
Types::MarkerPattern hal_marker_get_pattern();

} // namespace Hal
} // namespace RobofestDrone
