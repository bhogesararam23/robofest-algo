#pragma once

#include <stdint.h>
#include "../src/types.h"

namespace RobofestDrone {
namespace Hal {

// Initializes GPIO pins for digital inputs (kill switch), status LEDs, and optical marker drivers.
bool hal_gpio_init();

// Reads the state of the software-monitored kill switch pin.
// Note: Physical kill switch must also have hardware capability to cut power independently.
bool hal_kill_switch_active();

// Sets the optical guidance marker pattern (e.g. laser projection, directional LED array).
void hal_marker_set(Types::MarkerPattern pattern);

// Returns true if GPIO subsystem is initialized and healthy.
bool hal_gpio_is_healthy();

} // namespace Hal
} // namespace RobofestDrone
