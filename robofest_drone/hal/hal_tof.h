#pragma once

#include <stdint.h>
#include "../src/types.h"

namespace RobofestDrone {
namespace Hal {

// Initializes the downward-facing Time-of-Flight (ToF) rangefinder (e.g. VL53L1X/TFmini over I2C/UART).
// Must be replaced with real bus initialization on target hardware.
bool hal_tof_init();

// Non-blocking read of ground distance / altitude sample.
Types::TofSample hal_tof_read();

// Returns true if ToF sensor is online and returning valid ranges.
bool hal_tof_is_healthy();

} // namespace Hal
} // namespace RobofestDrone
