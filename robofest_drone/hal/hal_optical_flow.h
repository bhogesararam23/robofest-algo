#pragma once

#include <stdint.h>
#include "../src/types.h"

namespace RobofestDrone {
namespace Hal {

// Initializes the downward-facing optical flow sensor (e.g. PMW3901/CX-OF over SPI/UART).
// Must be replaced with real hardware bus driver initialization on target hardware.
bool hal_optical_flow_init();

// Non-blocking read of optical flow displacement and quality metric.
Types::OpticalFlowSample hal_optical_flow_read();

// Returns true if optical flow sensor is communicating and healthy.
bool hal_optical_flow_is_healthy();

} // namespace Hal
} // namespace RobofestDrone
