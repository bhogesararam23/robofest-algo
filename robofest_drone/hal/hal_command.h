#pragma once

#include <stdint.h>
#include "../src/types.h"

namespace RobofestDrone {
namespace Hal {

// Initializes onboard gesture recognition and optional voice keyword front-end hardware.
// Zero cloud / zero internet / zero outside-computer dependency.
bool hal_command_init();

// Returns true if command front-end hardware is initialized and healthy.
bool hal_command_is_healthy();

// Non-blocking read of the latest hand gesture recognition sample.
// Returns true if a valid new sample was retrieved from the onboard gesture pipeline.
bool hal_command_read_gesture(Types::CommandSample& sample);

// Non-blocking read of the latest voice keyword recognition sample.
// Returns true if a valid new sample was retrieved from the onboard voice processor.
bool hal_command_read_voice(Types::CommandSample& sample);

// Enables or disables gesture recognition capture.
void hal_command_enable_gesture(bool enabled);

// Enables or disables voice recognition capture.
void hal_command_enable_voice(bool enabled);

// Test/Mock sample injection functions
void hal_command_set_mock_gesture(const Types::CommandSample& sample);
void hal_command_clear_mock();

} // namespace Hal
} // namespace RobofestDrone
