#pragma once

#include <stdint.h>
#include "../src/types.h"

namespace RobofestDrone {
namespace Hal {

// Initializes onboard human detection sensor / neural accelerator interface.
bool hal_human_init();

// Returns true if human detection hardware pipeline is healthy.
bool hal_human_is_healthy();

// Non-blocking poll for latest human detection frame from onboard camera / NPU.
// Returns true if a new detection frame is available.
bool hal_human_read_detection(Types::HumanDetectionSample& out_sample);

// Test/simulation hook to inject mock human detection samples.
void hal_human_set_mock_detection(const Types::HumanDetectionSample& sample);

} // namespace Hal
} // namespace RobofestDrone
