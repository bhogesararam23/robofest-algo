#pragma once

#include <stdint.h>
#include "../src/types.h"

namespace RobofestDrone {
namespace Hal {

// Initializes non-volatile flash/EEPROM storage for mission blackbox logging.
bool hal_storage_init();

// Non-blocking write of a telemetry event record to persistent storage.
bool hal_storage_write_event(const Types::TelemetryEvent& event);

// Flushes internal write caches to persistent media.
bool hal_storage_flush();

// Returns available free storage space in bytes.
uint32_t hal_storage_get_free_space();

// Returns true if storage subsystem is operational.
bool hal_storage_is_healthy();

} // namespace Hal
} // namespace RobofestDrone
