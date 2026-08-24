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

// ============================================================================
// GENERIC KEYED BLOB STORAGE (REQ-DER-106, item 6)
// ----------------------------------------------------------------------------
// Small named binary records (e.g. calibrated vision profiles) persisted with
// write-all-then-swap semantics so a power cut mid-write cannot corrupt the
// previous record. On target: LittleFS file per key. On host: files under
// data/ next to the working directory.
// ============================================================================

constexpr uint16_t HAL_STORAGE_BLOB_MAX = 2048;

bool hal_storage_write_blob(const char* key, const uint8_t* data, uint16_t len);

// Reads into out (capacity HAL_STORAGE_BLOB_MAX). Returns bytes read, or
// -1 when missing / too large / storage unhealthy.
int hal_storage_read_blob(const char* key, uint8_t* out, uint16_t cap);

// Deletes a keyed blob (true when deleted or already absent).
bool hal_storage_delete_blob(const char* key);

} // namespace Hal
} // namespace RobofestDrone
