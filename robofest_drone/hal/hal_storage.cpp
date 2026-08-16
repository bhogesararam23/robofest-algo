#include "hal_storage.h"
#include "hal_system.h"

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_storage_initialized = false;
}

bool hal_storage_init() {
    // Note: Replace with SPIFFS/LittleFS/SD-MMC initialization on ESP32-S3.
    s_storage_initialized = true;
    hal_log("[HAL_STORAGE] Blackbox event storage initialized (safe default).");
    return true;
}

bool hal_storage_write_event(const Types::TelemetryEvent& event) {
    (void)event;
    if (!s_storage_initialized) {
        return false;
    }
    // Stub accepts event safely
    return true;
}

bool hal_storage_flush() {
    if (!s_storage_initialized) return false;
    // Note: Replace with real flash fsync / LittleFS sync.
    return true;
}

uint32_t hal_storage_get_free_space() {
    if (!s_storage_initialized) return 0;
    // Note: Replace with real LittleFS/FATFS free bytes query. Default 1MB in stub.
    return 1048576UL;
}

bool hal_storage_is_healthy() {
    return s_storage_initialized;
}

} // namespace Hal
} // namespace RobofestDrone
