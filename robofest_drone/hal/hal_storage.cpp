#include "hal_storage.h"
#include "hal_system.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(ARDUINO)
#include <LittleFS.h>
#endif

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_storage_initialized = false;

#if !defined(ARDUINO)
    // Host backend: one file per key under data/.
    constexpr const char* kHostDir = "data";

    void host_path(const char* key, char* out, size_t cap) {
        std::snprintf(out, cap, "%s/%s.bin", kHostDir, key);
    }

    void host_ensure_dir() {
        // POSIX mkdir via stdio workaround: system call is acceptable in the
        // host test backend (never compiled on target).
        std::system("if not exist data mkdir data");
    }
#endif
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

// ============================================================================
// GENERIC KEYED BLOB STORAGE
// ============================================================================

bool hal_storage_write_blob(const char* key, const uint8_t* data, uint16_t len) {
    if (!s_storage_initialized || key == nullptr ||
        (data == nullptr && len > 0) || len > HAL_STORAGE_BLOB_MAX) {
        return false;
    }

#if defined(ARDUINO)
    // Target backend: LittleFS atomic replace (write temp then rename).
    char path[64];
    std::snprintf(path, sizeof(path), "/blobs/%s.bin", key);
    if (!LittleFS.exists("/blobs")) {
        LittleFS.mkdir("/blobs");
    }
    char tmp[72];
    std::snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    {
        File f = LittleFS.open(tmp, "w");
        if (!f) return false;
        const size_t written = (len > 0) ? f.write(data, len) : 0;
        f.close();
        if (written != len) {
            LittleFS.remove(tmp);
            return false;
        }
    }
    LittleFS.remove(path);
    return LittleFS.rename(tmp, path);
#else
    char path[128];
    host_path(key, path, sizeof(path));
    host_ensure_dir();
    // Write-all-then-replace via temp file keeps the previous record intact
    // if this process dies mid-write.
    char tmp[136];
    std::snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE* f = std::fopen(tmp, "wb");
    if (f == nullptr) return false;
    if (len > 0 && std::fwrite(data, 1, len, f) != len) {
        std::fclose(f);
        std::remove(tmp);
        return false;
    }
    std::fclose(f);
    std::remove(path);
    return std::rename(tmp, path) == 0;
#endif
}

int hal_storage_read_blob(const char* key, uint8_t* out, uint16_t cap) {
    if (!s_storage_initialized || key == nullptr || out == nullptr || cap == 0) {
        return -1;
    }

#if defined(ARDUINO)
    char path[64];
    std::snprintf(path, sizeof(path), "/blobs/%s.bin", key);
    if (!LittleFS.exists(path)) return -1;
    File f = LittleFS.open(path, "r");
    if (!f) return -1;
    const size_t avail = f.size();
    if (avail > cap) {
        f.close();
        return -1;
    }
    const size_t n = (avail > 0) ? f.read(out, avail) : 0;
    f.close();
    return static_cast<int>(n);
#else
    char path[128];
    host_path(key, path, sizeof(path));
    FILE* f = std::fopen(path, "rb");
    if (f == nullptr) return -1;
    // Read one extra byte to detect records larger than the capacity.
    size_t n = std::fread(out, 1, static_cast<size_t>(cap) + 1, f);
    std::fclose(f);
    if (n > static_cast<size_t>(cap)) return -1;
    return static_cast<int>(n);
#endif
}

bool hal_storage_delete_blob(const char* key) {
    if (!s_storage_initialized || key == nullptr) return false;

#if defined(ARDUINO)
    char path[64];
    std::snprintf(path, sizeof(path), "/blobs/%s.bin", key);
    if (!LittleFS.exists(path)) return true;
    return LittleFS.remove(path);
#else
    char path[128];
    host_path(key, path, sizeof(path));
    return std::remove(path) == 0;
#endif
}

} // namespace Hal
} // namespace RobofestDrone
