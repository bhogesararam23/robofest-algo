#pragma once

#include <stdint.h>
#include "../config/vision_profiles.h"

namespace RobofestDrone {

// ============================================================================
// VISION PROFILE STORE (REQ-DER-106, item 6)
// ----------------------------------------------------------------------------
// Persists operator-calibrated marker profiles to flash so field calibration
// survives power cycles, with strict integrity checking:
//
//   record layout (little-endian):
//     [0..3]  magic "RVP1"
//     [4..5]  payload length (bytes)
//     [6..7]  reserved (0)
//     [8..11] CRC-32 over payload
//     [12..]  payload: VisionMarkerProfile table serialization
//
// Load result codes distinguish MISSING (no record) from CORRUPT (bad
// magic/size/CRC) - both fall back to built-in defaults; only OK mutates
// the live pipeline. This removes any external-toolchain dependency for
// basic operation.
// ============================================================================

class VisionPipeline;

enum class ProfileStoreStatus : uint8_t {
    STORE_OK = 0,
    STORE_MISSING,
    STORE_CORRUPT,
    STORE_IO_ERROR
};

// CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320). Pure function.
uint32_t profile_store_crc32(const uint8_t* data, uint32_t len);

// Serializes the live profile table into out (capacity in/out via cap).
// Returns required byte count; if > cap nothing is written.
uint16_t profile_store_serialize(
    const VisionPipeline& pipeline,
    uint8_t* out,
    uint16_t cap);

// Applies a serialized record to the live pipeline. Returns false when the
// payload is structurally invalid (bad type ids / count). No partial applies.
bool profile_store_deserialize(
    VisionPipeline& pipeline,
    const uint8_t* data,
    uint16_t len);

// Convenience wrappers around hal_storage blobs + integrity envelope.
ProfileStoreStatus profile_store_save(VisionPipeline& pipeline);
ProfileStoreStatus profile_store_load(VisionPipeline& pipeline);
void profile_store_erase();

} // namespace RobofestDrone
