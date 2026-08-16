#include "hal_radio.h"
#include "hal_system.h"

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_radio_initialized = false;
}

bool hal_radio_init() {
    // Note: Replace with real ESP-NOW esp_now_init() and peer registration on target ESP32-S3.
    // Purely local peer-to-peer radio. Zero internet / zero cloud connectivity.
    s_radio_initialized = true;
    hal_log("[HAL_RADIO] P2P swarm radio stub initialized (safe default).");
    return true;
}

bool hal_radio_send(const Types::SwarmPacket& packet) {
    if (!s_radio_initialized) {
        return false;
    }
    // Stub accepts valid length packets
    if (packet.payload_length > Types::SWARM_PAYLOAD_MAX_BYTES) {
        return false;
    }
    // Note: Replace with real esp_now_send() non-blocking transmit on hardware.
    return true;
}

bool hal_radio_receive(Types::SwarmPacket& packet) {
    (void)packet;
    // Stub: No packet queued in stub environment
    return false;
}

bool hal_radio_is_healthy() {
    return s_radio_initialized;
}

} // namespace Hal
} // namespace RobofestDrone
