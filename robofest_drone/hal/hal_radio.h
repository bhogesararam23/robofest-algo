#pragma once

#include <stdint.h>
#include "../src/types.h"

namespace RobofestDrone {
namespace Hal {

// Initializes the local peer-to-peer wireless transceiver (e.g., ESP-NOW or sub-GHz RF).
// Must not connect to Wi-Fi routers, cloud, internet, or base stations.
bool hal_radio_init();

// Non-blocking transmission of a swarm coordination packet over local P2P broadcast.
bool hal_radio_send(const Types::SwarmPacket& packet);

// Non-blocking reception of a swarm coordination packet from peer drones.
bool hal_radio_receive(Types::SwarmPacket& packet);

// Returns true if radio hardware is initialized and healthy.
bool hal_radio_is_healthy();

} // namespace Hal
} // namespace RobofestDrone
