#pragma once

#include <stdint.h>

namespace RobofestDrone {
namespace Hal {

// Initializes high-speed UART port for communicating with the Flight Controller.
bool hal_serial_init();

// Non-blocking write of raw byte stream to Flight Controller UART.
bool hal_serial_write(const uint8_t* data, uint16_t length);

// Non-blocking read of available bytes from Flight Controller UART into buffer.
// Returns number of bytes read, or -1 on error.
int16_t hal_serial_read(uint8_t* buffer, uint16_t max_length);

// Returns number of bytes available in serial RX buffer.
uint16_t hal_serial_available();

// Returns true if serial communication port is open and operational.
bool hal_serial_is_healthy();

} // namespace Hal
} // namespace RobofestDrone
