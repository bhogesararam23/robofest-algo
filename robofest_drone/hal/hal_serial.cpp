#include "hal_serial.h"
#include "hal_system.h"
#include "../config/mission_config.h"

namespace RobofestDrone {
namespace Hal {

// ============================================================================
// MATEK H743-SLIM V3 FLIGHT CONTROLLER SERIAL UART DRIVER HAL
// ============================================================================
// Target Hardware: Matek H743-SLIM V3 (STM32H743VIT6 @ 480 MHz, Dual IMU)
// - Dual IMU: MPU6000 & ICM-42605 hardware sensor fusion
// - UART Protocol: Custom high-speed CRC-16 framed binary / MSP / MAVLink bridge
// - Baud Rate: 921600 baud (Config::FC_SERIAL_BAUD_RATE)
// - Pinout on Seeed XIAO ESP32-S3: TX (GPIO 43 / D6), RX (GPIO 44 / D7) connected to FC UART1/UART2

namespace {
    static bool s_serial_initialized = false;
}

bool hal_serial_init() {
    // Note: Bind to Serial1.begin(Config::FC_SERIAL_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN)
    s_serial_initialized = true;
    hal_log("[HAL_SERIAL] Matek H743 Slim V3 serial UART initialized (921600 baud high-speed mode).");
    return true;
}

bool hal_serial_write(const uint8_t* data, uint16_t length) {
    if (!s_serial_initialized || data == nullptr || length == 0) {
        return false;
    }
    // Note: Replace with Serial1.write(data, length) on hardware
    return true;
}

int16_t hal_serial_read(uint8_t* buffer, uint16_t max_length) {
    if (!s_serial_initialized || buffer == nullptr || max_length == 0) {
        return -1;
    }
    // Note: Replace with non-blocking Serial1.readBytes(buffer, max_length) on hardware
    return 0; // 0 bytes available in test stub
}

uint16_t hal_serial_available() {
    if (!s_serial_initialized) return 0;
    // Note: Replace with Serial1.available()
    return 0;
}

bool hal_serial_is_healthy() {
    return s_serial_initialized;
}

} // namespace Hal
} // namespace RobofestDrone
