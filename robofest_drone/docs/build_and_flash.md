# Build and Flash Guide: Robofest Gujarat 6.0 Minefield Swarm Drone

---

## 1. Project Overview & Hardware Requirements

The **Robofest Gujarat 6.0 Minefield Swarm Drone** onboard software is built for an **ESP32-S3** mission computer interfaced to a low-level Flight Controller (e.g., Betaflight, INAV, ArduPilot, or PX4 running custom high-level bridge).

### Target Specifications:
- **MCU**: ESP32-S3 Dual-Core Xtensa LX7 @ 240 MHz.
- **Memory**: Minimum 512 KB internal SRAM + 8 MB Octal PSRAM.
- **Flash**: 16 MB SPI Flash.
- **Sensors Required**:
  - Optical Flow: PMW3901 or CX-OF (SPI / UART) for dead-reckoning displacement.
  - Downward Distance: VL53L1X Time-of-Flight or TFmini Plus lidar (I2C / UART).
  - Downward Vision Camera: OV2640 / OV5640 (DVP DMA frame interface).
  - Swarm Mesh Radio: ESP-NOW (2.4 GHz) or SX1262 LoRa packet transceiver.
  - Human Detection: Thermal Grid Sensor (AMG8833) or onboard camera bounding box processor.
  - Guidance Marker: High-visibility addressable WS2812B LED array or high-power strobes.
  - Flight Controller Link: Hardware UART (115200 or 921600 baud, 8N1).
  - Emergency Kill Switch: Hardware digital input pin (active LOW / open circuit fail-safe).

---

## 2. Opening and Building the Project

### Option A: Using PlatformIO (VS Code / CLI) — Recommended

1. **Install PlatformIO**:
   Install the PlatformIO extension in VS Code or install the CLI (`pip install platformio`).
2. **Open Project**:
   Open the `robofest_drone` directory in VS Code / IDE.
3. **Build Firmware**:
   Run the build task in PlatformIO toolbar or execute:
   ```bash
   pio run
   ```
4. **Compile Output**:
   The output binary `.bin` and `.elf` files are generated in `.pio/build/esp32s3/`.

### Option B: Using Desktop Toolchain (MinGW / GCC) for Verification

The codebase is written in standard embedded C++17 with clean zero-external-dependency abstractions:
```bash
cd robofest_drone
g++ -std=c++17 -Wall -Wextra -I./src -I./hal -I./config src/*.cpp hal/*.cpp -o robofest_drone_test.exe
./robofest_drone_test.exe
```

---

## 3. Flashing to ESP32-S3

1. **Connect Hardware**:
   Connect the ESP32-S3 mission computer to your PC via USB-C (USB-UART port or native USB).
2. **Select Serial Port**:
   PlatformIO automatically detects the COM port. If using CLI:
   ```bash
   pio run --target upload --upload-port COM_PORT
   ```
3. **Bootloader Recovery (if needed)**:
   Hold down `BOOT` (GPIO0), press and release `RST` (EN), then release `BOOT` to put the ESP32-S3 into ROM bootloader mode before flashing.

---

## 4. Opening the Serial Monitor & Boot Verification

1. **Open Monitor**:
   ```bash
   pio device monitor --baud 115200
   ```
2. **Expected Boot Output Sequence**:
   ```text
   [HAL_SYSTEM] System timers and non-blocking logger initialized.
   [BOOT] Starting Robofest Minefield Swarm Drone Onboard System...
   [HAL_GPIO] GPIO and kill switch input initialized (safe default).
   [HAL_STORAGE] Blackbox event storage initialized (safe default).
   [HAL_SERIAL] Flight controller serial UART initialized (safe default).
   [HAL_CAMERA] Camera stub initialized (safe default).
   [HAL_OPTICAL_FLOW] Optical flow stub initialized (safe default).
   [HAL_TOF] Time-of-Flight rangefinder stub initialized (safe default).
   [HAL_RADIO] P2P swarm radio stub initialized (safe default).
   [HAL_HUMAN] Onboard human detection HAL stub initialized (safe default).
   [HAL_MARKER] Visual guidance marker HAL stub initialized (safe default).
   [BOOT] System initialization complete. 20ms Main scheduler ready.
   ```

---

## 5. Startup Telemetry Events Reference

| Event ID | Constant | Meaning | Expected Action |
| :--- | :--- | :--- | :--- |
| `1000` | `TE_INIT_COMPLETE` | All 15 modules initialized | System enters `INIT` state |
| `1002` | `TE_CALIBRATION_PASSED` | Self-check passed | Transitions to `CALIBRATE` |
| `1001` | `TE_STATE_MACHINE_CALIBRATE` | Calibration started | Zeros origin, clears map |
| `2012` | `TE_CALIBRATION_COMPLETE` | Sensor settle complete | Transitions to `WAIT_FOR_START` |
| `1002` | `TE_STATE_MACHINE_WAIT_FOR_START` | Drone ready on ground | Awaits gesture/voice `START` |

---

## 6. Troubleshooting Self-Check Failures

If `TE_CALIBRATION_FAILED` is logged or `isSelfCheckPassed()` returns `false`:

1. **Hardware Kill Switch Tripped**:
   - Check digital input on `KILL_SWITCH_PIN` (GPIO 0). Ensure physical pull-up is intact.
2. **Flight Controller UART Link Lost**:
   - Verify TX/RX crossover on serial bridge. Ensure FC is powered and telemetry stream is active.
3. **Optical Flow / Rangefinder Disconnected**:
   - Check SPI clock line and I2C pull-up resistors (4.7kΩ).
4. **Flash Storage Mount Error**:
   - Verify LittleFS partition size in partition table. Storage must have at least 64 KB free space.
