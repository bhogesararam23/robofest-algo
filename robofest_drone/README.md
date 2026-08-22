# Robofest Gujarat 6.0 Minefield Swarm Drone — Onboard Flight Software

---

## 1. Executive Summary & Mission Architecture

This repository contains the complete, production-quality, deterministic onboard software stack for an autonomous minefield search and rescue swarm drone competing in **Robofest Gujarat 6.0**.

### Key Architectural Tenets:
- **100% Drone-Only Onboard Processing**: All localization, computer vision, mine mapping, A* safe path planning, swarm mesh coordination, human tracking, and safety supervisory logic execute entirely onboard the Seeed Studio XIAO ESP32-S3 Sense mission computer.
- **Zero GPS / GNSS / External Anchors**: Navigation relies strictly on downward optical flow dead-reckoning, Time-of-Flight lidar altitude rangefinding, and flight controller attitude telemetry.
- **Zero Cloud / Base Station / Remote Control**: No offboard computers, base stations, ground control stations, internet connections, or manual pilot controls are used.
- **Separation of Concerns**:
  - The **Seeed Studio XIAO ESP32-S3 Sense** executes mission orchestration, visual perception, multi-agent arbitration, path planning, and safety gating.
  - The **Matek H743-SLIM V3 Flight Controller (STM32H743 @ 480 MHz)** features dual IMU redundancy (MPU6000 + ICM-42605) for vibration filtering, high-rate attitude stabilization, and motor mixing.
  - The **Foxeer Reaper F4 65A 4-in-1 ESC (100A burst)** provides thermal efficiency and DShot telemetry.
  - The mission computer communicates with the flight controller via a 921600 baud UART bridge sending high-level setpoints (`HOLD`, `TAKEOFF`, `LAND`, `VELOCITY`, `EMERGENCY_STOP`).
- **Deterministic & Non-Blocking**: Zero dynamic memory allocation (`malloc`, `new`, STL containers) inside control loops; zero blocking delays (`delay()`, busy waits). All tasks run cooperatively under a 50 Hz (20 ms) scheduler.

---

## 2. Final Hardware Component Specifications & Bill of Materials (BOM)

| Component | Selected Hardware Model | Key Technical Specifications | Weight | Role in Architecture |
| :--- | :--- | :--- | :---: | :--- |
| **Mission Computer** | Seeed Studio XIAO ESP32-S3 Sense | Dual-Core Xtensa LX7 @ 240MHz, 8MB PSRAM, 8MB Flash | ~15 g | Perception, mine map fusion, A* path planning, swarm mesh, safety manager |
| **Flight Controller** | Matek H743-SLIM V3 (2-8S) | STM32H743 @ 480MHz, 2MB Flash, **Dual IMU (MPU6000 + ICM-42605)**, 7x UARTs | 7 g | Inner-loop attitude stabilization, dual IMU vibration filtering, motor mixing |
| **ESC** | Foxeer Reaper F4 65A 4-in-1 | 65A Continuous / 100A Burst, BLHeli_32 / AM32, DShot600, current telemetry | 14.2 g | High-efficiency motor drive, low thermal resistance |
| **BLDC Motors** | Darwin 1504 2300KV | Micro brushless motor set for 4S efficiency | 47 g | High thrust-to-weight ratio for agile 4-inch frame |
| **Propellers** | Gemfan Hurricane 4024 | 4.0" diameter, 2.4" pitch 3-blade | 6.5 g | Low vibration, optimized for 2300KV on 4S |
| **Camera** | OmniVision OV5640 5MP | DVP parallel interface directly to ESP32-S3 DMA | ~5 g | Downward mine and surface marker HSV color blob segmentation |
| **Ground Distance** | Holybro ST VL53L1X LiDAR | $0.04\text{ m}$ to $4.0\text{ m}$ range, $50\text{ Hz}$ I2C | 8 g | Precision ground clearance and AGL altitude hold |
| **Obstacle Avoidance**| LDRobot LD06 2D LiDAR | $0.02\text{ m}$ to $12.0\text{ m}$ 360° range, $4500\text{ Hz}$ sampling, 230400 baud UART | 50 g | Forward/lateral obstacle detection (poles, trees, boundary hazards) |
| **Battery** | Bonka 14.8V 2200mAh 35C 4S LiPo / Molicel P28A 4S1P Li-ion (2800mAh) | 4S1P 14.8V nominal (16.8V max, 14.4V low warning, 13.6V critical cutoff) | 210–220 g | 10+ minute hover mission endurance |
| **Airframe** | 4-inch Carbon Fiber Frame | Lightweight rigid unibody | 120–150 g | Total AUW: ~420 g |

---

## 3. Official Challenge Compliance Matrix

| Rule / Mandate | Official Requirement | Software Implementation | Compliance |
| :--- | :--- | :--- | :---: |
| **Fleet Composition** | $\ge 3$ autonomous drones (ideal 3–4) | Distributed P2P radio mesh with dynamic role failover | **PASS** |
| **Mission Duration** | $10\text{ minutes}$ ($600,000\text{ ms}$) max limit | Hardware timer watchdog forcing autonomous landing at $600\text{ s}$ | **PASS** |
| **Arena Dimensions** | $15\text{ m} \times 60\text{ m}$ field (Start, Minefield, Exit) | Software geofence ($0.5\text{ m}$ margin) with continuous velocity pushback | **PASS** |
| **Mine Clearance** | $\ge 1.0\text{ m}$ radial clearance from mine center | Exact continuous segment & waypoint obstacle clearance validation | **PASS** |
| **Positioning** | No GPS / No external positioning | Optical flow + ToF sensor fusion dead-reckoning with drift tracking | **PASS** |
| **Infrastructure** | No cloud / No internet / No base stations | P2P local broadcast radio packets with claim/yield deduplication | **PASS** |
| **Safety / Cutoff** | Emergency disarm / autonomous touchdown | Dedicated `SafetyManager` monitoring 22 faults + physical kill switch | **PASS** |

---

## 4. Directory Structure

```text
robofest_drone/
├── README.md                           # Master system overview and hardware calibration
├── platformio.ini                      # Seeed Studio XIAO ESP32S3 PlatformIO project config
├── docs/
│   ├── requirements.md                 # Complete official/derived requirements specification
│   ├── build_and_flash.md              # Firmware build and flashing instructions
│   └── safety_checklist.md             # Pre-flight and safety operational procedures
├── config/
│   ├── mission_config.h                # Arena geometry, 4S battery thresholds, and HW definitions
│   └── thresholds.h                    # Algorithmic gains, filter cutoffs, and safety limits
├── hal/                                # Hardware Abstraction Layer (Calibrated for Target HW)
│   ├── hal_system.h / .cpp             # Monotonic millisecond timers and non-blocking logging
│   ├── hal_camera.h / .cpp             # OV5640 DVP DMA camera interface
│   ├── hal_optical_flow.h / .cpp       # PMW3901 downward optical flow SPI interface
│   ├── hal_tof.h / .cpp                # Holybro ST VL53L1X ground distance sensor interface
│   ├── hal_lidar.h / .cpp              # LDRobot LD06 2D 360-degree LiDAR obstacle interface
│   ├── hal_radio.h / .cpp              # ESP-NOW / LoRa P2P packet transceiver
│   ├── hal_gpio.h / .cpp               # Hardware RC kill switch interrupt pin
│   ├── hal_serial.h / .cpp             # Matek H743 UART bridge @ 921600 baud
│   ├── hal_storage.h / .cpp            # Non-volatile flash LittleFS blackbox logger
│   ├── hal_command.h / .cpp            # Gesture & voice command sensor interface
│   ├── hal_human.h / .cpp              # Thermal / visual person detection sample interface
│   └── hal_marker.h / .cpp             # WS2812B guidance LED / marker optical output
├── src/                                # Core Algorithmic Subsystems
│   ├── main.cpp                        # Bootstrap, static context allocation, and 50 Hz loop
│   ├── calibration/                    # Onboard calibration (HSV tuner, drift tester)
│   ├── self_test/                      # Pre-flight hardware and software self-check
│   ├── scheduler.h / .cpp              # Non-blocking cooperative task dispatcher
│   ├── types.h                         # Fixed-size geometry, sample, and packet types
│   ├── system_state.h                  # Unified system-wide runtime context structure
│   ├── telemetry_events.h              # Master registry for all module event IDs (1000..3399)
│   ├── localization.h / .cpp           # Optical flow dead-reckoning + drift uncertainty filter
│   ├── geofence.h / .cpp               # Virtual boundary enforcement + continuous repulsion
│   ├── vision_pipeline.h / .cpp        # HSV segmentation, circularity filter, ray projection
│   ├── mine_map.h / .cpp               # Spatial deduplication, confidence fusion, stale decay
│   ├── path_planner.h / .cpp           # 1.0 m clearance A* corridor search & dynamic replanner
│   ├── swarm_comm.h / .cpp             # Distributed P2P heartbeats, role failover, claim/yield
│   ├── command_layer.h / .cpp          # Gesture/voice debouncing, confidence gating, lockout
│   ├── human_tracker.h / .cpp          # Person tracking, corridor deviation, exit confirmation
│   ├── marker_controller.h / .cpp      # Visual guidance pattern controller (PWM / blink)
│   ├── search_behavior.h / .cpp        # Autonomous lane holding, lawnmower sweeps, scan biasing
│   ├── safety_manager.h / .cpp         # Multi-fault watchdog, battery protection, safe action
│   ├── fc_bridge.h / .cpp              # High-level command serialization & UART framing
│   ├── telemetry.h / .cpp              # Circular RAM blackbox buffer & flash event flusher
│   └── state_machine.h / .cpp          # 13-state deterministic mission flow controller
├── tests/                              # Offline Host Unit Test Scaffolding
│   ├── test_main.cpp                   # Lightweight test runner
│   ├── test_geofence.cpp               # Geofence margin & drift shrinkage tests
│   ├── test_path_clearance.cpp         # Exact 1.0m mine clearance validation tests
│   ├── test_mine_dedup.cpp             # Mine deduplication & stale decay tests
│   ├── test_localization_math.cpp      # Optical flow velocity & yaw rotation tests
│   └── test_command_debounce.cpp       # Gesture debounce & confidence gating tests
└── sim/                                # SITL Simulation & Score Evaluation Toolset
    ├── sitl_harness.py                 # Multi-agent simulation runner
    ├── score_eval.py                   # Robofest score report generator
    ├── map_generator.py                # Synthetic arena & minefield generator
    ├── sensor_sim.py                   # Sensor noise & drift modeling
    ├── drone_model.py                  # Simulated drone autonomy model
    ├── swarm_model.py                  # Swarm mesh & role failover simulation
    └── scenarios.py                    # Preset test environments
```

---

## 5. Build, Test, and Flashing Guide

### 5.1 Run Offline Unit Tests (Desktop GCC / MinGW)
```bash
cd robofest_drone
g++ -std=c++17 -Wall -Wextra -I./src -I./hal -I./config tests/*.cpp src/vision_pipeline.cpp src/geofence.cpp src/path_planner.cpp src/mine_map.cpp src/command_layer.cpp src/telemetry.cpp src/calibration/hsv_tuner.cpp hal/*.cpp -o robofest_unit_tests.exe
./robofest_unit_tests.exe
```

### 5.2 Build and Flash to Seeed Studio XIAO ESP32-S3 Sense
```bash
# Build firmware binary
pio run -e seeed_xiao_esp32s3

# Flash to connected XIAO ESP32-S3 over USB-C
pio run -e seeed_xiao_esp32s3 --target upload

# Open serial diagnostic monitor @ 115200 baud
pio device monitor --baud 115200
```

### 5.3 Run SITL Swarm Simulation
```bash
python sim/sitl_harness.py --seed 1 --scenario nominal_map --drones 3 --mines 40 --duration 600 --output run_nominal.json --report run_nominal.md
```

---

## 6. Pre-Flight Safety Directives

> [!CAUTION]
> **SAFETY FIRST**: Always remove all propellers before powering up with USB connected or testing motor communication links. Never attempt flight without verifying the physical hardware kill switch functionality on `KILL_SWITCH_PIN` (GPIO 0).

