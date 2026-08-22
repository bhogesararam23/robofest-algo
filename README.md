# Robofest Gujarat 6.0 - Autonomous Minefield Swarm Drone Flight Software

## Project Overview

This repository contains the complete onboard flight software and simulation suite for an autonomous swarm of search and rescue drones competing in Robofest Gujarat 6.0.

The mission is to detect surface-laid mines and buried mine markers across a 15-meter by 60-meter field, calculate a safe walking path with at least 1.0-meter radial clearance from all mines, and guide a person-at-risk safely from the Start Zone to the Exit Zone within a 10-minute time limit.

## Key System Rules and Architecture

1. 100% Onboard Computing: All sensor reading, localization, computer vision, mine mapping, A* safe path planning, swarm communication, and safety management run entirely onboard each drone.
2. No GPS or External Positioning: Drones navigate without GPS, GNSS, motion-capture systems, or external beacons. Dead-reckoning uses downward optical flow, Time-of-Flight lidar altitude rangefinding, and flight controller attitude telemetry.
3. No Cloud or Remote Control: There are no ground control stations, cloud servers, internet connections, or manual remote pilot controls during autonomous flight.
4. Separation of Tasks:
   - Mission Computer (Seeed Studio XIAO ESP32-S3 Sense): Handles high-level mission logic, vision processing, mine mapping, path planning, swarm mesh communication, and safety supervision.
   - Flight Controller (Matek H743-SLIM V3): Handles low-level attitude stabilization, motor control, and sensor fusion using dual IMUs (MPU6000 and ICM-42605).
   - The mission computer sends high-level commands (HOLD, TAKEOFF, LAND, VELOCITY, EMERGENCY_STOP) to the flight controller over a 921600 baud serial UART connection.
5. Deterministic Real-Time Control: The software uses zero dynamic memory allocation inside control loops and no blocking delay calls. All tasks run cooperatively under a 50 Hz (20 ms) scheduler.

## Hardware Components and Specifications

| Component | Model | Specifications | Role |
| :--- | :--- | :--- | :--- |
| Companion Computer | Seeed Studio XIAO ESP32-S3 Sense | Dual-Core 240MHz, 8MB PSRAM, 8MB Flash | Vision, mapping, path planning, swarm mesh |
| Flight Controller | Matek H743-SLIM V3 | STM32H743 @ 480MHz, Dual IMU (MPU6000 + ICM-42605) | Attitude stabilization, motor mixing |
| ESC | Foxeer Reaper F4 65A 4-in-1 | 65A continuous, 100A burst, DShot600 | Brushless motor power control |
| BLDC Motors | T motor P1604 2850kv | Micro brushless motors | Propulsion on 4S power |
| Propellers | Gemfan Hurricane 4024 | 4.0 inch diameter, 2.4 inch pitch | Low vibration aerodynamic thrust |
| Camera | OmniVision OV5640 5MP | DVP parallel interface to ESP32-S3 | Downward mine and marker detection |
| Ground Distance | Holybro ST VL53L1X LiDAR | 0.04m to 4.0m range, 50Hz I2C | Altitude measurement and ground clearance |
| Obstacle Avoidance | LDRobot LD06 2D LiDAR | 0.02m to 12.0m 360-degree range, 230400 baud | Front and perimeter obstacle detection |
| Battery | 4S1P 14.8V (2200mAh to 2800mAh) | LiPo / Li-ion (16.8V max, 13.6V cutoff) | 10+ minutes autonomous flight power |
| Airframe | 4-inch Carbon Fiber Frame | Lightweight rigid structure | Total weight: ~420 grams |

## Repository Structure

```text
robofest_drone/
├── README.md               # Detailed firmware and hardware guide
├── platformio.ini          # PlatformIO build configuration for ESP32-S3
├── config/
│   ├── mission_config.h    # Arena dimensions, zones, and hardware parameters
│   └── thresholds.h        # Filter thresholds, safety limits, and gains
├── hal/                    # Hardware Abstraction Layer drivers
│   ├── hal_camera.h/.cpp   # OV5640 camera interface
│   ├── hal_optical_flow.h/.cpp # PMW3901 optical flow sensor
│   ├── hal_tof.h/.cpp      # Holybro ST VL53L1X rangefinder
│   ├── hal_lidar.h/.cpp    # LDRobot LD06 2D LiDAR scanner
│   ├── hal_serial.h/.cpp   # Serial link to Matek H743 FC
│   ├── hal_radio.h/.cpp    # Peer-to-peer swarm mesh radio
│   ├── hal_gpio.h/.cpp     # Hardware kill switch interface
│   ├── hal_storage.h/.cpp  # Flash blackbox event storage
│   ├── hal_command.h/.cpp  # Hand gesture and voice command interface
│   ├── hal_human.h/.cpp    # Person detection sensor interface
│   └── hal_marker.h/.cpp   # Visual guidance marker output
├── src/                    # Core onboard autonomy algorithms
│   ├── main.cpp            # Main entry point and 50Hz scheduler
│   ├── state_machine.h/.cpp # 13-state deterministic mission state machine
│   ├── localization.h/.cpp # Dead-reckoning and drift estimation
│   ├── geofence.h/.cpp     # Virtual arena boundary enforcement
│   ├── vision_pipeline.h/.cpp # HSV color segmentation and candidate filter
│   ├── mine_map.h/.cpp     # Deduplication, confidence fusion, and map decay
│   ├── path_planner.h/.cpp # 1.0m mine clearance A* corridor search
│   ├── swarm_comm.h/.cpp   # Heartbeat, role failover, and claim/yield logic
│   ├── command_layer.h/.cpp # Debouncing, confidence gating, and lockout
│   ├── human_tracker.h/.cpp # Person tracking and corridor deviation
│   ├── search_behavior.h/.cpp # Lawnmower lane search and peer separation
│   ├── safety_manager.h/.cpp # Watchdog, battery protection, and failsafes
│   ├── fc_bridge.h/.cpp    # High-level command serialization
│   ├── telemetry.h/.cpp    # RAM circular buffer and flash flusher
│   ├── calibration/        # Onboard HSV tuner and drift measurement
│   └── self_test/          # Pre-flight bench diagnostic check
├── tests/                  # Offline C++ unit test suite (17/17 passing)
│   ├── test_main.cpp       # Unit test runner
│   ├── test_geofence.cpp   # Geofence boundary and drift tests
│   ├── test_path_clearance.cpp # 1.0m clearance geometry tests
│   ├── test_mine_dedup.cpp # Mine deduplication tests
│   ├── test_localization_math.cpp # Optical flow velocity tests
│   └── test_command_debounce.cpp # Gesture debouncing tests
├── sim/                    # SITL simulation and scoring evaluation toolset
│   ├── sitl_harness.py     # Closed-loop multi-agent simulator
│   ├── score_eval.py       # Score calculator and report generator
│   ├── map_generator.py    # Synthetic arena and minefield generator
│   ├── sensor_sim.py       # Sensor noise and drift modeling
│   ├── drone_model.py      # Simulated drone flight model
│   ├── swarm_model.py      # Swarm coordination simulation
│   └── scenarios.py        # Preset test scenarios
└── docs/                   # Documentation and checklists
    ├── requirements.md     # Official and derived requirements
    ├── build_and_flash.md  # Build and flashing instructions
    └── safety_checklist.md # Pre-flight safety procedures
```

## How to Build and Run

### 1. Run Offline Unit Tests (C++17)

Run the unit tests on a host computer (Windows, Linux, or macOS) using standard GCC or Clang:

```bash
cd robofest_drone
g++ -std=c++17 -Wall -Wextra -I./src -I./hal -I./config tests/*.cpp src/vision_pipeline.cpp src/geofence.cpp src/path_planner.cpp src/mine_map.cpp src/command_layer.cpp src/telemetry.cpp src/calibration/hsv_tuner.cpp hal/*.cpp -o robofest_unit_tests.exe
./robofest_unit_tests.exe
```

### 2. Build and Flash to Seeed Studio XIAO ESP32-S3 Sense

Open the project in PlatformIO and build or flash:

```bash
cd robofest_drone

# Build firmware binary
pio run -e seeed_xiao_esp32s3

# Flash over USB-C
pio run -e seeed_xiao_esp32s3 --target upload

# Open serial diagnostic monitor at 115200 baud
pio device monitor --baud 115200
```

### 3. Run the SITL Swarm Simulation (Python 3)

Run the Software-In-The-Loop simulation to test swarm behaviors and scoring:

```bash
cd robofest_drone

# Run nominal simulation scenario
python sim/sitl_harness.py --seed 1 --scenario nominal_map --drones 3 --mines 40 --duration 600 --output run_nominal.json --report run_nominal.md

# Run high-density stress test
python sim/sitl_harness.py --seed 42 --scenario dense_map --drones 4 --mines 60 --duration 600 --output run_dense.json --report run_dense.md

# Evaluate score from a run file
python sim/score_eval.py run_nominal.json --report run_nominal_score.md
```
