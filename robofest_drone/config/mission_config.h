#pragma once

#include <stdint.h>

namespace RobofestDrone {
namespace Config {

// ============================================================================
// FIELD & ARENA DIMENSIONS
// ============================================================================

// official
constexpr float FIELD_LENGTH_M = 60.0f;

// official
constexpr float FIELD_WIDTH_M = 15.0f;

// official
constexpr float START_ZONE_LENGTH_M = 1.0f;

// official
constexpr float EXIT_ZONE_LENGTH_M = 1.0f;

// official
constexpr float MINEFIELD_LENGTH_M = 58.0f;

// official
constexpr float MINEFIELD_WIDTH_M = 15.0f;

// official
constexpr float MINE_CLEARANCE_RADIUS_M = 1.0f;

// official
constexpr uint32_t MISSION_TIME_LIMIT_MS = 600000UL;

// derived from uploaded logic
constexpr uint16_t MINE_COUNT_ESTIMATE = 40;


// ============================================================================
// ZONE Y RANGES (Longitudinal Corridor Along Y-Axis)
// ============================================================================

// official
constexpr float START_ZONE_Y_MIN = 0.0f;

// official
constexpr float START_ZONE_Y_MAX = 1.0f;

// official
constexpr float MINEFIELD_Y_MIN = 1.0f;

// official
constexpr float MINEFIELD_Y_MAX = 59.0f;

// official
constexpr float EXIT_ZONE_Y_MIN = 59.0f;

// official
constexpr float EXIT_ZONE_Y_MAX = 60.0f;


// ============================================================================
// FIELD COORDINATE LIMITS
// ============================================================================

// official
constexpr float FIELD_X_MIN = 0.0f;

// official
constexpr float FIELD_X_MAX = 15.0f;

// official
constexpr float FIELD_Y_MIN = 0.0f;

// official
constexpr float FIELD_Y_MAX = 60.0f;


// ============================================================================
// TAKEOFF OFFSET CONSTANTS
// ============================================================================

// tunable implementation default
constexpr float TAKEOFF_FIELD_X = 7.5f;

// tunable implementation default
constexpr float TAKEOFF_FIELD_Y = 0.5f;


// ============================================================================
// SOFTWARE GEOFENCE CONSTANTS
// ============================================================================

// tunable implementation default
constexpr float SOFTWARE_GEOFENCE_X_MIN = 0.5f;

// tunable implementation default
constexpr float SOFTWARE_GEOFENCE_X_MAX = 14.5f;

// tunable implementation default
constexpr float SOFTWARE_GEOFENCE_Y_MIN = 0.5f;

// tunable implementation default
constexpr float SOFTWARE_GEOFENCE_Y_MAX = 59.5f;

// tunable implementation default
constexpr float GEOFENCE_WARNING_BAND_M = 0.5f;

// tunable implementation default
constexpr float GEOFENCE_UNCERTAINTY_EXTRA_MARGIN_M = 0.5f;


// ============================================================================
// LOOP TIMING CONSTANTS
// ============================================================================

// derived from uploaded logic
constexpr uint32_t MAIN_LOOP_PERIOD_MS = 20UL;

// tunable implementation default
constexpr uint32_t VISION_PERIOD_MS = 66UL;

// tunable implementation default
constexpr uint32_t TELEMETRY_PERIOD_MS = 1000UL;

// tunable implementation default
constexpr uint32_t HEARTBEAT_PERIOD_MS = 250UL;

// tunable implementation default
constexpr uint32_t PEER_LOST_TIMEOUT_MS = 2000UL;


// ============================================================================
// DRONE IDENTITY CONSTANTS
// ============================================================================

// tunable implementation default
constexpr uint8_t DRONE_ID = 1;

// tunable implementation default
constexpr uint8_t DRONE_ROLE = 0;

// tunable implementation default
constexpr uint8_t SWARM_PACKET_VERSION = 1;


// ============================================================================
// MISSION ALTITUDE CONSTANTS
// ============================================================================

// tunable implementation default
constexpr float MISSION_ALTITUDE_M = 2.0f;

// tunable implementation default
constexpr float LANDING_ALTITUDE_STEP_M = 0.2f;


// ============================================================================
// HARDWARE COMPONENT TARGETS (ROBOFEST AERIAL FINAL SPECIFICATION)
// ============================================================================

// Companion Mission Computer: Seeed Studio XIAO ESP32-S3 Sense (Dual-Core @ 240MHz, 8MB PSRAM, 8MB Flash)
// Flight Controller: Matek H743-SLIM V3 (STM32H743VIT6 @ 480MHz, 2MB Flash, Dual IMU: MPU6000 + ICM-42605, 7x UARTs)
// ESC: Foxeer Reaper F4 65A 4-in-1 (100A burst, DShot600 / Telemetry)
// Motors: Darwin 1504 2300KV Brushless
// Propellers: Gemfan Hurricane 4024 (4.0x2.4")
// Camera: OmniVision OV5640 5MP DVP
// Ground Distance Sensor: Holybro ST VL53L1X LiDAR (up to 4.0m range)
// Front 360 Obstacle Avoidance: LDRobot LD06 2D LiDAR (12m range, 4500Hz, 230400 baud)
// Battery: 4S1P LiPo (Bonka 14.8V 2200mAh 35C) or Li-ion (Molicel INR18650 P28A 2800mAh 36.5A)

// tunable implementation default
constexpr uint8_t BATTERY_CELL_COUNT = 4;

// tunable implementation default (4.20V per cell)
constexpr float BATTERY_FULL_VOLTAGE = 16.8f;

// tunable implementation default (3.70V per cell)
constexpr float BATTERY_NOMINAL_VOLTAGE = 14.8f;

// tunable implementation default (3.60V per cell - mission return warning)
constexpr float BATTERY_LOW_VOLTAGE = 14.4f;

// tunable implementation default (3.40V per cell - mandatory landing cutoff)
constexpr float BATTERY_CRITICAL_VOLTAGE = 13.6f;

// derived from uploaded logic
constexpr uint32_t FC_SERIAL_BAUD_RATE = 921600UL;

// derived from uploaded logic
constexpr uint32_t FC_LINK_TIMEOUT_MS = 500UL;

// tunable implementation default
constexpr uint32_t CAMERA_STALL_TIMEOUT_MS = 1000UL;

// tunable implementation default
constexpr uint32_t RADIO_TIMEOUT_MS = 2000UL;

// tunable implementation default
constexpr uint8_t KILL_SWITCH_PIN = 0;

// derived from uploaded logic
constexpr float UNSAFE_PROXIMITY_DISTANCE_M = 1.0f;

// official
constexpr bool SURFACE_CONTACT_ALLOWED_ONLY_LANDING = true;

} // namespace Config
} // namespace RobofestDrone
