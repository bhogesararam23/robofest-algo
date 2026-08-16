#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"
#include "../hal/hal_serial.h"

namespace RobofestDrone {

// ============================================================================
// FC PACKET TYPES
// ============================================================================

constexpr uint8_t FC_PACKET_TYPE_COMMAND                     = 1;
constexpr uint8_t FC_PACKET_TYPE_STATUS                      = 2;
constexpr uint8_t FC_PACKET_TYPE_ACK                         = 3;
constexpr uint8_t FC_PACKET_TYPE_ERROR                       = 4;


// ============================================================================
// FC BRIDGE CLASS
// ============================================================================

class FcBridge {
public:
    FcBridge();

    void init();
    void reset();

    void update(uint32_t now_ms);

    bool sendHold(uint32_t now_ms);
    bool sendLand(uint32_t now_ms);
    bool sendTakeoff(float target_altitude_m, uint32_t now_ms);
    bool sendAltitudeCommand(float target_altitude_m, uint32_t now_ms);
    bool sendHeadingCommand(float heading_deg, uint32_t now_ms);
    bool sendVelocityCommand(const Types::Vec2& velocity, float target_altitude_m, float heading_deg, uint32_t now_ms);

    bool requestArm(uint32_t now_ms);
    bool requestDisarm(uint32_t now_ms);
    bool requestEmergencyStop(uint32_t now_ms);

    bool isArmed() const { return armed_; }
    bool isLinkHealthy() const { return link_healthy_; }
    bool isCommandWatchdogActive() const { return command_watchdog_active_; }

    float getBatteryVoltage() const { return latest_status_.battery_voltage; }
    float getBatteryCurrent() const { return latest_status_.battery_current; }
    bool isBatteryCurrentValid() const { return latest_status_.battery_current_valid; }

    Types::AttitudeSample getAttitude() const { return latest_status_; }
    float getRollDeg() const { return latest_status_.roll_deg; }
    float getPitchDeg() const { return latest_status_.pitch_deg; }
    float getYawDeg() const { return latest_status_.yaw_deg; }
    float getFcAltitude() const { return latest_status_.altitude_m; }

    uint32_t getLastRxTimeMs() const { return last_rx_ms_; }
    uint32_t getLastTxTimeMs() const { return last_tx_ms_; }
    uint16_t getLastError() const { return last_error_; }

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    uint8_t calculateCrc8(const uint8_t* data, uint16_t length) const;
    bool sendPacket(uint8_t packet_type, const uint8_t* payload, uint8_t payload_length, uint32_t now_ms);
    bool sendCommandInternal(Types::FcCommand cmd, float p1, float p2, float p3, float p4, uint32_t now_ms);
    void processIncomingBytes(uint32_t now_ms);
    void parseStatusPayload(const uint8_t* payload, uint8_t length, uint32_t now_ms);
    void setTelemetryEvent(uint16_t event_id);

private:
    Types::AttitudeSample latest_status_;
    bool armed_ = false;
    bool link_healthy_ = false;
    bool command_watchdog_active_ = false;

    uint32_t last_rx_ms_ = 0;
    uint32_t last_tx_ms_ = 0;
    uint32_t last_command_ms_ = 0;
    uint32_t last_status_packet_ms_ = 0;

    uint8_t tx_retry_count_ = 0;
    uint16_t last_error_ = 0;

    uint8_t rx_buffer_[Config::FC_RX_BUFFER_SIZE] = {};
    uint16_t rx_index_ = 0;

    uint8_t tx_buffer_[Config::FC_TX_BUFFER_SIZE] = {};

    uint16_t last_telemetry_event_id_ = TE_FC_BRIDGE_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void fc_bridge_init();
void fc_bridge_update(uint32_t now_ms);
bool fc_bridge_send_velocity(const Types::Vec2& vel, float alt, float yaw_deg, uint32_t now_ms);
bool fc_bridge_is_healthy();
Types::AttitudeSample fc_bridge_get_attitude();
FcBridge& fc_bridge_get_instance();

inline bool fc_bridge_is_link_healthy() { return fc_bridge_is_healthy(); }
inline float fc_bridge_get_battery_voltage() { return fc_bridge_get_instance().getBatteryVoltage(); }
inline bool fc_bridge_send_hold() { return fc_bridge_get_instance().sendHold(0); }
inline bool fc_bridge_send_land() { return fc_bridge_get_instance().sendLand(0); }
inline bool fc_bridge_send_emergency_stop() { return fc_bridge_get_instance().requestEmergencyStop(0); }

} // namespace RobofestDrone
