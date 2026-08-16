#include "fc_bridge.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static FcBridge s_global_fc_bridge;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

FcBridge::FcBridge() {
    reset();
}

void FcBridge::init() {
    reset();
    Hal::hal_serial_init();
}

void FcBridge::reset() {
    latest_status_ = Types::AttitudeSample();
    armed_ = false;
    link_healthy_ = false;
    command_watchdog_active_ = false;

    last_rx_ms_ = 0;
    last_tx_ms_ = 0;
    last_command_ms_ = 0;
    last_status_packet_ms_ = 0;

    tx_retry_count_ = 0;
    last_error_ = 0;

    rx_index_ = 0;
    std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
    std::memset(tx_buffer_, 0, sizeof(tx_buffer_));

    last_telemetry_event_id_ = TE_FC_BRIDGE_INITIALIZED;
    telemetry_event_valid_ = true;
}

void FcBridge::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}


// ============================================================================
// CRC8 IMPLEMENTATION (CRC-8-CCITT: POLY 0x07)
// ============================================================================

uint8_t FcBridge::calculateCrc8(const uint8_t* data, uint16_t length) const {
    if (data == nullptr || length == 0) return 0x00;
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


// ============================================================================
// PACKET TRANSMISSION
// ============================================================================

bool FcBridge::sendPacket(uint8_t packet_type, const uint8_t* payload, uint8_t payload_length, uint32_t now_ms) {
    if (payload_length > (Config::FC_TX_BUFFER_SIZE - 5)) {
        return false;
    }

    tx_buffer_[0] = Config::FC_PACKET_START_BYTE;
    tx_buffer_[1] = Config::FC_PROTOCOL_VERSION;
    tx_buffer_[2] = packet_type;
    tx_buffer_[3] = payload_length;

    if (payload != nullptr && payload_length > 0) {
        std::memcpy(&tx_buffer_[4], payload, payload_length);
    }

    // CRC8 over version, packet_type, length, and payload
    uint8_t crc = calculateCrc8(&tx_buffer_[1], payload_length + 3);
    tx_buffer_[4 + payload_length] = crc;

    uint16_t total_length = payload_length + 5;
    bool success = Hal::hal_serial_write(tx_buffer_, total_length);

    if (success) {
        last_tx_ms_ = now_ms;
        tx_retry_count_ = 0;
    } else {
        tx_retry_count_++;
        if (tx_retry_count_ > Config::FC_TX_RETRY_LIMIT) {
            setTelemetryEvent(TE_FC_TX_RETRY_LIMIT);
        }
        setTelemetryEvent(TE_FC_TX_FAILED);
    }

    return success;
}

bool FcBridge::sendCommandInternal(Types::FcCommand cmd, float p1, float p2, float p3, float p4, uint32_t now_ms) {
    if (std::isnan(p1) || std::isnan(p2) || std::isnan(p3) || std::isnan(p4)) {
        setTelemetryEvent(TE_FC_COMMAND_REJECTED_NAN);
        return false;
    }

    uint8_t payload[21];
    uint8_t cmd_type = static_cast<uint8_t>(cmd);

    uint16_t offset = 0;
    std::memcpy(&payload[offset], &cmd_type, sizeof(cmd_type)); offset += sizeof(cmd_type);
    std::memcpy(&payload[offset], &p1, sizeof(p1)); offset += sizeof(p1);
    std::memcpy(&payload[offset], &p2, sizeof(p2)); offset += sizeof(p2);
    std::memcpy(&payload[offset], &p3, sizeof(p3)); offset += sizeof(p3);
    std::memcpy(&payload[offset], &p4, sizeof(p4)); offset += sizeof(p4);
    std::memcpy(&payload[offset], &now_ms, sizeof(now_ms)); offset += sizeof(now_ms);

    bool ok = sendPacket(FC_PACKET_TYPE_COMMAND, payload, offset, now_ms);
    if (ok) {
        last_command_ms_ = now_ms;
        command_watchdog_active_ = false;
    }
    return ok;
}


// ============================================================================
// HIGH-LEVEL COMMAND DISPATCH
// ============================================================================

bool FcBridge::sendHold(uint32_t now_ms) {
    bool ok = sendCommandInternal(Types::FcCommand::HOLD, 0.0f, 0.0f, 0.0f, 0.0f, now_ms);
    if (ok) setTelemetryEvent(TE_FC_HOLD_SENT);
    return ok;
}

bool FcBridge::sendLand(uint32_t now_ms) {
    bool ok = sendCommandInternal(Types::FcCommand::LAND, 0.0f, 0.0f, 0.0f, 0.0f, now_ms);
    if (ok) setTelemetryEvent(TE_FC_LAND_SENT);
    return ok;
}

bool FcBridge::sendTakeoff(float target_altitude_m, uint32_t now_ms) {
    float clamped_alt = std::max(Config::FC_MIN_ALTITUDE_M, std::min(Config::FC_MAX_ALTITUDE_M, target_altitude_m));
    bool ok = sendCommandInternal(Types::FcCommand::TAKEOFF, clamped_alt, 0.0f, 0.0f, 0.0f, now_ms);
    if (ok) setTelemetryEvent(TE_FC_TAKEOFF_SENT);
    return ok;
}

bool FcBridge::sendAltitudeCommand(float target_altitude_m, uint32_t now_ms) {
    float clamped_alt = std::max(Config::FC_MIN_ALTITUDE_M, std::min(Config::FC_MAX_ALTITUDE_M, target_altitude_m));
    bool ok = sendCommandInternal(Types::FcCommand::ALTITUDE, clamped_alt, 0.0f, 0.0f, 0.0f, now_ms);
    if (ok) setTelemetryEvent(TE_FC_ALTITUDE_SENT);
    return ok;
}

bool FcBridge::sendHeadingCommand(float heading_deg, uint32_t now_ms) {
    float norm_heading = std::fmod(heading_deg, 360.0f);
    if (norm_heading < 0.0f) norm_heading += 360.0f;
    bool ok = sendCommandInternal(Types::FcCommand::HEADING, norm_heading, 0.0f, 0.0f, 0.0f, now_ms);
    if (ok) setTelemetryEvent(TE_FC_HEADING_SENT);
    return ok;
}

bool FcBridge::sendVelocityCommand(
    const Types::Vec2& velocity,
    float target_altitude_m,
    float heading_deg,
    uint32_t now_ms
) {
    float vx = std::max(-Config::FC_MAX_HORIZONTAL_SPEED_MPS, std::min(Config::FC_MAX_HORIZONTAL_SPEED_MPS, velocity.x));
    float vy = std::max(-Config::FC_MAX_HORIZONTAL_SPEED_MPS, std::min(Config::FC_MAX_HORIZONTAL_SPEED_MPS, velocity.y));
    float clamped_alt = std::max(Config::FC_MIN_ALTITUDE_M, std::min(Config::FC_MAX_ALTITUDE_M, target_altitude_m));
    float norm_heading = std::fmod(heading_deg, 360.0f);
    if (norm_heading < 0.0f) norm_heading += 360.0f;

    bool ok = sendCommandInternal(Types::FcCommand::VELOCITY, vx, vy, clamped_alt, norm_heading, now_ms);
    if (ok) setTelemetryEvent(TE_FC_VELOCITY_SENT);
    return ok;
}

bool FcBridge::requestArm(uint32_t now_ms) {
    setTelemetryEvent(TE_FC_ARM_REQUESTED);
    return sendCommandInternal(Types::FcCommand::ARM, Config::ARM_MAGIC_NUMBER, 0.0f, 0.0f, 0.0f, now_ms);
}

bool FcBridge::requestDisarm(uint32_t now_ms) {
    setTelemetryEvent(TE_FC_DISARM_REQUESTED);
    return sendCommandInternal(Types::FcCommand::DISARM, Config::DISARM_MAGIC_NUMBER, 0.0f, 0.0f, 0.0f, now_ms);
}

bool FcBridge::requestEmergencyStop(uint32_t now_ms) {
    setTelemetryEvent(TE_FC_EMERGENCY_STOP_SENT);
    return sendCommandInternal(Types::FcCommand::EMERGENCY_STOP, Config::EMERGENCY_MAGIC_NUMBER, 0.0f, 0.0f, 0.0f, now_ms);
}


// ============================================================================
// RECEIVE PARSER & STATUS DECODING
// ============================================================================

void FcBridge::parseStatusPayload(const uint8_t* payload, uint8_t length, uint32_t now_ms) {
    if (length < 32) {
        setTelemetryEvent(TE_FC_PACKET_INVALID);
        return;
    }

    uint8_t arm_val = 0, flight_m = 0;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f, alt = 0.0f, v_bat = 0.0f, i_bat = 0.0f;
    uint16_t err = 0;
    uint32_t ts = 0;

    uint16_t offset = 0;
    std::memcpy(&arm_val, &payload[offset], sizeof(arm_val)); offset += sizeof(arm_val);
    std::memcpy(&roll, &payload[offset], sizeof(roll)); offset += sizeof(roll);
    std::memcpy(&pitch, &payload[offset], sizeof(pitch)); offset += sizeof(pitch);
    std::memcpy(&yaw, &payload[offset], sizeof(yaw)); offset += sizeof(yaw);
    std::memcpy(&alt, &payload[offset], sizeof(alt)); offset += sizeof(alt);
    std::memcpy(&v_bat, &payload[offset], sizeof(v_bat)); offset += sizeof(v_bat);
    std::memcpy(&i_bat, &payload[offset], sizeof(i_bat)); offset += sizeof(i_bat);
    std::memcpy(&err, &payload[offset], sizeof(err)); offset += sizeof(err);
    std::memcpy(&flight_m, &payload[offset], sizeof(flight_m)); offset += sizeof(flight_m);
    std::memcpy(&ts, &payload[offset], sizeof(ts)); offset += sizeof(ts);

    latest_status_.valid = true;
    latest_status_.armed = (arm_val != 0);
    latest_status_.roll_deg = roll;
    latest_status_.pitch_deg = pitch;
    latest_status_.yaw_deg = yaw;
    latest_status_.altitude_m = alt;
    latest_status_.battery_voltage = (!std::isnan(v_bat) && v_bat > 0.0f) ? v_bat : 0.0f;
    latest_status_.battery_current = (!std::isnan(i_bat) && i_bat >= 0.0f) ? i_bat : 0.0f;
    latest_status_.battery_current_valid = (!std::isnan(i_bat) && i_bat >= 0.0f);
    latest_status_.error_flags = err;
    latest_status_.flight_mode = flight_m;
    latest_status_.timestamp_ms = now_ms;

    armed_ = latest_status_.armed;
    last_status_packet_ms_ = now_ms;
    last_rx_ms_ = now_ms;

    if (armed_) {
        setTelemetryEvent(TE_FC_ARMED);
    } else {
        setTelemetryEvent(TE_FC_DISARMED);
    }

    setTelemetryEvent(TE_FC_STATUS_RECEIVED);
}

void FcBridge::processIncomingBytes(uint32_t now_ms) {
    uint8_t temp_buf[32];
    int16_t bytes_read = Hal::hal_serial_read(temp_buf, sizeof(temp_buf));

    while (bytes_read > 0) {
        for (int16_t i = 0; i < bytes_read; ++i) {
            uint8_t byte = temp_buf[i];

            if (rx_index_ == 0) {
                if (byte == Config::FC_PACKET_START_BYTE) {
                    rx_buffer_[rx_index_++] = byte;
                }
            } else if (rx_index_ == 1) {
                if (byte == Config::FC_PROTOCOL_VERSION) {
                    rx_buffer_[rx_index_++] = byte;
                } else {
                    rx_index_ = 0; // Invalid version
                }
            } else if (rx_index_ == 2) {
                rx_buffer_[rx_index_++] = byte; // Packet type
            } else if (rx_index_ == 3) {
                rx_buffer_[rx_index_++] = byte; // Payload length
                if (byte > (Config::FC_RX_BUFFER_SIZE - 5)) {
                    rx_index_ = 0; // Length too large
                }
            } else {
                rx_buffer_[rx_index_++] = byte;
                uint8_t payload_len = rx_buffer_[3];
                uint16_t total_packet_len = payload_len + 5;

                if (rx_index_ >= total_packet_len) {
                    // Full packet received: verify CRC
                    uint8_t expected_crc = calculateCrc8(&rx_buffer_[1], payload_len + 3);
                    uint8_t received_crc = rx_buffer_[total_packet_len - 1];

                    if (expected_crc == received_crc) {
                        uint8_t p_type = rx_buffer_[2];
                        if (p_type == FC_PACKET_TYPE_STATUS) {
                            parseStatusPayload(&rx_buffer_[4], payload_len, now_ms);
                        }
                    } else {
                        setTelemetryEvent(TE_FC_PACKET_CRC_ERROR);
                    }

                    rx_index_ = 0; // Reset parser
                }
            }
        }

        bytes_read = Hal::hal_serial_read(temp_buf, sizeof(temp_buf));
    }
}


// ============================================================================
// MAIN UPDATE (50 Hz NON-BLOCKING)
// ============================================================================

void FcBridge::update(uint32_t now_ms) {
    processIncomingBytes(now_ms);

    // Link health evaluation
    if ((now_ms - last_status_packet_ms_) > Config::FC_STATUS_TIMEOUT_MS || !Hal::hal_serial_is_healthy()) {
        if (link_healthy_) {
            link_healthy_ = false;
            setTelemetryEvent(TE_FC_LINK_LOST);
        }
    } else {
        if (!link_healthy_) {
            link_healthy_ = true;
            setTelemetryEvent(TE_FC_LINK_HEALTHY);
        }
    }

    // Command watchdog auto-hold
    if (Config::FC_AUTO_HOLD_ON_COMMAND_TIMEOUT && armed_ && link_healthy_) {
        if ((now_ms - last_command_ms_) > Config::FC_COMMAND_TIMEOUT_MS) {
            sendHold(now_ms);
            command_watchdog_active_ = true;
            setTelemetryEvent(TE_FC_COMMAND_WATCHDOG_HOLD);
        }
    }
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void fc_bridge_init() {
    s_global_fc_bridge.init();
}

void fc_bridge_update(uint32_t now_ms) {
    s_global_fc_bridge.update(now_ms);
}

bool fc_bridge_send_velocity(const Types::Vec2& vel, float alt, float yaw_deg, uint32_t now_ms) {
    return s_global_fc_bridge.sendVelocityCommand(vel, alt, yaw_deg, now_ms);
}

bool fc_bridge_is_healthy() {
    return s_global_fc_bridge.isLinkHealthy();
}

Types::AttitudeSample fc_bridge_get_attitude() {
    return s_global_fc_bridge.getAttitude();
}

FcBridge& fc_bridge_get_instance() {
    return s_global_fc_bridge;
}

} // namespace RobofestDrone
