#include "swarm_comm.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static SwarmComm s_global_swarm_comm;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

SwarmComm::SwarmComm() {
    reset();
}

void SwarmComm::init() {
    reset();
    Hal::hal_radio_init();
}

void SwarmComm::reset() {
    for (uint8_t i = 0; i < Config::MAX_SWARM_DRONES; ++i) {
        peers_[i] = SwarmPeer();
        peers_[i].drone_id = i + 1;
    }

    self_drone_id_ = 1;
    self_role_ = Types::DroneRole::SCOUT_LEFT;
    self_state_ = Types::DroneState::INIT;

    shared_path_ = Types::SafePath();
    has_new_shared_path_ = false;
    path_assembly_buffer_ = Types::SafePath();
    received_chunks_mask_ = 0;

    shared_person_track_ = Types::HumanTrack();
    has_new_person_update_ = false;

    has_new_mine_update_ = false;
    last_mine_update_version_ = 0;

    has_help_request_ = false;
    help_request_reason_ = 0;
    help_request_source_ = 0;

    received_land_now_ = false;
    land_now_reason_ = 0;

    last_heartbeat_send_ms_ = 0;
    last_mine_update_send_ms_ = 0;
    last_person_update_send_ms_ = 0;
    last_path_update_send_ms_ = 0;
    last_role_check_ms_ = 0;
    last_peer_cleanup_ms_ = 0;

    last_telemetry_event_id_ = TE_SWARM_INITIALIZED;
    telemetry_event_valid_ = true;
}

void SwarmComm::setSelfInfo(uint8_t self_drone_id, Types::DroneRole self_role) {
    if (self_drone_id >= 1 && self_drone_id <= Config::MAX_SWARM_DRONES) {
        self_drone_id_ = self_drone_id;
        self_role_ = self_role;
    }
}

void SwarmComm::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}


// ============================================================================
// SPATIAL MINE HASH
// ============================================================================

uint16_t SwarmComm::computeMineHash(float x, float y) const {
    if (std::isnan(x) || std::isnan(y)) return 0;
    float safe_x = (x >= 0.0f) ? x : 0.0f;
    float safe_y = (y >= 0.0f) ? y : 0.0f;
    uint16_t qx = static_cast<uint16_t>(safe_x / Config::MINE_HASH_GRID_RESOLUTION_M);
    uint16_t qy = static_cast<uint16_t>(safe_y / Config::MINE_HASH_GRID_RESOLUTION_M);
    return static_cast<uint16_t>((qx * 131) ^ qy);
}


// ============================================================================
// SWARM STATUS & PEER INSPECTION
// ============================================================================

bool SwarmComm::isPeerAlive(uint8_t drone_id) const {
    if (drone_id >= 1 && drone_id <= Config::MAX_SWARM_DRONES) {
        return peers_[drone_id - 1].alive;
    }
    return false;
}

uint8_t SwarmComm::getActivePeerCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < Config::MAX_SWARM_DRONES; ++i) {
        if ((i + 1) != self_drone_id_ && peers_[i].alive) {
            count++;
        }
    }
    return count;
}

Types::DroneRole SwarmComm::getPeerRole(uint8_t drone_id) const {
    if (drone_id >= 1 && drone_id <= Config::MAX_SWARM_DRONES) {
        return peers_[drone_id - 1].role;
    }
    return Types::DroneRole::RESERVE;
}

Types::DroneState SwarmComm::getPeerState(uint8_t drone_id) const {
    if (drone_id >= 1 && drone_id <= Config::MAX_SWARM_DRONES) {
        return peers_[drone_id - 1].state;
    }
    return Types::DroneState::INIT;
}

bool SwarmComm::isSwarmHealthy() const {
    return getActivePeerCount() >= (Config::MIN_SWARM_DRONES - 1);
}

bool SwarmComm::isSwarmDegraded() const {
    uint8_t active = getActivePeerCount();
    return (active == (Config::MIN_SWARM_DRONES - 2));
}

bool SwarmComm::isSwarmCritical() const {
    return (getActivePeerCount() < (Config::MIN_SWARM_DRONES - 2)) || received_land_now_;
}

uint8_t SwarmComm::getLaneIdForRole(Types::DroneRole role) const {
    switch (role) {
        case Types::DroneRole::SCOUT_LEFT:   return Config::LANE_LEFT;
        case Types::DroneRole::SCOUT_RIGHT:  return Config::LANE_RIGHT;
        case Types::DroneRole::GUIDE_MARKER: return Config::LANE_CENTER;
        case Types::DroneRole::RESERVE:      return Config::LANE_NONE;
        default:                             return Config::LANE_NONE;
    }
}

uint8_t SwarmComm::getCoordinatorDroneId() const {
    if (peers_[self_drone_id_ - 1].alive || true) {
        // Find the lowest active drone ID
        for (uint8_t i = 0; i < Config::MAX_SWARM_DRONES; ++i) {
            uint8_t id = i + 1;
            if (id == self_drone_id_ || peers_[i].alive) {
                return id;
            }
        }
    }
    return self_drone_id_;
}


// ============================================================================
// DETERMINISTIC ROLE FAILOVER
// ============================================================================

bool SwarmComm::shouldReassignRoles() const {
    // Check if any primary role is uncovered among alive drones
    bool has_scout_left = (self_role_ == Types::DroneRole::SCOUT_LEFT);
    bool has_scout_right = (self_role_ == Types::DroneRole::SCOUT_RIGHT);
    bool has_guide = (self_role_ == Types::DroneRole::GUIDE_MARKER);

    for (uint8_t i = 0; i < Config::MAX_SWARM_DRONES; ++i) {
        if ((i + 1) != self_drone_id_ && peers_[i].alive) {
            if (peers_[i].role == Types::DroneRole::SCOUT_LEFT) has_scout_left = true;
            if (peers_[i].role == Types::DroneRole::SCOUT_RIGHT) has_scout_right = true;
            if (peers_[i].role == Types::DroneRole::GUIDE_MARKER) has_guide = true;
        }
    }

    return (!has_scout_left || !has_scout_right || !has_guide);
}

Types::DroneRole SwarmComm::getRecommendedRoleForSelf() const {
    return getRecommendedRoleForPeer(self_drone_id_);
}

Types::DroneRole SwarmComm::getRecommendedRoleForPeer(uint8_t drone_id) const {
    // Deterministic role mapping based on rank among currently alive drones
    uint8_t rank = 0;
    for (uint8_t i = 0; i < Config::MAX_SWARM_DRONES; ++i) {
        uint8_t peer_id = i + 1;
        if (peer_id == self_drone_id_ || peers_[i].alive) {
            if (peer_id == drone_id) {
                break;
            }
            rank++;
        }
    }

    switch (rank) {
        case 0:  return Types::DroneRole::SCOUT_LEFT;
        case 1:  return Types::DroneRole::SCOUT_RIGHT;
        case 2:  return Types::DroneRole::GUIDE_MARKER;
        default: return Types::DroneRole::RESERVE;
    }
}


// ============================================================================
// BROADCASTING API
// ============================================================================

bool SwarmComm::broadcastClaim(const Types::MineRecord& mine, uint32_t now_ms) {
    Types::SwarmPacket packet;
    packet.packet_type = Types::PacketType::CLAIM;
    packet.packet_version = Config::SWARM_PACKET_VERSION;
    packet.sender_drone_id = self_drone_id_;
    packet.timestamp_ms = now_ms;
    packet.map_version = mine.map_version;

    uint16_t hash = computeMineHash(mine.x, mine.y);
    uint8_t m_type = static_cast<uint8_t>(mine.marker_type);

    uint16_t offset = 0;
    std::memcpy(&packet.payload[offset], &hash, sizeof(hash)); offset += sizeof(hash);
    std::memcpy(&packet.payload[offset], &mine.mine_id, sizeof(mine.mine_id)); offset += sizeof(mine.mine_id);
    std::memcpy(&packet.payload[offset], &mine.x, sizeof(mine.x)); offset += sizeof(mine.x);
    std::memcpy(&packet.payload[offset], &mine.y, sizeof(mine.y)); offset += sizeof(mine.y);
    std::memcpy(&packet.payload[offset], &mine.confidence, sizeof(mine.confidence)); offset += sizeof(mine.confidence);
    std::memcpy(&packet.payload[offset], &mine.persistence_count, sizeof(mine.persistence_count)); offset += sizeof(mine.persistence_count);
    std::memcpy(&packet.payload[offset], &m_type, sizeof(m_type)); offset += sizeof(m_type);
    std::memcpy(&packet.payload[offset], &self_drone_id_, sizeof(self_drone_id_)); offset += sizeof(self_drone_id_);
    std::memcpy(&packet.payload[offset], &mine.map_version, sizeof(mine.map_version)); offset += sizeof(mine.map_version);
    std::memcpy(&packet.payload[offset], &now_ms, sizeof(now_ms)); offset += sizeof(now_ms);

    packet.payload_length = offset;
    bool ok = Hal::hal_radio_send(packet);
    if (ok) setTelemetryEvent(TE_SWARM_CLAIM_SENT);
    return ok;
}

bool SwarmComm::broadcastYield(uint16_t mine_hash, uint8_t reason, uint32_t now_ms) {
    Types::SwarmPacket packet;
    packet.packet_type = Types::PacketType::YIELD;
    packet.packet_version = Config::SWARM_PACKET_VERSION;
    packet.sender_drone_id = self_drone_id_;
    packet.timestamp_ms = now_ms;
    packet.map_version = 0;

    uint16_t offset = 0;
    std::memcpy(&packet.payload[offset], &mine_hash, sizeof(mine_hash)); offset += sizeof(mine_hash);
    std::memcpy(&packet.payload[offset], &reason, sizeof(reason)); offset += sizeof(reason);
    std::memcpy(&packet.payload[offset], &self_drone_id_, sizeof(self_drone_id_)); offset += sizeof(self_drone_id_);
    std::memcpy(&packet.payload[offset], &now_ms, sizeof(now_ms)); offset += sizeof(now_ms);

    packet.payload_length = offset;
    bool ok = Hal::hal_radio_send(packet);
    if (ok) setTelemetryEvent(TE_SWARM_YIELD_SENT);
    return ok;
}

bool SwarmComm::broadcastMineUpdate(const Types::MineRecord& mine, uint32_t now_ms) {
    Types::SwarmPacket packet;
    packet.packet_type = Types::PacketType::MINE_UPDATE;
    packet.packet_version = Config::SWARM_PACKET_VERSION;
    packet.sender_drone_id = self_drone_id_;
    packet.timestamp_ms = now_ms;
    packet.map_version = mine.map_version;

    uint16_t hash = computeMineHash(mine.x, mine.y);
    uint8_t status = static_cast<uint8_t>(mine.status);
    uint8_t m_type = static_cast<uint8_t>(mine.marker_type);

    uint16_t offset = 0;
    std::memcpy(&packet.payload[offset], &hash, sizeof(hash)); offset += sizeof(hash);
    std::memcpy(&packet.payload[offset], &mine.mine_id, sizeof(mine.mine_id)); offset += sizeof(mine.mine_id);
    std::memcpy(&packet.payload[offset], &mine.x, sizeof(mine.x)); offset += sizeof(mine.x);
    std::memcpy(&packet.payload[offset], &mine.y, sizeof(mine.y)); offset += sizeof(mine.y);
    std::memcpy(&packet.payload[offset], &mine.confidence, sizeof(mine.confidence)); offset += sizeof(mine.confidence);
    std::memcpy(&packet.payload[offset], &mine.persistence_count, sizeof(mine.persistence_count)); offset += sizeof(mine.persistence_count);
    std::memcpy(&packet.payload[offset], &status, sizeof(status)); offset += sizeof(status);
    std::memcpy(&packet.payload[offset], &m_type, sizeof(m_type)); offset += sizeof(m_type);
    std::memcpy(&packet.payload[offset], &mine.map_version, sizeof(mine.map_version)); offset += sizeof(mine.map_version);
    std::memcpy(&packet.payload[offset], &now_ms, sizeof(now_ms)); offset += sizeof(now_ms);

    packet.payload_length = offset;
    bool ok = Hal::hal_radio_send(packet);
    if (ok) setTelemetryEvent(TE_SWARM_MINE_UPDATE_SENT);
    return ok;
}

bool SwarmComm::broadcastPathUpdate(const Types::SafePath& path, uint32_t now_ms) {
    if (!path.valid || path.waypoint_count == 0) return false;

    uint8_t total_chunks = static_cast<uint8_t>(
        (path.waypoint_count + Config::PATH_UPDATE_CHUNK_MAX_WAYPOINTS - 1) / Config::PATH_UPDATE_CHUNK_MAX_WAYPOINTS
    );

    bool all_ok = true;
    for (uint8_t chunk = 0; chunk < total_chunks; ++chunk) {
        Types::SwarmPacket packet;
        packet.packet_type = Types::PacketType::PATH_UPDATE;
        packet.packet_version = Config::SWARM_PACKET_VERSION;
        packet.sender_drone_id = self_drone_id_;
        packet.timestamp_ms = now_ms;
        packet.map_version = path.path_version;

        uint8_t wp_start = chunk * Config::PATH_UPDATE_CHUNK_MAX_WAYPOINTS;
        uint8_t wp_end = std::min(static_cast<uint8_t>(wp_start + Config::PATH_UPDATE_CHUNK_MAX_WAYPOINTS), path.waypoint_count);

        uint16_t offset = 0;
        std::memcpy(&packet.payload[offset], &path.path_version, sizeof(path.path_version)); offset += sizeof(path.path_version);
        std::memcpy(&packet.payload[offset], &path.created_time, sizeof(path.created_time)); offset += sizeof(path.created_time);
        std::memcpy(&packet.payload[offset], &path.corridor_width_m, sizeof(path.corridor_width_m)); offset += sizeof(path.corridor_width_m);
        std::memcpy(&packet.payload[offset], &path.waypoint_count, sizeof(path.waypoint_count)); offset += sizeof(path.waypoint_count);
        std::memcpy(&packet.payload[offset], &chunk, sizeof(chunk)); offset += sizeof(chunk);
        std::memcpy(&packet.payload[offset], &total_chunks, sizeof(total_chunks)); offset += sizeof(total_chunks);
        std::memcpy(&packet.payload[offset], &path.path_version, sizeof(path.path_version)); offset += sizeof(path.path_version);

        for (uint8_t w = wp_start; w < wp_end; ++w) {
            std::memcpy(&packet.payload[offset], &path.waypoints[w].x, sizeof(float)); offset += sizeof(float);
            std::memcpy(&packet.payload[offset], &path.waypoints[w].y, sizeof(float)); offset += sizeof(float);
        }

        packet.payload_length = offset;
        all_ok &= Hal::hal_radio_send(packet);
    }

    if (all_ok) setTelemetryEvent(TE_SWARM_PATH_UPDATE_SENT);
    return all_ok;
}

bool SwarmComm::broadcastPersonUpdate(const Types::HumanTrack& human_track, uint32_t now_ms) {
    Types::SwarmPacket packet;
    packet.packet_type = Types::PacketType::PERSON_UPDATE;
    packet.packet_version = Config::SWARM_PACKET_VERSION;
    packet.sender_drone_id = self_drone_id_;
    packet.timestamp_ms = now_ms;
    packet.map_version = 0;

    uint8_t detected = human_track.human_detected ? 1 : 0;
    uint8_t exit_zone = human_track.human_in_exit_zone ? 1 : 0;

    uint16_t offset = 0;
    std::memcpy(&packet.payload[offset], &detected, sizeof(detected)); offset += sizeof(detected);
    std::memcpy(&packet.payload[offset], &human_track.field_x, sizeof(human_track.field_x)); offset += sizeof(human_track.field_x);
    std::memcpy(&packet.payload[offset], &human_track.field_y, sizeof(human_track.field_y)); offset += sizeof(human_track.field_y);
    std::memcpy(&packet.payload[offset], &human_track.lateral_deviation_m, sizeof(human_track.lateral_deviation_m)); offset += sizeof(human_track.lateral_deviation_m);
    std::memcpy(&packet.payload[offset], &human_track.forward_progress_m, sizeof(human_track.forward_progress_m)); offset += sizeof(human_track.forward_progress_m);
    std::memcpy(&packet.payload[offset], &human_track.tracking_confidence, sizeof(human_track.tracking_confidence)); offset += sizeof(human_track.tracking_confidence);
    std::memcpy(&packet.payload[offset], &exit_zone, sizeof(exit_zone)); offset += sizeof(exit_zone);
    std::memcpy(&packet.payload[offset], &now_ms, sizeof(now_ms)); offset += sizeof(now_ms);

    packet.payload_length = offset;
    bool ok = Hal::hal_radio_send(packet);
    if (ok) setTelemetryEvent(TE_SWARM_PERSON_UPDATE_SENT);
    return ok;
}

bool SwarmComm::broadcastHelpRequest(uint8_t reason, uint32_t now_ms) {
    Types::SwarmPacket packet;
    packet.packet_type = Types::PacketType::HELP_REQUEST;
    packet.packet_version = Config::SWARM_PACKET_VERSION;
    packet.sender_drone_id = self_drone_id_;
    packet.timestamp_ms = now_ms;
    packet.map_version = 0;

    uint8_t role = static_cast<uint8_t>(self_role_);
    uint8_t state = static_cast<uint8_t>(self_state_);

    uint16_t offset = 0;
    std::memcpy(&packet.payload[offset], &reason, sizeof(reason)); offset += sizeof(reason);
    std::memcpy(&packet.payload[offset], &self_drone_id_, sizeof(self_drone_id_)); offset += sizeof(self_drone_id_);
    std::memcpy(&packet.payload[offset], &role, sizeof(role)); offset += sizeof(role);
    std::memcpy(&packet.payload[offset], &state, sizeof(state)); offset += sizeof(state);
    std::memcpy(&packet.payload[offset], &now_ms, sizeof(now_ms)); offset += sizeof(now_ms);

    packet.payload_length = offset;
    bool ok = Hal::hal_radio_send(packet);
    if (ok) setTelemetryEvent(TE_SWARM_HELP_REQUEST_SENT);
    return ok;
}

bool SwarmComm::broadcastLandNow(uint8_t reason, uint32_t now_ms) {
    Types::SwarmPacket packet;
    packet.packet_type = Types::PacketType::LAND_NOW;
    packet.packet_version = Config::SWARM_PACKET_VERSION;
    packet.sender_drone_id = self_drone_id_;
    packet.timestamp_ms = now_ms;
    packet.map_version = 0;

    uint16_t offset = 0;
    std::memcpy(&packet.payload[offset], &reason, sizeof(reason)); offset += sizeof(reason);
    std::memcpy(&packet.payload[offset], &self_drone_id_, sizeof(self_drone_id_)); offset += sizeof(self_drone_id_);
    std::memcpy(&packet.payload[offset], &now_ms, sizeof(now_ms)); offset += sizeof(now_ms);

    packet.payload_length = offset;
    bool ok = Hal::hal_radio_send(packet);
    if (ok) setTelemetryEvent(TE_SWARM_LAND_NOW_SENT);
    return ok;
}

bool SwarmComm::getSharedPath(Types::SafePath& out_path) const {
    if (has_new_shared_path_ && shared_path_.valid) {
        out_path = shared_path_;
        return true;
    }
    return false;
}


// ============================================================================
// PACKET PROCESSING & DISPATCH
// ============================================================================

void SwarmComm::handleHeartbeat(const Types::SwarmPacket& packet, uint32_t now_ms) {
    if (packet.payload_length < 23) return;

    uint8_t sender = packet.sender_drone_id;
    if (sender < 1 || sender > Config::MAX_SWARM_DRONES) return;

    uint8_t role_val = 0, state_val = 0, loc_health_val = 0;
    float battery = 0.0f;
    uint32_t map_v = 0, path_v = 0, ts = 0;

    uint16_t offset = 1; // sender already read
    std::memcpy(&role_val, &packet.payload[offset], sizeof(role_val)); offset += sizeof(role_val);
    std::memcpy(&state_val, &packet.payload[offset], sizeof(state_val)); offset += sizeof(state_val);
    std::memcpy(&battery, &packet.payload[offset], sizeof(battery)); offset += sizeof(battery);
    std::memcpy(&loc_health_val, &packet.payload[offset], sizeof(loc_health_val)); offset += sizeof(loc_health_val);
    std::memcpy(&map_v, &packet.payload[offset], sizeof(map_v)); offset += sizeof(map_v);
    std::memcpy(&path_v, &packet.payload[offset], sizeof(path_v)); offset += sizeof(path_v);
    std::memcpy(&ts, &packet.payload[offset], sizeof(ts)); offset += sizeof(ts);

    SwarmPeer& peer = peers_[sender - 1];
    peer.known = true;
    peer.alive = true;
    peer.role = static_cast<Types::DroneRole>(role_val);
    peer.state = static_cast<Types::DroneState>(state_val);
    peer.battery_voltage = battery;
    peer.localization_health = static_cast<Types::LocalizationHealth>(loc_health_val);
    peer.map_version = map_v;
    peer.path_version = path_v;
    peer.last_heartbeat_ms = now_ms;
    peer.last_packet_ms = now_ms;
    peer.missed_heartbeats = 0;
    peer.lane_id = getLaneIdForRole(peer.role);

    setTelemetryEvent(TE_SWARM_HEARTBEAT_RECEIVED);
}

void SwarmComm::handleRoleAssign(const Types::SwarmPacket& packet, uint32_t now_ms) {
    (void)now_ms;
    if (packet.payload_length < 7) return;

    uint8_t target_id = packet.payload[0];
    uint8_t assigned_role = packet.payload[1];
    uint8_t coordinator_id = packet.payload[2];

    if (coordinator_id != getCoordinatorDroneId()) return;

    if (target_id == self_drone_id_ || target_id == 0xFF) {
        self_role_ = static_cast<Types::DroneRole>(assigned_role);
        setTelemetryEvent(TE_SWARM_ROLE_ACCEPTED);
    } else if (target_id >= 1 && target_id <= Config::MAX_SWARM_DRONES) {
        peers_[target_id - 1].role = static_cast<Types::DroneRole>(assigned_role);
        setTelemetryEvent(TE_SWARM_ROLE_ASSIGNED);
    }
}

void SwarmComm::handleClaim(const Types::SwarmPacket& packet, MineMap* optional_mine_map, uint32_t now_ms) {
    if (packet.payload_length < 28) return;

    uint16_t hash = 0, mine_id = 0, persistence = 0;
    float x = 0.0f, y = 0.0f, confidence = 0.0f;
    uint8_t marker_type = 0, sender_id = 0;
    uint32_t map_v = 0, ts = 0;

    uint16_t offset = 0;
    std::memcpy(&hash, &packet.payload[offset], sizeof(hash)); offset += sizeof(hash);
    std::memcpy(&mine_id, &packet.payload[offset], sizeof(mine_id)); offset += sizeof(mine_id);
    std::memcpy(&x, &packet.payload[offset], sizeof(x)); offset += sizeof(x);
    std::memcpy(&y, &packet.payload[offset], sizeof(y)); offset += sizeof(y);
    std::memcpy(&confidence, &packet.payload[offset], sizeof(confidence)); offset += sizeof(confidence);
    std::memcpy(&persistence, &packet.payload[offset], sizeof(persistence)); offset += sizeof(persistence);
    std::memcpy(&marker_type, &packet.payload[offset], sizeof(marker_type)); offset += sizeof(marker_type);
    std::memcpy(&sender_id, &packet.payload[offset], sizeof(sender_id)); offset += sizeof(sender_id);
    std::memcpy(&map_v, &packet.payload[offset], sizeof(map_v)); offset += sizeof(map_v);
    std::memcpy(&ts, &packet.payload[offset], sizeof(ts)); offset += sizeof(ts);

    if (optional_mine_map != nullptr) {
        // Fuse candidate detection from peer claim
        optional_mine_map->addDetection(x, y, confidence, static_cast<Types::VisionMarkerType>(marker_type), sender_id, now_ms);
    }

    setTelemetryEvent(TE_SWARM_CLAIM_RECEIVED);
}

void SwarmComm::handleYield(const Types::SwarmPacket& packet, uint32_t now_ms) {
    (void)now_ms;
    if (packet.payload_length < 8) return;
    setTelemetryEvent(TE_SWARM_YIELD_RECEIVED);
}

void SwarmComm::handleMineUpdate(const Types::SwarmPacket& packet, MineMap* optional_mine_map, uint32_t now_ms) {
    if (packet.payload_length < 28) return;

    uint16_t hash = 0, mine_id = 0, persistence = 0;
    float x = 0.0f, y = 0.0f, confidence = 0.0f;
    uint8_t status = 0, marker_type = 0;
    uint32_t map_v = 0, ts = 0;

    uint16_t offset = 0;
    std::memcpy(&hash, &packet.payload[offset], sizeof(hash)); offset += sizeof(hash);
    std::memcpy(&mine_id, &packet.payload[offset], sizeof(mine_id)); offset += sizeof(mine_id);
    std::memcpy(&x, &packet.payload[offset], sizeof(x)); offset += sizeof(x);
    std::memcpy(&y, &packet.payload[offset], sizeof(y)); offset += sizeof(y);
    std::memcpy(&confidence, &packet.payload[offset], sizeof(confidence)); offset += sizeof(confidence);
    std::memcpy(&persistence, &packet.payload[offset], sizeof(persistence)); offset += sizeof(persistence);
    std::memcpy(&status, &packet.payload[offset], sizeof(status)); offset += sizeof(status);
    std::memcpy(&marker_type, &packet.payload[offset], sizeof(marker_type)); offset += sizeof(marker_type);
    std::memcpy(&map_v, &packet.payload[offset], sizeof(map_v)); offset += sizeof(map_v);
    std::memcpy(&ts, &packet.payload[offset], sizeof(ts)); offset += sizeof(ts);

    if (optional_mine_map != nullptr) {
        optional_mine_map->addDetection(x, y, confidence, static_cast<Types::VisionMarkerType>(marker_type), packet.sender_drone_id, now_ms);
    }

    last_mine_update_version_ = map_v;
    has_new_mine_update_ = true;
    setTelemetryEvent(TE_SWARM_MINE_UPDATE_RECEIVED);
}

void SwarmComm::handlePathUpdate(const Types::SwarmPacket& packet, uint32_t now_ms) {
    (void)now_ms;
    if (packet.payload_length < 19) return;

    uint32_t path_v = 0, created_t = 0, map_v = 0;
    float corridor_w = 0.0f;
    uint8_t wp_count = 0, chunk_idx = 0, chunk_total = 0;

    uint16_t offset = 0;
    std::memcpy(&path_v, &packet.payload[offset], sizeof(path_v)); offset += sizeof(path_v);
    std::memcpy(&created_t, &packet.payload[offset], sizeof(created_t)); offset += sizeof(created_t);
    std::memcpy(&corridor_w, &packet.payload[offset], sizeof(corridor_w)); offset += sizeof(corridor_w);
    std::memcpy(&wp_count, &packet.payload[offset], sizeof(wp_count)); offset += sizeof(wp_count);
    std::memcpy(&chunk_idx, &packet.payload[offset], sizeof(chunk_idx)); offset += sizeof(chunk_idx);
    std::memcpy(&chunk_total, &packet.payload[offset], sizeof(chunk_total)); offset += sizeof(chunk_total);
    std::memcpy(&map_v, &packet.payload[offset], sizeof(map_v)); offset += sizeof(map_v);

    if (path_v <= shared_path_.path_version && shared_path_.valid) {
        return; // Ignore older path updates
    }

    if (chunk_idx == 0) {
        path_assembly_buffer_ = Types::SafePath();
        path_assembly_buffer_.path_version = path_v;
        path_assembly_buffer_.created_time = created_t;
        path_assembly_buffer_.corridor_width_m = corridor_w;
        path_assembly_buffer_.waypoint_count = wp_count;
        received_chunks_mask_ = 0;
    }

    uint8_t wp_start = chunk_idx * Config::PATH_UPDATE_CHUNK_MAX_WAYPOINTS;
    uint8_t num_wps_in_chunk = (packet.payload_length - offset) / (2 * sizeof(float));

    for (uint8_t w = 0; w < num_wps_in_chunk && (wp_start + w) < Config::PATH_MAX_WAYPOINTS; ++w) {
        float wx = 0.0f, wy = 0.0f;
        std::memcpy(&wx, &packet.payload[offset], sizeof(float)); offset += sizeof(float);
        std::memcpy(&wy, &packet.payload[offset], sizeof(float)); offset += sizeof(float);
        path_assembly_buffer_.waypoints[wp_start + w] = Types::PathWaypoint(wx, wy);
    }

    received_chunks_mask_ |= (1 << chunk_idx);

    uint8_t expected_mask = (1 << chunk_total) - 1;
    if (received_chunks_mask_ == expected_mask) {
        path_assembly_buffer_.valid = true;
        shared_path_ = path_assembly_buffer_;
        has_new_shared_path_ = true;
        setTelemetryEvent(TE_SWARM_PATH_UPDATE_RECEIVED);
    }
}

void SwarmComm::handlePersonUpdate(const Types::SwarmPacket& packet, uint32_t now_ms) {
    (void)now_ms;
    if (packet.payload_length < 26) return;

    uint8_t detected = 0, exit_zone = 0;
    float fx = 0.0f, fy = 0.0f, lat_dev = 0.0f, fwd_prog = 0.0f, conf = 0.0f;
    uint32_t ts = 0;

    uint16_t offset = 0;
    std::memcpy(&detected, &packet.payload[offset], sizeof(detected)); offset += sizeof(detected);
    std::memcpy(&fx, &packet.payload[offset], sizeof(fx)); offset += sizeof(fx);
    std::memcpy(&fy, &packet.payload[offset], sizeof(fy)); offset += sizeof(fy);
    std::memcpy(&lat_dev, &packet.payload[offset], sizeof(lat_dev)); offset += sizeof(lat_dev);
    std::memcpy(&fwd_prog, &packet.payload[offset], sizeof(fwd_prog)); offset += sizeof(fwd_prog);
    std::memcpy(&conf, &packet.payload[offset], sizeof(conf)); offset += sizeof(conf);
    std::memcpy(&exit_zone, &packet.payload[offset], sizeof(exit_zone)); offset += sizeof(exit_zone);
    std::memcpy(&ts, &packet.payload[offset], sizeof(ts)); offset += sizeof(ts);

    shared_person_track_.human_detected = (detected != 0);
    shared_person_track_.field_x = fx;
    shared_person_track_.field_y = fy;
    shared_person_track_.lateral_deviation_m = lat_dev;
    shared_person_track_.forward_progress_m = fwd_prog;
    shared_person_track_.tracking_confidence = conf;
    shared_person_track_.human_in_exit_zone = (exit_zone != 0);
    shared_person_track_.timestamp_ms = ts;
    has_new_person_update_ = true;

    setTelemetryEvent(TE_SWARM_PERSON_UPDATE_RECEIVED);
}

void SwarmComm::handleHelpRequest(const Types::SwarmPacket& packet, uint32_t now_ms) {
    (void)now_ms;
    if (packet.payload_length < 8) return;

    help_request_reason_ = packet.payload[0];
    help_request_source_ = packet.payload[1];
    has_help_request_ = true;

    setTelemetryEvent(TE_SWARM_HELP_REQUEST_RECEIVED);
}

void SwarmComm::handleLandNow(const Types::SwarmPacket& packet, uint32_t now_ms) {
    (void)now_ms;
    if (packet.payload_length < 6) return;

    land_now_reason_ = packet.payload[0];
    received_land_now_ = true;

    setTelemetryEvent(TE_SWARM_LAND_NOW_RECEIVED);
}

void SwarmComm::handlePacket(const Types::SwarmPacket& packet, MineMap* optional_mine_map, uint32_t now_ms) {
    if (packet.packet_version != Config::SWARM_PACKET_VERSION) {
        setTelemetryEvent(TE_SWARM_PACKET_INVALID);
        return;
    }

    if (packet.sender_drone_id == self_drone_id_) {
        return; // Ignore self-broadcast
    }

    if (packet.sender_drone_id < 1 || packet.sender_drone_id > Config::MAX_SWARM_DRONES) {
        setTelemetryEvent(TE_SWARM_PACKET_INVALID);
        return;
    }

    if ((now_ms >= packet.timestamp_ms) && ((now_ms - packet.timestamp_ms) > Config::SWARM_STALE_TIMESTAMP_TOLERANCE_MS)) {
        setTelemetryEvent(TE_SWARM_PACKET_STALE);
        return;
    }

    switch (packet.packet_type) {
        case Types::PacketType::HEARTBEAT:     handleHeartbeat(packet, now_ms); break;
        case Types::PacketType::ROLE_ASSIGN:   handleRoleAssign(packet, now_ms); break;
        case Types::PacketType::CLAIM:         handleClaim(packet, optional_mine_map, now_ms); break;
        case Types::PacketType::YIELD:         handleYield(packet, now_ms); break;
        case Types::PacketType::MINE_UPDATE:   handleMineUpdate(packet, optional_mine_map, now_ms); break;
        case Types::PacketType::PATH_UPDATE:   handlePathUpdate(packet, now_ms); break;
        case Types::PacketType::PERSON_UPDATE: handlePersonUpdate(packet, now_ms); break;
        case Types::PacketType::HELP_REQUEST:  handleHelpRequest(packet, now_ms); break;
        case Types::PacketType::LAND_NOW:      handleLandNow(packet, now_ms); break;
        default: break;
    }
}

void SwarmComm::processIncomingPackets(MineMap* optional_mine_map, uint32_t now_ms) {
    Types::SwarmPacket packet;
    while (Hal::hal_radio_receive(packet)) {
        handlePacket(packet, optional_mine_map, now_ms);
    }
}

void SwarmComm::updatePeerStatus(uint32_t now_ms) {
    for (uint8_t i = 0; i < Config::MAX_SWARM_DRONES; ++i) {
        if ((i + 1) == self_drone_id_) continue;

        if (peers_[i].known && peers_[i].alive) {
            uint32_t elapsed = now_ms - peers_[i].last_heartbeat_ms;
            if (elapsed > Config::PEER_LOST_TIMEOUT_MS) {
                peers_[i].alive = false;
                peers_[i].missed_heartbeats++;
                setTelemetryEvent(TE_SWARM_PEER_LOST);
            } else if (elapsed > Config::PEER_DEGRADED_TIMEOUT_MS) {
                setTelemetryEvent(TE_SWARM_PEER_DEGRADED);
            }
        }
    }
}


// ============================================================================
// MAIN UPDATE (50 Hz NON-BLOCKING)
// ============================================================================

void SwarmComm::update(uint32_t now_ms) {
    processIncomingPackets(nullptr, now_ms);
    updatePeerStatus(now_ms);

    // Periodic Heartbeat Transmission (250 ms)
    if ((now_ms - last_heartbeat_send_ms_) >= Config::HEARTBEAT_PERIOD_MS) {
        last_heartbeat_send_ms_ = now_ms;

        Types::SwarmPacket packet;
        packet.packet_type = Types::PacketType::HEARTBEAT;
        packet.packet_version = Config::SWARM_PACKET_VERSION;
        packet.sender_drone_id = self_drone_id_;
        packet.timestamp_ms = now_ms;
        packet.map_version = 0;

        uint8_t role_val = static_cast<uint8_t>(self_role_);
        uint8_t state_val = static_cast<uint8_t>(self_state_);
        float battery = 11.8f; // Stub battery level
        uint8_t loc_health = static_cast<uint8_t>(Types::LocalizationHealth::LOCALIZATION_GOOD);
        uint32_t map_v = 0;
        uint32_t path_v = 0;

        uint16_t offset = 0;
        std::memcpy(&packet.payload[offset], &self_drone_id_, sizeof(self_drone_id_)); offset += sizeof(self_drone_id_);
        std::memcpy(&packet.payload[offset], &role_val, sizeof(role_val)); offset += sizeof(role_val);
        std::memcpy(&packet.payload[offset], &state_val, sizeof(state_val)); offset += sizeof(state_val);
        std::memcpy(&packet.payload[offset], &battery, sizeof(battery)); offset += sizeof(battery);
        std::memcpy(&packet.payload[offset], &loc_health, sizeof(loc_health)); offset += sizeof(loc_health);
        std::memcpy(&packet.payload[offset], &map_v, sizeof(map_v)); offset += sizeof(map_v);
        std::memcpy(&packet.payload[offset], &path_v, sizeof(path_v)); offset += sizeof(path_v);
        std::memcpy(&packet.payload[offset], &now_ms, sizeof(now_ms)); offset += sizeof(now_ms);

        packet.payload_length = offset;
        if (Hal::hal_radio_send(packet)) {
            setTelemetryEvent(TE_SWARM_HEARTBEAT_SENT);
        }
    }
}

void SwarmComm::updateWithMissionData(
    MineMap& mine_map,
    const Types::SafePath& active_path,
    const Types::HumanTrack& human_track,
    Types::DroneState drone_state,
    uint32_t now_ms
) {
    self_state_ = drone_state;

    processIncomingPackets(&mine_map, now_ms);
    updatePeerStatus(now_ms);

    // Periodic Heartbeat Transmission
    if ((now_ms - last_heartbeat_send_ms_) >= Config::HEARTBEAT_PERIOD_MS) {
        last_heartbeat_send_ms_ = now_ms;

        Types::SwarmPacket packet;
        packet.packet_type = Types::PacketType::HEARTBEAT;
        packet.packet_version = Config::SWARM_PACKET_VERSION;
        packet.sender_drone_id = self_drone_id_;
        packet.timestamp_ms = now_ms;
        packet.map_version = mine_map.getMapVersion();

        uint8_t role_val = static_cast<uint8_t>(self_role_);
        uint8_t state_val = static_cast<uint8_t>(self_state_);
        float battery = 11.8f;
        uint8_t loc_health = static_cast<uint8_t>(Types::LocalizationHealth::LOCALIZATION_GOOD);
        uint32_t map_v = mine_map.getMapVersion();
        uint32_t path_v = active_path.path_version;

        uint16_t offset = 0;
        std::memcpy(&packet.payload[offset], &self_drone_id_, sizeof(self_drone_id_)); offset += sizeof(self_drone_id_);
        std::memcpy(&packet.payload[offset], &role_val, sizeof(role_val)); offset += sizeof(role_val);
        std::memcpy(&packet.payload[offset], &state_val, sizeof(state_val)); offset += sizeof(state_val);
        std::memcpy(&packet.payload[offset], &battery, sizeof(battery)); offset += sizeof(battery);
        std::memcpy(&packet.payload[offset], &loc_health, sizeof(loc_health)); offset += sizeof(loc_health);
        std::memcpy(&packet.payload[offset], &map_v, sizeof(map_v)); offset += sizeof(map_v);
        std::memcpy(&packet.payload[offset], &path_v, sizeof(path_v)); offset += sizeof(path_v);
        std::memcpy(&packet.payload[offset], &now_ms, sizeof(now_ms)); offset += sizeof(now_ms);

        packet.payload_length = offset;
        if (Hal::hal_radio_send(packet)) {
            setTelemetryEvent(TE_SWARM_HEARTBEAT_SENT);
        }
    }

    // Role Failover Watchdog
    if ((now_ms - last_role_check_ms_) >= Config::ROLE_REASSIGN_TIMEOUT_MS) {
        last_role_check_ms_ = now_ms;
        if (getCoordinatorDroneId() == self_drone_id_ && shouldReassignRoles()) {
            Types::DroneRole recommended = getRecommendedRoleForSelf();
            if (recommended != self_role_) {
                self_role_ = recommended;
                setTelemetryEvent(TE_SWARM_ROLE_REASSIGNED);
            }
        }
    }

    // Path Sharing (1000 ms)
    if (active_path.valid && (now_ms - last_path_update_send_ms_) >= Config::PATH_UPDATE_PERIOD_MS) {
        last_path_update_send_ms_ = now_ms;
        broadcastPathUpdate(active_path, now_ms);
    }

    // Human Track Sharing (500 ms)
    if (human_track.human_detected && (now_ms - last_person_update_send_ms_) >= Config::PERSON_UPDATE_PERIOD_MS) {
        last_person_update_send_ms_ = now_ms;
        broadcastPersonUpdate(human_track, now_ms);
    }
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void swarm_comm_init() {
    s_global_swarm_comm.init();
}

void swarm_comm_update(uint32_t now_ms) {
    s_global_swarm_comm.update(now_ms);
}

bool swarm_comm_is_healthy() {
    return s_global_swarm_comm.isSwarmHealthy();
}

uint8_t swarm_comm_get_active_peers() {
    return s_global_swarm_comm.getActivePeerCount();
}

SwarmComm& swarm_comm_get_instance() {
    return s_global_swarm_comm;
}

} // namespace RobofestDrone
