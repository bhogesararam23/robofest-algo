#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "mine_map.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"
#include "../hal/hal_radio.h"

namespace RobofestDrone {

// ============================================================================
// SWARM PEER STRUCTURE
// ============================================================================

struct SwarmPeer {
    uint8_t drone_id = 0;
    bool known = false;
    bool alive = false;
    Types::DroneRole role = Types::DroneRole::RESERVE;
    Types::DroneState state = Types::DroneState::INIT;
    float battery_voltage = 0.0f;
    Types::LocalizationHealth localization_health = Types::LocalizationHealth::LOCALIZATION_GOOD;
    uint32_t last_heartbeat_ms = 0;
    uint32_t last_packet_ms = 0;
    uint32_t map_version = 0;
    uint32_t path_version = 0;
    uint8_t missed_heartbeats = 0;
    uint8_t lane_id = Config::LANE_NONE;
    bool role_assign_pending = false;
};


// ============================================================================
// SWARM COMMUNICATION CLASS
// ============================================================================

class SwarmComm {
public:
    SwarmComm();

    void init();
    void reset();

    void setSelfInfo(uint8_t self_drone_id, Types::DroneRole self_role);

    void update(uint32_t now_ms);

    void updateWithMissionData(
        MineMap& mine_map,
        const Types::SafePath& active_path,
        const Types::HumanTrack& human_track,
        Types::DroneState drone_state,
        uint32_t now_ms
    );

    bool isPeerAlive(uint8_t drone_id) const;
    uint8_t getActivePeerCount() const;
    Types::DroneRole getPeerRole(uint8_t drone_id) const;
    Types::DroneState getPeerState(uint8_t drone_id) const;

    bool isSwarmHealthy() const;
    bool isSwarmDegraded() const;
    bool isSwarmCritical() const;

    bool shouldReassignRoles() const;
    Types::DroneRole getRecommendedRoleForSelf() const;
    Types::DroneRole getRecommendedRoleForPeer(uint8_t drone_id) const;
    uint8_t getLaneIdForRole(Types::DroneRole role) const;

    bool broadcastClaim(const Types::MineRecord& mine, uint32_t now_ms);
    bool broadcastYield(uint16_t mine_hash, uint8_t reason, uint32_t now_ms);
    bool broadcastMineUpdate(const Types::MineRecord& mine, uint32_t now_ms);
    bool broadcastPathUpdate(const Types::SafePath& path, uint32_t now_ms);
    bool broadcastPersonUpdate(const Types::HumanTrack& human_track, uint32_t now_ms);
    bool broadcastHelpRequest(uint8_t reason, uint32_t now_ms);
    bool broadcastLandNow(uint8_t reason, uint32_t now_ms);

    // Cross-drone vision fusion (item 11): share one raw observation.
    bool broadcastVisionObs(const Types::VisionObsPayload& obs, uint32_t now_ms);

    // Periodic sweep: broadcasts not-yet-shared local detections as
    // VISION_OBS packets (rate-limited, few per call). Call from the mission
    // loop alongside updateWithMissionData.
    void pumpVisionObs(MineMap& mine_map, uint32_t now_ms);

    uint32_t getVisionObsSentCount() const { return vision_obs_sent_; }
    uint32_t getVisionObsReceivedCount() const { return vision_obs_received_; }

    bool receivedLandNow() const { return received_land_now_; }
    uint8_t getLandNowReason() const { return land_now_reason_; }

    bool hasNewSharedPath() const { return has_new_shared_path_; }
    bool getSharedPath(Types::SafePath& out_path) const;
    uint32_t getSharedPathVersion() const { return shared_path_.path_version; }

    bool hasNewMineUpdate() const { return has_new_mine_update_; }
    uint32_t getLastMineUpdateVersion() const { return last_mine_update_version_; }

    bool hasNewPersonUpdate() const { return has_new_person_update_; }
    Types::HumanTrack getSharedPersonTrack() const { return shared_person_track_; }

    bool hasHelpRequest() const { return has_help_request_; }
    uint8_t getHelpRequestReason() const { return help_request_reason_; }
    uint8_t getHelpRequestSource() const { return help_request_source_; }

    uint16_t computeMineHash(float x, float y) const;

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    void processIncomingPackets(MineMap* optional_mine_map, uint32_t now_ms);
    void handlePacket(const Types::SwarmPacket& packet, MineMap* optional_mine_map, uint32_t now_ms);
    void handleHeartbeat(const Types::SwarmPacket& packet, uint32_t now_ms);
    void handleRoleAssign(const Types::SwarmPacket& packet, uint32_t now_ms);
    void handleClaim(const Types::SwarmPacket& packet, MineMap* optional_mine_map, uint32_t now_ms);
    void handleYield(const Types::SwarmPacket& packet, uint32_t now_ms);
    void handleMineUpdate(const Types::SwarmPacket& packet, MineMap* optional_mine_map, uint32_t now_ms);
    void handlePathUpdate(const Types::SwarmPacket& packet, uint32_t now_ms);
    void handlePersonUpdate(const Types::SwarmPacket& packet, uint32_t now_ms);
    void handleHelpRequest(const Types::SwarmPacket& packet, uint32_t now_ms);
    void handleLandNow(const Types::SwarmPacket& packet, uint32_t now_ms);
    void handleVisionObs(const Types::SwarmPacket& packet, MineMap* optional_mine_map, uint32_t now_ms);

    void updatePeerStatus(uint32_t now_ms);
    uint8_t getCoordinatorDroneId() const;
    void setTelemetryEvent(uint16_t event_id);

private:
    SwarmPeer peers_[Config::MAX_SWARM_DRONES] = {};
    uint8_t self_drone_id_ = 1;
    Types::DroneRole self_role_ = Types::DroneRole::SCOUT_LEFT;
    Types::DroneState self_state_ = Types::DroneState::INIT;

    Types::SafePath shared_path_;
    bool has_new_shared_path_ = false;
    Types::SafePath path_assembly_buffer_;
    uint8_t received_chunks_mask_ = 0;

    Types::HumanTrack shared_person_track_;
    bool has_new_person_update_ = false;

    bool has_new_mine_update_ = false;
    uint32_t last_mine_update_version_ = 0;

    bool has_help_request_ = false;
    uint8_t help_request_reason_ = 0;
    uint8_t help_request_source_ = 0;

    bool received_land_now_ = false;
    uint8_t land_now_reason_ = 0;

    uint32_t vision_obs_sent_ = 0;
    uint32_t vision_obs_received_ = 0;
    uint32_t last_vision_obs_pump_ms_ = 0;

    uint32_t last_heartbeat_send_ms_ = 0;
    uint32_t last_mine_update_send_ms_ = 0;
    uint32_t last_person_update_send_ms_ = 0;
    uint32_t last_path_update_send_ms_ = 0;
    uint32_t last_role_check_ms_ = 0;
    uint32_t last_peer_cleanup_ms_ = 0;

    uint16_t last_telemetry_event_id_ = TE_SWARM_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void swarm_comm_init();
void swarm_comm_update(uint32_t now_ms);
bool swarm_comm_is_healthy();
uint8_t swarm_comm_get_active_peers();
SwarmComm& swarm_comm_get_instance();

} // namespace RobofestDrone
