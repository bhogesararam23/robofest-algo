#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

// ============================================================================
// SEARCH INPUTS STRUCT
// ============================================================================

struct SearchInputs {
    uint32_t now_ms = 0;

    Types::DroneRole role = Types::DroneRole::SCOUT_LEFT;
    Types::DroneState drone_state = Types::DroneState::INIT;

    Types::Pose2D pose;
    float altitude_m = 0.0f;
    bool localization_healthy = true;
    bool camera_healthy = true;

    Types::GeofenceStatus geofence_status = Types::GeofenceStatus::GEOFENCE_INSIDE;
    Types::Vec2 geofence_correction;

    uint8_t active_scan_command = 0;
    bool forward_command_active = false;

    uint8_t lane_id = 0;
    bool lane_assigned_by_swarm = false;
    bool swarm_role_failover_active = false;

    uint8_t active_peer_count = 0;
    float nearest_peer_distance_m = 100.0f;
    bool nearest_peer_distance_valid = false;

    uint16_t confirmed_mine_count = 0;
    uint16_t candidate_mine_count = 0;
    bool route_possible = false;
    bool path_planner_requests_more_scan = false;

    Types::SafetyAction safety_action = Types::SafetyAction::CONTINUE;
};


// ============================================================================
// SEARCH BEHAVIOR CLASS
// ============================================================================

class SearchBehavior {
public:
    SearchBehavior();

    void init();
    void reset();

    void update(const SearchInputs& inputs);

    Types::Vec2 getVelocityCommand() const { return velocity_command_; }
    float getTargetX() const { return target_x_; }
    float getTargetY() const { return target_y_; }

    float getCoveragePercent() const { return coverage_percent_; }
    bool isCoverageSufficient() const { return coverage_sufficient_; }
    bool needsMoreScan() const { return needs_more_scan_; }

    uint8_t getActiveLane() const { return active_lane_id_; }
    Types::DroneRole getActiveRole() const { return active_role_; }

    void setRole(Types::DroneRole role);
    void setLaneId(uint8_t lane_id);
    void setActive(bool active);
    bool isActive() const { return active_; }

    void commandScanLeft(uint32_t now_ms);
    void commandScanRight(uint32_t now_ms);
    void commandForward(uint32_t now_ms);
    void clearScanCommand();

    uint8_t getActiveScanCommand() const { return scan_command_; }
    uint32_t getScanCommandExpiryMs() const { return scan_command_expiry_ms_; }

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    void updateCoverage(const Types::Pose2D& pose, uint32_t now_ms);
    bool findNearestUnscannedCell(const Types::Pose2D& pose, Types::Vec2& out_target);
    float getLaneCenterX(uint8_t lane_id) const;
    void setTelemetryEvent(uint16_t event_id);

private:
    Types::Vec2 velocity_command_;
    float target_x_ = 0.0f;
    float target_y_ = 0.0f;

    Types::DroneRole active_role_ = Types::DroneRole::SCOUT_LEFT;
    uint8_t active_lane_id_ = Config::LANE_LEFT;
    bool active_ = false;

    uint8_t scan_command_ = 0;
    uint32_t scan_command_expiry_ms_ = 0;

    uint8_t coverage_grid_[Config::SEARCH_COVERAGE_GRID_COLS * Config::SEARCH_COVERAGE_GRID_ROWS] = {};
    float coverage_percent_ = 0.0f;
    bool coverage_sufficient_ = false;
    bool needs_more_scan_ = true;

    float forward_direction_ = 1.0f;
    uint32_t last_coverage_update_ms_ = 0;
    uint32_t unscanned_search_start_ms_ = 0;
    bool searching_unscanned_cell_ = false;
    Types::Vec2 unscanned_target_;

    uint32_t hold_start_ms_ = 0;

    uint16_t last_telemetry_event_id_ = TE_SEARCH_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void search_behavior_init(Types::DroneRole role);
void search_behavior_update(const SearchInputs& inputs);
Types::Vec2 search_behavior_get_velocity();
bool search_behavior_is_coverage_sufficient();
SearchBehavior& search_behavior_get_instance();

} // namespace RobofestDrone
