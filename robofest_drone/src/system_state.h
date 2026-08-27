#pragma once

#include <stdint.h>
#include "types.h"

namespace RobofestDrone {

struct SystemState {
    Types::DroneState drone_state = Types::DroneState::INIT;
    Types::DroneRole drone_role = Types::DroneRole::SCOUT_LEFT;
    Types::SafetyAction safety_action = Types::SafetyAction::CONTINUE;
    Types::LocalizationHealth localization_health = Types::LocalizationHealth::LOCALIZATION_GOOD;
    Types::GeofenceStatus geofence_status = Types::GeofenceStatus::GEOFENCE_INSIDE;
    Types::Pose2D pose;
    float altitude_m = 0.0f;
    float drift_uncertainty_m = 0.0f;
    float battery_voltage = 0.0f;
    bool armed = false;
    bool kill_switch_active = false;
    bool fc_link_healthy = false;
    bool camera_healthy = false;
    bool optical_flow_healthy = false;
    bool tof_healthy = false;
    bool radio_healthy = false;
    bool localization_healthy = true;
    bool route_possible = false;
    bool path_valid = false;
    bool swarm_healthy = true;
    bool swarm_degraded = false;
    bool swarm_critical = false;
    uint8_t active_peer_count = 0;
    bool human_detected = false;
    bool human_off_path = false;
    bool human_in_exit_zone = false;

    // Moving target landing tracking fields
    bool target_tracked = false;
    float target_field_x = 0.0f;
    float target_field_y = 0.0f;
    float target_velocity_x = 0.0f;
    float target_velocity_y = 0.0f;

    Types::Vec2 search_velocity;
    bool search_coverage_sufficient = false;
    bool search_needs_more_scan = true;
    bool mission_timer_running = false;
    uint32_t mission_start_ms = 0;
    uint32_t current_time_ms = 0;
};

} // namespace RobofestDrone
