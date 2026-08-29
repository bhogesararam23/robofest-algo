#include "mission_integration.h"
#include "localization.h"
#include "geofence.h"
#include "vision_pipeline.h"
#include "mine_map.h"
#include "buried_detector.h"
#include "path_planner.h"
#include "swarm_comm.h"
#include "command_layer.h"
#include "human_tracker.h"
#include "marker_controller.h"
#include "safety_manager.h"
#include "fc_bridge.h"
#include "state_machine.h"
#include "search_behavior.h"
#include "telemetry.h"

#include "../hal/hal_system.h"
#include "../hal/hal_gpio.h"
#include "../hal/hal_optical_flow.h"
#include "../hal/hal_tof.h"
#include "../hal/hal_human.h"
#include "../hal/hal_marker.h"
#include "../hal/hal_radio.h"

#include <cmath>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static MissionIntegration s_global_mission_integration;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

MissionIntegration::MissionIntegration() {
    reset();
}

void MissionIntegration::reset() {
    ctx_ = nullptr;
    last_vision_update_ms_ = 0;
    last_path_plan_ms_ = 0;
    last_human_track_ms_ = 0;
    last_marker_update_ms_ = 0;
    last_telemetry_update_ms_ = 0;
    calibration_start_ms_ = 0;
    calibration_in_progress_ = false;
    last_telemetry_event_id_ = TE_INIT_COMPLETE;
    telemetry_event_valid_ = true;
}

void MissionIntegration::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}

bool MissionIntegration::validateConfiguration() const {
    if (Config::FIELD_LENGTH_M < 10.0f || Config::FIELD_WIDTH_M < 5.0f) return false;
    if (Config::MINE_CLEARANCE_RADIUS_M < 1.0f) return false;
    if (Config::MISSION_TIME_LIMIT_MS != 600000UL) return false;
    if (Types::MAX_MINES == 0 || Types::MAX_PATH_WAYPOINTS < 2) return false;
    if (Config::SOFTWARE_GEOFENCE_X_MIN <= Config::FIELD_X_MIN) return false;
    if (Config::SOFTWARE_GEOFENCE_X_MAX >= Config::FIELD_X_MAX) return false;
    if (Config::SOFTWARE_GEOFENCE_Y_MIN <= Config::FIELD_Y_MIN) return false;
    if (Config::SOFTWARE_GEOFENCE_Y_MAX >= Config::FIELD_Y_MAX) return false;
    return true;
}

void MissionIntegration::init(SystemContext& ctx) {
    ctx_ = &ctx;

    // 1. Validate arena geometry & configuration parameters
    bool config_valid = validateConfiguration();
    if (!config_valid) {
        setTelemetryEvent(TE_CALIBRATION_FAILED);
        ctx_->self_check_passed = false;
        return;
    }

    // 2. Initialize Telemetry & Logging
    if (ctx_->telemetry != nullptr) {
        ctx_->telemetry->init();
    }

    // 3. Initialize Localization Module
    if (ctx_->localization != nullptr) {
        ctx_->localization->init();
    }

    // 4. Initialize Geofence Module
    if (ctx_->geofence != nullptr) {
        ctx_->geofence->init();
    }

    // 5. Initialize Vision Pipeline Module
    if (ctx_->vision_pipeline != nullptr) {
        ctx_->vision_pipeline->init();
    }

    // 6. Initialize Mine Map Module
    if (ctx_->mine_map != nullptr) {
        ctx_->mine_map->init();
    }

    // 7. Initialize Safe Path Planner Module
    if (ctx_->path_planner != nullptr) {
        ctx_->path_planner->init();
    }

    // Determine initial swarm role based on drone identity
    Types::DroneRole default_role = (Config::DRONE_ID == 1) ? Types::DroneRole::SCOUT_LEFT :
                                    ((Config::DRONE_ID == 2) ? Types::DroneRole::SCOUT_RIGHT :
                                     Types::DroneRole::GUIDE_MARKER);
    ctx_->system_state.drone_role = default_role;

    // 8. Initialize Swarm Communication Module
    if (ctx_->swarm_comm != nullptr) {
        ctx_->swarm_comm->init();
        ctx_->swarm_comm->setSelfInfo(Config::DRONE_ID, default_role);
    }

    // 9. Initialize Command Layer Module
    if (ctx_->command_layer != nullptr) {
        ctx_->command_layer->init();
    }

    // 10. Initialize Human Tracker Module
    if (ctx_->human_tracker != nullptr) {
        ctx_->human_tracker->init();
    }

    // 11. Initialize Marker Controller Module
    if (ctx_->marker_controller != nullptr) {
        ctx_->marker_controller->init();
    }

    // 12. Initialize Safety Manager Module
    if (ctx_->safety_manager != nullptr) {
        ctx_->safety_manager->init();
    }

    // 13. Initialize Flight Controller Bridge Module
    if (ctx_->fc_bridge != nullptr) {
        ctx_->fc_bridge->init();
    }

    // 14. Initialize State Machine Module
    if (ctx_->state_machine != nullptr) {
        ctx_->state_machine->init();
    }

    // 15. Initialize Search Behavior Module
    if (ctx_->search_behavior != nullptr) {
        ctx_->search_behavior->init();
        ctx_->search_behavior->setRole(default_role);
    }

    // Mark self check passed initially; verified in runSelfCheck()
    ctx_->self_check_passed = true;
    setTelemetryEvent(TE_INIT_COMPLETE);
}


// ============================================================================
// SELF-CHECK & CALIBRATION
// ============================================================================

void MissionIntegration::runSelfCheck(uint32_t now_ms) {
    if (ctx_ == nullptr) return;
    (void)now_ms;

    bool pass = true;

    // Check critical HAL subsystems
    if (!Hal::hal_gpio_is_healthy()) pass = false;
    if (ctx_->fc_bridge == nullptr) pass = false;
    if (ctx_->safety_manager == nullptr) pass = false;
    if (ctx_->state_machine == nullptr) pass = false;
    if (ctx_->localization == nullptr) pass = false;

    ctx_->self_check_passed = pass;

    if (pass) {
        setTelemetryEvent(TE_CALIBRATION_PASSED);
    } else {
        setTelemetryEvent(TE_CALIBRATION_FAILED);
    }
}

void MissionIntegration::runCalibration(uint32_t now_ms) {
    if (ctx_ == nullptr) return;

    if (!calibration_in_progress_) {
        calibration_start_ms_ = now_ms;
        calibration_in_progress_ = true;

        if (ctx_->localization != nullptr) ctx_->localization->reset();
        if (ctx_->geofence != nullptr) ctx_->geofence->reset();
        if (ctx_->mine_map != nullptr) ctx_->mine_map->reset();
        if (ctx_->path_planner != nullptr) ctx_->path_planner->reset();
        if (ctx_->human_tracker != nullptr) ctx_->human_tracker->reset();
        if (ctx_->search_behavior != nullptr) ctx_->search_behavior->reset();
        if (ctx_->marker_controller != nullptr) ctx_->marker_controller->reset();
        if (ctx_->fc_bridge != nullptr) ctx_->fc_bridge->sendHold(now_ms);
    }

    // Allow 200 ms for sensor settle
    if (now_ms - calibration_start_ms_ >= 200UL) {
        ctx_->calibration_complete = true;
        calibration_in_progress_ = false;
        setTelemetryEvent(TE_CALIBRATION_COMPLETE);
    }
}


// ============================================================================
// SENSOR ACQUISITION & INTEGRATION PIPELINE
// ============================================================================

void MissionIntegration::readSensors(uint32_t now_ms) {
    if (ctx_ == nullptr) return;
    (void)now_ms;

    ctx_->latest_flow = Hal::hal_optical_flow_read();
    ctx_->latest_tof = Hal::hal_tof_read();

    if (ctx_->fc_bridge != nullptr) {
        ctx_->latest_attitude = ctx_->fc_bridge->getAttitude();
        ctx_->system_state.battery_voltage = ctx_->fc_bridge->getBatteryVoltage();
        ctx_->system_state.armed = ctx_->fc_bridge->isArmed();
        ctx_->system_state.fc_link_healthy = ctx_->fc_bridge->isLinkHealthy();
    }

    Hal::hal_human_read_detection(ctx_->latest_human_detection);
    ctx_->system_state.kill_switch_active = Hal::hal_kill_switch_active();
    ctx_->sensors_read_valid = true;
}

void MissionIntegration::updateLocalization(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->localization == nullptr) return;

    ctx_->localization->update(
        ctx_->latest_flow,
        ctx_->latest_tof,
        ctx_->latest_attitude,
        now_ms
    );

    ctx_->system_state.pose = ctx_->localization->getPose();
    ctx_->system_state.altitude_m = ctx_->localization->getAltitude();
    ctx_->system_state.drift_uncertainty_m = ctx_->localization->getDriftUncertainty();
    ctx_->system_state.localization_health = ctx_->localization->getHealth();
    ctx_->system_state.localization_healthy = ctx_->localization->isLocalizationHealthy();
}

void MissionIntegration::updateGeofence(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->geofence == nullptr) return;

    ctx_->geofence->update(
        ctx_->system_state.pose,
        ctx_->system_state.drift_uncertainty_m,
        now_ms
    );

    ctx_->latest_geofence_correction = ctx_->geofence->correctionVectorToCenter();
    ctx_->system_state.geofence_status = ctx_->geofence->getStatus();
}

void MissionIntegration::updateVision(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->vision_pipeline == nullptr) return;

    // Run at vision period
    if (now_ms - last_vision_update_ms_ < Config::VISION_PERIOD_MS) {
        return;
    }
    last_vision_update_ms_ = now_ms;

    ctx_->vision_pipeline->update(
        ctx_->system_state.pose,
        ctx_->system_state.altitude_m,
        ctx_->latest_attitude,
        now_ms
    );

    ctx_->system_state.camera_healthy = ctx_->vision_pipeline->isHealthy();

    // Feed new candidates into mine map
    if (ctx_->mine_map != nullptr) {
        uint8_t count = ctx_->vision_pipeline->getCandidateCount();
        for (uint8_t i = 0; i < count; ++i) {
            Types::VisionCandidate cand = ctx_->vision_pipeline->getCandidate(i);
            ctx_->mine_map->addDetectionFromCandidate(cand, Config::DRONE_ID, now_ms);
        }
    }

    // Buried-mine sweep (item 13): texture + depth + spectral fusion on the
    // center ROI. Emits low-confidence candidates that need peer votes or a
    // closer pass to confirm.
    if (Config::BURIED_DETECT_ENABLED && ctx_->buried_detector != nullptr &&
        ctx_->mine_map != nullptr &&
        ctx_->vision_pipeline->isHealthy() && !ctx_->vision_pipeline->isNightModeActive()) {
        BuriedAnomaly anomaly = ctx_->buried_detector->update(
            nullptr, 0, 0, nullptr, 0, now_ms); // depth-only feed here; texture
        // arrives via the working buffer hook below when the pipeline exposes it.
        if (anomaly.detected &&
            ctx_->buried_detector->readyToEmit(now_ms)) {
            // Project straight-down offset: anomaly sits at drone ground track.
            Types::VisionCandidate buried;
            buried.world_x = ctx_->system_state.pose.field_x;
            buried.world_y = ctx_->system_state.pose.field_y;
            buried.confidence =
                Config::BURIED_CANDIDATE_CONFIDENCE * anomaly.score;
            buried.marker_type = Types::VisionMarkerType::BURIED_SURFACE_MARKER;
            buried.timestamp_ms = now_ms;
            ctx_->mine_map->addDetectionFromCandidate(
                buried, Config::DRONE_ID, now_ms);
            ctx_->buried_detector->markEmitted(now_ms);
        }
    }
}

void MissionIntegration::updateMineMap(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->mine_map == nullptr) return;

    ctx_->mine_map->setSelfDroneId(Config::DRONE_ID);
    ctx_->mine_map->update(now_ms);
}

void MissionIntegration::updatePathPlanner(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->path_planner == nullptr || ctx_->mine_map == nullptr) return;

    // Run at 20 Hz cadence
    if (now_ms - last_path_plan_ms_ < 50UL) {
        return;
    }
    last_path_plan_ms_ = now_ms;

    Types::DroneState state = (ctx_->state_machine != nullptr) ? ctx_->state_machine->getState() : Types::DroneState::INIT;

    if (state == Types::DroneState::PLANNING) {
        bool found = ctx_->path_planner->computePath(*ctx_->mine_map, now_ms);
        if (found) {
            ctx_->active_path = ctx_->path_planner->getPath();
            ctx_->system_state.path_valid = true;
            ctx_->system_state.route_possible = true;
        } else {
            ctx_->system_state.path_valid = false;
            ctx_->system_state.route_possible = false;
        }
    } else if (state == Types::DroneState::GUIDING) {
        ctx_->system_state.path_valid = ctx_->path_planner->pathStillValid(*ctx_->mine_map, now_ms);
    }
}

void MissionIntegration::updateSwarm(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->swarm_comm == nullptr || ctx_->mine_map == nullptr) return;

    Types::DroneState state = (ctx_->state_machine != nullptr) ? ctx_->state_machine->getState() : Types::DroneState::INIT;

    ctx_->swarm_comm->setSelfInfo(Config::DRONE_ID, ctx_->system_state.drone_role);
    ctx_->swarm_comm->updateWithMissionData(
        *ctx_->mine_map,
        ctx_->active_path,
        ctx_->latest_human_track,
        state,
        now_ms
    );

    // Cross-drone vision fusion (item 11): broadcast fresh local detections
    // as VISION_OBS so peers can distance-weight and vote on them.
    ctx_->swarm_comm->pumpVisionObs(*ctx_->mine_map, now_ms);

    ctx_->system_state.swarm_healthy = ctx_->swarm_comm->isSwarmHealthy();
    ctx_->system_state.swarm_degraded = ctx_->swarm_comm->isSwarmDegraded();
    ctx_->system_state.swarm_critical = ctx_->swarm_comm->isSwarmCritical();
    ctx_->system_state.active_peer_count = ctx_->swarm_comm->getActivePeerCount();

    // Check role failover
    if (ctx_->swarm_comm->shouldReassignRoles()) {
        ctx_->system_state.drone_role = ctx_->swarm_comm->getRecommendedRoleForSelf();
        if (ctx_->search_behavior != nullptr) {
            ctx_->search_behavior->setRole(ctx_->system_state.drone_role);
        }
    }
}

void MissionIntegration::updateCommandLayer(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->command_layer == nullptr) return;

    Types::DroneState state = (ctx_->state_machine != nullptr) ? ctx_->state_machine->getState() : Types::DroneState::INIT;
    ctx_->command_layer->setSystemState(state);
    ctx_->command_layer->update(now_ms);

    if (ctx_->command_layer->isCommandValid()) {
        Types::CommandType cmd = ctx_->command_layer->getLatestCommand();
        if (cmd == Types::CommandType::SCAN_LEFT && ctx_->search_behavior != nullptr) {
            ctx_->search_behavior->commandScanLeft(now_ms);
        } else if (cmd == Types::CommandType::SCAN_RIGHT && ctx_->search_behavior != nullptr) {
            ctx_->search_behavior->commandScanRight(now_ms);
        }
        ctx_->command_layer->consumeCommand();
    }
}

void MissionIntegration::updateHumanTracker(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->human_tracker == nullptr) return;

    if (now_ms - last_human_track_ms_ < 50UL) {
        return;
    }
    last_human_track_ms_ = now_ms;

    ctx_->human_tracker->update(
        ctx_->latest_human_detection,
        ctx_->active_path,
        ctx_->system_state.pose,
        ctx_->system_state.altitude_m,
        ctx_->latest_attitude,
        now_ms
    );

    ctx_->latest_human_track = ctx_->human_tracker->getTrack();
    ctx_->system_state.human_detected = ctx_->latest_human_track.human_detected;
    ctx_->system_state.human_off_path = ctx_->latest_human_track.human_off_path;
    ctx_->system_state.human_in_exit_zone = ctx_->latest_human_track.human_in_exit_zone;

    if (ctx_->system_state.human_detected) {
        ctx_->system_state.target_tracked = true;
        ctx_->system_state.target_field_x = ctx_->latest_human_track.field_x;
        ctx_->system_state.target_field_y = ctx_->latest_human_track.field_y;
        ctx_->system_state.target_velocity_x = ctx_->latest_human_track.velocity_x;
        ctx_->system_state.target_velocity_y = ctx_->latest_human_track.velocity_y;
    } else {
        ctx_->system_state.target_tracked = false;
    }
}

void MissionIntegration::updateSearchBehavior(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->search_behavior == nullptr || ctx_->mine_map == nullptr) return;

    Types::DroneState state = (ctx_->state_machine != nullptr) ? ctx_->state_machine->getState() : Types::DroneState::INIT;

    SearchInputs inputs;
    inputs.now_ms = now_ms;
    inputs.role = ctx_->system_state.drone_role;
    inputs.drone_state = state;
    inputs.pose = ctx_->system_state.pose;
    inputs.altitude_m = ctx_->system_state.altitude_m;
    inputs.localization_healthy = ctx_->system_state.localization_healthy;
    inputs.camera_healthy = ctx_->system_state.camera_healthy;
    inputs.geofence_status = ctx_->system_state.geofence_status;
    inputs.geofence_correction = ctx_->latest_geofence_correction;
    inputs.active_scan_command = ctx_->search_behavior->getActiveScanCommand();
    inputs.active_peer_count = ctx_->system_state.active_peer_count;
    inputs.confirmed_mine_count = ctx_->mine_map->getConfirmedCount();
    inputs.candidate_mine_count = ctx_->mine_map->getCandidateCount();
    inputs.route_possible = ctx_->system_state.route_possible;
    inputs.path_planner_requests_more_scan = (ctx_->path_planner != nullptr && ctx_->path_planner->needsMoreScan());
    inputs.safety_action = ctx_->system_state.safety_action;

    ctx_->search_behavior->update(inputs);

    ctx_->system_state.search_velocity = ctx_->search_behavior->getVelocityCommand();
    ctx_->system_state.search_coverage_sufficient = ctx_->search_behavior->isCoverageSufficient();
    ctx_->system_state.search_needs_more_scan = ctx_->search_behavior->needsMoreScan();
}

void MissionIntegration::updateSafetyManager(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->safety_manager == nullptr) return;

    Types::SafetyInputs inputs;
    inputs.now_ms = now_ms;
    inputs.kill_switch_active = ctx_->system_state.kill_switch_active;
    inputs.battery_present = (ctx_->system_state.battery_voltage > 5.0f);
    inputs.battery_voltage = ctx_->system_state.battery_voltage;
    inputs.fc_link_healthy = ctx_->system_state.fc_link_healthy;
    inputs.fc_armed = ctx_->system_state.armed;
    inputs.drone_state = (ctx_->state_machine != nullptr) ? ctx_->state_machine->getState() : Types::DroneState::INIT;
    inputs.mission_timer_running = ctx_->mission_active;
    inputs.mission_elapsed_ms = (ctx_->state_machine != nullptr) ? ctx_->state_machine->getMissionElapsedMs() : 0;
    inputs.localization_health = ctx_->system_state.localization_health;
    inputs.drift_uncertainty_m = ctx_->system_state.drift_uncertainty_m;
    inputs.altitude_m = ctx_->system_state.altitude_m;
    inputs.camera_healthy = ctx_->system_state.camera_healthy;
    inputs.optical_flow_healthy = ctx_->latest_flow.valid;
    inputs.tof_healthy = ctx_->latest_tof.valid;
    inputs.radio_healthy = Hal::hal_radio_is_healthy();
    inputs.swarm_healthy = ctx_->system_state.swarm_healthy;
    inputs.swarm_degraded = ctx_->system_state.swarm_degraded;
    inputs.swarm_critical = ctx_->system_state.swarm_critical;
    inputs.active_peer_count = ctx_->system_state.active_peer_count;
    inputs.human_detected = ctx_->system_state.human_detected;
    inputs.nearest_human_distance_m = ctx_->latest_human_track.distance_to_drone_m;
    inputs.nearest_human_distance_valid = ctx_->latest_human_track.human_detected;
    inputs.geofence_status = ctx_->system_state.geofence_status;
    inputs.path_valid = ctx_->system_state.path_valid;

    ctx_->safety_manager->update(inputs);
    ctx_->system_state.safety_action = ctx_->safety_manager->recommendedAction();
}

void MissionIntegration::updateStateMachine(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->state_machine == nullptr) return;

    StateMachineInputs inputs;
    inputs.now_ms = now_ms;
    inputs.self_check_passed = ctx_->self_check_passed;
    inputs.calibration_complete = ctx_->calibration_complete;
    inputs.calibration_failed = !ctx_->self_check_passed;

    if (ctx_->command_layer != nullptr) {
        inputs.latest_command = ctx_->command_layer->getLatestCommand();
        inputs.command_valid = ctx_->command_layer->isCommandValid();
        inputs.command_confidence = ctx_->command_layer->getLatestConfidence();
    }

    inputs.kill_switch_active = ctx_->system_state.kill_switch_active;
    inputs.fc_link_healthy = ctx_->system_state.fc_link_healthy;
    inputs.fc_armed = ctx_->system_state.armed;

    if (ctx_->safety_manager != nullptr) {
        inputs.battery_low = ctx_->safety_manager->isBatteryLow();
        inputs.battery_critical = ctx_->safety_manager->isBatteryCritical();
        inputs.start_command_allowed = ctx_->safety_manager->shouldAllowTakeoff();
    }

    inputs.localization_health = ctx_->system_state.localization_health;
    inputs.current_altitude_m = ctx_->system_state.altitude_m;
    inputs.target_altitude_m = Config::MISSION_ALTITUDE_M;
    inputs.takeoff_complete = (ctx_->system_state.altitude_m >= Config::MISSION_ALTITUDE_M * 0.9f);

    inputs.peers_healthy = ctx_->system_state.swarm_healthy;
    inputs.roles_confirmed = (ctx_->system_state.active_peer_count >= (Config::MIN_SWARM_DRONES - 1));
    inputs.swarm_degraded = ctx_->system_state.swarm_degraded;
    inputs.swarm_critical = ctx_->system_state.swarm_critical;

    inputs.search_coverage_sufficient = ctx_->system_state.search_coverage_sufficient;
    inputs.route_possible = ctx_->system_state.route_possible;
    inputs.path_valid = ctx_->system_state.path_valid;
    inputs.human_off_path = ctx_->system_state.human_off_path;
    inputs.human_in_exit_zone = ctx_->system_state.human_in_exit_zone;
    inputs.mission_complete_confirmed = (ctx_->system_state.human_in_exit_zone && ctx_->system_state.path_valid);

    inputs.safety_action = ctx_->system_state.safety_action;

    ctx_->state_machine->update(inputs, now_ms);

    ctx_->system_state.drone_state = ctx_->state_machine->getState();
    ctx_->system_state.mission_timer_running = ctx_->state_machine->isMissionTimerRunning();
    ctx_->mission_active = ctx_->system_state.mission_timer_running;
}

void MissionIntegration::updateMarkerController(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->marker_controller == nullptr) return;

    if (now_ms - last_marker_update_ms_ < Config::MARKER_GUIDANCE_UPDATE_MS) {
        return;
    }
    last_marker_update_ms_ = now_ms;

    Types::DroneState state = (ctx_->state_machine != nullptr) ? ctx_->state_machine->getState() : Types::DroneState::INIT;

    ctx_->marker_controller->update(
        state,
        ctx_->active_path,
        ctx_->latest_human_track,
        ctx_->system_state.safety_action,
        now_ms
    );
}

void MissionIntegration::updateFlightCommands(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->fc_bridge == nullptr) return;

    // Priority 1: Emergency Stop Cut
    if (ctx_->system_state.safety_action == Types::SafetyAction::EMERGENCY_CUT ||
        ctx_->system_state.kill_switch_active) {
        ctx_->fc_bridge->requestEmergencyStop(now_ms);
        return;
    }

    // Priority 2: Forced Landing
    if (ctx_->system_state.safety_action == Types::SafetyAction::LAND) {
        ctx_->fc_bridge->sendLand(now_ms);
        return;
    }

    // Priority 3: Safety Hold
    if (ctx_->system_state.safety_action == Types::SafetyAction::HOLD) {
        ctx_->fc_bridge->sendHold(now_ms);
        return;
    }

    // Priority 4: State Machine Setpoint Dispatch
    Types::DroneState state = (ctx_->state_machine != nullptr) ? ctx_->state_machine->getState() : Types::DroneState::INIT;

    switch (state) {
        case Types::DroneState::INIT:
        case Types::DroneState::CALIBRATE:
        case Types::DroneState::WAIT_FOR_START:
        case Types::DroneState::HOLD:
        case Types::DroneState::PLANNING:
        case Types::DroneState::MISSION_COMPLETE:
            ctx_->fc_bridge->sendHold(now_ms);
            break;

        case Types::DroneState::TAKEOFF:
            ctx_->fc_bridge->sendTakeoff(Config::MISSION_ALTITUDE_M, now_ms);
            break;

        case Types::DroneState::FORMATION:
        case Types::DroneState::SEARCHING:
            ctx_->fc_bridge->sendVelocityCommand(
                ctx_->system_state.search_velocity,
                Config::MISSION_ALTITUDE_M,
                0.0f,
                now_ms
            );
            break;

        case Types::DroneState::GUIDING:
            if (ctx_->system_state.human_detected && ctx_->system_state.path_valid && !ctx_->system_state.human_off_path) {
                // Gentle forward guiding velocity along path
                ctx_->fc_bridge->sendVelocityCommand(
                    Types::Vec2(0.0f, 0.35f),
                    Config::MISSION_ALTITUDE_M,
                    0.0f,
                    now_ms
                );
            } else {
                ctx_->fc_bridge->sendHold(now_ms);
            }
            break;

        case Types::DroneState::LANDING:
            if (ctx_->system_state.target_tracked) {
                // Predictive Visual Servoing using field coordinates and target velocity
                float target_speed = std::sqrt(ctx_->system_state.target_velocity_x * ctx_->system_state.target_velocity_x + 
                                               ctx_->system_state.target_velocity_y * ctx_->system_state.target_velocity_y);
                
                // Dynamic descent rate calculation (shallow approach for fast targets)
                float base_descent = Config::LANDING_ALTITUDE_STEP_M;
                float closure = std::min(1.5f, std::max(0.3f, ctx_->system_state.altitude_m / 2.0f));
                float speed_penalty = 1.0f / (1.0f + target_speed);
                float descent_rate = base_descent * closure * speed_penalty;

                // Time To Impact (TTI)
                float tti_s = ctx_->system_state.altitude_m / std::max(0.1f, descent_rate);

                // Predictive intercept coordinates
                float predict_x = ctx_->system_state.target_field_x + ctx_->system_state.target_velocity_x * std::min(2.0f, tti_s);
                float predict_y = ctx_->system_state.target_field_y + ctx_->system_state.target_velocity_y * std::min(2.0f, tti_s);

                float err_x = predict_x - ctx_->system_state.pose.field_x;
                float err_y = predict_y - ctx_->system_state.pose.field_y;
                
                // Servoing gain (P-controller)
                const float kp = 0.5f; 
                float vx = err_x * kp;
                float vy = err_y * kp;

                // Cap lateral velocities
                const float cap = 0.5f;
                float mag = std::sqrt(vx * vx + vy * vy);
                if (mag > cap) {
                    vx *= (cap / mag);
                    vy *= (cap / mag);
                }
                
                // Set altitude target slightly below current to force descent at the calculated rate
                float target_altitude_m = std::max(0.0f, ctx_->system_state.altitude_m - descent_rate);

                // If we are very close to the ground, fallback to raw sendLand
                if (ctx_->system_state.altitude_m <= 0.3f) {
                    ctx_->fc_bridge->sendLand(now_ms);
                } else {
                    ctx_->fc_bridge->sendVelocityCommand(Types::Vec2(vx, vy), target_altitude_m, ctx_->system_state.pose.yaw_deg, now_ms);
                }
            } else {
                ctx_->fc_bridge->sendLand(now_ms);
            }
            break;

        case Types::DroneState::DISARMED:
            ctx_->fc_bridge->requestDisarm(now_ms);
            break;

        case Types::DroneState::EMERGENCY:
        default:
            ctx_->fc_bridge->requestEmergencyStop(now_ms);
            break;
    }
}

void MissionIntegration::updateTelemetry(uint32_t now_ms) {
    if (ctx_ == nullptr || ctx_->telemetry == nullptr || ctx_->mine_map == nullptr) return;

    if (now_ms - last_telemetry_update_ms_ < Config::TELEMETRY_PERIOD_MS) {
        return;
    }
    last_telemetry_update_ms_ = now_ms;

    Types::TelemetryInputs inputs;
    inputs.now_ms = now_ms;
    inputs.drone_state = (ctx_->state_machine != nullptr) ? ctx_->state_machine->getState() : Types::DroneState::INIT;
    inputs.safety_action = ctx_->system_state.safety_action;
    inputs.localization_health = ctx_->system_state.localization_health;
    inputs.geofence_status = ctx_->system_state.geofence_status;
    inputs.battery_voltage = ctx_->system_state.battery_voltage;
    inputs.drift_uncertainty_m = ctx_->system_state.drift_uncertainty_m;
    inputs.altitude_m = ctx_->system_state.altitude_m;
    inputs.confirmed_mine_count = ctx_->mine_map->getConfirmedCount();
    inputs.candidate_mine_count = ctx_->mine_map->getCandidateCount();
    inputs.path_valid = ctx_->system_state.path_valid;
    inputs.path_version = (ctx_->path_planner != nullptr) ? ctx_->path_planner->getPathVersion() : 0;
    inputs.human_detected = ctx_->system_state.human_detected;
    inputs.human_off_path = ctx_->system_state.human_off_path;
    inputs.human_in_exit_zone = ctx_->system_state.human_in_exit_zone;
    inputs.swarm_healthy = ctx_->system_state.swarm_healthy;
    inputs.swarm_degraded = ctx_->system_state.swarm_degraded;
    inputs.swarm_critical = ctx_->system_state.swarm_critical;
    inputs.active_peer_count = ctx_->system_state.active_peer_count;
    inputs.storage_healthy = ctx_->telemetry->isStorageHealthy();

    ctx_->telemetry->update(inputs);
}


// ============================================================================
// MAIN SCHEDULER DISPATCH
// ============================================================================

void MissionIntegration::update(uint32_t now_ms) {
    if (ctx_ == nullptr) return;
    ctx_->now_ms = now_ms;

    // Check state-specific calibration/self-check phases
    Types::DroneState state = (ctx_->state_machine != nullptr) ? ctx_->state_machine->getState() : Types::DroneState::INIT;

    if (state == Types::DroneState::INIT) {
        runSelfCheck(now_ms);
    } else if (state == Types::DroneState::CALIBRATE) {
        runCalibration(now_ms);
    }

    // Master deterministic 14-step dependency-ordered pipeline
    readSensors(now_ms);
    updateLocalization(now_ms);
    updateGeofence(now_ms);
    updateVision(now_ms);
    updateMineMap(now_ms);
    updatePathPlanner(now_ms);
    updateSwarm(now_ms);
    updateCommandLayer(now_ms);
    updateHumanTracker(now_ms);
    updateSearchBehavior(now_ms);
    updateSafetyManager(now_ms);
    updateStateMachine(now_ms);
    updateMarkerController(now_ms);
    updateFlightCommands(now_ms);
    updateTelemetry(now_ms);
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void mission_integration_init(SystemContext& ctx) {
    s_global_mission_integration.init(ctx);
}

void mission_integration_update(uint32_t now_ms) {
    s_global_mission_integration.update(now_ms);
}

MissionIntegration& mission_integration_get_instance() {
    return s_global_mission_integration;
}

} // namespace RobofestDrone
