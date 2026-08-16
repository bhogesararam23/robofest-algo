#pragma once

#include <stdint.h>
#include "types.h"
#include "system_state.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

// Forward declarations of all subsystem modules
class Localization;
class Geofence;
class VisionPipeline;
class MineMap;
class PathPlanner;
class SwarmComm;
class CommandLayer;
class HumanTracker;
class MarkerController;
class SafetyManager;
class FcBridge;
class StateMachine;
class SearchBehavior;
class Telemetry;

// ============================================================================
// SYSTEM CONTEXT STRUCTURE
// ============================================================================

struct SystemContext {
    Localization* localization = nullptr;
    Geofence* geofence = nullptr;
    VisionPipeline* vision_pipeline = nullptr;
    MineMap* mine_map = nullptr;
    PathPlanner* path_planner = nullptr;
    SwarmComm* swarm_comm = nullptr;
    CommandLayer* command_layer = nullptr;
    HumanTracker* human_tracker = nullptr;
    MarkerController* marker_controller = nullptr;
    SafetyManager* safety_manager = nullptr;
    FcBridge* fc_bridge = nullptr;
    StateMachine* state_machine = nullptr;
    SearchBehavior* search_behavior = nullptr;
    Telemetry* telemetry = nullptr;

    SystemState system_state;
    Types::OpticalFlowSample latest_flow;
    Types::TofSample latest_tof;
    Types::AttitudeSample latest_attitude;
    Types::HumanDetectionSample latest_human_detection;
    Types::SafePath active_path;
    Types::HumanTrack latest_human_track;
    Types::Vec2 latest_geofence_correction;
    Types::SwarmPacket latest_rx_packet;

    bool sensors_read_valid = false;
    bool localization_ready = false;
    bool self_check_passed = false;
    bool calibration_complete = false;
    bool mission_active = false;

    uint32_t now_ms = 0;
};


// ============================================================================
// MISSION INTEGRATION CLASS
// ============================================================================

class MissionIntegration {
public:
    MissionIntegration();

    void init(SystemContext& ctx);
    void reset();

    void runSelfCheck(uint32_t now_ms);
    void runCalibration(uint32_t now_ms);

    void readSensors(uint32_t now_ms);

    void updateLocalization(uint32_t now_ms);
    void updateGeofence(uint32_t now_ms);
    void updateVision(uint32_t now_ms);
    void updateMineMap(uint32_t now_ms);
    void updatePathPlanner(uint32_t now_ms);
    void updateSwarm(uint32_t now_ms);
    void updateCommandLayer(uint32_t now_ms);
    void updateHumanTracker(uint32_t now_ms);
    void updateSearchBehavior(uint32_t now_ms);
    void updateSafetyManager(uint32_t now_ms);
    void updateStateMachine(uint32_t now_ms);
    void updateMarkerController(uint32_t now_ms);
    void updateFlightCommands(uint32_t now_ms);
    void updateTelemetry(uint32_t now_ms);

    void update(uint32_t now_ms);

    bool isSelfCheckPassed() const { return ctx_ != nullptr && ctx_->self_check_passed; }
    bool isCalibrationComplete() const { return ctx_ != nullptr && ctx_->calibration_complete; }
    bool isMissionActive() const { return ctx_ != nullptr && ctx_->mission_active; }

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    bool validateConfiguration() const;
    void setTelemetryEvent(uint16_t event_id);

private:
    SystemContext* ctx_ = nullptr;
    uint32_t last_vision_update_ms_ = 0;
    uint32_t last_path_plan_ms_ = 0;
    uint32_t last_human_track_ms_ = 0;
    uint32_t last_marker_update_ms_ = 0;
    uint32_t last_telemetry_update_ms_ = 0;

    uint32_t calibration_start_ms_ = 0;
    bool calibration_in_progress_ = false;

    uint16_t last_telemetry_event_id_ = TE_INIT_COMPLETE;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void mission_integration_init(SystemContext& ctx);
void mission_integration_update(uint32_t now_ms);
MissionIntegration& mission_integration_get_instance();

} // namespace RobofestDrone
