#include <stdint.h>
#include <cmath>
#include "system_state.h"
#include "scheduler.h"
#include "types.h"
#include "state_machine_types.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"

// HAL includes
#include "../hal/hal_system.h"
#include "../hal/hal_camera.h"
#include "../hal/hal_optical_flow.h"
#include "../hal/hal_tof.h"
#include "../hal/hal_radio.h"
#include "../hal/hal_gpio.h"
#include "../hal/hal_serial.h"
#include "../hal/hal_storage.h"
#include "../hal/hal_human.h"
#include "../hal/hal_marker.h"

// Module includes
#include "localization.h"
#include "geofence.h"
#include "vision_pipeline.h"
#include "mine_map.h"
#include "path_planner.h"
#include "swarm_comm.h"
#include "buried_detector.h"
#include "search_behavior.h"
#include "command_layer.h"
#include "human_tracker.h"
#include "marker_controller.h"
#include "safety_manager.h"
#include "fc_bridge.h"
#include "telemetry.h"
#include "state_machine.h"
#include "mission_integration.h"

namespace RobofestDrone {

// ============================================================================
// STATIC SUBSYSTEM INSTANCES & CONTEXT
// ============================================================================

static SystemContext s_system_context;
static Localization s_localization;
static Geofence s_geofence;
static VisionPipeline s_vision_pipeline;
static MineMap s_mine_map;
static PathPlanner s_path_planner;
static SwarmComm s_swarm_comm;
static BuriedDetector s_buried_detector;
static CommandLayer s_command_layer;
static HumanTracker s_human_tracker;
static MarkerController s_marker_controller;
static SafetyManager s_safety_manager;
static FcBridge s_fc_bridge;
static StateMachine s_state_machine;
static SearchBehavior s_search_behavior;
static Telemetry s_telemetry;
static MissionIntegration s_mission_integration;

static Scheduler s_scheduler;


// ============================================================================
// SCHEDULER TASK WRAPPERS (STRICT DEPENDENCY ORDER)
// ============================================================================

void task_read_sensors(uint32_t now_ms) {
    s_mission_integration.readSensors(now_ms);
}

void task_update_localization(uint32_t now_ms) {
    s_mission_integration.updateLocalization(now_ms);
}

void task_update_geofence(uint32_t now_ms) {
    s_mission_integration.updateGeofence(now_ms);
}

void task_update_vision(uint32_t now_ms) {
    s_mission_integration.updateVision(now_ms);
}

void task_update_mine_map(uint32_t now_ms) {
    s_mission_integration.updateMineMap(now_ms);
}

void task_update_path_planner(uint32_t now_ms) {
    s_mission_integration.updatePathPlanner(now_ms);
}

void task_update_swarm(uint32_t now_ms) {
    s_mission_integration.updateSwarm(now_ms);
}

void task_update_command_layer(uint32_t now_ms) {
    s_mission_integration.updateCommandLayer(now_ms);
}

void task_update_human_tracker(uint32_t now_ms) {
    s_mission_integration.updateHumanTracker(now_ms);
}

void task_update_search_behavior(uint32_t now_ms) {
    s_mission_integration.updateSearchBehavior(now_ms);
}

void task_update_safety_manager(uint32_t now_ms) {
    s_mission_integration.updateSafetyManager(now_ms);
}

void task_update_state_machine(uint32_t now_ms) {
    s_mission_integration.updateStateMachine(now_ms);
}

void task_update_marker_controller(uint32_t now_ms) {
    s_mission_integration.updateMarkerController(now_ms);
}

void task_update_flight_commands(uint32_t now_ms) {
    s_mission_integration.updateFlightCommands(now_ms);
}

void task_update_telemetry(uint32_t now_ms) {
    s_mission_integration.updateTelemetry(now_ms);
}


// ============================================================================
// APPLICATION BOOTSTRAP & MAIN INITIALIZATION
// ============================================================================

void robofest_setup() {
    // 1. HAL System Time and Non-Blocking Logging
    Hal::hal_system_init();
    Hal::hal_log("[BOOT] Starting Robofest Minefield Swarm Drone Onboard System...");

    // 2. Hardware Subsystem Initializations
    if (!Hal::hal_gpio_init()) {
        Hal::hal_log("[BOOT][WARN] GPIO initialization warning.");
    }
    if (!Hal::hal_storage_init()) {
        Hal::hal_log("[BOOT][WARN] Storage initialization warning.");
    }
    if (!Hal::hal_serial_init()) {
        Hal::hal_log("[BOOT][WARN] Flight controller serial UART initialization warning.");
    }
    if (!Hal::hal_camera_init()) {
        Hal::hal_log("[BOOT][WARN] Camera sensor initialization warning.");
    }
    if (!Hal::hal_optical_flow_init()) {
        Hal::hal_log("[BOOT][WARN] Optical flow sensor initialization warning.");
    }
    if (!Hal::hal_tof_init()) {
        Hal::hal_log("[BOOT][WARN] ToF altitude sensor initialization warning.");
    }
    if (!Hal::hal_radio_init()) {
        Hal::hal_log("[BOOT][WARN] Swarm P2P radio initialization warning.");
    }
    if (!Hal::hal_human_init()) {
        Hal::hal_log("[BOOT][WARN] Human detection HAL initialization warning.");
    }
    if (!Hal::hal_marker_init()) {
        Hal::hal_log("[BOOT][WARN] Marker output HAL initialization warning.");
    }

    // 3. Wire Subsystem Pointers into SystemContext
    s_system_context.localization = &s_localization;
    s_system_context.geofence = &s_geofence;
    s_system_context.vision_pipeline = &s_vision_pipeline;
    s_system_context.mine_map = &s_mine_map;
    s_system_context.path_planner = &s_path_planner;
    s_system_context.swarm_comm = &s_swarm_comm;
    s_system_context.buried_detector = &s_buried_detector;
    s_system_context.command_layer = &s_command_layer;
    s_system_context.human_tracker = &s_human_tracker;
    s_system_context.marker_controller = &s_marker_controller;
    s_system_context.safety_manager = &s_safety_manager;
    s_system_context.fc_bridge = &s_fc_bridge;
    s_system_context.state_machine = &s_state_machine;
    s_system_context.search_behavior = &s_search_behavior;
    s_system_context.telemetry = &s_telemetry;

    // 4. Initialize Mission Integration & Subsystems in deterministic order
    s_mission_integration.init(s_system_context);

    // 5. Register Cooperative Scheduler Tasks (preserves exact dependency pipeline)
    s_scheduler.init();
    s_scheduler.registerTask("read_sensors",        Config::MAIN_LOOP_PERIOD_MS, task_read_sensors);
    s_scheduler.registerTask("localization",        Config::MAIN_LOOP_PERIOD_MS, task_update_localization);
    s_scheduler.registerTask("geofence",            Config::MAIN_LOOP_PERIOD_MS, task_update_geofence);
    s_scheduler.registerTask("vision_pipeline",     Config::VISION_PERIOD_MS,    task_update_vision);
    s_scheduler.registerTask("mine_map",            Config::MAIN_LOOP_PERIOD_MS, task_update_mine_map);
    s_scheduler.registerTask("path_planner",        50UL,                        task_update_path_planner);
    s_scheduler.registerTask("swarm_comm",          Config::MAIN_LOOP_PERIOD_MS, task_update_swarm);
    s_scheduler.registerTask("command_layer",       Config::MAIN_LOOP_PERIOD_MS, task_update_command_layer);
    s_scheduler.registerTask("human_tracker",       50UL,                        task_update_human_tracker);
    s_scheduler.registerTask("search_behavior",     Config::MAIN_LOOP_PERIOD_MS, task_update_search_behavior);
    s_scheduler.registerTask("safety_manager",      Config::MAIN_LOOP_PERIOD_MS, task_update_safety_manager);
    s_scheduler.registerTask("state_machine",       Config::MAIN_LOOP_PERIOD_MS, task_update_state_machine);
    s_scheduler.registerTask("marker_controller",   50UL,                        task_update_marker_controller);
    s_scheduler.registerTask("flight_commands",     Config::MAIN_LOOP_PERIOD_MS, task_update_flight_commands);
    s_scheduler.registerTask("telemetry",           Config::TELEMETRY_PERIOD_MS, task_update_telemetry);

    Hal::hal_log("[BOOT] System initialization complete. 20ms Main scheduler ready.");
}

void robofest_loop() {
    uint32_t now_ms = Hal::hal_millis();

    // Check master hardware kill switch
    if (Hal::hal_kill_switch_active()) {
        s_system_context.system_state.drone_state = Types::DroneState::EMERGENCY;
        s_system_context.system_state.safety_action = Types::SafetyAction::EMERGENCY_CUT;
        s_fc_bridge.requestEmergencyStop(now_ms);
    }

    // Execute cooperative non-blocking scheduler tasks
    s_scheduler.run(now_ms);
}

} // namespace RobofestDrone

// ============================================================================
// ARDUINO / ESP32 ENTRY POINT COMPATIBILITY WRAPPERS
// ============================================================================

void setup() {
    RobofestDrone::robofest_setup();
}

void loop() {
    RobofestDrone::robofest_loop();
}

#ifndef ARDUINO
// Standard C++ main function for desktop builds, verification, and automated testing
int main() {
    RobofestDrone::robofest_setup();

    // Execute multiple deterministic test cycles to verify non-blocking execution
    for (int cycle = 0; cycle < 50; ++cycle) {
        RobofestDrone::robofest_loop();
    }

    RobofestDrone::Hal::hal_log("[SYSTEM] Main loop test cycles completed successfully.");
    return 0;
}
#endif
