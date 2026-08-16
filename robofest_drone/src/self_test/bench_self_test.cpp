#include "bench_self_test.h"
#include "../../hal/hal_system.h"
#include "../../hal/hal_camera.h"
#include "../../hal/hal_tof.h"
#include "../../hal/hal_optical_flow.h"
#include "../../hal/hal_radio.h"
#include "../../hal/hal_storage.h"
#include "../../hal/hal_gpio.h"
#include "../fc_bridge.h"
#include "../telemetry.h"
#include "../../config/mission_config.h"
#include "../../config/thresholds.h"

#include <cstdio>

namespace RobofestDrone {

BenchTestReport run_bench_self_test(SystemContext& ctx, uint32_t now_ms) {
    BenchTestReport report;
    (void)now_ms;

    Hal::hal_log("[BENCH_TEST] Starting Pre-Flight Hardware & Software Diagnostic...");

    // 1. Camera check
    report.camera_ok = Hal::hal_camera_is_healthy();

    // 2. Time-of-Flight rangefinder check (Holybro ST VL53L1X)
    Types::TofSample tof = Hal::hal_tof_read();
    report.tof_ok = (Hal::hal_tof_is_healthy() || (tof.valid && tof.altitude_m < 3.5f));

    // 3. Optical flow sensor check
    Types::OpticalFlowSample flow = Hal::hal_optical_flow_read();
    report.optical_flow_ok = Hal::hal_optical_flow_is_healthy() || flow.valid;

    // 4. Radio P2P transceiver check
    report.radio_ok = Hal::hal_radio_is_healthy();

    // 5. Persistent flash storage check
    report.storage_ok = Hal::hal_storage_is_healthy();

    // 6. Flight Controller UART link & state
    if (ctx.fc_bridge != nullptr) {
        report.fc_link_ok = ctx.fc_bridge->isLinkHealthy();
        report.fc_disarmed_ok = !ctx.fc_bridge->isArmed();
    } else {
        report.fc_link_ok = false;
        report.fc_disarmed_ok = false;
    }

    // 7. Kill switch GPIO readability
    report.kill_switch_ok = Hal::hal_gpio_is_healthy();

    // 8. Battery Voltage Check
    float vbat = (ctx.fc_bridge != nullptr) ? ctx.fc_bridge->getBatteryVoltage() : 15.0f;
    report.battery_voltage_ok = (vbat >= Config::BATTERY_CRITICAL_VOLTAGE);

    // 9. Config Sanity Verification
    bool config_ok = true;
    if (Config::MINE_CLEARANCE_RADIUS_M < 1.0f) config_ok = false;
    if (Config::MISSION_TIME_LIMIT_MS != 600000UL) config_ok = false;
    if (Config::FIELD_LENGTH_M != 60.0f || Config::FIELD_WIDTH_M != 15.0f) config_ok = false;
    report.config_sanity_ok = config_ok;

    // 10. Telemetry Buffer Readiness
    report.telemetry_ok = (ctx.telemetry != nullptr);

    // Overall verdict
    report.overall_passed = (report.storage_ok &&
                             report.kill_switch_ok &&
                             report.config_sanity_ok &&
                             report.telemetry_ok);

    print_bench_test_report(report);

    ctx.self_check_passed = report.overall_passed;
    return report;
}

void print_bench_test_report(const BenchTestReport& report) {
    char buf[128];
    Hal::hal_log("==================== BENCH SELF-TEST REPORT ====================");
    std::snprintf(buf, sizeof(buf), " [1] Camera Sensor:        [%s]", report.camera_ok ? "PASS" : "WARN");
    Hal::hal_log(buf);
    std::snprintf(buf, sizeof(buf), " [2] ToF Ground Range:     [%s]", report.tof_ok ? "PASS" : "WARN");
    Hal::hal_log(buf);
    std::snprintf(buf, sizeof(buf), " [3] Optical Flow Sensor:  [%s]", report.optical_flow_ok ? "PASS" : "WARN");
    Hal::hal_log(buf);
    std::snprintf(buf, sizeof(buf), " [4] Swarm P2P Radio:      [%s]", report.radio_ok ? "PASS" : "WARN");
    Hal::hal_log(buf);
    std::snprintf(buf, sizeof(buf), " [5] Persistent Storage:   [%s]", report.storage_ok ? "PASS" : "FAIL");
    Hal::hal_log(buf);
    std::snprintf(buf, sizeof(buf), " [6] FC Serial UART Link:  [%s]", report.fc_link_ok ? "PASS" : "WARN");
    Hal::hal_log(buf);
    std::snprintf(buf, sizeof(buf), " [7] FC Disarmed State:    [%s]", report.fc_disarmed_ok ? "PASS" : "WARN");
    Hal::hal_log(buf);
    std::snprintf(buf, sizeof(buf), " [8] Kill Switch GPIO:     [%s]", report.kill_switch_ok ? "PASS" : "FAIL");
    Hal::hal_log(buf);
    std::snprintf(buf, sizeof(buf), " [9] Battery Voltage:      [%s]", report.battery_voltage_ok ? "PASS" : "WARN");
    Hal::hal_log(buf);
    std::snprintf(buf, sizeof(buf), " [10] Config Sanity Rules: [%s]", report.config_sanity_ok ? "PASS" : "FAIL");
    Hal::hal_log(buf);
    std::snprintf(buf, sizeof(buf), " [11] Telemetry Logger:    [%s]", report.telemetry_ok ? "PASS" : "FAIL");
    Hal::hal_log(buf);
    Hal::hal_log("----------------------------------------------------------------");
    std::snprintf(buf, sizeof(buf), " >>> OVERALL BENCH VERDICT: [%s] <<<", report.overall_passed ? "PASS (READY)" : "FAIL (BLOCKED)");
    Hal::hal_log(buf);
    Hal::hal_log("================================================================");
}

} // namespace RobofestDrone
