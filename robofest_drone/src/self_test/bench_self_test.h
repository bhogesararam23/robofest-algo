#pragma once

#include <stdint.h>
#include "../mission_integration.h"

namespace RobofestDrone {

struct BenchTestReport {
    bool camera_ok = false;
    bool tof_ok = false;
    bool optical_flow_ok = false;
    bool radio_ok = false;
    bool storage_ok = false;
    bool fc_link_ok = false;
    bool fc_disarmed_ok = false;
    bool kill_switch_ok = false;
    bool battery_voltage_ok = false;
    bool config_sanity_ok = false;
    bool telemetry_ok = false;

    bool overall_passed = false;
};

// Run complete hardware and software bench self-test
BenchTestReport run_bench_self_test(SystemContext& ctx, uint32_t now_ms);

// Formatted report printer
void print_bench_test_report(const BenchTestReport& report);

} // namespace RobofestDrone
