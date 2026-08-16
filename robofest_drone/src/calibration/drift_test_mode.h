#pragma once

#include <stdint.h>
#include "../types.h"
#include "../localization.h"

namespace RobofestDrone {

struct DriftTestResults {
    uint32_t elapsed_time_ms = 0;
    float start_x = 0.0f;
    float start_y = 0.0f;
    float final_x = 0.0f;
    float final_y = 0.0f;
    float total_distance_m = 0.0f;
    float loop_closure_drift_m = 0.0f;
    float drift_rate_mps = 0.0f;
    bool completed = false;
};

class DriftTestMode {
public:
    DriftTestMode();

    void init();
    void reset();

    // Start a dead-reckoning drift measurement test
    bool startTest(Localization& loc, uint32_t now_ms);
    void update(Localization& loc, float altitude_m, const Types::AttitudeSample& attitude, uint32_t now_ms);
    void endTest(Localization& loc, uint32_t now_ms);

    bool isRunning() const { return test_active_; }
    const DriftTestResults& getResults() const { return results_; }

    // Sensor Calibration Helpers
    void logFocalLengthCheck(int16_t pixel_shift_x, int16_t pixel_shift_y, float tof_alt_m, uint32_t dt_us);
    void logImuBiasCheck(const Types::AttitudeSample& attitude, uint32_t now_ms);

private:
    bool test_active_ = false;
    uint32_t start_time_ms_ = 0;
    uint32_t last_log_ms_ = 0;
    uint32_t last_imu_log_ms_ = 0;

    Types::Pose2D last_pose_;
    DriftTestResults results_;
};

} // namespace RobofestDrone
