#pragma once

#include <stdint.h>
#include "../mission_integration.h"
#include "hsv_tuner.h"
#include "drift_test_mode.h"

namespace RobofestDrone {

enum class CalibrationSubstate : uint8_t {
    IDLE = 0,
    HSV_TUNING,
    DRIFT_TESTING,
    FOCAL_IMU_CHECK
};

class CalibrationMode {
public:
    CalibrationMode();

    void init(SystemContext& ctx);
    void reset();

    void update(uint32_t now_ms);

    void setSubstate(CalibrationSubstate substate);
    CalibrationSubstate getSubstate() const { return substate_; }

    HsvTuner& getHsvTuner() { return hsv_tuner_; }
    DriftTestMode& getDriftTestMode() { return drift_test_mode_; }

    // Parse serial string commands for calibration control
    bool handleSerialCommand(const char* cmd_line, uint32_t now_ms);

private:
    SystemContext* ctx_ = nullptr;
    CalibrationSubstate substate_ = CalibrationSubstate::IDLE;

    HsvTuner hsv_tuner_;
    DriftTestMode drift_test_mode_;

    uint32_t last_status_print_ms_ = 0;
};

// Facade
CalibrationMode& calibration_mode_get_instance();

} // namespace RobofestDrone
