#include "calibration_mode.h"
#include "../../hal/hal_system.h"
#include "../../hal/hal_camera.h"
#include "../../hal/hal_optical_flow.h"
#include "../../hal/hal_tof.h"
#include "../fc_bridge.h"
#include <cstring>
#include <cstdio>

namespace RobofestDrone {

namespace {
    static CalibrationMode s_global_calibration_mode;
}

CalibrationMode::CalibrationMode() {
    reset();
}

void CalibrationMode::init(SystemContext& ctx) {
    ctx_ = &ctx;
    hsv_tuner_.init();
    drift_test_mode_.init();
    substate_ = CalibrationSubstate::IDLE;
    last_status_print_ms_ = 0;

    Hal::hal_log("[CALIBRATION_MODE] Initialized safely. Motor arming is DISABLED.");
}

void CalibrationMode::reset() {
    substate_ = CalibrationSubstate::IDLE;
    hsv_tuner_.reset();
    drift_test_mode_.reset();
    last_status_print_ms_ = 0;
}

void CalibrationMode::setSubstate(CalibrationSubstate substate) {
    substate_ = substate;
    Hal::hal_log("[CALIBRATION_MODE] Substate transitioned.");
}

void CalibrationMode::update(uint32_t now_ms) {
    if (ctx_ == nullptr) return;

    // Safety: Enforce HOLD/DISARM to FC at all times in calibration mode
    if (ctx_->fc_bridge != nullptr) {
        ctx_->fc_bridge->sendHold(now_ms);
    }

    // Process Substates
    if (substate_ == CalibrationSubstate::HSV_TUNING) {
        // Read camera frame if available
        Hal::CameraFrame frame;
        if (Hal::hal_camera_get_frame(frame)) {
            if (frame.data != nullptr && frame.width > 0 && frame.height > 0) {
                hsv_tuner_.processFrame(frame.data, frame.width, frame.height);
            }
        }
        if (now_ms - last_status_print_ms_ >= 2000UL) {
            last_status_print_ms_ = now_ms;
            hsv_tuner_.printStatus();
        }
    } else if (substate_ == CalibrationSubstate::DRIFT_TESTING) {
        if (ctx_->localization != nullptr) {
            // Ingest latest optical flow and ToF for dead-reckoning update
            Types::OpticalFlowSample flow = Hal::hal_optical_flow_read();
            Types::TofSample tof = Hal::hal_tof_read();
            Types::AttitudeSample att = (ctx_->fc_bridge != nullptr) ? ctx_->fc_bridge->getAttitude() : Types::AttitudeSample();

            ctx_->localization->update(flow, tof, att, now_ms);
            drift_test_mode_.update(*ctx_->localization, tof.altitude_m, att, now_ms);
        }
    } else if (substate_ == CalibrationSubstate::FOCAL_IMU_CHECK) {
        if (now_ms - last_status_print_ms_ >= 2000UL) {
            last_status_print_ms_ = now_ms;
            Types::OpticalFlowSample flow = Hal::hal_optical_flow_read();
            Types::TofSample tof = Hal::hal_tof_read();
            Types::AttitudeSample att = (ctx_->fc_bridge != nullptr) ? ctx_->fc_bridge->getAttitude() : Types::AttitudeSample();

            drift_test_mode_.logFocalLengthCheck(
                static_cast<int16_t>(flow.pixel_shift_x),
                static_cast<int16_t>(flow.pixel_shift_y),
                tof.altitude_m,
                20000UL
            );
            drift_test_mode_.logImuBiasCheck(att, now_ms);
        }
    }
}

bool CalibrationMode::handleSerialCommand(const char* cmd_line, uint32_t now_ms) {
    if (cmd_line == nullptr || ctx_ == nullptr) return false;

    if (std::strcmp(cmd_line, "MODE HSV") == 0) {
        setSubstate(CalibrationSubstate::HSV_TUNING);
        return true;
    } else if (std::strcmp(cmd_line, "MODE DRIFT_START") == 0) {
        setSubstate(CalibrationSubstate::DRIFT_TESTING);
        if (ctx_->localization != nullptr) {
            return drift_test_mode_.startTest(*ctx_->localization, now_ms);
        }
        return false;
    } else if (std::strcmp(cmd_line, "MODE DRIFT_END") == 0) {
        if (ctx_->localization != nullptr) {
            drift_test_mode_.endTest(*ctx_->localization, now_ms);
        }
        setSubstate(CalibrationSubstate::IDLE);
        return true;
    } else if (std::strcmp(cmd_line, "MODE FOCAL_IMU") == 0) {
        setSubstate(CalibrationSubstate::FOCAL_IMU_CHECK);
        return true;
    } else if (std::strcmp(cmd_line, "MODE IDLE") == 0) {
        setSubstate(CalibrationSubstate::IDLE);
        return true;
    }

    // Try routing to HSV tuner
    if (substate_ == CalibrationSubstate::HSV_TUNING) {
        return hsv_tuner_.parseCommand(cmd_line);
    }

    return false;
}

CalibrationMode& calibration_mode_get_instance() {
    return s_global_calibration_mode;
}

} // namespace RobofestDrone
