#include "localization.h"
#include <cmath>

namespace RobofestDrone {

namespace {
    static Localization s_global_localization;
    constexpr float M_PI_F = 3.14159265358979323846f;
    constexpr float DEG_TO_RAD = M_PI_F / 180.0f;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

Localization::Localization() {
    reset();
}

void Localization::init() {
    reset();
}

void Localization::reset() {
    local_x_ = 0.0f;
    local_y_ = 0.0f;
    velocity_x_ = 0.0f;
    velocity_y_ = 0.0f;
    fused_altitude_ = 0.0f;
    flow_altitude_estimate_ = 0.0f;
    tof_altitude_raw_ = 0.0f;
    altitude_bias_ = Config::ALTITUDE_BIAS_DEFAULT_M;
    drift_x_ = 0.0f;
    drift_y_ = 0.0f;
    drift_uncertainty_ = 0.0f;
    focal_length_px_ = Config::OPTICAL_FLOW_FOCAL_LENGTH_PX;
    flow_offset_x_ = Config::FLOW_ALIGNMENT_X_DEFAULT;
    flow_offset_y_ = Config::FLOW_ALIGNMENT_Y_DEFAULT;
    flow_quality_threshold_ = Config::FLOW_QUALITY_MIN;

    takeoff_field_x_ = Config::TAKEOFF_FIELD_X;
    takeoff_field_y_ = Config::TAKEOFF_FIELD_Y;

    current_yaw_deg_ = 0.0f;

    flow_usable_ = false;
    tof_usable_ = false;
    initialized_ = true;
    calibration_active_ = false;

    health_ = Types::LocalizationHealth::LOCALIZATION_GOOD;

    last_update_ms_ = 0;
    last_valid_flow_ms_ = 0;
    last_valid_tof_ms_ = 0;
    last_telemetry_event_id_ = TE_LOCALIZATION_INITIALIZED;

    calibration_altitude_sum_ = 0.0f;
    calibration_sample_count_ = 0;
}

void Localization::resetOrigin() {
    local_x_ = 0.0f;
    local_y_ = 0.0f;
    velocity_x_ = 0.0f;
    velocity_y_ = 0.0f;
    drift_x_ = 0.0f;
    drift_y_ = 0.0f;
    drift_uncertainty_ = 0.0f;
    health_ = Types::LocalizationHealth::LOCALIZATION_DEGRADED;
}


// ============================================================================
// CONFIGURATION SETTERS
// ============================================================================

void Localization::setTakeoffFieldOffset(float field_x, float field_y) {
    takeoff_field_x_ = field_x;
    takeoff_field_y_ = field_y;
}

void Localization::setFocalLengthPx(float focal_length_px) {
    if (focal_length_px > 0.0f) {
        focal_length_px_ = focal_length_px;
    }
}

void Localization::setAltitudeBias(float bias_m) {
    altitude_bias_ = bias_m;
}

void Localization::setFlowAlignmentOffset(float x_offset, float y_offset) {
    flow_offset_x_ = x_offset;
    flow_offset_y_ = y_offset;
}

void Localization::setFlowQualityThreshold(float threshold) {
    if (threshold >= 0.0f && threshold <= 1.0f) {
        flow_quality_threshold_ = threshold;
    }
}


// ============================================================================
// CALIBRATION INTERFACE (NON-BLOCKING)
// ============================================================================

void Localization::beginCalibration() {
    calibration_active_ = true;
    calibration_altitude_sum_ = 0.0f;
    calibration_sample_count_ = 0;
    setTelemetryEvent(TE_CALIBRATION_STARTED);
}

void Localization::addCalibrationSample(float measured_altitude_m) {
    if (calibration_active_ && measured_altitude_m >= 0.0f) {
        calibration_altitude_sum_ += measured_altitude_m;
        calibration_sample_count_++;
    }
}

bool Localization::finishCalibration() {
    if (!calibration_active_) {
        return false;
    }

    calibration_active_ = false;
    if (calibration_sample_count_ >= 5) {
        altitude_bias_ = calibration_altitude_sum_ / static_cast<float>(calibration_sample_count_);
        setTelemetryEvent(TE_CALIBRATION_COMPLETE);
        return true;
    }

    setTelemetryEvent(TE_CALIBRATION_FAILED);
    return false;
}


// ============================================================================
// TELEMETRY HELPER
// ============================================================================

void Localization::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
}


// ============================================================================
// ALTITUDE FUSION
// ============================================================================

void Localization::updateAltitude(const Types::TofSample& tof_sample, uint32_t now_ms) {
    if (tof_sample.valid && tof_sample.altitude_m >= 0.0f) {
        tof_altitude_raw_ = tof_sample.altitude_m - altitude_bias_;
        if (tof_altitude_raw_ < 0.0f) {
            tof_altitude_raw_ = 0.0f;
        }

        if (!tof_usable_) {
            // First valid reading after timeout / initialization
            fused_altitude_ = tof_altitude_raw_;
        } else {
            // Complementary filter fusion
            fused_altitude_ = Config::TOF_ALTITUDE_FILTER_ALPHA * tof_altitude_raw_ +
                              (1.0f - Config::TOF_ALTITUDE_FILTER_ALPHA) * fused_altitude_;
        }

        tof_usable_ = true;
        last_valid_tof_ms_ = now_ms;
    } else {
        if ((now_ms - last_valid_tof_ms_) > Config::TOF_TIMEOUT_MS) {
            tof_usable_ = false;
            setTelemetryEvent(TE_TOF_TIMEOUT);
        }
    }
}


// ============================================================================
// OPTICAL FLOW DEAD RECKONING
// ============================================================================

void Localization::updateOpticalFlow(
    const Types::OpticalFlowSample& flow_sample,
    const Types::AttitudeSample& attitude_sample,
    float dt,
    uint32_t now_ms
) {
    if (focal_length_px_ <= 0.0f || dt <= 0.0f) {
        flow_usable_ = false;
        velocity_x_ = 0.0f;
        velocity_y_ = 0.0f;
        return;
    }

    // 1. Check for extreme or corrupted pixel shifts
    if (std::abs(flow_sample.pixel_shift_x) > Config::MAX_PIXEL_SHIFT_PER_UPDATE_PX ||
        std::abs(flow_sample.pixel_shift_y) > Config::MAX_PIXEL_SHIFT_PER_UPDATE_PX) {
        flow_usable_ = false;
        velocity_x_ = 0.0f;
        velocity_y_ = 0.0f;
        setTelemetryEvent(TE_FLOW_REJECTED_BAD_SHIFT);
        drift_uncertainty_ += 0.05f;
        return;
    }

    // 2. Check attitude tilt angle for optical flow perspective distortion
    bool excessive_tilt = false;
    if (attitude_sample.valid) {
        if (std::abs(attitude_sample.roll_deg) > Config::MAX_TILT_ANGLE_DEG ||
            std::abs(attitude_sample.pitch_deg) > Config::MAX_TILT_ANGLE_DEG) {
            excessive_tilt = true;
        }
    }

    // 3. Flow usability evaluation
    bool is_usable = flow_sample.valid &&
                     (flow_sample.quality >= flow_quality_threshold_) &&
                     (fused_altitude_ >= Config::MIN_FLOW_ALTITUDE_M) &&
                     !excessive_tilt;

    if (is_usable) {
        flow_usable_ = true;
        last_valid_flow_ms_ = now_ms;

        // Apply alignment offset corrections
        float corrected_shift_x = flow_sample.pixel_shift_x - flow_offset_x_;
        float corrected_shift_y = flow_sample.pixel_shift_y - flow_offset_y_;

        // Compute body-frame translational velocities (m/s)
        float v_body_x = (corrected_shift_x / focal_length_px_) * (fused_altitude_ / dt);
        float v_body_y = (corrected_shift_y / focal_length_px_) * (fused_altitude_ / dt);

        // Convert body-frame velocity to world/field-frame using heading yaw
        float yaw_rad = current_yaw_deg_ * DEG_TO_RAD;
        float cos_yaw = std::cos(yaw_rad);
        float sin_yaw = std::sin(yaw_rad);

        velocity_x_ = v_body_x * cos_yaw - v_body_y * sin_yaw;
        velocity_y_ = v_body_x * sin_yaw + v_body_y * cos_yaw;

        // Integrate world-frame displacements into takeoff-relative local position
        local_x_ += velocity_x_ * dt;
        local_y_ += velocity_y_ * dt;
    } else {
        flow_usable_ = false;
        velocity_x_ = 0.0f;
        velocity_y_ = 0.0f;

        if (flow_sample.valid && flow_sample.quality < flow_quality_threshold_) {
            setTelemetryEvent(TE_LOW_FLOW_QUALITY);
        }

        if ((now_ms - last_valid_flow_ms_) > Config::FLOW_TIMEOUT_MS) {
            setTelemetryEvent(TE_OPTICAL_FLOW_TIMEOUT);
        }
    }
}


// ============================================================================
// DRIFT UNCERTAINTY & HEALTH EVALUATION
// ============================================================================

void Localization::updateDriftAndHealth(float dt, uint32_t now_ms) {
    // 1. Dynamic drift uncertainty model
    if (flow_usable_ && tof_usable_) {
        drift_uncertainty_ -= Config::DRIFT_RECOVERY_RATE_MPS * dt;
        if (drift_uncertainty_ < 0.0f) {
            drift_uncertainty_ = 0.0f;
        }
    } else {
        drift_uncertainty_ += Config::DRIFT_GROWTH_RATE_MPS * dt;
        if (drift_uncertainty_ > Config::DRIFT_UNCERTAINTY_MAX_M) {
            drift_uncertainty_ = Config::DRIFT_UNCERTAINTY_MAX_M;
        }
    }

    if (drift_uncertainty_ >= Config::DRIFT_UNCERTAINTY_LIMIT_M) {
        setTelemetryEvent(TE_DRIFT_UNCERTAINTY_HIGH);
    }

    // 2. Health classification
    bool flow_timeout = (now_ms - last_valid_flow_ms_) > Config::FLOW_TIMEOUT_MS;
    bool tof_timeout  = (now_ms - last_valid_tof_ms_) > Config::TOF_TIMEOUT_MS;

    if (!initialized_ || drift_uncertainty_ >= Config::DRIFT_UNCERTAINTY_LIMIT_M || (flow_timeout && tof_timeout)) {
        health_ = Types::LocalizationHealth::LOCALIZATION_UNRECOVERABLE;
        setTelemetryEvent(TE_LOCALIZATION_UNRECOVERABLE);
    } else if (!flow_usable_ || !tof_usable_ || drift_uncertainty_ > Config::DRIFT_UNCERTAINTY_RECOVERED_M) {
        health_ = Types::LocalizationHealth::LOCALIZATION_DEGRADED;
        setTelemetryEvent(TE_LOCALIZATION_DEGRADED);
    } else {
        health_ = Types::LocalizationHealth::LOCALIZATION_GOOD;
        setTelemetryEvent(TE_LOCALIZATION_GOOD);
    }
}


// ============================================================================
// MAIN UPDATE FUNCTION
// ============================================================================

void Localization::update(
    const Types::OpticalFlowSample& flow_sample,
    const Types::TofSample& tof_sample,
    const Types::AttitudeSample& attitude_sample,
    uint32_t now_ms
) {
    if (last_update_ms_ == 0) {
        last_update_ms_ = now_ms;
        last_valid_flow_ms_ = now_ms;
        last_valid_tof_ms_ = now_ms;
        if (tof_sample.valid) {
            updateAltitude(tof_sample, now_ms);
        }
        return;
    }

    uint32_t dt_ms = now_ms - last_update_ms_;
    if (dt_ms == 0) {
        // Prevent division by zero if called multiple times in same millisecond
        return;
    }

    float dt = 0.0f;
    if (dt_ms > Config::MAX_VALID_DT_MS) {
        setTelemetryEvent(TE_FLOW_REJECTED_BAD_DT);
        drift_uncertainty_ += 0.10f;
        dt = static_cast<float>(Config::MAIN_LOOP_PERIOD_MS) / 1000.0f; // Safe fallback dt
    } else {
        dt = static_cast<float>(dt_ms) / 1000.0f;
    }

    // Update current heading yaw from attitude
    if (attitude_sample.valid) {
        current_yaw_deg_ = attitude_sample.yaw_deg;
    }

    // Execute sequential sensor fusion steps
    updateAltitude(tof_sample, now_ms);
    updateOpticalFlow(flow_sample, attitude_sample, dt, now_ms);
    updateDriftAndHealth(dt, now_ms);

    last_update_ms_ = now_ms;
}


// ============================================================================
// POSE & STATE GETTERS
// ============================================================================

Types::Pose2D Localization::getPose() const {
    Types::Pose2D pose;
    pose.local_x = local_x_;
    pose.local_y = local_y_;
    pose.field_x = takeoff_field_x_ + local_x_;
    pose.field_y = takeoff_field_y_ + local_y_;
    pose.yaw_deg = current_yaw_deg_;
    pose.timestamp_ms = last_update_ms_;
    return pose;
}

Types::Pose2D Localization::getLocalPose() const {
    Types::Pose2D pose;
    pose.local_x = local_x_;
    pose.local_y = local_y_;
    pose.field_x = local_x_;
    pose.field_y = local_y_;
    pose.yaw_deg = current_yaw_deg_;
    pose.timestamp_ms = last_update_ms_;
    return pose;
}

Types::Pose2D Localization::getFieldPose() const {
    Types::Pose2D pose;
    pose.local_x = local_x_;
    pose.local_y = local_y_;
    pose.field_x = takeoff_field_x_ + local_x_;
    pose.field_y = takeoff_field_y_ + local_y_;
    pose.yaw_deg = current_yaw_deg_;
    pose.timestamp_ms = last_update_ms_;
    return pose;
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void localization_init() {
    s_global_localization.init();
}

void localization_update(
    const Types::OpticalFlowSample& flow,
    const Types::TofSample& tof,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
) {
    s_global_localization.update(flow, tof, attitude, now_ms);
}

Types::Pose2D localization_get_pose() {
    return s_global_localization.getPose();
}

float localization_get_altitude() {
    return s_global_localization.getAltitude();
}

float localization_get_drift_uncertainty() {
    return s_global_localization.getDriftUncertainty();
}

bool localization_is_healthy() {
    return s_global_localization.isLocalizationHealthy();
}

Localization& localization_get_instance() {
    return s_global_localization;
}

} // namespace RobofestDrone
