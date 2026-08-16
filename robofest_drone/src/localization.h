#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

// ============================================================================
// LOCALIZATION CLASS
// ============================================================================

class Localization {
public:
    Localization();

    void init();
    void reset();
    void resetOrigin();

    void update(
        const Types::OpticalFlowSample& flow_sample,
        const Types::TofSample& tof_sample,
        const Types::AttitudeSample& attitude_sample,
        uint32_t now_ms
    );

    Types::Pose2D getPose() const;
    Types::Pose2D getLocalPose() const;
    Types::Pose2D getFieldPose() const;

    float getAltitude() const { return fused_altitude_; }
    float getFusedAltitude() const { return fused_altitude_; }

    Types::Vec2 getVelocity() const { return Types::Vec2(velocity_x_, velocity_y_); }
    float getVelocityX() const { return velocity_x_; }
    float getVelocityY() const { return velocity_y_; }

    Types::Vec2 getDriftEstimate() const { return Types::Vec2(drift_x_, drift_y_); }
    float getDriftUncertainty() const { return drift_uncertainty_; }

    bool isLocalizationHealthy() const { return health_ == Types::LocalizationHealth::LOCALIZATION_GOOD; }
    Types::LocalizationHealth getHealth() const { return health_; }

    bool isFlowUsable() const { return flow_usable_; }
    bool isTofUsable() const { return tof_usable_; }

    void setTakeoffFieldOffset(float field_x, float field_y);
    void setFocalLengthPx(float focal_length_px);
    void setAltitudeBias(float bias_m);
    void setFlowAlignmentOffset(float x_offset, float y_offset);
    void setFlowQualityThreshold(float threshold);

    void beginCalibration();
    void addCalibrationSample(float measured_altitude_m);
    bool finishCalibration();

    uint32_t getLastUpdateTimeMs() const { return last_update_ms_; }
    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }

private:
    void updateAltitude(const Types::TofSample& tof_sample, uint32_t now_ms);
    void updateOpticalFlow(const Types::OpticalFlowSample& flow_sample, const Types::AttitudeSample& attitude_sample, float dt, uint32_t now_ms);
    void updateDriftAndHealth(float dt, uint32_t now_ms);
    void setTelemetryEvent(uint16_t event_id);

private:
    float local_x_ = 0.0f;
    float local_y_ = 0.0f;
    float velocity_x_ = 0.0f;
    float velocity_y_ = 0.0f;
    float fused_altitude_ = 0.0f;
    float flow_altitude_estimate_ = 0.0f;
    float tof_altitude_raw_ = 0.0f;
    float altitude_bias_ = Config::ALTITUDE_BIAS_DEFAULT_M;
    float drift_x_ = 0.0f;
    float drift_y_ = 0.0f;
    float drift_uncertainty_ = 0.0f;
    float focal_length_px_ = Config::OPTICAL_FLOW_FOCAL_LENGTH_PX;
    float flow_offset_x_ = Config::FLOW_ALIGNMENT_X_DEFAULT;
    float flow_offset_y_ = Config::FLOW_ALIGNMENT_Y_DEFAULT;
    float flow_quality_threshold_ = Config::FLOW_QUALITY_MIN;

    float takeoff_field_x_ = Config::TAKEOFF_FIELD_X;
    float takeoff_field_y_ = Config::TAKEOFF_FIELD_Y;

    float current_yaw_deg_ = 0.0f;

    bool flow_usable_ = false;
    bool tof_usable_ = false;
    bool initialized_ = false;
    bool calibration_active_ = false;

    Types::LocalizationHealth health_ = Types::LocalizationHealth::LOCALIZATION_GOOD;

    uint32_t last_update_ms_ = 0;
    uint32_t last_valid_flow_ms_ = 0;
    uint32_t last_valid_tof_ms_ = 0;
    uint16_t last_telemetry_event_id_ = TE_LOCALIZATION_INITIALIZED;

    float calibration_altitude_sum_ = 0.0f;
    uint16_t calibration_sample_count_ = 0;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void localization_init();
void localization_update(
    const Types::OpticalFlowSample& flow,
    const Types::TofSample& tof,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
);
Types::Pose2D localization_get_pose();
float localization_get_altitude();
float localization_get_drift_uncertainty();
bool localization_is_healthy();
Localization& localization_get_instance();

} // namespace RobofestDrone
