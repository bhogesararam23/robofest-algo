#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

// ============================================================================
// GEOFENCE CLASS
// ============================================================================

class Geofence {
public:
    Geofence();

    void init();
    void reset();

    void update(const Types::Pose2D& pose, float drift_uncertainty_m, uint32_t now_ms);

    bool isInside() const { return status_ == Types::GeofenceStatus::GEOFENCE_INSIDE; }
    bool isNearBoundary() const { return status_ == Types::GeofenceStatus::GEOFENCE_NEAR_LIMIT; }
    bool isWarning() const { return status_ == Types::GeofenceStatus::GEOFENCE_WARNING; }
    bool isOutside() const { return status_ == Types::GeofenceStatus::GEOFENCE_OUTSIDE; }

    Types::GeofenceStatus getStatus() const { return status_; }
    Types::GeofenceStatus getPreviousStatus() const { return previous_status_; }

    Types::Vec2 correctionVectorToCenter() const { return correction_vector_; }
    Types::Vec2 applyBoundaryAwareVelocityBias(
        const Types::Vec2& commanded_velocity,
        const Types::Pose2D& pose,
        float drift_uncertainty_m
    ) const;

    float getEffectiveXMin() const { return effective_x_min_; }
    float getEffectiveXMax() const { return effective_x_max_; }
    float getEffectiveYMin() const { return effective_y_min_; }
    float getEffectiveYMax() const { return effective_y_max_; }

    float getUncertaintyMargin() const { return uncertainty_margin_m_; }
    float getDistanceToNearestBoundary(const Types::Pose2D& pose) const;

    void setBaseLimits(float x_min, float x_max, float y_min, float y_max);
    void setWarningBand(float warning_band_m);
    void setUncertaintyMarginScale(float scale);
    void setMaxCorrectionSpeed(float max_correction_mps);
    void setCorrectionGain(float gain);

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    void updateEffectiveLimits(float drift_uncertainty_m);
    Types::GeofenceStatus evaluateRawStatus(const Types::Pose2D& pose) const;
    void computeCorrectionVector(const Types::Pose2D& pose);
    void setTelemetryEvent(uint16_t event_id);

private:
    float base_x_min_ = Config::SOFTWARE_GEOFENCE_X_MIN;
    float base_x_max_ = Config::SOFTWARE_GEOFENCE_X_MAX;
    float base_y_min_ = Config::SOFTWARE_GEOFENCE_Y_MIN;
    float base_y_max_ = Config::SOFTWARE_GEOFENCE_Y_MAX;

    float effective_x_min_ = Config::SOFTWARE_GEOFENCE_X_MIN;
    float effective_x_max_ = Config::SOFTWARE_GEOFENCE_X_MAX;
    float effective_y_min_ = Config::SOFTWARE_GEOFENCE_Y_MIN;
    float effective_y_max_ = Config::SOFTWARE_GEOFENCE_Y_MAX;

    float warning_band_m_ = Config::GEOFENCE_WARNING_BAND_M;
    float near_limit_band_m_ = Config::GEOFENCE_NEAR_LIMIT_BAND_M;
    float uncertainty_margin_scale_ = Config::GEOFENCE_UNCERTAINTY_MARGIN_SCALE;
    float uncertainty_margin_m_ = 0.0f;
    float max_correction_speed_mps_ = Config::GEOFENCE_MAX_CORRECTION_SPEED_MPS;
    float correction_gain_ = Config::GEOFENCE_CORRECTION_GAIN;

    Types::Vec2 correction_vector_;

    Types::GeofenceStatus status_ = Types::GeofenceStatus::GEOFENCE_INSIDE;
    Types::GeofenceStatus previous_status_ = Types::GeofenceStatus::GEOFENCE_INSIDE;
    Types::GeofenceStatus raw_candidate_status_ = Types::GeofenceStatus::GEOFENCE_INSIDE;

    bool initialized_ = false;

    uint32_t last_update_ms_ = 0;
    uint32_t status_change_ms_ = 0;

    uint16_t last_telemetry_event_id_ = TE_GEOFENCE_INITIALIZED;
    bool telemetry_event_valid_ = false;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void geofence_init();
void geofence_update(const Types::Pose2D& pose, float drift_uncertainty_m);
void geofence_update_with_time(const Types::Pose2D& pose, float drift_uncertainty_m, uint32_t now_ms);
Types::GeofenceStatus geofence_get_status();
Types::Vec2 geofence_get_correction_vector();
Types::Vec2 geofence_apply_velocity_bias(
    const Types::Vec2& commanded_velocity,
    const Types::Pose2D& pose,
    float drift_uncertainty_m
);
Geofence& geofence_get_instance();

} // namespace RobofestDrone
