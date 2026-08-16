#include "geofence.h"
#include <cmath>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static Geofence s_global_geofence;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

Geofence::Geofence() {
    reset();
}

void Geofence::init() {
    reset();
}

void Geofence::reset() {
    base_x_min_ = Config::SOFTWARE_GEOFENCE_X_MIN;
    base_x_max_ = Config::SOFTWARE_GEOFENCE_X_MAX;
    base_y_min_ = Config::SOFTWARE_GEOFENCE_Y_MIN;
    base_y_max_ = Config::SOFTWARE_GEOFENCE_Y_MAX;

    effective_x_min_ = base_x_min_;
    effective_x_max_ = base_x_max_;
    effective_y_min_ = base_y_min_;
    effective_y_max_ = base_y_max_;

    warning_band_m_ = Config::GEOFENCE_WARNING_BAND_M;
    near_limit_band_m_ = Config::GEOFENCE_NEAR_LIMIT_BAND_M;
    uncertainty_margin_scale_ = Config::GEOFENCE_UNCERTAINTY_MARGIN_SCALE;
    uncertainty_margin_m_ = 0.0f;
    max_correction_speed_mps_ = Config::GEOFENCE_MAX_CORRECTION_SPEED_MPS;
    correction_gain_ = Config::GEOFENCE_CORRECTION_GAIN;

    correction_vector_ = Types::Vec2(0.0f, 0.0f);

    status_ = Types::GeofenceStatus::GEOFENCE_INSIDE;
    previous_status_ = Types::GeofenceStatus::GEOFENCE_INSIDE;
    raw_candidate_status_ = Types::GeofenceStatus::GEOFENCE_INSIDE;

    initialized_ = true;

    last_update_ms_ = 0;
    status_change_ms_ = 0;

    last_telemetry_event_id_ = TE_GEOFENCE_INITIALIZED;
    telemetry_event_valid_ = true;
}


// ============================================================================
// CONFIGURATION SETTERS
// ============================================================================

void Geofence::setBaseLimits(float x_min, float x_max, float y_min, float y_max) {
    if (x_min < x_max && y_min < y_max) {
        base_x_min_ = x_min;
        base_x_max_ = x_max;
        base_y_min_ = y_min;
        base_y_max_ = y_max;
    }
}

void Geofence::setWarningBand(float warning_band_m) {
    if (warning_band_m > 0.0f) {
        warning_band_m_ = warning_band_m;
    }
}

void Geofence::setUncertaintyMarginScale(float scale) {
    if (scale >= 0.0f) {
        uncertainty_margin_scale_ = scale;
    }
}

void Geofence::setMaxCorrectionSpeed(float max_correction_mps) {
    if (max_correction_mps > 0.0f) {
        max_correction_speed_mps_ = max_correction_mps;
    }
}

void Geofence::setCorrectionGain(float gain) {
    if (gain > 0.0f) {
        correction_gain_ = gain;
    }
}

void Geofence::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}


// ============================================================================
// DRIFT-AWARE MARGIN RECOMPUTATION
// ============================================================================

void Geofence::updateEffectiveLimits(float drift_uncertainty_m) {
    float safe_uncertainty = (drift_uncertainty_m > 0.0f) ? drift_uncertainty_m : 0.0f;
    uncertainty_margin_m_ = safe_uncertainty * uncertainty_margin_scale_;

    if (uncertainty_margin_m_ > Config::GEOFENCE_MAX_UNCERTAINTY_MARGIN_M) {
        uncertainty_margin_m_ = Config::GEOFENCE_MAX_UNCERTAINTY_MARGIN_M;
        setTelemetryEvent(TE_GEOFENCE_MARGIN_MAXIMUM);
    } else if (uncertainty_margin_m_ > 0.0f) {
        setTelemetryEvent(TE_GEOFENCE_MARGIN_INCREASED);
    }

    effective_x_min_ = base_x_min_ + uncertainty_margin_m_;
    effective_x_max_ = base_x_max_ - uncertainty_margin_m_;
    effective_y_min_ = base_y_min_ + uncertainty_margin_m_;
    effective_y_max_ = base_y_max_ - uncertainty_margin_m_;

    // Prevent corridor collapse under high uncertainty by centering minimum box
    if ((effective_x_max_ - effective_x_min_) < Config::GEOFENCE_MIN_EFFECTIVE_WIDTH_M) {
        float center_x = (Config::FIELD_X_MIN + Config::FIELD_X_MAX) * 0.5f;
        effective_x_min_ = center_x - (Config::GEOFENCE_MIN_EFFECTIVE_WIDTH_M * 0.5f);
        effective_x_max_ = center_x + (Config::GEOFENCE_MIN_EFFECTIVE_WIDTH_M * 0.5f);
        setTelemetryEvent(TE_GEOFENCE_EFFECTIVE_LIMITS_TOO_SMALL);
    }

    if ((effective_y_max_ - effective_y_min_) < Config::GEOFENCE_MIN_EFFECTIVE_WIDTH_M) {
        float center_y = (Config::FIELD_Y_MIN + Config::FIELD_Y_MAX) * 0.5f;
        effective_y_min_ = center_y - (Config::GEOFENCE_MIN_EFFECTIVE_WIDTH_M * 0.5f);
        effective_y_max_ = center_y + (Config::GEOFENCE_MIN_EFFECTIVE_WIDTH_M * 0.5f);
        setTelemetryEvent(TE_GEOFENCE_EFFECTIVE_LIMITS_TOO_SMALL);
    }
}


// ============================================================================
// BOUNDARY DISTANCE & STATUS EVALUATION
// ============================================================================

float Geofence::getDistanceToNearestBoundary(const Types::Pose2D& pose) const {
    if (std::isnan(pose.field_x) || std::isnan(pose.field_y)) {
        return 0.0f;
    }

    float dist_x_min = pose.field_x - effective_x_min_;
    float dist_x_max = effective_x_max_ - pose.field_x;
    float dist_y_min = pose.field_y - effective_y_min_;
    float dist_y_max = effective_y_max_ - pose.field_y;

    float min_dist_x = (dist_x_min < dist_x_max) ? dist_x_min : dist_x_max;
    float min_dist_y = (dist_y_min < dist_y_max) ? dist_y_min : dist_y_max;

    return (min_dist_x < min_dist_y) ? min_dist_x : min_dist_y;
}

Types::GeofenceStatus Geofence::evaluateRawStatus(const Types::Pose2D& pose) const {
    if (std::isnan(pose.field_x) || std::isnan(pose.field_y)) {
        return Types::GeofenceStatus::GEOFENCE_OUTSIDE;
    }

    float dist = getDistanceToNearestBoundary(pose);

    if (dist < 0.0f) {
        return Types::GeofenceStatus::GEOFENCE_OUTSIDE;
    }
    if (dist < near_limit_band_m_) {
        return Types::GeofenceStatus::GEOFENCE_NEAR_LIMIT;
    }
    if (dist < warning_band_m_) {
        return Types::GeofenceStatus::GEOFENCE_WARNING;
    }

    return Types::GeofenceStatus::GEOFENCE_INSIDE;
}


// ============================================================================
// CONTINUOUS CORRECTION VECTOR GENERATION
// ============================================================================

void Geofence::computeCorrectionVector(const Types::Pose2D& pose) {
    if (std::isnan(pose.field_x) || std::isnan(pose.field_y)) {
        float center_x = (Config::FIELD_X_MIN + Config::FIELD_X_MAX) * 0.5f;
        float center_y = (Config::FIELD_Y_MIN + Config::FIELD_Y_MAX) * 0.5f;
        correction_vector_ = Types::Vec2(0.0f, 0.0f);
        (void)center_x;
        (void)center_y;
        return;
    }

    float center_x = (effective_x_min_ + effective_x_max_) * 0.5f;
    float center_y = (effective_y_min_ + effective_y_max_) * 0.5f;

    float corr_x = 0.0f;
    float corr_y = 0.0f;

    if (status_ == Types::GeofenceStatus::GEOFENCE_OUTSIDE) {
        // Strong proportional restoring vector toward arena center
        corr_x = correction_gain_ * (center_x - pose.field_x);
        corr_y = correction_gain_ * (center_y - pose.field_y);
    } else {
        // Continuous boundary steering within warning and near-limit bands
        if (pose.field_x < (effective_x_min_ + warning_band_m_)) {
            float error_x = (effective_x_min_ + warning_band_m_) - pose.field_x;
            corr_x += correction_gain_ * error_x;
        }
        if (pose.field_x > (effective_x_max_ - warning_band_m_)) {
            float error_x = pose.field_x - (effective_x_max_ - warning_band_m_);
            corr_x -= correction_gain_ * error_x;
        }

        if (pose.field_y < (effective_y_min_ + warning_band_m_)) {
            float error_y = (effective_y_min_ + warning_band_m_) - pose.field_y;
            corr_y += correction_gain_ * error_y;
        }
        if (pose.field_y > (effective_y_max_ - warning_band_m_)) {
            float error_y = pose.field_y - (effective_y_max_ - warning_band_m_);
            corr_y -= correction_gain_ * error_y;
        }
    }

    // Clamp correction vector to configured max correction speed
    float magnitude = std::sqrt(corr_x * corr_x + corr_y * corr_y);
    if (magnitude > max_correction_speed_mps_) {
        float scale = max_correction_speed_mps_ / magnitude;
        corr_x *= scale;
        corr_y *= scale;
    }

    correction_vector_ = Types::Vec2(corr_x, corr_y);

    if (magnitude > 0.05f) {
        setTelemetryEvent(TE_GEOFENCE_CORRECTION_ACTIVE);
    }
}


// ============================================================================
// MAIN GEOFENCE UPDATE (50 Hz NON-BLOCKING)
// ============================================================================

void Geofence::update(const Types::Pose2D& pose, float drift_uncertainty_m, uint32_t now_ms) {
    if (!initialized_) {
        init();
    }

    // 1. Recompute drift-aware effective bounds
    updateEffectiveLimits(drift_uncertainty_m);

    // 2. Evaluate raw status from geometry
    Types::GeofenceStatus raw_status = evaluateRawStatus(pose);

    // 3. Status hysteresis filtering
    if (raw_status == Types::GeofenceStatus::GEOFENCE_OUTSIDE) {
        // Immediate outside detection (no hysteresis delay for safety critical state)
        if (status_ != Types::GeofenceStatus::GEOFENCE_OUTSIDE) {
            previous_status_ = status_;
            status_ = Types::GeofenceStatus::GEOFENCE_OUTSIDE;
            status_change_ms_ = now_ms;
            setTelemetryEvent(TE_GEOFENCE_OUTSIDE);
        }
    } else if (raw_status != status_) {
        if (raw_status != raw_candidate_status_) {
            raw_candidate_status_ = raw_status;
            status_change_ms_ = now_ms;
        } else if ((now_ms - status_change_ms_) >= Config::GEOFENCE_STATUS_HYSTERESIS_MS) {
            previous_status_ = status_;
            status_ = raw_status;

            if (status_ == Types::GeofenceStatus::GEOFENCE_INSIDE) {
                setTelemetryEvent(TE_GEOFENCE_INSIDE);
            } else if (status_ == Types::GeofenceStatus::GEOFENCE_WARNING) {
                setTelemetryEvent(TE_GEOFENCE_WARNING);
            } else if (status_ == Types::GeofenceStatus::GEOFENCE_NEAR_LIMIT) {
                setTelemetryEvent(TE_GEOFENCE_NEAR_LIMIT);
            }
        }
    } else {
        raw_candidate_status_ = status_;
    }

    // 4. Compute continuous correction vector
    computeCorrectionVector(pose);

    last_update_ms_ = now_ms;
}


// ============================================================================
// VELOCITY BIASING & INTENT PRESERVATION
// ============================================================================

Types::Vec2 Geofence::applyBoundaryAwareVelocityBias(
    const Types::Vec2& commanded_velocity,
    const Types::Pose2D& pose,
    float drift_uncertainty_m
) const {
    (void)drift_uncertainty_m;

    if (std::isnan(commanded_velocity.x) || std::isnan(commanded_velocity.y) ||
        std::isnan(pose.field_x) || std::isnan(pose.field_y)) {
        return correction_vector_;
    }

    float vx = commanded_velocity.x;
    float vy = commanded_velocity.y;

    switch (status_) {
        case Types::GeofenceStatus::GEOFENCE_INSIDE:
            // Full commanded velocity preserved
            break;

        case Types::GeofenceStatus::GEOFENCE_WARNING:
            // Moderate speed reduction + add boundary correction vector
            vx = (vx * Config::GEOFENCE_SPEED_REDUCTION_FACTOR_WARNING) + correction_vector_.x;
            vy = (vy * Config::GEOFENCE_SPEED_REDUCTION_FACTOR_WARNING) + correction_vector_.y;
            break;

        case Types::GeofenceStatus::GEOFENCE_NEAR_LIMIT:
            // Inhibit velocity components moving outward towards boundary
            if (pose.field_x > (effective_x_max_ - near_limit_band_m_) && vx > 0.0f) {
                vx = 0.0f;
            }
            if (pose.field_x < (effective_x_min_ + near_limit_band_m_) && vx < 0.0f) {
                vx = 0.0f;
            }
            if (pose.field_y > (effective_y_max_ - near_limit_band_m_) && vy > 0.0f) {
                vy = 0.0f;
            }
            if (pose.field_y < (effective_y_min_ + near_limit_band_m_) && vy < 0.0f) {
                vy = 0.0f;
            }

            vx = (vx * Config::GEOFENCE_SPEED_REDUCTION_FACTOR_NEAR_LIMIT) + correction_vector_.x;
            vy = (vy * Config::GEOFENCE_SPEED_REDUCTION_FACTOR_NEAR_LIMIT) + correction_vector_.y;
            break;

        case Types::GeofenceStatus::GEOFENCE_OUTSIDE:
            // Strong override: zero outward velocity and guide back to center
            vx = correction_vector_.x * Config::GEOFENCE_SPEED_REDUCTION_FACTOR_OUTSIDE;
            vy = correction_vector_.y * Config::GEOFENCE_SPEED_REDUCTION_FACTOR_OUTSIDE;
            break;
    }

    // Clamp final velocity to maximum allowable translational velocity
    float total_speed = std::sqrt(vx * vx + vy * vy);
    if (total_speed > Config::MAX_HORIZONTAL_SPEED_MPS && total_speed > 0.0001f) {
        float scale = Config::MAX_HORIZONTAL_SPEED_MPS / total_speed;
        vx *= scale;
        vy *= scale;
    }

    return Types::Vec2(vx, vy);
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void geofence_init() {
    s_global_geofence.init();
}

void geofence_update(const Types::Pose2D& pose, float drift_uncertainty_m) {
    s_global_geofence.update(pose, drift_uncertainty_m, pose.timestamp_ms);
}

void geofence_update_with_time(const Types::Pose2D& pose, float drift_uncertainty_m, uint32_t now_ms) {
    s_global_geofence.update(pose, drift_uncertainty_m, now_ms);
}

Types::GeofenceStatus geofence_get_status() {
    return s_global_geofence.getStatus();
}

Types::Vec2 geofence_get_correction_vector() {
    return s_global_geofence.correctionVectorToCenter();
}

Types::Vec2 geofence_apply_velocity_bias(
    const Types::Vec2& commanded_velocity,
    const Types::Pose2D& pose,
    float drift_uncertainty_m
) {
    return s_global_geofence.applyBoundaryAwareVelocityBias(commanded_velocity, pose, drift_uncertainty_m);
}

Geofence& geofence_get_instance() {
    return s_global_geofence;
}

} // namespace RobofestDrone
