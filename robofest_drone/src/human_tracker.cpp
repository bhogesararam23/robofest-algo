#include "human_tracker.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static HumanTracker s_global_human_tracker;
    constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

HumanTracker::HumanTracker() {
    reset();
}

void HumanTracker::init() {
    reset();
    initialized_ = true;
    setTelemetryEvent(TE_HUMAN_TRACKER_INITIALIZED);
}

void HumanTracker::reset() {
    track_ = Types::HumanTrack();
    initialized_ = false;
    track_active_ = false;
    human_detected_ = false;

    estimated_x_ = 0.0f;
    estimated_y_ = 0.0f;
    velocity_x_ = 0.0f;
    velocity_y_ = 0.0f;
    tracking_confidence_ = 0.0f;

    last_detection_ms_ = 0;
    last_update_ms_ = 0;
    lost_since_ms_ = 0;
    off_path_since_ms_ = 0;
    exit_zone_since_ms_ = 0;

    corridor_width_m_ = 1.0f;
    off_path_threshold_m_ = Config::HUMAN_OFF_PATH_DISTANCE_M;

    last_telemetry_event_id_ = TE_HUMAN_TRACKER_INITIALIZED;
    telemetry_event_valid_ = true;
}

void HumanTracker::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}

void HumanTracker::startRecoveryTimer(uint32_t now_ms) {
    lost_since_ms_ = now_ms;
}

bool HumanTracker::isTrackingRecovered(uint32_t now_ms) const {
    return track_active_ && human_detected_ && (now_ms - last_detection_ms_ < Config::HUMAN_LOST_TIMEOUT_MS);
}

bool HumanTracker::isTrackingLost(uint32_t now_ms) const {
    if (!track_active_) return true;
    return (now_ms - last_detection_ms_) >= Config::HUMAN_RECOVERY_TIMEOUT_MS;
}


// ============================================================================
// DETECTION VALIDATION & RAY PROJECTION
// ============================================================================

bool HumanTracker::validateDetection(const Types::HumanDetectionSample& detection, uint32_t now_ms) {
    (void)now_ms;
    if (!detection.valid) {
        setTelemetryEvent(TE_HUMAN_DETECTION_REJECTED_INVALID);
        return false;
    }
    if (std::isnan(detection.confidence) || detection.confidence < Config::HUMAN_MIN_DETECTION_CONFIDENCE) {
        setTelemetryEvent(TE_HUMAN_DETECTION_REJECTED_LOW_CONFIDENCE);
        return false;
    }

    if (detection.field_position_valid) {
        if (std::isnan(detection.field_x) || std::isnan(detection.field_y)) {
            setTelemetryEvent(TE_HUMAN_DETECTION_REJECTED_INVALID);
            return false;
        }
        return true;
    }

    if (std::isnan(detection.pixel_x) || std::isnan(detection.pixel_y)) {
        setTelemetryEvent(TE_HUMAN_DETECTION_REJECTED_INVALID);
        return false;
    }

    return true;
}

bool HumanTracker::projectPixelToField(
    const Types::HumanDetectionSample& detection,
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    float& out_field_x,
    float& out_field_y
) {
    if (detection.field_position_valid) {
        out_field_x = detection.field_x;
        out_field_y = detection.field_y;
        return true;
    }

    if (std::isnan(fused_altitude_m) || fused_altitude_m < 0.20f) {
        setTelemetryEvent(TE_HUMAN_PROJECTION_DEGRADED);
        return false;
    }

    float px = detection.pixel_x;
    float py = detection.pixel_y;
    if (Config::HUMAN_USE_FOOT_POINT && detection.pixel_height > 0.0f) {
        py += detection.pixel_height * 0.5f;
    }

    float cx = Config::IMAGE_CENTER_X;
    float cy = Config::IMAGE_CENTER_Y;

    float h_fov_rad = Config::H_FOV_DEG * DEG_TO_RAD_F;
    float v_fov_rad = Config::V_FOV_DEG * DEG_TO_RAD_F;

    float angle_x = ((px - cx) / Config::IMAGE_WIDTH) * h_fov_rad;
    float angle_y = ((py - cy) / Config::IMAGE_HEIGHT) * v_fov_rad;

    float body_dx = fused_altitude_m * std::tan(angle_x);
    float body_dy = fused_altitude_m * std::tan(angle_y);

    if (attitude.valid) {
        float pitch_rad = -attitude.pitch_deg * DEG_TO_RAD_F;
        float roll_rad = attitude.roll_deg * DEG_TO_RAD_F;
        body_dx += fused_altitude_m * std::tan(pitch_rad);
        body_dy += fused_altitude_m * std::tan(roll_rad);
    }

    float yaw_rad = drone_pose.yaw_deg * DEG_TO_RAD_F;
    float cos_yaw = std::cos(yaw_rad);
    float sin_yaw = std::sin(yaw_rad);

    out_field_x = drone_pose.field_x + (body_dx * cos_yaw - body_dy * sin_yaw);
    out_field_y = drone_pose.field_y + (body_dx * sin_yaw + body_dy * cos_yaw);

    // Clamp to field perimeter
    out_field_x = std::max(Config::FIELD_X_MIN, std::min(Config::FIELD_X_MAX, out_field_x));
    out_field_y = std::max(Config::FIELD_Y_MIN, std::min(Config::FIELD_Y_MAX, out_field_y));

    return true;
}


// ============================================================================
// GEOMETRY & PATH PROGRESS HELPERS
// ============================================================================

float HumanTracker::pointToPolylineDistance(float x, float y, const Types::SafePath& path) const {
    if (!path.valid || path.waypoint_count < 2) {
        return 0.0f;
    }

    float min_dist_sq = 1e9f;

    for (uint8_t i = 0; i < path.waypoint_count - 1; ++i) {
        float x1 = path.waypoints[i].x;
        float y1 = path.waypoints[i].y;
        float x2 = path.waypoints[i + 1].x;
        float y2 = path.waypoints[i + 1].y;

        float seg_dx = x2 - x1;
        float seg_dy = y2 - y1;
        float seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

        if (seg_len_sq < 1e-6f) {
            float dx = x - x1;
            float dy = y - y1;
            float dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq) min_dist_sq = dist_sq;
            continue;
        }

        float t = ((x - x1) * seg_dx + (y - y1) * seg_dy) / seg_len_sq;
        t = std::max(0.0f, std::min(1.0f, t));

        float proj_x = x1 + t * seg_dx;
        float proj_y = y1 + t * seg_dy;

        float dx = x - proj_x;
        float dy = y - proj_y;
        float dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
        }
    }

    return std::sqrt(min_dist_sq);
}

float HumanTracker::getForwardProgressAlongPath(float x, float y, const Types::SafePath& path) const {
    if (!path.valid || path.waypoint_count < 2) {
        return y; // fallback to forward field coordinate
    }

    float accumulated_length = 0.0f;
    float best_total_progress = 0.0f;
    float min_dist_sq = 1e9f;

    for (uint8_t i = 0; i < path.waypoint_count - 1; ++i) {
        float x1 = path.waypoints[i].x;
        float y1 = path.waypoints[i].y;
        float x2 = path.waypoints[i + 1].x;
        float y2 = path.waypoints[i + 1].y;

        float seg_dx = x2 - x1;
        float seg_dy = y2 - y1;
        float seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

        if (seg_len < 1e-4f) continue;

        float t = ((x - x1) * seg_dx + (y - y1) * seg_dy) / (seg_len * seg_len);
        t = std::max(0.0f, std::min(1.0f, t));

        float proj_x = x1 + t * seg_dx;
        float proj_y = y1 + t * seg_dy;

        float dx = x - proj_x;
        float dy = y - proj_y;
        float dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_total_progress = accumulated_length + t * seg_len;
        }

        accumulated_length += seg_len;
    }

    return best_total_progress;
}


// ============================================================================
// MAIN UPDATE FUNCTION (20 Hz NON-BLOCKING)
// ============================================================================

void HumanTracker::update(
    const Types::HumanDetectionSample& detection,
    const Types::SafePath& active_path,
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
) {
    float dt = (last_update_ms_ > 0 && now_ms >= last_update_ms_) ?
               (static_cast<float>(now_ms - last_update_ms_) / 1000.0f) : 0.05f;
    last_update_ms_ = now_ms;

    bool valid_detection = validateDetection(detection, now_ms);
    float meas_x = 0.0f, meas_y = 0.0f;
    bool projected = false;

    if (valid_detection) {
        projected = projectPixelToField(detection, drone_pose, fused_altitude_m, attitude, meas_x, meas_y);
    }

    if (valid_detection && projected) {
        if (!track_active_) {
            // Initialize new track
            estimated_x_ = meas_x;
            estimated_y_ = meas_y;
            velocity_x_ = 0.0f;
            velocity_y_ = 0.0f;
            tracking_confidence_ = detection.confidence;
            track_active_ = true;
            human_detected_ = true;
            setTelemetryEvent(TE_HUMAN_DETECTED);
        } else {
            // Update filter
            if (dt > 0.001f) {
                float raw_vx = (meas_x - estimated_x_) / dt;
                float raw_vy = (meas_y - estimated_y_) / dt;

                raw_vx = std::max(-Config::HUMAN_MAX_PLAUSIBLE_SPEED_MPS, std::min(Config::HUMAN_MAX_PLAUSIBLE_SPEED_MPS, raw_vx));
                raw_vy = std::max(-Config::HUMAN_MAX_PLAUSIBLE_SPEED_MPS, std::min(Config::HUMAN_MAX_PLAUSIBLE_SPEED_MPS, raw_vy));

                velocity_x_ = Config::HUMAN_VELOCITY_ALPHA * raw_vx + (1.0f - Config::HUMAN_VELOCITY_ALPHA) * velocity_x_;
                velocity_y_ = Config::HUMAN_VELOCITY_ALPHA * raw_vy + (1.0f - Config::HUMAN_VELOCITY_ALPHA) * velocity_y_;
            }

            estimated_x_ = Config::HUMAN_TRACK_ALPHA * meas_x + (1.0f - Config::HUMAN_TRACK_ALPHA) * (estimated_x_ + velocity_x_ * dt);
            estimated_y_ = Config::HUMAN_TRACK_ALPHA * meas_y + (1.0f - Config::HUMAN_TRACK_ALPHA) * (estimated_y_ + velocity_y_ * dt);

            tracking_confidence_ = std::min(1.0f, tracking_confidence_ + 0.10f);
            human_detected_ = true;
            setTelemetryEvent(TE_HUMAN_TRACK_UPDATED);
        }
        last_detection_ms_ = now_ms;
    } else {
        // No valid measurement: propagate track with velocity prediction
        if (track_active_) {
            estimated_x_ += velocity_x_ * dt;
            estimated_y_ += velocity_y_ * dt;
            tracking_confidence_ = std::max(0.0f, tracking_confidence_ - 0.05f);
            setTelemetryEvent(TE_HUMAN_TRACK_PREDICTED);

            if ((now_ms - last_detection_ms_) > Config::HUMAN_LOST_TIMEOUT_MS) {
                if (human_detected_) {
                    human_detected_ = false;
                    setTelemetryEvent(TE_HUMAN_LOST_SHORT_TERM);
                }
            }

            if ((now_ms - last_detection_ms_) > Config::HUMAN_RECOVERY_TIMEOUT_MS) {
                track_active_ = false;
                velocity_x_ = 0.0f;
                velocity_y_ = 0.0f;
                setTelemetryEvent(TE_HUMAN_LOST_RECOVERY_TIMEOUT);
            }
        }
    }

    // Populate track output
    track_.human_detected = human_detected_;
    track_.field_x = estimated_x_;
    track_.field_y = estimated_y_;
    track_.velocity_x = velocity_x_;
    track_.velocity_y = velocity_y_;
    track_.tracking_confidence = tracking_confidence_;
    track_.timestamp_ms = now_ms;

    // Lateral deviation & forward progress
    if (active_path.valid) {
        track_.lateral_deviation_m = pointToPolylineDistance(estimated_x_, estimated_y_, active_path);
        track_.forward_progress_m = getForwardProgressAlongPath(estimated_x_, estimated_y_, active_path);
    } else {
        track_.lateral_deviation_m = 0.0f;
        track_.forward_progress_m = estimated_y_;
    }

    // Off-path evaluation with debounce
    float effective_off_path_threshold = std::max(off_path_threshold_m_, (active_path.valid && active_path.corridor_width_m > 0.0f) ? (active_path.corridor_width_m * 0.5f) : Config::HUMAN_OFF_PATH_DISTANCE_M);
    if (track_active_ && active_path.valid && track_.lateral_deviation_m > effective_off_path_threshold) {
        if (off_path_since_ms_ == 0) off_path_since_ms_ = now_ms;
        if ((now_ms - off_path_since_ms_) >= Config::HUMAN_OFF_PATH_DEBOUNCE_MS) {
            if (!track_.human_off_path) {
                track_.human_off_path = true;
                setTelemetryEvent(TE_HUMAN_OFF_PATH_DETECTED);
            }
        }
    } else {
        if (track_.human_off_path) {
            setTelemetryEvent(TE_HUMAN_BACK_ON_PATH);
        }
        off_path_since_ms_ = 0;
        track_.human_off_path = false;
    }

    // Exit Zone evaluation with confirmation timeout
    bool in_exit_zone_bounds = (estimated_y_ >= (Config::EXIT_ZONE_Y_MIN + Config::HUMAN_EXIT_ZONE_MARGIN_M)) &&
                              (estimated_y_ <= Config::EXIT_ZONE_Y_MAX) &&
                              (estimated_x_ >= Config::FIELD_X_MIN) &&
                              (estimated_x_ <= Config::FIELD_X_MAX);

    if (track_active_ && in_exit_zone_bounds) {
        if (exit_zone_since_ms_ == 0) {
            exit_zone_since_ms_ = now_ms;
            setTelemetryEvent(TE_HUMAN_EXIT_ZONE_DETECTED);
        }
        if ((now_ms - exit_zone_since_ms_) >= Config::HUMAN_EXIT_CONFIRM_TIMEOUT_MS) {
            if (!track_.human_in_exit_zone) {
                track_.human_in_exit_zone = true;
                setTelemetryEvent(TE_HUMAN_EXIT_ZONE_CONFIRMED);
            }
        }
    } else {
        if (track_.human_in_exit_zone) {
            setTelemetryEvent(TE_HUMAN_EXIT_ZONE_LOST);
        }
        exit_zone_since_ms_ = 0;
        track_.human_in_exit_zone = false;
    }

    // Distance to drone & Proximity Warnings
    float dx = estimated_x_ - drone_pose.field_x;
    float dy = estimated_y_ - drone_pose.field_y;
    track_.distance_to_drone_m = std::sqrt(dx * dx + dy * dy);

    if (track_active_) {
        if (track_.distance_to_drone_m <= Config::HUMAN_PROXIMITY_CRITICAL_M) {
            setTelemetryEvent(TE_HUMAN_PROXIMITY_CRITICAL);
        } else if (track_.distance_to_drone_m <= Config::HUMAN_PROXIMITY_WARNING_M) {
            setTelemetryEvent(TE_HUMAN_PROXIMITY_WARNING);
        }
    }
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void human_tracker_init() {
    s_global_human_tracker.init();
}

void human_tracker_update(
    const Types::HumanDetectionSample& detection,
    const Types::SafePath& active_path,
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
) {
    s_global_human_tracker.update(detection, active_path, drone_pose, fused_altitude_m, attitude, now_ms);
}

Types::HumanTrack human_tracker_get_track() {
    return s_global_human_tracker.getTrack();
}

bool human_tracker_is_detected() {
    return s_global_human_tracker.isHumanDetected();
}

bool human_tracker_is_off_path() {
    return s_global_human_tracker.isHumanOffPath();
}

bool human_tracker_is_in_exit_zone() {
    return s_global_human_tracker.isHumanInExitZone();
}

HumanTracker& human_tracker_get_instance() {
    return s_global_human_tracker;
}

} // namespace RobofestDrone
