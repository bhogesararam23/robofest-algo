#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

// ============================================================================
// HUMAN TRACKER CLASS
// ============================================================================

class HumanTracker {
public:
    HumanTracker();

    void init();
    void reset();

    void update(
        const Types::HumanDetectionSample& detection,
        const Types::SafePath& active_path,
        const Types::Pose2D& drone_pose,
        float fused_altitude_m,
        const Types::AttitudeSample& attitude,
        uint32_t now_ms
    );

    Types::HumanTrack getTrack() const { return track_; }

    bool isHumanDetected() const { return human_detected_; }
    float getTrackingConfidence() const { return tracking_confidence_; }

    float getLateralDeviation() const { return track_.lateral_deviation_m; }
    float getForwardProgress() const { return track_.forward_progress_m; }
    float getDistanceToDrone() const { return track_.distance_to_drone_m; }

    bool isHumanOffPath() const { return track_.human_off_path; }
    bool isHumanInExitZone() const { return track_.human_in_exit_zone; }

    void setPathCorridorWidth(float corridor_width_m) { corridor_width_m_ = corridor_width_m; }
    void setOffPathThreshold(float threshold_m) { off_path_threshold_m_ = threshold_m; }

    void startRecoveryTimer(uint32_t now_ms);
    bool isTrackingRecovered(uint32_t now_ms) const;
    bool isTrackingLost(uint32_t now_ms) const;

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    bool validateDetection(const Types::HumanDetectionSample& detection, uint32_t now_ms);
    bool projectPixelToField(
        const Types::HumanDetectionSample& detection,
        const Types::Pose2D& drone_pose,
        float fused_altitude_m,
        const Types::AttitudeSample& attitude,
        float& out_field_x,
        float& out_field_y
    );

    float pointToPolylineDistance(float x, float y, const Types::SafePath& path) const;
    float getForwardProgressAlongPath(float x, float y, const Types::SafePath& path) const;
    void setTelemetryEvent(uint16_t event_id);

private:
    Types::HumanTrack track_;

    bool initialized_ = false;
    bool track_active_ = false;
    bool human_detected_ = false;

    float estimated_x_ = 0.0f;
    float estimated_y_ = 0.0f;
    float velocity_x_ = 0.0f;
    float velocity_y_ = 0.0f;
    float tracking_confidence_ = 0.0f;

    uint32_t last_detection_ms_ = 0;
    uint32_t last_update_ms_ = 0;
    uint32_t lost_since_ms_ = 0;
    uint32_t off_path_since_ms_ = 0;
    uint32_t exit_zone_since_ms_ = 0;

    float corridor_width_m_ = 1.0f;
    float off_path_threshold_m_ = Config::HUMAN_OFF_PATH_DISTANCE_M;

    uint16_t last_telemetry_event_id_ = TE_HUMAN_TRACKER_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void human_tracker_init();
void human_tracker_update(
    const Types::HumanDetectionSample& detection,
    const Types::SafePath& active_path,
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
);
Types::HumanTrack human_tracker_get_track();
bool human_tracker_is_detected();
bool human_tracker_is_off_path();
bool human_tracker_is_in_exit_zone();
HumanTracker& human_tracker_get_instance();

} // namespace RobofestDrone
