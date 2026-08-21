#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"
#include "../hal/hal_camera.h"

namespace RobofestDrone {

// ============================================================================
// MARKER PROFILE STRUCTURE
// ============================================================================

struct VisionMarkerProfile {
    Types::VisionMarkerType profile_type = Types::VisionMarkerType::ON_GROUND_MINE;
    bool enabled = true;
    uint8_t h_min = 0;
    uint8_t h_max = 15;
    uint8_t s_min = 100;
    uint8_t s_max = 255;
    uint8_t v_min = 100;
    uint8_t v_max = 255;
    float min_area_px = Config::BLOB_AREA_MIN_PX;
    float max_area_px = Config::BLOB_AREA_MAX_PX;
    float circularity_min = Config::CIRCULARITY_MIN;
    float confidence_bias = 0.0f;
    uint16_t expected_marker_area_px = 400;
};


// ============================================================================
// INTERNAL BLOB & TRACK STRUCTURES
// ============================================================================

struct VisionBlob {
    float centroid_x = 0.0f;
    float centroid_y = 0.0f;
    float area = 0.0f;
    float perimeter = 0.0f;
    float circularity = 0.0f;
    uint16_t x_min = 0;
    uint16_t x_max = 0;
    uint16_t y_min = 0;
    uint16_t y_max = 0;
    bool valid = false;
};

struct VisionPersistenceTrack {
    uint16_t track_id = 0;
    float world_x = 0.0f;
    float world_y = 0.0f;
    float pixel_x = 0.0f;
    float pixel_y = 0.0f;
    float average_confidence = 0.0f;
    float circularity = 0.0f;
    float area = 0.0f;
    float normalized_area_score = 0.0f;
    uint16_t persistence_count = 0;
    uint32_t last_seen_ms = 0;
    Types::VisionMarkerType marker_type = Types::VisionMarkerType::UNKNOWN;
    uint32_t frame_id = 0;
    bool active = false;
};


// ============================================================================
// VISION PIPELINE CLASS
// ============================================================================

class VisionPipeline {
public:
    VisionPipeline();

    void init();
    void reset();

    void update(
        const Types::Pose2D& drone_pose,
        float fused_altitude_m,
        const Types::AttitudeSample& attitude,
        uint32_t now_ms
    );

    uint8_t getCandidateCount() const { return candidate_count_; }
    Types::VisionCandidate getCandidate(uint8_t index) const;

    void setActiveProfile(Types::VisionMarkerType profile);
    Types::VisionMarkerType getActiveProfile() const { return active_profile_type_; }

    bool isHealthy() const { return camera_healthy_; }
    bool isFrameFresh(uint32_t now_ms) const;

    uint32_t getLastFrameTimeMs() const { return last_frame_time_ms_; }
    uint32_t getLastProcessTimeMs() const { return last_process_time_ms_; }
    uint32_t getLastProcessingDurationUs() const { return processing_duration_us_; }

    void setCameraCalibration(
        float h_fov_deg,
        float v_fov_deg,
        float image_center_x,
        float image_center_y
    );

    void setAttitudeCompensationEnabled(bool enabled) { attitude_compensation_enabled_ = enabled; }
    void setDownscaleEnabled(bool enabled) { downscale_enabled_ = enabled; }

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    void initProfiles();
    void segmentHsvMask(const Hal::CameraFrame& frame, const VisionMarkerProfile& profile);
    uint8_t extractBlobs(const VisionMarkerProfile& profile);
    void projectToWorld(float pixel_x, float pixel_y, float altitude_m, const Types::Pose2D& drone_pose, const Types::AttitudeSample& attitude, float& out_world_x, float& out_world_y, bool& out_valid);
    void updatePersistenceTracks(const VisionBlob* blobs, uint8_t blob_count, const Types::Pose2D& drone_pose, float fused_altitude_m, const Types::AttitudeSample& attitude, uint32_t now_ms);
    void pruneStaleTracks(uint32_t now_ms);
    void applyMorphologyCleanup();
    void setTelemetryEvent(uint16_t event_id);

private:
    VisionMarkerProfile profile_on_ground_;
    VisionMarkerProfile profile_buried_;
    Types::VisionMarkerType active_profile_type_ = Types::VisionMarkerType::ON_GROUND_MINE;

    Types::VisionCandidate candidates_[Config::VISION_MAX_CANDIDATES] = {};
    uint8_t candidate_count_ = 0;

    VisionPersistenceTrack tracks_[Config::VISION_MAX_PERSISTENCE_TRACKS] = {};
    uint16_t next_track_id_ = 1;

    float h_fov_deg_ = Config::H_FOV_DEG;
    float v_fov_deg_ = Config::V_FOV_DEG;
    float image_center_x_ = Config::IMAGE_CENTER_X;
    float image_center_y_ = Config::IMAGE_CENTER_Y;

    bool attitude_compensation_enabled_ = Config::VISION_ATTITUDE_COMPENSATION_ENABLED;
    bool downscale_enabled_ = Config::VISION_DOWNSCALE_ENABLED;

    bool camera_healthy_ = false;
    uint32_t last_frame_time_ms_ = 0;
    uint32_t last_process_time_ms_ = 0;
    uint32_t processing_duration_us_ = 0;
    uint32_t frame_count_ = 0;
    uint16_t frame_timeout_count_ = 0;
    uint16_t dropped_frame_count_ = 0;

    uint16_t last_telemetry_event_id_ = TE_VISION_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void vision_pipeline_init();
void vision_pipeline_update(
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    uint32_t now_ms
);
void vision_pipeline_update_full(
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
);
uint8_t vision_pipeline_get_candidates(Types::VisionCandidate* out_candidates, uint8_t max_count);
VisionPipeline& vision_pipeline_get_instance();

} // namespace RobofestDrone
