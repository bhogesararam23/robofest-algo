#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"
#include "../config/vision_profiles.h"
#include "../hal/hal_camera.h"
#include "frame_adapter.h"
#include "vision_geom.h"
#include "profile_store.h"

namespace RobofestDrone {

// ============================================================================
// MARKER PROFILE STRUCTURE (RUNTIME - initialized from Config::VISION_PROFILE_TABLE)
// ============================================================================

struct VisionMarkerProfile {
    Types::VisionMarkerType profile_type = Types::VisionMarkerType::UNKNOWN;
    bool enabled = true;

    // Primary HSV band ("sunny" calibration). Hue scale 0-180 like OpenCV.
    uint8_t h_min = 0;
    uint8_t h_max = 15; // h_min > h_max => wraparound band
    uint8_t s_min = 100;
    uint8_t s_max = 255;
    uint8_t v_min = 100;
    uint8_t v_max = 255;

    // Alternate HSV band ("overcast" calibration), selected by adaptive lighting.
    bool has_alt_band = false;
    uint8_t alt_h_min = 0;
    uint8_t alt_h_max = 15;
    uint8_t alt_s_min = 100;
    uint8_t alt_s_max = 255;
    uint8_t alt_v_min = 100;
    uint8_t alt_v_max = 255;

    float min_area_px = Config::BLOB_AREA_MIN_PX;
    float max_area_px = Config::BLOB_AREA_MAX_PX;
    float circularity_min = Config::CIRCULARITY_MIN;
    float confidence_bias = 0.0f;
    uint16_t expected_marker_area_px = 400;

    // Shape descriptor gates (Step 8)
    float aspect_min = 1.0f;   // normalized bbox ratio max(w,h)/min(w,h) >= 1
    float aspect_max = 10.0f;
    float extent_min = 0.0f;   // blob area / bbox area
    float extent_max = 1.05f;
    float solidity_min = 0.0f; // blob area / convex hull area
    uint8_t corners_min = 0;   // corners_min == 0 disables corner gating & DP pass
    uint8_t corners_max = 255;
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
    // Shape descriptors (Steps 7-9): all derived from the BFS pass data.
    float aspect_ratio = 1.0f; // normalized bbox ratio, >= 1.0
    float extent = 1.0f;       // area / bbox area
    float solidity = 1.0f;     // area / convex hull area
    uint8_t corner_count = 0;  // DP vertices on hull polyline (0 = not computed)
    Types::VisionMarkerType profile_type = Types::VisionMarkerType::UNKNOWN;
    bool valid = false;
    // Phase 3 additions (items 7/8):
    float concavity_depth = -1.0f;  // >0.25 indicates star/cross-like concavity
    uint8_t contour_corner_count = 0; // corners from raw traced contour
    float shadow_ratio = 0.0f;        // fraction of shadow-flagged px in bbox
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
    float extent = 0.0f;
    float solidity = 0.0f;
    uint8_t corner_count = 0;
    uint16_t persistence_count = 0;
    uint32_t last_seen_ms = 0;
    Types::VisionMarkerType marker_type = Types::VisionMarkerType::UNKNOWN;
    uint32_t frame_id = 0;
    bool active = false;
};

// Lighting variant selection (Step 6)
enum class VisionLightingMode : uint8_t {
    SUNNY_PRIMARY = 0,
    OVERCAST_ALT = 1
};


// ============================================================================
// TESTABLE DESCRIPTOR / SCORING HELPERS (free functions, no pipeline state)
// ============================================================================

bool vision_hsv_in_band(
    uint8_t h, uint8_t s, uint8_t v,
    uint8_t h_min, uint8_t h_max,
    uint8_t s_min, uint8_t s_max,
    uint8_t v_min, uint8_t v_max);

// Soft range score: 1 inside [lo,hi], linear falloff over a margin outside.
float vision_range_score(float value, float lo, float hi);

// Soft floor score: 1 at/above floor, linear falloff below it.
float vision_floor_score(float value, float floor_value);

// Corner-count score against [min,max] gate; neutral 1.0 when gating disabled.
float vision_corner_score(uint8_t corners, uint8_t cmin, uint8_t cmax);

// Weighted blend of aspect/extent/solidity/corner match for one blob/profile pair.
float vision_shape_match(
    float aspect, float extent, float solidity, uint8_t corners,
    const VisionMarkerProfile& profile);

// Confidence formula (Step 8): shape bucket + area bucket + bias, clamped 0..100.
float vision_blob_confidence(const VisionBlob& blob, const VisionMarkerProfile& profile);

// Shoelace area of a simple polygon.
float vision_polygon_area(const VisionPoint* pts, uint8_t n);

// Monotone-chain convex hull. Returns hull vertex count (<= out_cap).
uint8_t vision_convex_hull(const VisionPoint* pts, uint8_t n, VisionPoint* out, uint8_t out_cap);

// Douglas-Peucker vertex count on a closed polyline (e.g. a hull).
uint8_t vision_poly_corner_count(const VisionPoint* poly, uint8_t n, float epsilon);


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

    // Legacy focus API: enables ONLY the matching profile row.
    void setActiveProfile(Types::VisionMarkerType profile);
    Types::VisionMarkerType getActiveProfile() const { return active_profile_type_; }

    // Multi-profile scanning (default mode): every enabled row runs each frame.
    void setAllProfilesEnabled(bool enabled);
    void setProfileEnabled(Types::VisionMarkerType type, bool enabled);

    uint8_t getProfileCount() const { return profile_count_; }
    VisionMarkerProfile* getProfileByIndex(uint8_t index);
    const VisionMarkerProfile* getProfileByIndex(uint8_t index) const;
    VisionMarkerProfile* getProfileByType(Types::VisionMarkerType type);
    const VisionMarkerProfile* getProfileByType(Types::VisionMarkerType type) const;
    void restoreProfileDefaults();

    VisionLightingMode getLightingMode() const { return lighting_mode_; }
    float getLastFrameMeanV() const { return frame_mean_v_; }
    float getLastExposureGain() const { return exposure_gain_; }

    // Night/low-light operation (REQ-DER-115): auto-engaged by the pipeline
    // from frame mean-V with hysteresis; isNightModeActive() reports state.
    bool isNightModeActive() const { return night_active_; }
    float getLastHazeSeverity() const { return last_haze_severity_; }
    uint16_t getLastShadowPixelCount() const { return last_shadow_pixels_; }

    // Profile persistence (REQ-DER-106): CRC-checked save/load of calibrated
    // bands to flash; load falls back to defaults on MISSING/CORRUPT.
    bool saveCalibratedProfiles();
    ProfileStoreStatus loadCalibratedProfiles();

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
    void measureExposureAndLighting(const Hal::CameraFrame& frame, uint32_t now_ms);
    void segmentMultiLabelMask(const Hal::CameraFrame& frame);
    void applyMorphologyCleanup();
    uint8_t extractBlobs();
    void projectToWorld(float pixel_x, float pixel_y, float altitude_m, const Types::Pose2D& drone_pose, const Types::AttitudeSample& attitude, float& out_world_x, float& out_world_y, bool& out_valid);
    void updatePersistenceTracks(const VisionBlob* blobs, uint8_t blob_count, const Types::Pose2D& drone_pose, float fused_altitude_m, const Types::AttitudeSample& attitude, uint32_t now_ms);
    void pruneStaleTracks(uint32_t now_ms);
    void setTelemetryEvent(uint16_t event_id);

private:
    VisionMarkerProfile profiles_[Config::VISION_PROFILE_MAX];
    uint8_t profile_count_ = 0;
    Types::VisionMarkerType active_profile_type_ = Types::VisionMarkerType::ON_GROUND_MINE;
    bool scan_all_profiles_ = true;

    // Label map: 0 = background, n+1 = profiles_[n]. Reuses the old mask buffer.
    uint32_t labels_present_bits_ = 0;
    uint32_t label_pixel_counts_[Config::VISION_PROFILE_MAX] = {};

    VisionLightingMode lighting_mode_ = VisionLightingMode::SUNNY_PRIMARY;
    uint32_t last_lighting_switch_ms_ = 0;
    uint32_t last_exposure_log_ms_ = 0;
    float frame_mean_v_ = 0.0f;
    float exposure_gain_ = 1.0f;
    float gain_r_ = 1.0f;
    float gain_g_ = 1.0f;
    float gain_b_ = 1.0f;

    // Phase 3 state: enhancement chain (items 5/8/15)
    bool night_active_ = false;
    uint32_t last_night_switch_ms_ = 0;
    float last_haze_severity_ = 0.0f;
    uint16_t last_shadow_pixels_ = 0;
    FrameTransform frame_tf_;

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
