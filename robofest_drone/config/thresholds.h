#pragma once

#include <stdint.h>

namespace RobofestDrone {
namespace Config {

// ============================================================================
// HSV COLOR RANGE STRUCT
// ============================================================================

struct HsvColor {
    uint8_t h = 0;
    uint8_t s = 0;
    uint8_t v = 0;

    constexpr HsvColor() : h(0), s(0), v(0) {}
    constexpr HsvColor(uint8_t _h, uint8_t _s, uint8_t _v) : h(_h), s(_s), v(_v) {}
};


// ============================================================================
// LOCALIZATION THRESHOLDS
// ============================================================================

// tunable implementation default
constexpr float OPTICAL_FLOW_FOCAL_LENGTH_PX = 400.0f;

// tunable implementation default
constexpr float TOF_ALTITUDE_FILTER_ALPHA = 0.85f;

// tunable implementation default
constexpr float FLOW_QUALITY_MIN = 0.30f;

// tunable implementation default
constexpr float DRIFT_UNCERTAINTY_LIMIT_M = 1.0f;

// tunable implementation default
constexpr float DRIFT_UNCERTAINTY_RECOVERED_M = 0.60f;

// tunable implementation default
constexpr float DRIFT_UNCERTAINTY_MAX_M = 5.0f;

// tunable implementation default
constexpr float MIN_FLOW_ALTITUDE_M = 0.20f;

// tunable implementation default
constexpr uint32_t MAX_VALID_DT_MS = 200UL;

// tunable implementation default
constexpr uint32_t TOF_TIMEOUT_MS = 500UL;

// tunable implementation default
constexpr uint32_t FLOW_TIMEOUT_MS = 500UL;

// tunable implementation default
constexpr float ALTITUDE_BIAS_DEFAULT_M = 0.0f;

// tunable implementation default
constexpr float FLOW_ALIGNMENT_X_DEFAULT = 0.0f;

// tunable implementation default
constexpr float FLOW_ALIGNMENT_Y_DEFAULT = 0.0f;

// tunable implementation default
constexpr float MAX_PIXEL_SHIFT_PER_UPDATE_PX = 100.0f;

// tunable implementation default
constexpr float MAX_TILT_ANGLE_DEG = 12.0f;

// tunable implementation default
constexpr float DRIFT_GROWTH_RATE_MPS = 0.15f;

// tunable implementation default
constexpr float DRIFT_RECOVERY_RATE_MPS = 0.05f;

// derived from uploaded logic
constexpr uint16_t LOCALIZATION_RATE_HZ = 50;


// ============================================================================
// SOFTWARE GEOFENCE THRESHOLDS
// ============================================================================

// tunable implementation default
constexpr float GEOFENCE_NEAR_LIMIT_BAND_M = 0.25f;

// tunable implementation default
constexpr float GEOFENCE_UNCERTAINTY_MARGIN_SCALE = 1.0f;

// tunable implementation default
constexpr float GEOFENCE_MAX_UNCERTAINTY_MARGIN_M = 1.5f;

// tunable implementation default
constexpr float GEOFENCE_MIN_EFFECTIVE_WIDTH_M = 2.0f;

// tunable implementation default
constexpr float GEOFENCE_MAX_CORRECTION_SPEED_MPS = 1.0f;

// tunable implementation default
constexpr float GEOFENCE_CORRECTION_GAIN = 0.35f;

// tunable implementation default
constexpr uint32_t GEOFENCE_STATUS_HYSTERESIS_MS = 200UL;

// tunable implementation default
constexpr float GEOFENCE_SPEED_REDUCTION_FACTOR_WARNING = 0.75f;

// tunable implementation default
constexpr float GEOFENCE_SPEED_REDUCTION_FACTOR_NEAR_LIMIT = 0.50f;

// tunable implementation default
constexpr float GEOFENCE_SPEED_REDUCTION_FACTOR_OUTSIDE = 0.25f;

// tunable implementation default
constexpr float MAX_HORIZONTAL_SPEED_MPS = 1.5f;


// ============================================================================
// VISION THRESHOLDS
// ============================================================================

// tunable implementation default
 constexpr uint16_t VISION_RATE_HZ = 15;

// tunable implementation default
 constexpr bool RGB565_LE_BYTE_ORDER = true; // true = LSB first (this code's assumption), false = MSB first
 // TODO: Verify empirically with camera module - test with pure red, green, blue targets
 // and confirm computed hue values match expectations before trusting HSV thresholds.

// tunable implementation default

// tunable implementation default
constexpr uint16_t VISION_PROCESS_WIDTH = 160;

// tunable implementation default
constexpr uint16_t VISION_PROCESS_HEIGHT = 120;

// tunable implementation default
constexpr uint8_t VISION_MAX_BLOBS = 24;

// tunable implementation default
constexpr uint8_t VISION_MAX_CANDIDATES = 16;

// Re-tuned (Step 11): multi-color scanning yields more simultaneous blobs,
// so the track pool was widened from 24 to 32 slots.
constexpr uint8_t VISION_MAX_PERSISTENCE_TRACKS = 32;

// tunable implementation default
constexpr uint16_t IMAGE_WIDTH = 320;

// tunable implementation default
constexpr uint16_t IMAGE_HEIGHT = 240;

// tunable implementation default
constexpr float H_FOV_DEG = 60.0f;

// tunable implementation default
constexpr float V_FOV_DEG = 45.0f;

// tunable implementation default
constexpr float IMAGE_CENTER_X = 160.0f;

// tunable implementation default
constexpr float IMAGE_CENTER_Y = 120.0f;

// derived from uploaded logic
constexpr float CIRCULARITY_MIN = 0.70f;

// tunable implementation default
constexpr float BLOB_AREA_MIN_PX = 25.0f;

// tunable implementation default
constexpr float BLOB_AREA_MAX_PX = 2500.0f;

// tunable implementation default
constexpr float CONFIDENCE_REPORT_MIN = 50.0f;
// Re-tuned (Step 11): with many profiles scanning simultaneously the noise floor
// rises, so the reporting bar was nudged from 45 to 50.

// derived from uploaded logic
constexpr float PERSISTENCE_RADIUS_M = 0.25f;

// tunable implementation default
constexpr uint16_t PERSISTENCE_COUNT_MIN = 5;

// tunable implementation default
constexpr uint32_t PERSISTENCE_TIMEOUT_MS = 1500UL;

// tunable implementation default
constexpr float MARKER_AREA_NOMINAL_PX = 400.0f;

// tunable implementation default
constexpr float EDGE_REJECT_MARGIN_PX = 8.0f;

// tunable implementation default
constexpr bool GLARE_REJECT_ENABLED = true;

// tunable implementation default
constexpr float GLARE_AREA_MAX_PX = 5000.0f;

// tunable implementation default
constexpr bool VISION_DOWNSCALE_ENABLED = true;

// --- Phase 3 enhancement chain additions (complements ITEM 8/15 blocks
// further below; REQ-DER-108/115 + dehazing) ---

// Gamma applied to the working buffer while in night mode (<1 brightens).
constexpr float VISION_NIGHT_GAMMA = 0.65f;

// Exit threshold for night mode (must exceed VISION_NIGHT_MEAN_V_MAX).
constexpr uint8_t VISION_NIGHT_MEAN_V_EXIT = 78;

// Dehazing engages above this severity (0..1, from dark-channel estimate).
constexpr float VISION_DEHAZE_SEVERITY_MIN = 0.45f;

// Confidence penalty (0..100 scale) for blobs dominated by shadow-flagged px.
constexpr float VISION_SHADOW_PENALTY_CONF = 12.0f;

// Minimum blob area (native px) before contour tracing runs (item 7 cost gate).
constexpr uint16_t VISION_CONTOUR_MIN_BLOB_AREA_PX = 48;

// Turn-angle threshold for contour-based corner counting (degrees).
constexpr float VISION_CONTOUR_CORNER_ANGLE_DEG = 35.0f;

// tunable implementation default
constexpr bool VISION_ATTITUDE_COMPENSATION_ENABLED = true;

// tunable implementation default
constexpr float MIN_PROJECTION_ALTITUDE_M = 0.25f;

// --- Morphological noise cleanup on binary mask (erode then dilate) ---
// tunable implementation default
constexpr bool VISION_MORPHOLOGY_ENABLED = true;

// tunable implementation default (radius=1 means 3x3 kernel)
constexpr uint8_t VISION_MORPHOLOGY_RADIUS = 1;

// --- Spatial blur (3x3 box average) on RGB before HSV conversion ---
// tunable implementation default
constexpr bool VISION_BLUR_ENABLED = true;

// --- Camera exposure / white-balance locking ---
// tunable implementation default
constexpr bool CAMERA_AUTO_EXPOSURE_ENABLED = false;

// tunable implementation default (0 = use HAL/sensor default)
constexpr int32_t CAMERA_MANUAL_EXPOSURE_VALUE = 0;

// tunable implementation default
constexpr bool CAMERA_AUTO_WHITEBALANCE_ENABLED = false;


// ============================================================================
// MINE MARKER HSV THRESHOLD PLACEHOLDERS
// ============================================================================

// tunable implementation default
constexpr HsvColor ON_GROUND_MINE_HSV_LOW = {0, 100, 100};

// tunable implementation default
constexpr HsvColor ON_GROUND_MINE_HSV_HIGH = {15, 255, 255};

// tunable implementation default
constexpr HsvColor BURIED_SURFACE_MARKER_HSV_LOW = {25, 120, 120};

// tunable implementation default
constexpr HsvColor BURIED_SURFACE_MARKER_HSV_HIGH = {40, 255, 255};


// ============================================================================
// VISION PROFILE TABLE / SHAPE / LIGHTING / EXPOSURE CONFIGURATION
// ============================================================================

// Maximum number of marker profiles in the runtime table (data-driven, Step 1).
constexpr uint8_t VISION_PROFILE_MAX = 16;

// --- Shape descriptor pipeline (Steps 7-9) ---
// Corner counting via Douglas-Peucker on the blob boundary polyline.
constexpr bool VISION_CORNER_DETECT_ENABLED = true;

// tunable implementation default (epsilon = ratio * bbox diagonal)
constexpr float SHAPE_CORNER_EPSILON_RATIO = 0.035f;

// Static buffers for boundary-pixel collection and convex hull computation.
constexpr uint16_t VISION_BOUNDARY_POINTS_MAX = 384;
constexpr uint16_t VISION_HULL_POINTS_MAX = 192;

// Labels with fewer pixels than this skip the morphology pass entirely.
// Must stay below BLOB_AREA_MIN_PX expressed in process-resolution pixels
// (25 full-frame px / (2x2 downscale)^2 = ~6) so real markers are never erased.
constexpr uint16_t VISION_MORPH_MIN_LABEL_PX = 6;

// --- Confidence blend weights (Step 8) ---
// confidence = min(circ_term) + shape_match*W + area_term + bias, clamped 0..100.
constexpr float CONF_WEIGHT_CIRCULARITY = 24.0f;
constexpr float CONF_WEIGHT_SHAPE_MATCH = 36.0f;
constexpr float CONF_WEIGHT_AREA = 40.0f;

// Circularity term saturates when it reaches this value.
constexpr float CONF_CIRC_PERFECT_AT = 0.85f;

// Soft falloff margin for descriptor gates: outside [min,max] the score decays
// linearly over this fraction of the gate span before reaching zero.
constexpr float CONF_GATE_SOFT_MARGIN_RATIO = 0.25f;


// ============================================================================
// CONCAVE / COMPLEX SHAPE CLASSIFICATION (ITEM 7, REQ-DER-107)
// ============================================================================

// Ordered contour tracing on the raw blob boundary (preserves concavities).
constexpr bool VISION_CONTOUR_TRACE_ENABLED = true;

// Contour vertices kept per traced boundary (bounded static scratch).
constexpr uint16_t VISION_CONTOUR_POINTS_MAX = 512;

// A concavity counts as a defect when deeper than this ratio of the bbox
// diagonal, and star/concave classification needs at least this many defects.
constexpr float SHAPE_DEFECT_DEPTH_MIN_RATIO = 0.07f;
constexpr uint8_t SHAPE_STAR_MIN_DEFECTS = 3;

constexpr float SHAPE_STAR_SOLIDITY_MAX = 0.80f;
constexpr float SHAPE_LINE_ASPECT_MIN = 3.5f;
constexpr float SHAPE_CIRCLE_CIRCULARITY_MIN = 0.82f;
constexpr float SHAPE_POLY_SOLIDITY_MIN = 0.85f;


// ============================================================================
// ADVANCED LIGHTING PREPROCESSING (ITEM 8, REQ-DER-108)
// ============================================================================

// Tile-based contrast-limited histogram equalization on the V channel,
// applied after exposure normalization and before HSV segmentation.
constexpr bool VISION_CLAHE_ENABLED = true;

// Clip limit expressed as a multiple of the uniform-histogram level.
constexpr uint8_t VISION_CLAHE_CLIP_LIMIT = 4;

// Tile grid is chosen so each tile is roughly this many process-pixels wide.
constexpr uint8_t VISION_CLAHE_TILE_TARGET_PX = 40;

// Gray-world white balance applied to the adapted RGB working buffer.
constexpr bool VISION_AWB_GRAY_WORLD_ENABLED = true;
constexpr float VISION_AWB_STRENGTH = 0.6f;

// Chromaticity-based shadow detection: a dark pixel whose saturation stays
// below S_MAX while its value collapses under the local mean is classified a
// physical shadow rather than a dark object. Used to gate the BLACK profile
// so ground shadows stop registering as black markers.
constexpr bool VISION_SHADOW_GATING_ENABLED = true;
constexpr uint8_t VISION_SHADOW_V_RATIO_PCT = 55;
constexpr uint8_t VISION_SHADOW_S_MAX = 90;

// ============================================================================
// NIGHT / LOW-LIGHT OPERATION (ITEM 15, REQ-DER-115)
// ============================================================================

// Temporal exponential blend applied to the adapted frame: new frame weight =
// NUM/DEN (e.g. 1/4). Suppresses heavy low-shutter sensor noise.
constexpr bool VISION_NIGHT_TEMPORAL_BLEND = true;
constexpr uint8_t VISION_NIGHT_BLEND_NUM = 1;
constexpr uint8_t VISION_NIGHT_BLEND_DEN = 4;

// Threshold relaxation multipliers engaged while night mode is active:
// sensor noise raises the false-positive floor, so the reporting bar and
// circularity gate are eased instead of missing real markers.
constexpr float NIGHT_CONFIDENCE_REPORT_SCALE = 0.85f;
constexpr float NIGHT_CIRCULARITY_BIAS = -0.05f;

// Mean-V below this engages night mode automatically (hysteresis shared with
// the sunny/overcast selector).
constexpr uint8_t VISION_NIGHT_MEAN_V_MAX = 60;


// ============================================================================
// FRAME ADAPTER (ITEM 5, REQ-DER-105)
// ============================================================================

// Route every camera frame through aspect-correct conversion + bilinear
// resize into the working grid before segmentation. When disabled or on
// allocation failure the legacy strided-sampling path is used unchanged.
constexpr bool VISION_FRAME_ADAPTER_ENABLED = true;


// ============================================================================
// EXPOSURE NORMALIZATION & LIGHTING FALLBACK (STEPS 5-6)
// ============================================================================

// Software gain normalization: frame mean-V is measured on a subsample grid and
// a clamped scalar gain drives RGB toward the target mean before HSV conversion.
constexpr bool VISION_EXPOSURE_NORM_ENABLED = true;

// tunable implementation default
constexpr uint8_t VISION_EXPOSURE_TARGET_MEAN_V = 135;

// tunable implementation default
constexpr float VISION_EXPOSURE_GAIN_MIN = 0.70f;

// tunable implementation default
constexpr float VISION_EXPOSURE_GAIN_MAX = 1.50f;

// Lighting-condition fallback: each profile carries two calibrated bands.
// Frame mean-V >= threshold selects "sunny" (primary); below selects "overcast" (alt).
constexpr bool VISION_LIGHTING_ADAPTIVE_ENABLED = true;

// tunable implementation default
constexpr uint8_t VISION_LIGHTING_V_SUNNY_MIN = 115;

// Minimum interval between lighting mode flips (anti-flicker hysteresis).
constexpr uint32_t VISION_LIGHTING_HYSTERESIS_MS = 2000UL;


// ============================================================================
// MINE MAP THRESHOLDS
// ============================================================================

// tunable implementation default
constexpr float SAME_DRONE_DEDUP_RADIUS_M = 0.30f;

// tunable implementation default
constexpr float CROSS_DRONE_DEDUP_RADIUS_M = 0.50f;

// tunable implementation default
constexpr float MINE_CONFIRM_CONFIDENCE_MIN = 70.0f;

// tunable implementation default
constexpr uint16_t MINE_CONFIRM_PERSISTENCE_MIN = 5;

// tunable implementation default
constexpr uint32_t MINE_STALE_TIMEOUT_MS = 15000UL;

// tunable implementation default
constexpr uint32_t CANDIDATE_DECAY_INTERVAL_MS = 1000UL;

// tunable implementation default
constexpr float CANDIDATE_CONFIDENCE_DECAY_FACTOR = 0.90f;

// tunable implementation default
constexpr float CANDIDATE_MIN_CONFIDENCE_AFTER_DECAY = 20.0f;

// tunable implementation default
constexpr float MINE_FUSION_POSITION_GAIN = 0.30f;

// tunable implementation default
constexpr float MINE_FUSION_CONFIDENCE_GAIN = 0.40f;

// tunable implementation default
constexpr uint32_t MINE_MAP_VERSION_START = 0;

// tunable implementation default
constexpr float CLAIM_READY_CONFIDENCE_MIN = 65.0f;

// tunable implementation default
constexpr uint16_t CLAIM_READY_PERSISTENCE_MIN = 4;

// tunable implementation default
constexpr uint32_t CLAIM_TIMEOUT_MS = 5000UL;

// tunable implementation default
constexpr float MINE_MAP_BOUNDARY_MARGIN_M = 1.0f;

// tunable implementation default
constexpr bool MINE_REJECT_OUTSIDE_FIELD = true;

// tunable implementation default
constexpr uint32_t MINE_DETECTION_MAX_AGE_MS = 500UL;


// ============================================================================
// PATH PLANNER THRESHOLDS
// ============================================================================

// tunable implementation default
constexpr float PATH_GRID_RESOLUTION_M = 0.5f;

// tunable implementation default
constexpr uint16_t PATH_MAX_GRID_COLS = 32;

// tunable implementation default
constexpr uint16_t PATH_MAX_GRID_ROWS = 122;

// tunable implementation default
constexpr uint16_t PATH_MAX_GRID_CELLS = 3904;

// tunable implementation default
constexpr uint8_t PATH_MAX_WAYPOINTS = 32;

// tunable implementation default
constexpr uint16_t PATH_MAX_ASTAR_NODES = 3904;

// tunable implementation default
constexpr uint8_t PATH_SMOOTHING_ITERATIONS = 2;

// tunable implementation default
constexpr float PATH_EXACT_CLEARANCE_STEP_M = 0.25f;

// tunable implementation default
constexpr float PATH_CORRIDOR_WIDTH_M = 1.0f;

// tunable implementation default
constexpr float PATH_DEVIATION_TOLERANCE_M = 0.75f;

// tunable implementation default
constexpr uint32_t PATH_REPLAN_COOLDOWN_MS = 2000UL;

// tunable implementation default
constexpr float PATH_MIN_SEGMENT_LENGTH_M = 0.35f;

// tunable implementation default
constexpr float PATH_MAX_SEGMENT_LENGTH_M = 3.0f;

// tunable implementation default
constexpr float PATH_OBSTACLE_INFLATION_RADIUS_M = 0.5f;

// tunable implementation default
constexpr bool PATH_ALLOW_DIAGONAL_MOVEMENT = true;

// tunable implementation default
constexpr bool PATH_PREVENT_CORNER_CUTTING = true;

// tunable implementation default
constexpr float PATH_EDGE_COST_STRAIGHT = 1.0f;

// tunable implementation default
constexpr float PATH_EDGE_COST_DIAGONAL = 1.41421356f;

// tunable implementation default
constexpr float PATH_START_ANCHOR_X = 7.5f;

// tunable implementation default
constexpr float PATH_START_ANCHOR_Y = 0.5f;

// tunable implementation default
constexpr float PATH_EXIT_ANCHOR_X = 7.5f;

// tunable implementation default
constexpr float PATH_EXIT_ANCHOR_Y = 59.5f;

// tunable implementation default
constexpr bool PATH_REQUIRE_MINE_OBSERVATION_BEFORE_GUIDING = true;


// ============================================================================
// COMMAND RECOGNITION THRESHOLDS
// ============================================================================

// official or derived challenge constants
constexpr bool COMMAND_START_REQUIRED = true;
constexpr bool COMMAND_FORWARD_SUPPORTED = true;
constexpr bool COMMAND_PAUSE_SUPPORTED = true;
constexpr bool COMMAND_SCAN_LEFT_SUPPORTED = true;
constexpr bool COMMAND_SCAN_RIGHT_SUPPORTED = true;
constexpr bool COMMAND_STOP_ABORT_SUPPORTED = true;

// tunable implementation default
constexpr float START_CONFIDENCE_MIN = 0.70f;

// tunable implementation default
constexpr float FORWARD_CONFIDENCE_MIN = 0.60f;

// tunable implementation default
constexpr float PAUSE_CONFIDENCE_MIN = 0.80f;

// tunable implementation default
constexpr float SCAN_CONFIDENCE_MIN = 0.60f;

// tunable implementation default
constexpr float STOP_CONFIDENCE_MIN = 0.90f;

// tunable implementation default
constexpr uint8_t COMMAND_HYSTERESIS_FRAMES = 3;

// tunable implementation default
constexpr uint32_t COMMAND_DEBOUNCE_MS = 300UL;

// tunable implementation default
constexpr uint32_t COMMAND_LOCKOUT_MS = 1500UL;

// tunable implementation default
constexpr uint32_t STOP_COMMAND_LOCKOUT_MS = 5000UL;

// tunable implementation default
constexpr uint32_t COMMAND_VALID_WINDOW_MS = 1000UL;

// tunable implementation default
constexpr float COMMAND_SOURCE_CONFLICT_MARGIN = 0.15f;

// tunable implementation default
constexpr float COMMAND_FUSION_BONUS = 0.10f;

// tunable implementation default
constexpr uint32_t COMMAND_STALE_SAMPLE_TIMEOUT_MS = 300UL;

// tunable implementation default
constexpr uint32_t SCAN_COMMAND_ACTIVE_MS = 5000UL;

// tunable implementation default
constexpr uint32_t COMMAND_NONE_GRACE_MS = 100UL;

// tunable implementation default
constexpr bool GESTURE_ENABLED_DEFAULT = true;

// tunable implementation default
constexpr bool VOICE_ENABLED_DEFAULT = false;

// tunable implementation default
constexpr bool DEBUG_COMMANDS_ENABLED_DEFAULT = false;

// tunable implementation default
constexpr bool START_ALLOW_ONLY_BEFORE_TAKEOFF = true;

// tunable implementation default
constexpr bool PAUSE_ALLOW_DURING_FLIGHT_ONLY = true;

// tunable implementation default
constexpr bool STOP_ALLOW_ALWAYS = true;

// tunable implementation default
constexpr bool STOP_REQUIRE_DOUBLE_CONFIRM = false;

// tunable implementation default
constexpr uint32_t STOP_DOUBLE_CONFIRM_WINDOW_MS = 3000UL;


// ============================================================================
// HUMAN TRACKING THRESHOLDS
// ============================================================================

// tunable implementation default
constexpr uint8_t HUMAN_TRACK_RATE_HZ = 20;

// tunable implementation default
constexpr float HUMAN_MIN_DETECTION_CONFIDENCE = 0.55f;

// tunable implementation default
constexpr float HUMAN_TRACK_CONFIDENCE_MIN = 0.60f;

// tunable implementation default
constexpr float HUMAN_TRACK_ALPHA = 0.35f;

// tunable implementation default
constexpr float HUMAN_VELOCITY_ALPHA = 0.25f;

// tunable implementation default
constexpr uint32_t HUMAN_LOST_TIMEOUT_MS = 1500UL;

// tunable implementation default
constexpr float HUMAN_OFF_PATH_DISTANCE_M = 0.75f;

// tunable implementation default
constexpr uint32_t HUMAN_OFF_PATH_DEBOUNCE_MS = 500UL;

// tunable implementation default
constexpr uint32_t HUMAN_RECOVERY_TIMEOUT_MS = 3000UL;

// tunable implementation default
constexpr uint32_t HUMAN_EXIT_CONFIRM_TIMEOUT_MS = 2000UL;

// tunable implementation default
constexpr float HUMAN_MAX_PLAUSIBLE_SPEED_MPS = 2.5f;

// tunable implementation default
constexpr float HUMAN_PROXIMITY_WARNING_M = 1.0f;

// tunable implementation default
constexpr float HUMAN_PROXIMITY_CRITICAL_M = 0.5f;

// tunable implementation default
constexpr float HUMAN_EXIT_ZONE_MARGIN_M = 0.10f;

// tunable implementation default
constexpr bool HUMAN_USE_FOOT_POINT = true;

// tunable implementation default
constexpr float HUMAN_PIXEL_NOISE_LIMIT_PX = 80.0f;

// ============================================================================
// HUMAN MOTION DETECTOR (ITEM 2 CLASSICAL BACKEND, REQ-DER-102)
// ============================================================================

// Grayscale frame-difference threshold for motion pixels.
constexpr uint8_t HUMAN_DIFF_THRESHOLD = 18;

// Motion components smaller than this many working pixels are noise.
constexpr uint16_t HUMAN_MOTION_MIN_BLOB_PX = 8;

// Motion covering more than this fraction of the frame is global change
// (shake / flicker), not a person.
constexpr float HUMAN_MOTION_MAX_AREA_RATIO = 0.25f;

// Bounding-box aspect gate (people from above stay roughly compact).
constexpr float HUMAN_MOTION_MAX_ASPECT = 3.5f;

// Consecutive plausible frames required before emitting a detection.
constexpr uint8_t HUMAN_MOTION_CONFIRM_HITS = 2;

// Minimum interval between emitted detections from the same walk-by.
constexpr uint32_t HUMAN_MOTION_REFRACTORY_MS = 1200UL;

// Neural-model path: run ESP-DL pedestrian inference every Nth vision cycle
// so the classical detector owns the remaining slots.
constexpr uint8_t HUMAN_MODEL_RUN_DIVIDER = 4;

// Confidence floor applied to model outputs before they leave the HAL.
constexpr float HUMAN_MODEL_CONFIDENCE_MIN = 0.55f;

// ============================================================================
// SWARM COMMUNICATION THRESHOLDS
// ============================================================================

// official or derived swarm requirements
constexpr uint8_t MIN_SWARM_DRONES = 3;
constexpr uint8_t MAX_SWARM_DRONES = 4;

// tunable implementation default
constexpr uint32_t PEER_DEGRADED_TIMEOUT_MS = 1000UL;

// tunable implementation default
constexpr uint32_t CLAIM_SUPPRESSION_MS = 8000UL;

// tunable implementation default
constexpr uint32_t MINE_UPDATE_PERIOD_MS = 1000UL;

// tunable implementation default
constexpr uint32_t PERSON_UPDATE_PERIOD_MS = 500UL;

// tunable implementation default
constexpr uint32_t PATH_UPDATE_PERIOD_MS = 1000UL;

// tunable implementation default
constexpr uint32_t ROLE_REASSIGN_TIMEOUT_MS = 3000UL;

// tunable implementation default
constexpr uint8_t RADIO_TX_RETRY_LIMIT = 2;

// tunable implementation default
constexpr uint8_t PATH_UPDATE_CHUNK_MAX_WAYPOINTS = 4;

// tunable implementation default
constexpr float CLAIM_STRENGTH_MARGIN = 5.0f;

// tunable implementation default
constexpr float MINE_HASH_GRID_RESOLUTION_M = 0.10f;

// tunable implementation default
constexpr uint32_t SWARM_STALE_TIMESTAMP_TOLERANCE_MS = 5000UL;

// tunable implementation default
constexpr uint8_t LANE_NONE = 0;
constexpr uint8_t LANE_LEFT = 1;
constexpr uint8_t LANE_RIGHT = 2;
constexpr uint8_t LANE_CENTER = 3;

// ============================================================================
// SAFETY MANAGER THRESHOLDS
// ============================================================================

// tunable implementation default
constexpr float BATTERY_HYSTERESIS_VOLTAGE = 0.10f;

// tunable implementation default
constexpr uint32_t BATTERY_FAULT_DEBOUNCE_MS = 1000UL;

// tunable implementation default
constexpr uint32_t FC_LINK_LOST_LAND_MS = 2000UL;

// tunable implementation default
constexpr uint32_t FC_LINK_LOST_EMERGENCY_MS = 5000UL;

// tunable implementation default
constexpr uint32_t CAMERA_STALL_HOLD_MS = 2000UL;

// tunable implementation default
constexpr uint32_t CAMERA_STALL_LAND_MS = 5000UL;

// tunable implementation default
constexpr uint32_t RADIO_TIMEOUT_HOLD_MS = 5000UL;

// tunable implementation default
constexpr uint32_t SWARM_CRITICAL_HOLD_MS = 5000UL;

// tunable implementation default
constexpr float LOCALIZATION_DEGRADED_UNCERTAINTY_M = 0.75f;

// tunable implementation default
constexpr float LOCALIZATION_UNRECOVERABLE_UNCERTAINTY_M = 1.0f;

// tunable implementation default
constexpr uint32_t LOCALIZATION_FAULT_DEBOUNCE_MS = 500UL;

// tunable implementation default
constexpr uint32_t GEOFENCE_OUTSIDE_DEBOUNCE_MS = 200UL;

// tunable implementation default
constexpr bool GEOFENCE_WARNING_HOLD_ENABLED = false;

// tunable implementation default
constexpr bool GEOFENCE_OUTSIDE_LAND_ENABLED = true;

// tunable implementation default
constexpr float COLLISION_RISK_PEER_DISTANCE_M = 1.5f;

// tunable implementation default
constexpr float COLLISION_RISK_PEER_DISTANCE_CRITICAL_M = 0.75f;

// tunable implementation default
constexpr float UNSAFE_PROXIMITY_HUMAN_DISTANCE_M = 1.0f;

// tunable implementation default
constexpr float UNSAFE_PROXIMITY_HUMAN_CRITICAL_M = 0.5f;

// tunable implementation default
constexpr uint32_t PROXIMITY_FAULT_DEBOUNCE_MS = 300UL;

// tunable implementation default
constexpr float SURFACE_CONTACT_ALTITUDE_M = 0.12f;

// tunable implementation default
constexpr uint32_t SURFACE_CONTACT_DEBOUNCE_MS = 500UL;

// tunable implementation default
constexpr uint32_t WATCHDOG_MODULE_UPDATE_TIMEOUT_MS = 500UL;

// tunable implementation default
constexpr uint32_t SAFETY_FAULT_CLEAR_HYSTERESIS_MS = 1000UL;

// tunable implementation default
constexpr bool EMERGENCY_LATCH_UNTIL_RESET = true;

// ============================================================================
// FLIGHT CONTROLLER BRIDGE THRESHOLDS
// ============================================================================

// tunable implementation default
constexpr uint32_t FC_UART_BAUDRATE = 115200;

// tunable implementation default
constexpr uint8_t FC_PROTOCOL_VERSION = 1;

// tunable implementation default
constexpr uint8_t FC_PACKET_START_BYTE = 0xAA;

// tunable implementation default
constexpr uint32_t FC_HEARTBEAT_PERIOD_MS = 100UL;

// tunable implementation default
constexpr uint32_t FC_STATUS_TIMEOUT_MS = 500UL;

// tunable implementation default
constexpr uint32_t FC_COMMAND_TIMEOUT_MS = 200UL;

// tunable implementation default
constexpr uint8_t FC_TX_RETRY_LIMIT = 2;

// tunable implementation default
constexpr uint16_t FC_RX_BUFFER_SIZE = 128;

// tunable implementation default
constexpr uint16_t FC_TX_BUFFER_SIZE = 64;

// tunable implementation default
constexpr float FC_MAX_HORIZONTAL_SPEED_MPS = 1.5f;

// tunable implementation default
constexpr float FC_MAX_VERTICAL_SPEED_MPS = 0.8f;

// tunable implementation default
constexpr float FC_MIN_ALTITUDE_M = 0.0f;

// tunable implementation default
constexpr float FC_MAX_ALTITUDE_M = 3.0f;

// tunable implementation default
constexpr float FC_MAX_HEADING_RATE_DEG_S = 90.0f;

// tunable implementation default
constexpr uint32_t FC_TAKEOFF_TIMEOUT_MS = 20000UL;

// tunable implementation default
constexpr uint32_t FC_LANDING_TIMEOUT_MS = 30000UL;

// tunable implementation default
constexpr uint32_t FC_EMERGENCY_STOP_REPEAT_MS = 100UL;

// tunable implementation default
constexpr bool FC_AUTO_HOLD_ON_COMMAND_TIMEOUT = true;

// tunable implementation default
constexpr bool FC_AUTO_LAND_ON_LINK_LOST = false;

// tunable implementation default
constexpr float ARM_MAGIC_NUMBER = 41234.0f; // 0xA112

// tunable implementation default
constexpr float DISARM_MAGIC_NUMBER = 53594.0f; // 0xD15A

// tunable implementation default
constexpr float EMERGENCY_MAGIC_NUMBER = 58736.0f; // 0xE570

// ============================================================================
// MARKER CONTROLLER THRESHOLDS
// ============================================================================

// tunable implementation default
constexpr bool MARKER_OUTPUT_ENABLED_DEFAULT = true;

// tunable implementation default
constexpr uint8_t MARKER_BRIGHTNESS_DEFAULT_PERCENT = 80;

// tunable implementation default
constexpr uint32_t MARKER_GUIDANCE_UPDATE_MS = 50UL;

// tunable implementation default
constexpr uint32_t MARKER_PATTERN_MIN_HOLD_MS = 200UL;

// tunable implementation default
constexpr uint32_t MARKER_BLINK_PERIOD_MS = 300UL;

// tunable implementation default
constexpr uint32_t MARKER_EMERGENCY_BLINK_PERIOD_MS = 100UL;

// tunable implementation default
constexpr uint32_t MARKER_CAUTION_BLINK_PERIOD_MS = 500UL;

// tunable implementation default
constexpr bool MARKER_USE_LIGHT_CUES = true;

// tunable implementation default
constexpr bool MARKER_ALLOW_PROJECTED_CUES = false;

// tunable implementation default
constexpr bool MARKER_PHYSICAL_MECHANISM_ENABLED = false;

// ============================================================================
// SEARCH BEHAVIOR THRESHOLDS
// ============================================================================

// tunable implementation default
constexpr bool SEARCH_ACTIVE_ENABLED_DEFAULT = false;

// tunable implementation default
constexpr float SEARCH_FORWARD_SPEED_MPS = 0.6f;

// tunable implementation default
constexpr float SEARCH_MAX_SPEED_MPS = 1.0f;

// tunable implementation default
constexpr float SEARCH_MIN_SPEED_MPS = 0.05f;

// tunable implementation default
constexpr float SEARCH_LANE_LEFT_CENTER_X = 3.5f;

// tunable implementation default
constexpr float SEARCH_LANE_RIGHT_CENTER_X = 11.5f;

// tunable implementation default
constexpr float SEARCH_LANE_CENTER_CENTER_X = 7.5f;

// tunable implementation default
constexpr float SEARCH_RESERVE_HOLD_X = 7.5f;

// tunable implementation default
constexpr float SEARCH_RESERVE_HOLD_Y = 2.0f;

// tunable implementation default
constexpr float SEARCH_LANE_HOLD_GAIN = 0.35f;

// tunable implementation default
constexpr float SEARCH_Y_LOOKAHEAD_M = 1.5f;

// tunable implementation default
constexpr float SEARCH_Y_HOLD_GAIN = 0.25f;

// tunable implementation default
constexpr float SEARCH_SCAN_BIAS_SPEED_MPS = 0.30f;

// tunable implementation default
constexpr uint32_t SEARCH_SCAN_COMMAND_TIMEOUT_MS = 5000UL;

// tunable implementation default
constexpr float SEARCH_TURNAROUND_MARGIN_M = 1.0f;

// tunable implementation default
constexpr float SEARCH_LANE_SHIFT_M = 1.0f;

// tunable implementation default
constexpr bool SEARCH_SERPENTINE_ENABLED = true;

// tunable implementation default
constexpr float SEARCH_COVERAGE_GRID_RESOLUTION_M = 0.5f;

// tunable implementation default
constexpr uint16_t SEARCH_COVERAGE_GRID_COLS = 30;

// tunable implementation default
constexpr uint16_t SEARCH_COVERAGE_GRID_ROWS = 116;

// tunable implementation default
constexpr float SEARCH_SENSOR_FOOTPRINT_RADIUS_M = 1.0f;

// tunable implementation default
constexpr float SEARCH_COVERAGE_SUFFICIENT_PERCENT = 75.0f;

// tunable implementation default
constexpr uint32_t SEARCH_COVERAGE_UPDATE_MS = 100UL;

// tunable implementation default
constexpr float SEARCH_PEER_SEPARATION_M = 2.0f;

// tunable implementation default
constexpr float SEARCH_PEER_CRITICAL_SEPARATION_M = 1.0f;

// tunable implementation default
constexpr float SEARCH_PEER_AVOID_GAIN = 0.4f;

// tunable implementation default
constexpr float SEARCH_GEOFENCE_CORRECTION_GAIN = 0.5f;

// tunable implementation default
constexpr float SEARCH_MAX_GEOFENCE_CORRECTION_MPS = 0.6f;

// tunable implementation default
constexpr uint32_t SEARCH_UNSCANNED_CELL_SEARCH_TIMEOUT_MS = 10000UL;

// tunable implementation default
constexpr uint32_t SEARCH_HOLD_AFTER_COMPLETE_MS = 2000UL;

// ============================================================================
// TELEMETRY & STORAGE THRESHOLDS
// ============================================================================

// tunable implementation default
constexpr uint16_t MAX_TELEMETRY_EVENTS = 128;

// tunable implementation default
constexpr uint16_t TELEMETRY_MAX_EVENTS_PER_SECOND = 50;

// tunable implementation default
constexpr bool TELEMETRY_PERSIST_CRITICAL_EVENTS = true;

// tunable implementation default
constexpr bool TELEMETRY_PERSIST_PERIODIC_SUMMARY = false;

// tunable implementation default
constexpr uint8_t TELEMETRY_MIN_LOG_PRIORITY = 1; // TELEMETRY_SEVERITY_INFO

// tunable implementation default
constexpr uint32_t TELEMETRY_DRIFT_LOG_INTERVAL_MS = 2000UL;

// tunable implementation default
constexpr uint32_t TELEMETRY_SUMMARY_LOG_INTERVAL_MS = 5000UL;

// tunable implementation default
constexpr uint32_t TELEMETRY_STORAGE_RETRY_MS = 3000UL;

// tunable implementation default
constexpr bool TELEMETRY_FLUSH_BEFORE_LANDING = true;

// tunable implementation default
constexpr uint16_t TELEMETRY_CRITICAL_QUEUE_SIZE = 16;


// ============================================================================
// CROSS-DRONE VISION FUSION & CONSENSUS (ITEMS 11/12, REQ-DER-111/112)
// ============================================================================

// Observation weight = confidence * 1/(1 + distance^2 / REF_DIST^2).
constexpr float VISION_FUSION_REF_DISTANCE_M = 2.5f;

// Minimum weighted agreement for a classification to be trusted.
constexpr float MARKER_CONSENSUS_AGREEMENT_MIN = 0.60f;

// Below this total vote weight the marker is flagged ambiguous regardless.
constexpr float MARKER_CONSENSUS_MIN_WEIGHT = 0.8f;

// Interval between VISION_OBS broadcasts of not-yet-shared local detections.
constexpr uint32_t VISION_OBS_BROADCAST_INTERVAL_MS = 500UL;


// ============================================================================
// BURIED-MINE DETECTION (ITEM 13, REQ-DER-113 + multispectral hooks item 3n)
// ============================================================================

constexpr bool BURIED_DETECT_ENABLED = true;

// Texture anomaly: local variance must exceed this multiple of the field mean.
constexpr float BURIED_TEXTURE_RATIO_MIN = 1.8f;

// Plane-fit residual (m) above which soil is considered disturbed.
constexpr float BURIED_PLANE_RESIDUAL_M = 0.035f;

// Combined anomaly score required to emit a buried-mine candidate.
constexpr float BURIED_SCORE_EMIT_MIN = 0.55f;

// Buried candidates enter the map with this confidence (needs peer votes or
// a closer pass to confirm).
constexpr float BURIED_CANDIDATE_CONFIDENCE = 40.0f;

// Multispectral index weights (NDVI proxy etc.). With no NIR provider these
// act as RGB-proxy vegetation stress indicators.
constexpr float BURIED_NDVI_WEIGHT = 0.25f;
constexpr float BURIED_MOISTURE_WEIGHT = 0.15f;


// ============================================================================
// DYNAMIC OBSTACLE TTC & MOVING-TARGET LANDING (ITEMS 14 / 6n)
// ============================================================================

// Evasion triggers when predicted time-to-collision falls below this.
constexpr float OBSTACLE_TTC_CRITICAL_S = 2.0f;

// Obstacles closer than this are treated as immediate (TTC ~ 0).
constexpr float OBSTACLE_IMMEDIATE_RADIUS_M = 0.6f;

// Moving-target landing: constant-velocity prediction horizon (s).
constexpr float LANDING_PREDICT_HORIZON_S = 1.5f;

// Terminal visual-servo gain: descent velocity scales with pixel error.
constexpr float LANDING_SERVO_GAIN_PX_TO_MPS = 0.004f;

// Target is "centered" when lateral servo error < this many native px.
constexpr float LANDING_SERVO_TOLERANCE_PX = 12.0f;

} // namespace Config
} // namespace RobofestDrone
