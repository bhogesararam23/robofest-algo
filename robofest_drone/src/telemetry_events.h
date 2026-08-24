#pragma once

#include <stdint.h>

namespace RobofestDrone {

// ============================================================================
// STATE MACHINE TELEMETRY EVENT IDS (1000 - 1099)
// ============================================================================

constexpr uint16_t TE_STATE_TRANSITION                       = 1000;
constexpr uint16_t TE_INIT_COMPLETE                          = 1000;
constexpr uint16_t TE_STATE_MACHINE_INIT                     = 1000;
constexpr uint16_t TE_STATE_MACHINE_BOOT                     = 1001;
constexpr uint16_t TE_STATE_MACHINE_CALIBRATE                = 1001;
constexpr uint16_t TE_STATE_MACHINE_STANDBY                  = 1002;
constexpr uint16_t TE_CALIBRATION_PASSED                     = 1002;
constexpr uint16_t TE_STATE_MACHINE_WAIT_FOR_START          = 1002;
constexpr uint16_t TE_STATE_MACHINE_TAKEOFF                  = 1003;
constexpr uint16_t TE_STATE_MACHINE_FORMATION                = 1004;
constexpr uint16_t TE_STATE_MACHINE_SEARCHING                = 1005;
constexpr uint16_t TE_STATE_MACHINE_PLANNING                 = 1006;
constexpr uint16_t TE_STATE_MACHINE_GUIDING                  = 1007;
constexpr uint16_t TE_STATE_MACHINE_MISSION_COMPLETE         = 1008;
constexpr uint16_t TE_STATE_MACHINE_LANDING                  = 1009;
constexpr uint16_t TE_STATE_MACHINE_DISARMED                 = 1010;
constexpr uint16_t TE_STATE_MACHINE_HOLD                     = 1011;
constexpr uint16_t TE_STATE_MACHINE_EMERGENCY                = 1012;
constexpr uint16_t TE_MISSION_TIMER_STARTED                  = 1013;
constexpr uint16_t TE_MISSION_TIMER_TIMEOUT                  = 1014;
constexpr uint16_t TE_MISSION_TIMER_EXPIRED                  = 1014;
constexpr uint16_t TE_HOLD_ENTERED                           = 1015;
constexpr uint16_t TE_HOLD_EXITED                            = 1016;
constexpr uint16_t TE_EMERGENCY_ENTERED                      = 1017;
constexpr uint16_t TE_TAKEOFF_REQUESTED                      = 1018;
constexpr uint16_t TE_LANDING_REQUESTED                      = 1019;
constexpr uint16_t TE_DISARM_REQUESTED                       = 1020;
constexpr uint16_t TE_START_COMMAND_ACCEPTED                 = 1021;
constexpr uint16_t TE_PAUSE_COMMAND_ACCEPTED                 = 1022;
constexpr uint16_t TE_RESUME_COMMAND_ACCEPTED                = 1023;
constexpr uint16_t TE_STOP_COMMAND_ACCEPTED                  = 1024;
constexpr uint16_t TE_PATH_VALID_TRANSITION                  = 1025;
constexpr uint16_t TE_PATH_INVALID_TRANSITION                = 1026;
constexpr uint16_t TE_HUMAN_EXIT_REACHED                     = 1027;
constexpr uint16_t TE_KILL_SWITCH_ACTIVE                     = 1028;
constexpr uint16_t TE_SM_LOCALIZATION_DEGRADED               = 1029;
constexpr uint16_t TE_SM_LOCALIZATION_UNRECOVERABLE          = 1030;
constexpr uint16_t TE_BATTERY_LOW                            = 1031;
constexpr uint16_t TE_BATTERY_CRITICAL                       = 1032;
constexpr uint16_t TE_SM_FC_LINK_LOST                        = 1033;
constexpr uint16_t TE_SWARM_DEGRADED                         = 1034;
constexpr uint16_t TE_SWARM_CRITICAL                         = 1035;


// ============================================================================
// LOCALIZATION TELEMETRY EVENT IDS (2000 - 2099)
// ============================================================================

constexpr uint16_t TE_LOCALIZATION_INITIALIZED               = 2000;
constexpr uint16_t TE_LOCALIZATION_GOOD                      = 2001;
constexpr uint16_t TE_LOCALIZATION_DEGRADED                  = 2002;
constexpr uint16_t TE_LOCALIZATION_UNRECOVERABLE             = 2003;
constexpr uint16_t TE_OPTICAL_FLOW_TIMEOUT                   = 2004;
constexpr uint16_t TE_TOF_TIMEOUT                            = 2005;
constexpr uint16_t TE_LOW_FLOW_QUALITY                       = 2006;
constexpr uint16_t TE_DRIFT_UNCERTAINTY_HIGH                 = 2007;
constexpr uint16_t TE_ALTITUDE_INVALID                       = 2008;
constexpr uint16_t TE_FLOW_REJECTED_BAD_DT                   = 2009;
constexpr uint16_t TE_FLOW_REJECTED_BAD_SHIFT                = 2010;
constexpr uint16_t TE_CALIBRATION_STARTED                    = 2011;
constexpr uint16_t TE_CALIBRATION_COMPLETE                   = 2012;
constexpr uint16_t TE_CALIBRATION_FAILED                     = 2013;


// ============================================================================
// GEOFENCE TELEMETRY EVENT IDS (2100 - 2199)
// ============================================================================

constexpr uint16_t TE_GEOFENCE_INITIALIZED                   = 2100;
constexpr uint16_t TE_GEOFENCE_INSIDE                        = 2101;
constexpr uint16_t TE_GEOFENCE_WARNING                       = 2102;
constexpr uint16_t TE_GEOFENCE_NEAR_LIMIT                    = 2103;
constexpr uint16_t TE_GEOFENCE_OUTSIDE                       = 2104;
constexpr uint16_t TE_GEOFENCE_MARGIN_INCREASED              = 2105;
constexpr uint16_t TE_GEOFENCE_MARGIN_MAXIMUM                = 2106;
constexpr uint16_t TE_GEOFENCE_EFFECTIVE_LIMITS_TOO_SMALL    = 2107;
constexpr uint16_t TE_GEOFENCE_CORRECTION_ACTIVE             = 2108;
constexpr uint16_t TE_GEOFENCE_VELOCITY_LIMITED              = 2109;


// ============================================================================
// VISION PIPELINE TELEMETRY EVENT IDS (2200 - 2299)
// ============================================================================

constexpr uint16_t TE_VISION_INITIALIZED                     = 2200;
constexpr uint16_t TE_VISION_FRAME_OK                        = 2201;
constexpr uint16_t TE_VISION_FRAME_TIMEOUT                   = 2202;
constexpr uint16_t TE_VISION_FRAME_INVALID                   = 2203;
constexpr uint16_t TE_VISION_MASK_EMPTY                      = 2204;
constexpr uint16_t TE_VISION_BLOB_REJECTED_AREA              = 2205;
constexpr uint16_t TE_VISION_BLOB_REJECTED_CIRCULARITY       = 2206;
constexpr uint16_t TE_VISION_BLOB_REJECTED_EDGE              = 2207;
constexpr uint16_t TE_VISION_BLOB_REJECTED_GLARE             = 2208;
constexpr uint16_t TE_VISION_CANDIDATE_REPORTED              = 2209;
constexpr uint16_t TE_VISION_TRACK_CREATED                   = 2210;
constexpr uint16_t TE_VISION_TRACK_FUSED                     = 2211;
constexpr uint16_t TE_VISION_TRACK_EXPIRED                   = 2212;
constexpr uint16_t TE_VISION_PROJECTION_DEGRADED             = 2213;
constexpr uint16_t TE_VISION_ATTITUDE_INVALID                = 2214;
constexpr uint16_t TE_VISION_ALTITUDE_TOO_LOW                = 2215;
constexpr uint16_t TE_VISION_PROCESSING_SLOW                 = 2216;
constexpr uint16_t TE_VISION_UNSUPPORTED_PIXEL_FORMAT         = 2217;
constexpr uint16_t TE_VISION_BLOB_EXCEEDED_SCAN_BUFFER        = 2218;
constexpr uint16_t TE_VISION_MORPHOLOGY_APPLIED               = 2219;
constexpr uint16_t TE_VISION_BLOB_REJECTED_SHAPE             = 2220;
constexpr uint16_t TE_VISION_LIGHTING_MODE_CHANGED           = 2221;
constexpr uint16_t TE_VISION_EXPOSURE_METRIC                 = 2222;
constexpr uint16_t TE_VISION_PROFILE_TABLE_LOADED            = 2223;

constexpr uint16_t TE_FRAME_ACQUIRED                         = 2201;
constexpr uint16_t TE_FRAME_DROPPED                          = 2202;
constexpr uint16_t TE_BLOB_DETECTED                          = 2203;
constexpr uint16_t TE_CANDIDATE_EMITTED                      = 2204;
constexpr uint16_t TE_CANDIDATE_PERSISTED                    = 2205;
constexpr uint16_t TE_CANDIDATE_REJECTED                     = 2206;


// ============================================================================
// MINE MAP TELEMETRY EVENT IDS (2300 - 2399)
// ============================================================================

constexpr uint16_t TE_MINE_MAP_INITIALIZED                   = 2300;
constexpr uint16_t TE_MINE_CANDIDATE_CREATED                 = 2301;
constexpr uint16_t TE_MINE_CANDIDATE_ADDED                   = 2301;
constexpr uint16_t TE_MINE_CANDIDATE_FUSED                   = 2302;
constexpr uint16_t TE_MINE_FUSED_EXISTING                    = 2302;
constexpr uint16_t TE_MINE_CONFIRMED                         = 2303;
constexpr uint16_t TE_MINE_REJECTED_STALE                    = 2304;
constexpr uint16_t TE_MINE_STALE_DECAYED                     = 2304;
constexpr uint16_t TE_MINE_REJECTED_LOW_CONFIDENCE           = 2305;
constexpr uint16_t TE_MINE_CLAIM_SUBMITTED                   = 2305;
constexpr uint16_t TE_MINE_REJECTED_OUTSIDE_FIELD            = 2306;
constexpr uint16_t TE_MINE_OCCUPANCY_EXPORTED                = 2306;
constexpr uint16_t TE_MINE_REJECTED_INVALID_INPUT            = 2307;
constexpr uint16_t TE_MINE_MAP_FULL                          = 2308;
constexpr uint16_t TE_MINE_CLAIM_READY                       = 2309;
constexpr uint16_t TE_MINE_CLAIM_ACCEPTED                    = 2310;
constexpr uint16_t TE_MINE_CLAIM_RELEASED                    = 2311;
constexpr uint16_t TE_MINE_CLAIM_EXPIRED                     = 2312;
constexpr uint16_t TE_MINE_MAP_VERSION_CHANGED               = 2313;
constexpr uint16_t TE_MINE_DECAY_RUN                         = 2314;


// ============================================================================
// PATH PLANNER TELEMETRY EVENT IDS (2400 - 2499)
// ============================================================================

constexpr uint16_t TE_PATH_PLANNER_INITIALIZED               = 2400;
constexpr uint16_t TE_PLANNER_INITIALIZED                    = 2400;
constexpr uint16_t TE_PATH_PLANNER_RESET                     = 2401;
constexpr uint16_t TE_PLANNER_OCCUPANCY_BUILT                = 2401;
constexpr uint16_t TE_PATH_PLAN_STARTED                      = 2402;
constexpr uint16_t TE_PLANNER_SEARCH_STARTED                 = 2402;
constexpr uint16_t TE_PATH_PLAN_SUCCESS                      = 2403;
constexpr uint16_t TE_PLANNER_PATH_FOUND                     = 2403;
constexpr uint16_t TE_PATH_PLAN_FAILED                       = 2404;
constexpr uint16_t TE_PLANNER_PATH_FAILED                    = 2404;
constexpr uint16_t TE_PATH_START_BLOCKED                     = 2405;
constexpr uint16_t TE_PATH_EXIT_BLOCKED                      = 2406;
constexpr uint16_t TE_PATH_NO_ROUTE_FOUND                    = 2407;
constexpr uint16_t TE_PATH_CLEARANCE_VALIDATION_FAILED       = 2408;
constexpr uint16_t TE_PLANNER_CLEARANCE_VIOLATION            = 2408;
constexpr uint16_t TE_PATH_SMOOTHED                          = 2409;
constexpr uint16_t TE_PLANNER_PATH_SMOOTHED                  = 2409;
constexpr uint16_t TE_PATH_SMOOTHING_REJECTED                = 2410;
constexpr uint16_t TE_PATH_MAP_VERSION_CHANGED               = 2411;
constexpr uint16_t TE_PATH_INVALIDATED                       = 2412;
constexpr uint16_t TE_PATH_REPLAN_REQUESTED                  = 2413;
constexpr uint16_t TE_PATH_HUMAN_OFF_CORRIDOR               = 2414;
constexpr uint16_t TE_PATH_NEEDS_MORE_SCAN                   = 2415;
constexpr uint16_t TE_PATH_EXIT_REACHED_HELPER               = 2416;
constexpr uint16_t TE_PLANNER_CLEARANCE_VALIDATED            = 2403;


// ============================================================================
// SWARM COMM TELEMETRY EVENT IDS (2500 - 2599)
// ============================================================================

constexpr uint16_t TE_SWARM_INITIALIZED                      = 2500;
constexpr uint16_t TE_SWARM_HEARTBEAT_SENT                   = 2501;
constexpr uint16_t TE_SWARM_HEARTBEAT_RECEIVED               = 2502;
constexpr uint16_t TE_SWARM_PEER_ALIVE                       = 2503;
constexpr uint16_t TE_SWARM_PEER_DEGRADED                    = 2504;
constexpr uint16_t TE_SWARM_PEER_LOST                        = 2505;
constexpr uint16_t TE_SWARM_PEER_TIMEOUT                     = 2505;
constexpr uint16_t TE_SWARM_ROLE_ASSIGNED                    = 2506;
constexpr uint16_t TE_SWARM_COORDINATOR_ELECTED              = 2506;
constexpr uint16_t TE_SWARM_ROLE_ACCEPTED                    = 2507;
constexpr uint16_t TE_SWARM_ROLE_REASSIGNED                  = 2508;
constexpr uint16_t TE_SWARM_ROLE_FAILOVER                    = 2508;
constexpr uint16_t TE_SWARM_CLAIM_SENT                       = 2509;
constexpr uint16_t TE_SWARM_CLAIM_RECEIVED                   = 2510;
constexpr uint16_t TE_SWARM_CLAIM_ACCEPTED                   = 2511;
constexpr uint16_t TE_SWARM_CLAIM_REJECTED                   = 2512;
constexpr uint16_t TE_SWARM_YIELD_SENT                       = 2513;
constexpr uint16_t TE_SWARM_CLAIM_YIELDED                    = 2513;
constexpr uint16_t TE_SWARM_YIELD_RECEIVED                   = 2514;
constexpr uint16_t TE_SWARM_MINE_UPDATE_SENT                 = 2515;
constexpr uint16_t TE_SWARM_MINE_UPDATE_RECEIVED             = 2516;
constexpr uint16_t TE_SWARM_PATH_UPDATE_SENT                 = 2517;
constexpr uint16_t TE_SWARM_PATH_UPDATE_RECEIVED             = 2518;
constexpr uint16_t TE_SWARM_PATH_REASSEMBLED                 = 2518;
constexpr uint16_t TE_SWARM_PERSON_UPDATE_SENT               = 2519;
constexpr uint16_t TE_SWARM_PERSON_UPDATE_RECEIVED           = 2520;
constexpr uint16_t TE_SWARM_HELP_REQUEST_SENT                = 2521;
constexpr uint16_t TE_SWARM_HELP_REQUEST_RECEIVED            = 2522;
constexpr uint16_t TE_SWARM_LAND_NOW_SENT                    = 2523;
constexpr uint16_t TE_SWARM_LAND_NOW_RECEIVED                = 2524;
constexpr uint16_t TE_SWARM_PACKET_INVALID                   = 2525;
constexpr uint16_t TE_SWARM_PACKET_STALE                     = 2526;
constexpr uint16_t TE_SWARM_RADIO_UNHEALTHY                  = 2527;
constexpr uint16_t TE_SWARM_VISION_OBS_SENT                  = 2528;
constexpr uint16_t TE_SWARM_VISION_OBS_RECEIVED              = 2529;


// ============================================================================
// COMMAND LAYER TELEMETRY EVENT IDS (2600 - 2699)
// ============================================================================

constexpr uint16_t TE_COMMAND_INITIALIZED                    = 2600;
constexpr uint16_t TE_COMMAND_LAYER_INITIALIZED              = 2600;
constexpr uint16_t TE_COMMAND_START_ACCEPTED                 = 2601;
constexpr uint16_t TE_COMMAND_FORWARD_ACCEPTED               = 2602;
constexpr uint16_t TE_COMMAND_PAUSE_ACCEPTED                 = 2603;
constexpr uint16_t TE_COMMAND_SCAN_ACCEPTED                  = 2604;
constexpr uint16_t TE_COMMAND_SCAN_LEFT_ACCEPTED             = 2604;
constexpr uint16_t TE_COMMAND_SCAN_RIGHT_ACCEPTED            = 2605;
constexpr uint16_t TE_COMMAND_STOP_ACCEPTED                  = 2606;
constexpr uint16_t TE_COMMAND_STOP_ABORT_ACCEPTED            = 2606;
constexpr uint16_t TE_COMMAND_REJECTED_LOW_CONFIDENCE        = 2607;
constexpr uint16_t TE_COMMAND_REJECTED_CONFIDENCE            = 2607;
constexpr uint16_t TE_COMMAND_REJECTED_LOCKOUT               = 2608;
constexpr uint16_t TE_COMMAND_REJECTED_DEBOUNCE              = 2609;
constexpr uint16_t TE_COMMAND_REJECTED_STATE_NOT_ALLOWED     = 2610;
constexpr uint16_t TE_COMMAND_REJECTED_STATE                 = 2610;
constexpr uint16_t TE_COMMAND_SOURCE_CONFLICT                = 2611;
constexpr uint16_t TE_COMMAND_STALE_SAMPLE                   = 2612;
constexpr uint16_t TE_COMMAND_VALID_WINDOW_EXPIRED           = 2613;
constexpr uint16_t TE_COMMAND_GESTURE_ENABLED                = 2614;
constexpr uint16_t TE_COMMAND_GESTURE_DISABLED               = 2615;
constexpr uint16_t TE_COMMAND_VOICE_ENABLED                  = 2616;
constexpr uint16_t TE_COMMAND_VOICE_DISABLED                 = 2617;
constexpr uint16_t TE_COMMAND_DEBUG_USED                     = 2618;
constexpr uint16_t TE_COMMAND_SCAN_EXPIRED                   = 2619;


// ============================================================================
// SAFETY MANAGER TELEMETRY EVENT IDS (2700 - 2799)
// ============================================================================

constexpr uint16_t TE_SAFETY_INITIALIZED                     = 2700;
constexpr uint16_t TE_SAFETY_RESET                           = 2701;
constexpr uint16_t TE_SAFETY_KILL_SWITCH_ACTIVE              = 2702;
constexpr uint16_t TE_SAFETY_BATTERY_LOW                     = 2703;
constexpr uint16_t TE_SAFETY_BATTERY_CRITICAL                = 2704;
constexpr uint16_t TE_SAFETY_FC_LINK_LOST                    = 2705;
constexpr uint16_t TE_SAFETY_CAMERA_STALL                    = 2706;
constexpr uint16_t TE_SAFETY_OPTICAL_FLOW_FAILURE            = 2707;
constexpr uint16_t TE_SAFETY_TOF_FAILURE                     = 2708;
constexpr uint16_t TE_SAFETY_RADIO_TIMEOUT                   = 2709;
constexpr uint16_t TE_SAFETY_SWARM_DEGRADED                  = 2710;
constexpr uint16_t TE_SAFETY_SWARM_CRITICAL                  = 2711;
constexpr uint16_t TE_SAFETY_LOCALIZATION_DEGRADED           = 2712;
constexpr uint16_t TE_SAFETY_LOCALIZATION_UNRECOVERABLE       = 2713;
constexpr uint16_t TE_SAFETY_MISSION_TIMEOUT                 = 2714;
constexpr uint16_t TE_SAFETY_GEOFENCE_WARNING                = 2715;
constexpr uint16_t TE_SAFETY_GEOFENCE_NEAR_LIMIT             = 2716;
constexpr uint16_t TE_SAFETY_GEOFENCE_OUTSIDE                = 2717;
constexpr uint16_t TE_SAFETY_COLLISION_RISK                  = 2718;
constexpr uint16_t TE_SAFETY_UNSAFE_PROXIMITY_HUMAN          = 2719;
constexpr uint16_t TE_SAFETY_UNSAFE_PROXIMITY_PEER           = 2720;
constexpr uint16_t TE_SAFETY_SURFACE_CONTACT                 = 2721;
constexpr uint16_t TE_SAFETY_WATCHDOG_STALL                  = 2722;
constexpr uint16_t TE_SAFETY_ACTION_CONTINUE                 = 2723;
constexpr uint16_t TE_SAFETY_ACTION_HOLD                     = 2724;
constexpr uint16_t TE_SAFETY_ACTION_LAND                     = 2725;
constexpr uint16_t TE_SAFETY_ACTION_EMERGENCY_CUT            = 2726;
constexpr uint16_t TE_SAFETY_FAULT_CLEARED                   = 2727;
constexpr uint16_t TE_SAFETY_EMERGENCY_ACKNOWLEDGED          = 2728;


// ============================================================================
// FLIGHT CONTROLLER BRIDGE TELEMETRY EVENT IDS (2800 - 2899)
// ============================================================================

constexpr uint16_t TE_FC_BRIDGE_INITIALIZED                  = 2800;
constexpr uint16_t TE_FC_LINK_HEALTHY                        = 2801;
constexpr uint16_t TE_FC_LINK_LOST                           = 2802;
constexpr uint16_t TE_FC_STATUS_RECEIVED                     = 2803;
constexpr uint16_t TE_FC_STATUS_TIMEOUT                      = 2804;
constexpr uint16_t TE_FC_PACKET_CRC_ERROR                    = 2805;
constexpr uint16_t TE_FC_PACKET_INVALID                      = 2806;
constexpr uint16_t TE_FC_TX_FAILED                           = 2807;
constexpr uint16_t TE_FC_TX_RETRY_LIMIT                      = 2808;
constexpr uint16_t TE_FC_ARMED                               = 2809;
constexpr uint16_t TE_FC_DISARMED                            = 2810;
constexpr uint16_t TE_FC_HOLD_SENT                           = 2811;
constexpr uint16_t TE_FC_LAND_SENT                           = 2812;
constexpr uint16_t TE_FC_TAKEOFF_SENT                        = 2813;
constexpr uint16_t TE_FC_VELOCITY_SENT                       = 2814;
constexpr uint16_t TE_FC_ALTITUDE_SENT                       = 2815;
constexpr uint16_t TE_FC_HEADING_SENT                        = 2816;
constexpr uint16_t TE_FC_ARM_REQUESTED                       = 2817;
constexpr uint16_t TE_FC_DISARM_REQUESTED                    = 2818;
constexpr uint16_t TE_FC_EMERGENCY_STOP_SENT                 = 2819;
constexpr uint16_t TE_FC_COMMAND_WATCHDOG_HOLD               = 2820;
constexpr uint16_t TE_FC_COMMAND_REJECTED_NAN                = 2821;
constexpr uint16_t TE_FC_COMMAND_REJECTED_LIMIT              = 2822;
constexpr uint16_t TE_FC_BATTERY_VOLTAGE_INVALID             = 2823;
constexpr uint16_t TE_FC_BATTERY_CURRENT_INVALID             = 2824;


// ============================================================================
// HUMAN TRACKER TELEMETRY EVENT IDS (2900 - 2999)
// ============================================================================

constexpr uint16_t TE_HUMAN_TRACKER_INITIALIZED              = 2900;
constexpr uint16_t TE_HUMAN_DETECTED                         = 2901;
constexpr uint16_t TE_HUMAN_TRACK_UPDATED                    = 2902;
constexpr uint16_t TE_HUMAN_TRACK_PREDICTED                  = 2903;
constexpr uint16_t TE_HUMAN_LOST_SHORT_TERM                  = 2904;
constexpr uint16_t TE_HUMAN_LOST_RECOVERY_TIMEOUT            = 2905;
constexpr uint16_t TE_HUMAN_TRACK_RESET                      = 2906;
constexpr uint16_t TE_HUMAN_OFF_PATH_DETECTED                = 2907;
constexpr uint16_t TE_HUMAN_BACK_ON_PATH                     = 2908;
constexpr uint16_t TE_HUMAN_EXIT_ZONE_DETECTED               = 2909;
constexpr uint16_t TE_HUMAN_EXIT_ZONE_CONFIRMED              = 2910;
constexpr uint16_t TE_HUMAN_EXIT_ZONE_LOST                   = 2911;
constexpr uint16_t TE_HUMAN_DETECTION_REJECTED_LOW_CONFIDENCE= 2912;
constexpr uint16_t TE_HUMAN_DETECTION_REJECTED_INVALID       = 2913;
constexpr uint16_t TE_HUMAN_PROJECTION_DEGRADED              = 2914;
constexpr uint16_t TE_HUMAN_PROXIMITY_WARNING                = 2915;
constexpr uint16_t TE_HUMAN_PROXIMITY_CRITICAL               = 2916;


// ============================================================================
// MARKER CONTROLLER TELEMETRY EVENT IDS (3000 - 3099)
// ============================================================================

constexpr uint16_t TE_MARKER_INITIALIZED                     = 3000;
constexpr uint16_t TE_MARKER_OUTPUT_ENABLED                  = 3001;
constexpr uint16_t TE_MARKER_OUTPUT_DISABLED                 = 3002;
constexpr uint16_t TE_MARKER_PATTERN_OFF                     = 3003;
constexpr uint16_t TE_MARKER_PATTERN_FORWARD                 = 3004;
constexpr uint16_t TE_MARKER_PATTERN_STOP                    = 3005;
constexpr uint16_t TE_MARKER_PATTERN_LEFT                    = 3006;
constexpr uint16_t TE_MARKER_PATTERN_RIGHT                   = 3007;
constexpr uint16_t TE_MARKER_PATTERN_SAFE_PATH               = 3008;
constexpr uint16_t TE_MARKER_PATTERN_EMERGENCY               = 3009;
constexpr uint16_t TE_MARKER_PATTERN_MISSION_COMPLETE        = 3010;
constexpr uint16_t TE_MARKER_PATTERN_CAUTION                 = 3011;
constexpr uint16_t TE_MARKER_PATTERN_REJOIN_LEFT             = 3012;
constexpr uint16_t TE_MARKER_PATTERN_REJOIN_RIGHT            = 3013;
constexpr uint16_t TE_MARKER_PATTERN_LANDING                 = 3014;
constexpr uint16_t TE_MARKER_OVERRIDE_STARTED                = 3015;
constexpr uint16_t TE_MARKER_OVERRIDE_EXPIRED                = 3016;
constexpr uint16_t TE_MARKER_HAL_ERROR                       = 3017;


// ============================================================================
// SEARCH BEHAVIOR TELEMETRY EVENT IDS (3100 - 3199)
// ============================================================================

constexpr uint16_t TE_SEARCH_INITIALIZED                     = 3100;
constexpr uint16_t TE_SEARCH_ACTIVATED                       = 3101;
constexpr uint16_t TE_SEARCH_DEACTIVATED                     = 3102;
constexpr uint16_t TE_SEARCH_LANE_ASSIGNED                   = 3103;
constexpr uint16_t TE_SEARCH_ROLE_UPDATED                    = 3104;
constexpr uint16_t TE_SEARCH_FORWARD_STARTED                 = 3105;
constexpr uint16_t TE_SEARCH_TURNAROUND                      = 3106;
constexpr uint16_t TE_SEARCH_SCAN_LEFT_COMMAND               = 3107;
constexpr uint16_t TE_SEARCH_SCAN_RIGHT_COMMAND              = 3108;
constexpr uint16_t TE_SEARCH_SCAN_COMMAND_EXPIRED            = 3109;
constexpr uint16_t TE_SEARCH_COVERAGE_UPDATED                = 3110;
constexpr uint16_t TE_SEARCH_COVERAGE_SUFFICIENT             = 3111;
constexpr uint16_t TE_SEARCH_NEEDS_MORE_SCAN                 = 3112;
constexpr uint16_t TE_SEARCH_UNSCANNED_CELL_TARGET          = 3113;
constexpr uint16_t TE_SEARCH_PEER_SEPARATION_WARNING         = 3114;
constexpr uint16_t TE_SEARCH_PEER_SEPARATION_CRITICAL        = 3115;
constexpr uint16_t TE_SEARCH_GEOFENCE_CORRECTION_ACTIVE      = 3116;
constexpr uint16_t TE_SEARCH_HOLD_ENTERED                    = 3117;
constexpr uint16_t TE_SEARCH_RESERVE_HOLD                    = 3118;
constexpr uint16_t TE_SEARCH_SERPENTINE_SHIFT                = 3119;


// ============================================================================
// TELEMETRY & LOGGING MODULE EVENTS (3200 - 3299)
// ============================================================================

constexpr uint16_t TE_TELEMETRY_INITIALIZED                  = 3200;
constexpr uint16_t TE_TELEMETRY_RESET                        = 3201;
constexpr uint16_t TE_TELEMETRY_STORAGE_HEALTHY              = 3202;
constexpr uint16_t TE_TELEMETRY_STORAGE_UNHEALTHY            = 3203;
constexpr uint16_t TE_TELEMETRY_BUFFER_FULL                  = 3204;
constexpr uint16_t TE_TELEMETRY_CRITICAL_EVENT_DROPPED       = 3205;
constexpr uint16_t TE_TELEMETRY_FLUSH_STARTED                = 3206;
constexpr uint16_t TE_TELEMETRY_FLUSH_COMPLETE               = 3207;
constexpr uint16_t TE_TELEMETRY_PERIODIC_SUMMARY             = 3208;
constexpr uint16_t TE_TELEMETRY_MISSION_SUMMARY              = 3209;

} // namespace RobofestDrone
