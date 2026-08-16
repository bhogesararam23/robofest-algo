#include "search_behavior.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static SearchBehavior s_global_search_behavior;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

SearchBehavior::SearchBehavior() {
    reset();
}

void SearchBehavior::init() {
    reset();
    active_ = Config::SEARCH_ACTIVE_ENABLED_DEFAULT;
    setTelemetryEvent(TE_SEARCH_INITIALIZED);
}

void SearchBehavior::reset() {
    velocity_command_ = Types::Vec2(0.0f, 0.0f);
    target_x_ = Config::SEARCH_LANE_LEFT_CENTER_X;
    target_y_ = Config::MINEFIELD_Y_MIN;

    active_role_ = Types::DroneRole::SCOUT_LEFT;
    active_lane_id_ = Config::LANE_LEFT;
    active_ = Config::SEARCH_ACTIVE_ENABLED_DEFAULT;

    scan_command_ = 0;
    scan_command_expiry_ms_ = 0;

    std::memset(coverage_grid_, 0, sizeof(coverage_grid_));
    coverage_percent_ = 0.0f;
    coverage_sufficient_ = false;
    needs_more_scan_ = true;

    forward_direction_ = 1.0f;
    last_coverage_update_ms_ = 0;
    unscanned_search_start_ms_ = 0;
    searching_unscanned_cell_ = false;
    unscanned_target_ = Types::Vec2(0.0f, 0.0f);

    hold_start_ms_ = 0;

    last_telemetry_event_id_ = TE_SEARCH_INITIALIZED;
    telemetry_event_valid_ = true;
}

void SearchBehavior::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}

void SearchBehavior::setActive(bool active) {
    active_ = active;
    if (active) {
        setTelemetryEvent(TE_SEARCH_ACTIVATED);
    } else {
        setTelemetryEvent(TE_SEARCH_DEACTIVATED);
        velocity_command_ = Types::Vec2(0.0f, 0.0f);
    }
}

void SearchBehavior::setRole(Types::DroneRole role) {
    active_role_ = role;
    switch (role) {
        case Types::DroneRole::SCOUT_LEFT:
            setLaneId(Config::LANE_LEFT);
            break;
        case Types::DroneRole::SCOUT_RIGHT:
            setLaneId(Config::LANE_RIGHT);
            break;
        case Types::DroneRole::GUIDE_MARKER:
            setLaneId(Config::LANE_CENTER);
            break;
        case Types::DroneRole::RESERVE:
        default:
            setLaneId(Config::LANE_NONE);
            break;
    }
    setTelemetryEvent(TE_SEARCH_ROLE_UPDATED);
}

void SearchBehavior::setLaneId(uint8_t lane_id) {
    active_lane_id_ = lane_id;
    target_x_ = getLaneCenterX(lane_id);
    setTelemetryEvent(TE_SEARCH_LANE_ASSIGNED);
}

float SearchBehavior::getLaneCenterX(uint8_t lane_id) const {
    switch (lane_id) {
        case Config::LANE_LEFT:
            return Config::SEARCH_LANE_LEFT_CENTER_X;
        case Config::LANE_RIGHT:
            return Config::SEARCH_LANE_RIGHT_CENTER_X;
        case Config::LANE_CENTER:
            return Config::SEARCH_LANE_CENTER_CENTER_X;
        case Config::LANE_NONE:
        default:
            return Config::SEARCH_RESERVE_HOLD_X;
    }
}


// ============================================================================
// SCAN COMMAND DISPATCH
// ============================================================================

void SearchBehavior::commandScanLeft(uint32_t now_ms) {
    scan_command_ = 1;
    scan_command_expiry_ms_ = now_ms + Config::SEARCH_SCAN_COMMAND_TIMEOUT_MS;
    setTelemetryEvent(TE_SEARCH_SCAN_LEFT_COMMAND);
}

void SearchBehavior::commandScanRight(uint32_t now_ms) {
    scan_command_ = 2;
    scan_command_expiry_ms_ = now_ms + Config::SEARCH_SCAN_COMMAND_TIMEOUT_MS;
    setTelemetryEvent(TE_SEARCH_SCAN_RIGHT_COMMAND);
}

void SearchBehavior::commandForward(uint32_t now_ms) {
    (void)now_ms;
    clearScanCommand();
    forward_direction_ = 1.0f;
    setTelemetryEvent(TE_SEARCH_FORWARD_STARTED);
}

void SearchBehavior::clearScanCommand() {
    scan_command_ = 0;
    scan_command_expiry_ms_ = 0;
}


// ============================================================================
// COVERAGE GRID & UNMET CELL SEARCH
// ============================================================================

void SearchBehavior::updateCoverage(const Types::Pose2D& pose, uint32_t now_ms) {
    if (now_ms - last_coverage_update_ms_ < Config::SEARCH_COVERAGE_UPDATE_MS) {
        return;
    }
    last_coverage_update_ms_ = now_ms;

    float res = Config::SEARCH_COVERAGE_GRID_RESOLUTION_M;
    float footprint = Config::SEARCH_SENSOR_FOOTPRINT_RADIUS_M;

    // Check if drone is inside minefield zone
    if (pose.field_y < Config::MINEFIELD_Y_MIN || pose.field_y > Config::MINEFIELD_Y_MAX ||
        pose.field_x < Config::FIELD_X_MIN || pose.field_x > Config::FIELD_X_MAX) {
        return;
    }

    int min_c = std::max(0, static_cast<int>((pose.field_x - footprint) / res));
    int max_c = std::min(static_cast<int>(Config::SEARCH_COVERAGE_GRID_COLS) - 1, static_cast<int>((pose.field_x + footprint) / res));

    int min_r = std::max(0, static_cast<int>((pose.field_y - Config::MINEFIELD_Y_MIN - footprint) / res));
    int max_r = std::min(static_cast<int>(Config::SEARCH_COVERAGE_GRID_ROWS) - 1, static_cast<int>((pose.field_y - Config::MINEFIELD_Y_MIN + footprint) / res));

    float footprint_sq = footprint * footprint;

    for (int r = min_r; r <= max_r; ++r) {
        float cell_y = Config::MINEFIELD_Y_MIN + (r + 0.5f) * res;
        float dy = cell_y - pose.field_y;

        for (int c = min_c; c <= max_c; ++c) {
            float cell_x = (c + 0.5f) * res;
            float dx = cell_x - pose.field_x;

            if ((dx * dx + dy * dy) <= footprint_sq) {
                uint16_t idx = r * Config::SEARCH_COVERAGE_GRID_COLS + c;
                coverage_grid_[idx] = 1; // SCANNED
            }
        }
    }

    // Compute total coverage percentage
    uint16_t total_cells = Config::SEARCH_COVERAGE_GRID_COLS * Config::SEARCH_COVERAGE_GRID_ROWS;
    uint16_t scanned_cells = 0;
    for (uint16_t i = 0; i < total_cells; ++i) {
        if (coverage_grid_[i] > 0) scanned_cells++;
    }

    coverage_percent_ = (static_cast<float>(scanned_cells) / static_cast<float>(total_cells)) * 100.0f;
    setTelemetryEvent(TE_SEARCH_COVERAGE_UPDATED);
}

bool SearchBehavior::findNearestUnscannedCell(const Types::Pose2D& pose, Types::Vec2& out_target) {
    float res = Config::SEARCH_COVERAGE_GRID_RESOLUTION_M;
    float best_dist_sq = 1e9f;
    bool found = false;

    // Search preferred lane first
    float lane_x = target_x_;

    for (uint16_t r = 0; r < Config::SEARCH_COVERAGE_GRID_ROWS; ++r) {
        float cell_y = Config::MINEFIELD_Y_MIN + (r + 0.5f) * res;
        for (uint16_t c = 0; c < Config::SEARCH_COVERAGE_GRID_COLS; ++c) {
            uint16_t idx = r * Config::SEARCH_COVERAGE_GRID_COLS + c;
            if (coverage_grid_[idx] == 0) { // UNSCANNED
                float cell_x = (c + 0.5f) * res;
                float lane_bias = std::abs(cell_x - lane_x) * 2.0f; // lane preference
                float dx = cell_x - pose.field_x;
                float dy = cell_y - pose.field_y;
                float dist_sq = dx * dx + dy * dy + lane_bias * lane_bias;

                if (dist_sq < best_dist_sq) {
                    best_dist_sq = dist_sq;
                    out_target = Types::Vec2(cell_x, cell_y);
                    found = true;
                }
            }
        }
    }

    return found;
}


// ============================================================================
// MAIN UPDATE FUNCTION (50 Hz NON-BLOCKING)
// ============================================================================

void SearchBehavior::update(const SearchInputs& inputs) {
    uint32_t now = inputs.now_ms;

    // Check scan command expiration
    if (scan_command_ != 0 && now >= scan_command_expiry_ms_) {
        clearScanCommand();
        setTelemetryEvent(TE_SEARCH_SCAN_COMMAND_EXPIRED);
    }

    // Role assignment sync from swarm / inputs
    if (inputs.swarm_role_failover_active || inputs.lane_assigned_by_swarm) {
        if (inputs.role != active_role_) {
            setRole(inputs.role);
        }
        if (inputs.lane_id != active_lane_id_ && inputs.lane_id != Config::LANE_NONE) {
            setLaneId(inputs.lane_id);
        }
    }

    // Update Coverage Grid
    if (inputs.camera_healthy && inputs.localization_healthy &&
        (inputs.drone_state == Types::DroneState::SEARCHING || inputs.drone_state == Types::DroneState::FORMATION)) {
        updateCoverage(inputs.pose, now);
    }

    // Evaluate Coverage Sufficiency
    if (coverage_percent_ >= Config::SEARCH_COVERAGE_SUFFICIENT_PERCENT &&
        (!inputs.path_planner_requests_more_scan || inputs.route_possible)) {
        if (!coverage_sufficient_) {
            coverage_sufficient_ = true;
            needs_more_scan_ = false;
            setTelemetryEvent(TE_SEARCH_COVERAGE_SUFFICIENT);
        }
    } else {
        coverage_sufficient_ = false;
        needs_more_scan_ = true;
    }

    // Safety & Permission Gating
    if (inputs.safety_action != Types::SafetyAction::CONTINUE ||
        !inputs.localization_healthy ||
        (inputs.drone_state != Types::DroneState::SEARCHING && inputs.drone_state != Types::DroneState::FORMATION)) {
        velocity_command_ = Types::Vec2(0.0f, 0.0f);
        return;
    }

    // Handle RESERVE Role
    if (active_role_ == Types::DroneRole::RESERVE && !inputs.swarm_role_failover_active) {
        float dx = Config::SEARCH_RESERVE_HOLD_X - inputs.pose.field_x;
        float dy = Config::SEARCH_RESERVE_HOLD_Y - inputs.pose.field_y;
        float vx = std::max(-Config::SEARCH_MAX_SPEED_MPS, std::min(Config::SEARCH_MAX_SPEED_MPS, dx * Config::SEARCH_LANE_HOLD_GAIN));
        float vy = std::max(-Config::SEARCH_MAX_SPEED_MPS, std::min(Config::SEARCH_MAX_SPEED_MPS, dy * Config::SEARCH_Y_HOLD_GAIN));
        velocity_command_ = Types::Vec2(vx, vy);
        setTelemetryEvent(TE_SEARCH_RESERVE_HOLD);
        return;
    }

    // Handle FORMATION State
    if (inputs.drone_state == Types::DroneState::FORMATION) {
        float start_x = getLaneCenterX(active_lane_id_);
        float start_y = Config::MINEFIELD_Y_MIN + 1.0f;

        float dx = start_x - inputs.pose.field_x;
        float dy = start_y - inputs.pose.field_y;

        float vx = std::max(-Config::SEARCH_MAX_SPEED_MPS, std::min(Config::SEARCH_MAX_SPEED_MPS, dx * Config::SEARCH_LANE_HOLD_GAIN));
        float vy = std::max(-Config::SEARCH_MAX_SPEED_MPS, std::min(Config::SEARCH_MAX_SPEED_MPS, dy * Config::SEARCH_Y_HOLD_GAIN));

        velocity_command_ = Types::Vec2(vx, vy);
        return;
    }

    // Handle SEARCHING State
    if (inputs.drone_state == Types::DroneState::SEARCHING) {
        // Serpentine Turnaround Check
        if (Config::SEARCH_SERPENTINE_ENABLED) {
            if (forward_direction_ > 0.0f && inputs.pose.field_y >= (Config::MINEFIELD_Y_MAX - Config::SEARCH_TURNAROUND_MARGIN_M)) {
                forward_direction_ = -1.0f;
                target_x_ += (active_lane_id_ == Config::LANE_LEFT) ? Config::SEARCH_LANE_SHIFT_M : -Config::SEARCH_LANE_SHIFT_M;
                setTelemetryEvent(TE_SEARCH_TURNAROUND);
                setTelemetryEvent(TE_SEARCH_SERPENTINE_SHIFT);
            } else if (forward_direction_ < 0.0f && inputs.pose.field_y <= (Config::MINEFIELD_Y_MIN + Config::SEARCH_TURNAROUND_MARGIN_M)) {
                forward_direction_ = 1.0f;
                target_x_ = getLaneCenterX(active_lane_id_);
                setTelemetryEvent(TE_SEARCH_TURNAROUND);
            }
        }

        // Clamp target_x_ to safe field bounds
        target_x_ = std::max(Config::FIELD_X_MIN + 1.5f, std::min(Config::FIELD_X_MAX - 1.5f, target_x_));
        target_y_ = inputs.pose.field_y + forward_direction_ * Config::SEARCH_Y_LOOKAHEAD_M;

        // Base velocity commands
        float vx = (target_x_ - inputs.pose.field_x) * Config::SEARCH_LANE_HOLD_GAIN;
        float vy = forward_direction_ * Config::SEARCH_FORWARD_SPEED_MPS;

        // Scan command bias
        if (scan_command_ == 1) {
            vx -= Config::SEARCH_SCAN_BIAS_SPEED_MPS;
        } else if (scan_command_ == 2) {
            vx += Config::SEARCH_SCAN_BIAS_SPEED_MPS;
        }

        // Geofence continuous steering
        float g_corr_x = std::max(-Config::SEARCH_MAX_GEOFENCE_CORRECTION_MPS, std::min(Config::SEARCH_MAX_GEOFENCE_CORRECTION_MPS, inputs.geofence_correction.x * Config::SEARCH_GEOFENCE_CORRECTION_GAIN));
        float g_corr_y = std::max(-Config::SEARCH_MAX_GEOFENCE_CORRECTION_MPS, std::min(Config::SEARCH_MAX_GEOFENCE_CORRECTION_MPS, inputs.geofence_correction.y * Config::SEARCH_GEOFENCE_CORRECTION_GAIN));

        vx += g_corr_x;
        vy += g_corr_y;

        if (inputs.geofence_status == Types::GeofenceStatus::GEOFENCE_WARNING ||
            inputs.geofence_status == Types::GeofenceStatus::GEOFENCE_NEAR_LIMIT) {
            vy *= 0.5f; // Damping forward speed near boundary
            setTelemetryEvent(TE_SEARCH_GEOFENCE_CORRECTION_ACTIVE);
        } else if (inputs.geofence_status == Types::GeofenceStatus::GEOFENCE_OUTSIDE) {
            vy = 0.0f; // Prioritize returning inside
        }

        // Peer separation avoidance
        if (inputs.nearest_peer_distance_valid) {
            if (inputs.nearest_peer_distance_m <= Config::SEARCH_PEER_CRITICAL_SEPARATION_M) {
                vx = 0.0f;
                vy = 0.0f;
                setTelemetryEvent(TE_SEARCH_PEER_SEPARATION_CRITICAL);
            } else if (inputs.nearest_peer_distance_m <= Config::SEARCH_PEER_SEPARATION_M) {
                vy *= 0.5f; // Slow down
                setTelemetryEvent(TE_SEARCH_PEER_SEPARATION_WARNING);
            }
        }

        // Speed clamping
        float speed = std::sqrt(vx * vx + vy * vy);
        if (speed > Config::SEARCH_MAX_SPEED_MPS) {
            float scale = Config::SEARCH_MAX_SPEED_MPS / speed;
            vx *= scale;
            vy *= scale;
        }

        velocity_command_ = Types::Vec2(vx, vy);
    }
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void search_behavior_init(Types::DroneRole role) {
    s_global_search_behavior.init();
    s_global_search_behavior.setRole(role);
}

void search_behavior_update(const SearchInputs& inputs) {
    s_global_search_behavior.update(inputs);
}

Types::Vec2 search_behavior_get_velocity() {
    return s_global_search_behavior.getVelocityCommand();
}

bool search_behavior_is_coverage_sufficient() {
    return s_global_search_behavior.isCoverageSufficient();
}

SearchBehavior& search_behavior_get_instance() {
    return s_global_search_behavior;
}

} // namespace RobofestDrone
