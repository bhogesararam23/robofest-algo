#include "path_planner.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static PathPlanner s_global_path_planner;

    // Static priority queue (min-heap) arrays for A* search (embedded zero-allocation)
    static uint16_t s_heap[Config::PATH_MAX_ASTAR_NODES];
    static uint16_t s_heap_size = 0;

    void heapPush(uint16_t node_idx, const float* f_scores) {
        s_heap[s_heap_size] = node_idx;
        uint16_t curr = s_heap_size++;
        while (curr > 0) {
            uint16_t parent = (curr - 1) / 2;
            if (f_scores[s_heap[curr]] < f_scores[s_heap[parent]]) {
                uint16_t temp = s_heap[curr];
                s_heap[curr] = s_heap[parent];
                s_heap[parent] = temp;
                curr = parent;
            } else {
                break;
            }
        }
    }

    uint16_t heapPop(const float* f_scores) {
        uint16_t root = s_heap[0];
        s_heap[0] = s_heap[--s_heap_size];
        uint16_t curr = 0;
        while (true) {
            uint16_t left = 2 * curr + 1;
            uint16_t right = 2 * curr + 2;
            uint16_t smallest = curr;

            if (left < s_heap_size && f_scores[s_heap[left]] < f_scores[s_heap[smallest]]) {
                smallest = left;
            }
            if (right < s_heap_size && f_scores[s_heap[right]] < f_scores[s_heap[smallest]]) {
                smallest = right;
            }
            if (smallest != curr) {
                uint16_t temp = s_heap[curr];
                s_heap[curr] = s_heap[smallest];
                s_heap[smallest] = temp;
                curr = smallest;
            } else {
                break;
            }
        }
        return root;
    }
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

PathPlanner::PathPlanner() {
    reset();
}

void PathPlanner::init() {
    reset();
}

void PathPlanner::reset() {
    active_path_ = Types::SafePath();
    active_path_.corridor_width_m = Config::PATH_CORRIDOR_WIDTH_M;
    candidate_path_ = Types::SafePath();
    candidate_path_.corridor_width_m = Config::PATH_CORRIDOR_WIDTH_M;

    grid_resolution_ = Config::PATH_GRID_RESOLUTION_M;
    grid_cols_ = static_cast<uint16_t>(std::ceil(Config::FIELD_WIDTH_M / grid_resolution_));  // 30
    grid_rows_ = static_cast<uint16_t>(std::ceil(Config::FIELD_LENGTH_M / grid_resolution_)); // 120

    if (grid_cols_ > Config::PATH_MAX_GRID_COLS) grid_cols_ = Config::PATH_MAX_GRID_COLS;
    if (grid_rows_ > Config::PATH_MAX_GRID_ROWS) grid_rows_ = Config::PATH_MAX_GRID_ROWS;

    path_version_ = 0;
    path_map_version_ = 0;
    last_plan_attempt_ms_ = 0;
    last_successful_plan_ms_ = 0;

    needs_more_scan_ = false;
    no_path_reason_ = PATH_OK;

    last_telemetry_event_id_ = TE_PATH_PLANNER_INITIALIZED;
    telemetry_event_valid_ = true;
}

void PathPlanner::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}

Types::PathWaypoint PathPlanner::getWaypoint(uint8_t index) const {
    if (index < active_path_.waypoint_count) {
        return active_path_.waypoints[index];
    }
    return Types::PathWaypoint();
}


// ============================================================================
// OCCUPANCY GRID BUILDING
// ============================================================================

bool PathPlanner::buildOccupancyGrid(const MineMap& mine_map) {
    if (Config::PATH_REQUIRE_MINE_OBSERVATION_BEFORE_GUIDING &&
        mine_map.getConfirmedCount() == 0 && mine_map.getMineCount() == 0) {
        needs_more_scan_ = true;
    }

    return mine_map.exportOccupancyGrid(grid_, grid_cols_, grid_rows_, grid_resolution_, Config::MINE_CLEARANCE_RADIUS_M);
}


// ============================================================================
// A* GRAPH SEARCH
// ============================================================================

bool PathPlanner::runAStarSearch(uint16_t start_col, uint16_t start_row, uint16_t exit_col, uint16_t exit_row) {
    uint16_t total_nodes = grid_cols_ * grid_rows_;
    if (total_nodes > Config::PATH_MAX_ASTAR_NODES) {
        return false;
    }

    // Initialize search arrays
    for (uint16_t i = 0; i < total_nodes; ++i) {
        g_score_[i] = 1e9f;
        f_score_[i] = 1e9f;
        came_from_[i] = -1;
        node_flags_[i] = 0; // 0 = unvisited, 1 = in open set, 2 = closed
    }

    s_heap_size = 0;

    uint16_t start_idx = start_row * grid_cols_ + start_col;
    uint16_t exit_idx = exit_row * grid_cols_ + exit_col;

    g_score_[start_idx] = 0.0f;
    float dx = static_cast<float>(exit_col) - static_cast<float>(start_col);
    float dy = static_cast<float>(exit_row) - static_cast<float>(start_row);
    f_score_[start_idx] = std::sqrt(dx * dx + dy * dy);

    heapPush(start_idx, f_score_);
    node_flags_[start_idx] = 1;

    // 8-neighbor directional deltas
    static const int dir_c[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    static const int dir_r[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const float dir_cost[8] = {
        Config::PATH_EDGE_COST_STRAIGHT,
        Config::PATH_EDGE_COST_DIAGONAL,
        Config::PATH_EDGE_COST_STRAIGHT,
        Config::PATH_EDGE_COST_DIAGONAL,
        Config::PATH_EDGE_COST_STRAIGHT,
        Config::PATH_EDGE_COST_DIAGONAL,
        Config::PATH_EDGE_COST_STRAIGHT,
        Config::PATH_EDGE_COST_DIAGONAL
    };

    while (s_heap_size > 0) {
        uint16_t current_idx = heapPop(f_score_);
        if (current_idx == exit_idx) {
            return true; // Exit reached!
        }

        node_flags_[current_idx] = 2; // Closed

        uint16_t c_col = current_idx % grid_cols_;
        uint16_t c_row = current_idx / grid_cols_;

        for (int d = 0; d < 8; ++d) {
            if (!Config::PATH_ALLOW_DIAGONAL_MOVEMENT && (d % 2 != 0)) {
                continue;
            }

            int n_col = static_cast<int>(c_col) + dir_c[d];
            int n_row = static_cast<int>(c_row) + dir_r[d];

            if (n_col < 0 || n_col >= grid_cols_ || n_row < 0 || n_row >= grid_rows_) {
                continue;
            }

            uint16_t n_idx = static_cast<uint16_t>(n_row * grid_cols_ + n_col);

            // Blocked obstacle check
            if (grid_[n_idx] == 1 || node_flags_[n_idx] == 2) {
                continue;
            }

            // Diagonal corner cutting prevention
            if (Config::PATH_PREVENT_CORNER_CUTTING && (d % 2 != 0)) {
                uint16_t adj1 = static_cast<uint16_t>(c_row * grid_cols_ + n_col);
                uint16_t adj2 = static_cast<uint16_t>(n_row * grid_cols_ + c_col);
                if (grid_[adj1] == 1 || grid_[adj2] == 1) {
                    continue;
                }
            }

            float tentative_g = g_score_[current_idx] + dir_cost[d];
            if (tentative_g < g_score_[n_idx]) {
                came_from_[n_idx] = static_cast<int16_t>(current_idx);
                g_score_[n_idx] = tentative_g;

                float h_dx = static_cast<float>(exit_col) - static_cast<float>(n_col);
                float h_dy = static_cast<float>(exit_row) - static_cast<float>(n_row);
                f_score_[n_idx] = tentative_g + std::sqrt(h_dx * h_dx + h_dy * h_dy);

                if (node_flags_[n_idx] != 1) {
                    heapPush(n_idx, f_score_);
                    node_flags_[n_idx] = 1;
                }
            }
        }
    }

    return false; // No route possible
}


// ============================================================================
// PATH EXTRACTION & DECIMATION
// ============================================================================

void PathPlanner::extractGridPath(uint16_t start_idx, uint16_t exit_idx) {
    candidate_path_ = Types::SafePath();
    candidate_path_.corridor_width_m = Config::PATH_CORRIDOR_WIDTH_M;

    // Backtrack from exit to start
    static uint16_t temp_indices[Config::PATH_MAX_ASTAR_NODES];
    uint16_t count = 0;
    int16_t curr = static_cast<int16_t>(exit_idx);

    while (curr >= 0 && count < Config::PATH_MAX_ASTAR_NODES) {
        temp_indices[count++] = static_cast<uint16_t>(curr);
        if (curr == static_cast<int16_t>(start_idx)) {
            break;
        }
        curr = came_from_[curr];
    }

    if (count < 2) return;

    // Convert reversed indices to forward waypoints in field coordinates
    uint8_t wp_count = 0;
    for (int i = static_cast<int>(count) - 1; i >= 0; --i) {
        uint16_t idx = temp_indices[i];
        uint16_t col = idx % grid_cols_;
        uint16_t row = idx / grid_cols_;

        float wx = Config::FIELD_X_MIN + (static_cast<float>(col) + 0.5f) * grid_resolution_;
        float wy = Config::FIELD_Y_MIN + (static_cast<float>(row) + 0.5f) * grid_resolution_;

        // Decimate nearly collinear points along straight lines
        if (wp_count >= 2) {
            float prev_x = candidate_path_.waypoints[wp_count - 1].x;
            float prev_y = candidate_path_.waypoints[wp_count - 1].y;
            float pprev_x = candidate_path_.waypoints[wp_count - 2].x;
            float pprev_y = candidate_path_.waypoints[wp_count - 2].y;

            float cross_product = (wx - prev_x) * (prev_y - pprev_y) - (wy - prev_y) * (prev_x - pprev_x);
            if (std::abs(cross_product) < 0.001f) {
                // Replace previous collinear point
                candidate_path_.waypoints[wp_count - 1] = Types::PathWaypoint(wx, wy);
                continue;
            }
        }

        if (wp_count < Config::PATH_MAX_WAYPOINTS) {
            candidate_path_.waypoints[wp_count++] = Types::PathWaypoint(wx, wy);
        }
    }

    candidate_path_.waypoint_count = wp_count;
    candidate_path_.valid = (wp_count >= 2);
}


// ============================================================================
// PATH SMOOTHING
// ============================================================================

void PathPlanner::smoothCandidatePath() {
    if (!candidate_path_.valid || candidate_path_.waypoint_count < 3) {
        return;
    }

    // Moving-average smoothing preserving Start and Exit endpoints
    for (uint8_t iter = 0; iter < Config::PATH_SMOOTHING_ITERATIONS; ++iter) {
        for (uint8_t i = 1; i < candidate_path_.waypoint_count - 1; ++i) {
            float smoothed_x = 0.25f * candidate_path_.waypoints[i - 1].x +
                               0.50f * candidate_path_.waypoints[i].x +
                               0.25f * candidate_path_.waypoints[i + 1].x;

            float smoothed_y = 0.25f * candidate_path_.waypoints[i - 1].y +
                               0.50f * candidate_path_.waypoints[i].y +
                               0.25f * candidate_path_.waypoints[i + 1].y;

            candidate_path_.waypoints[i].x = smoothed_x;
            candidate_path_.waypoints[i].y = smoothed_y;
        }
    }
}


// ============================================================================
// EXACT CLEARANCE VALIDATION (1.0 METER RULE)
// ============================================================================

float PathPlanner::pointSegmentDistance(
    float px,
    float py,
    float ax,
    float ay,
    float bx,
    float by
) const {
    float abx = bx - ax;
    float aby = by - ay;
    float apx = px - ax;
    float apy = py - ay;

    float l2 = abx * abx + aby * aby;
    if (l2 < 1e-6f) {
        return std::sqrt(apx * apx + apy * apy);
    }

    float t = (apx * abx + apy * aby) / l2;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    float closest_x = ax + t * abx;
    float closest_y = ay + t * aby;

    float dx = px - closest_x;
    float dy = py - closest_y;
    return std::sqrt(dx * dx + dy * dy);
}

bool PathPlanner::validateExactClearance(const Types::SafePath& path, const MineMap& mine_map) const {
    if (!path.valid || path.waypoint_count < 2) {
        return false;
    }

    static Types::MineRecord confirmed_mines[Types::MAX_MINES];
    uint16_t confirmed_count = mine_map.getConfirmedMines(confirmed_mines, Types::MAX_MINES);

    if (confirmed_count == 0) {
        return true;
    }

    constexpr float SAFE_CLEARANCE_LIMIT = Config::MINE_CLEARANCE_RADIUS_M * 0.999f;

    for (uint16_t m = 0; m < confirmed_count; ++m) {
        float mx = confirmed_mines[m].x;
        float my = confirmed_mines[m].y;

        // 1. Check each waypoint
        for (uint8_t w = 0; w < path.waypoint_count; ++w) {
            float dx = path.waypoints[w].x - mx;
            float dy = path.waypoints[w].y - my;
            if (std::sqrt(dx * dx + dy * dy) < SAFE_CLEARANCE_LIMIT) {
                return false;
            }
        }

        // 2. Check each continuous segment
        for (uint8_t s = 0; s < path.waypoint_count - 1; ++s) {
            float dist = pointSegmentDistance(
                mx, my,
                path.waypoints[s].x, path.waypoints[s].y,
                path.waypoints[s + 1].x, path.waypoints[s + 1].y
            );
            if (dist < SAFE_CLEARANCE_LIMIT) {
                return false;
            }
        }
    }

    return true;
}


// ============================================================================
// MAIN PATH COMPUTATION
// ============================================================================

bool PathPlanner::computePath(const MineMap& mine_map, uint32_t now_ms) {
    return computePathFromPosition(mine_map, Config::PATH_START_ANCHOR_X, Config::PATH_START_ANCHOR_Y, now_ms);
}

bool PathPlanner::computePathFromPosition(
    const MineMap& mine_map,
    float start_x,
    float start_y,
    uint32_t now_ms
) {
    last_plan_attempt_ms_ = now_ms;
    setTelemetryEvent(TE_PATH_PLAN_STARTED);

    // 1. Coordinate boundary validation
    if (start_x < Config::FIELD_X_MIN || start_x > Config::FIELD_X_MAX ||
        start_y < Config::START_ZONE_Y_MIN || start_y > Config::FIELD_LENGTH_M) {
        no_path_reason_ = PATH_ERROR_INVALID_INPUT;
        setTelemetryEvent(TE_PATH_PLAN_FAILED);
        return false;
    }

    // 2. Convert start and exit positions to grid cells
    uint16_t start_col = static_cast<uint16_t>(start_x / grid_resolution_);
    uint16_t start_row = static_cast<uint16_t>(start_y / grid_resolution_);
    uint16_t exit_col  = static_cast<uint16_t>(Config::PATH_EXIT_ANCHOR_X / grid_resolution_);
    uint16_t exit_row  = static_cast<uint16_t>(Config::PATH_EXIT_ANCHOR_Y / grid_resolution_);

    if (start_col >= grid_cols_) start_col = grid_cols_ - 1;
    if (start_row >= grid_rows_) start_row = grid_rows_ - 1;
    if (exit_col >= grid_cols_)  exit_col = grid_cols_ - 1;
    if (exit_row >= grid_rows_)  exit_row = grid_rows_ - 1;

    // 3. Build inflated occupancy grid
    if (!buildOccupancyGrid(mine_map)) {
        no_path_reason_ = PATH_ERROR_GRID_TOO_SMALL;
        setTelemetryEvent(TE_PATH_PLAN_FAILED);
        return false;
    }

    // 4. Check start/exit cell clearances
    uint16_t start_idx = start_row * grid_cols_ + start_col;
    uint16_t exit_idx  = exit_row * grid_cols_ + exit_col;

    if (grid_[start_idx] == 1) {
        no_path_reason_ = PATH_ERROR_START_BLOCKED;
        needs_more_scan_ = true;
        setTelemetryEvent(TE_PATH_START_BLOCKED);
        return false;
    }

    if (grid_[exit_idx] == 1) {
        no_path_reason_ = PATH_ERROR_EXIT_BLOCKED;
        needs_more_scan_ = true;
        setTelemetryEvent(TE_PATH_EXIT_BLOCKED);
        return false;
    }

    // 5. Run A* graph search
    if (!runAStarSearch(start_col, start_row, exit_col, exit_row)) {
        no_path_reason_ = PATH_ERROR_NO_ROUTE_FOUND;
        needs_more_scan_ = true;
        setTelemetryEvent(TE_PATH_NO_ROUTE_FOUND);
        return false;
    }

    // 6. Extract candidate path
    extractGridPath(start_idx, exit_idx);
    if (!candidate_path_.valid) {
        no_path_reason_ = PATH_ERROR_NO_ROUTE_FOUND;
        setTelemetryEvent(TE_PATH_PLAN_FAILED);
        return false;
    }

    // Ensure first waypoint is exactly start anchor and last is exit anchor
    candidate_path_.waypoints[0] = Types::PathWaypoint(start_x, start_y);
    candidate_path_.waypoints[candidate_path_.waypoint_count - 1] =
        Types::PathWaypoint(Config::PATH_EXIT_ANCHOR_X, Config::PATH_EXIT_ANCHOR_Y);

    // 7. Path Smoothing & Exact Validation
    Types::SafePath unsmoothed = candidate_path_;
    smoothCandidatePath();

    if (validateExactClearance(candidate_path_, mine_map)) {
        setTelemetryEvent(TE_PATH_SMOOTHED);
    } else {
        // Fallback to unsmoothed grid path
        candidate_path_ = unsmoothed;
        setTelemetryEvent(TE_PATH_SMOOTHING_REJECTED);

        if (!validateExactClearance(candidate_path_, mine_map)) {
            no_path_reason_ = PATH_ERROR_CLEARANCE_VALIDATION_FAILED;
            setTelemetryEvent(TE_PATH_CLEARANCE_VALIDATION_FAILED);
            return false;
        }
    }

    // 8. Commit active path
    active_path_ = candidate_path_;
    active_path_.valid = true;
    active_path_.created_time = now_ms;
    active_path_.corridor_width_m = Config::PATH_CORRIDOR_WIDTH_M;
    path_version_++;
    active_path_.path_version = path_version_;
    path_map_version_ = mine_map.getMapVersion();
    last_successful_plan_ms_ = now_ms;

    needs_more_scan_ = false;
    no_path_reason_ = PATH_OK;
    setTelemetryEvent(TE_PATH_PLAN_SUCCESS);
    return true;
}


// ============================================================================
// VALIDATION & SPATIAL HELPERS
// ============================================================================

bool PathPlanner::pathStillValid(const MineMap& mine_map, uint32_t now_ms) const {
    (void)now_ms;

    if (!active_path_.valid || active_path_.waypoint_count < 2) {
        return false;
    }

    if (mine_map.getMapVersion() != path_map_version_) {
        return false;
    }

    return validateExactClearance(active_path_, mine_map);
}

bool PathPlanner::pathStillValidForHuman(
    const MineMap& mine_map,
    const Types::HumanTrack& human_track,
    uint32_t now_ms
) const {
    if (!pathStillValid(mine_map, now_ms)) {
        return false;
    }

    if (human_track.human_detected && human_track.lateral_deviation_m > Config::PATH_DEVIATION_TOLERANCE_M) {
        return false;
    }

    return true;
}

float PathPlanner::nearestDeviationFromPath(float x, float y) const {
    if (!active_path_.valid || active_path_.waypoint_count < 2) {
        return 1000.0f;
    }

    float min_dist = 1000.0f;
    for (uint8_t i = 0; i < active_path_.waypoint_count - 1; ++i) {
        float dist = pointSegmentDistance(
            x, y,
            active_path_.waypoints[i].x, active_path_.waypoints[i].y,
            active_path_.waypoints[i + 1].x, active_path_.waypoints[i + 1].y
        );
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    return min_dist;
}

bool PathPlanner::isPointOnSafePath(float x, float y) const {
    return nearestDeviationFromPath(x, y) <= (active_path_.corridor_width_m * 0.5f);
}

bool PathPlanner::isPointInExitZone(float x, float y) const {
    return (y >= Config::EXIT_ZONE_Y_MIN && y <= Config::EXIT_ZONE_Y_MAX &&
            x >= Config::FIELD_X_MIN && x <= Config::FIELD_X_MAX);
}

float PathPlanner::getForwardProgressAlongPath(float x, float y) const {
    if (!active_path_.valid || active_path_.waypoint_count < 2) {
        return 0.0f;
    }

    float total_progress = 0.0f;
    float best_dist = 1000.0f;
    float progress_at_closest = 0.0f;

    for (uint8_t i = 0; i < active_path_.waypoint_count - 1; ++i) {
        float ax = active_path_.waypoints[i].x;
        float ay = active_path_.waypoints[i].y;
        float bx = active_path_.waypoints[i + 1].x;
        float by = active_path_.waypoints[i + 1].y;

        float abx = bx - ax;
        float aby = by - ay;
        float seg_len = std::sqrt(abx * abx + aby * aby);

        float dist = pointSegmentDistance(x, y, ax, ay, bx, by);
        if (dist < best_dist) {
            best_dist = dist;
            float t = (seg_len > 0.001f) ? ((x - ax) * abx + (y - ay) * aby) / (seg_len * seg_len) : 0.0f;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            progress_at_closest = total_progress + t * seg_len;
        }

        total_progress += seg_len;
    }

    return progress_at_closest;
}

bool PathPlanner::shouldReplan(
    const MineMap& mine_map,
    const Types::HumanTrack& human_track,
    uint32_t now_ms
) const {
    if (!active_path_.valid) {
        return true;
    }

    if (mine_map.getMapVersion() != path_map_version_) {
        return true;
    }

    if (!validateExactClearance(active_path_, mine_map)) {
        return true;
    }

    if (human_track.human_detected && human_track.lateral_deviation_m > Config::PATH_DEVIATION_TOLERANCE_M) {
        if ((now_ms - last_plan_attempt_ms_) > Config::PATH_REPLAN_COOLDOWN_MS) {
            return true;
        }
    }

    return false;
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void path_planner_init() {
    s_global_path_planner.init();
}

void path_planner_update(uint32_t now_ms) {
    (void)now_ms;
}

bool path_planner_compute(uint32_t now_ms) {
    return s_global_path_planner.computePath(mine_map_get_instance(), now_ms);
}

Types::SafePath path_planner_get_path() {
    return s_global_path_planner.getPath();
}

bool path_planner_is_path_valid() {
    return s_global_path_planner.getPath().valid;
}

PathPlanner& path_planner_get_instance() {
    return s_global_path_planner;
}

} // namespace RobofestDrone
