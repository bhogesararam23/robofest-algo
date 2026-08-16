#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "mine_map.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

// ============================================================================
// NO PATH REASON CODES
// ============================================================================

constexpr uint8_t PATH_OK                                    = 0;
constexpr uint8_t PATH_ERROR_NO_CONFIRMED_MINES_REQUIRED     = 1;
constexpr uint8_t PATH_ERROR_START_BLOCKED                   = 2;
constexpr uint8_t PATH_ERROR_EXIT_BLOCKED                    = 3;
constexpr uint8_t PATH_ERROR_NO_ROUTE_FOUND                  = 4;
constexpr uint8_t PATH_ERROR_CLEARANCE_VALIDATION_FAILED     = 5;
constexpr uint8_t PATH_ERROR_INVALID_INPUT                   = 6;
constexpr uint8_t PATH_ERROR_GRID_TOO_SMALL                  = 7;
constexpr uint8_t PATH_ERROR_MAP_CHANGED                     = 8;
constexpr uint8_t PATH_ERROR_HUMAN_OFF_CORRIDOR              = 9;
constexpr uint8_t PATH_ERROR_NOT_INITIALIZED                 = 10;


// ============================================================================
// PATH PLANNER CLASS
// ============================================================================

class PathPlanner {
public:
    PathPlanner();

    void init();
    void reset();

    bool computePath(const MineMap& mine_map, uint32_t now_ms);

    bool computePathFromPosition(
        const MineMap& mine_map,
        float start_x,
        float start_y,
        uint32_t now_ms
    );

    bool pathStillValid(const MineMap& mine_map, uint32_t now_ms) const;

    bool pathStillValidForHuman(
        const MineMap& mine_map,
        const Types::HumanTrack& human_track,
        uint32_t now_ms
    ) const;

    Types::SafePath getPath() const { return active_path_; }

    uint8_t getWaypointCount() const { return active_path_.waypoint_count; }
    Types::PathWaypoint getWaypoint(uint8_t index) const;

    float getCorridorWidth() const { return active_path_.corridor_width_m; }
    uint32_t getPathVersion() const { return path_version_; }
    uint32_t getPathCreatedTime() const { return active_path_.created_time; }

    float nearestDeviationFromPath(float x, float y) const;
    bool isPointOnSafePath(float x, float y) const;
    bool isPointInExitZone(float x, float y) const;
    float getForwardProgressAlongPath(float x, float y) const;

    bool shouldReplan(
        const MineMap& mine_map,
        const Types::HumanTrack& human_track,
        uint32_t now_ms
    ) const;

    bool needsMoreScan() const { return needs_more_scan_; }
    uint8_t getNoPathReason() const { return no_path_reason_; }

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    bool buildOccupancyGrid(const MineMap& mine_map);
    bool runAStarSearch(uint16_t start_col, uint16_t start_row, uint16_t exit_col, uint16_t exit_row);
    void extractGridPath(uint16_t start_idx, uint16_t exit_idx);
    void smoothCandidatePath();
    bool validateExactClearance(const Types::SafePath& path, const MineMap& mine_map) const;
    float pointSegmentDistance(float px, float py, float ax, float ay, float bx, float by) const;
    void setTelemetryEvent(uint16_t event_id);

private:
    Types::SafePath active_path_;
    Types::SafePath candidate_path_;

    uint8_t grid_[Config::PATH_MAX_GRID_CELLS] = {};
    float g_score_[Config::PATH_MAX_ASTAR_NODES] = {};
    float f_score_[Config::PATH_MAX_ASTAR_NODES] = {};
    int16_t came_from_[Config::PATH_MAX_ASTAR_NODES] = {};
    uint8_t node_flags_[Config::PATH_MAX_ASTAR_NODES] = {}; // 0 = unvisited, 1 = open, 2 = closed

    uint16_t grid_cols_ = 0;
    uint16_t grid_rows_ = 0;
    float grid_resolution_ = Config::PATH_GRID_RESOLUTION_M;

    uint32_t path_version_ = 0;
    uint32_t path_map_version_ = 0;
    uint32_t last_plan_attempt_ms_ = 0;
    uint32_t last_successful_plan_ms_ = 0;

    bool needs_more_scan_ = false;
    uint8_t no_path_reason_ = PATH_OK;

    uint16_t last_telemetry_event_id_ = TE_PATH_PLANNER_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void path_planner_init();
void path_planner_update(uint32_t now_ms);
bool path_planner_compute(uint32_t now_ms);
Types::SafePath path_planner_get_path();
bool path_planner_is_path_valid();
PathPlanner& path_planner_get_instance();

} // namespace RobofestDrone
