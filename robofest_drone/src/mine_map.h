#pragma once

#include <stdint.h>
#include "types.h"
#include "telemetry_events.h"
#include "../config/mission_config.h"
#include "../config/thresholds.h"

namespace RobofestDrone {

// ============================================================================
// MINE MAP CLASS
// ============================================================================

class MineMap {
public:
    MineMap();

    void init();
    void reset();

    void setSelfDroneId(uint8_t self_drone_id) { self_drone_id_ = self_drone_id; }

    void addDetection(
        float x,
        float y,
        float confidence,
        Types::VisionMarkerType marker_type,
        uint8_t source_drone_id,
        uint32_t now_ms
    );

    void addDetectionFromCandidate(
        const Types::VisionCandidate& candidate,
        uint8_t source_drone_id,
        uint32_t now_ms
    );

    // ------------------------------------------------------------------
    // CROSS-DRONE FUSION + CLASSIFICATION CONSENSUS
    // (items 11/12, REQ-DER-111/112)
    // ------------------------------------------------------------------

    // Fuses one peer observation: confidence x inverse-distance weighted
    // position merge plus a classification vote into the marker's history.
    void addVisionObservation(
        const Types::VisionObsPayload& obs,
        uint8_t source_drone_id,
        uint32_t now_ms
    );

    // Computes the current classification consensus for a mine.
    Types::MarkerConsensus getMarkerConsensus(uint16_t mine_id) const;

    // True when the swarm should ignore this marker's classification
    // (votes too split / weight too low) per REQ item 12 policy.
    bool isMarkerAmbiguous(uint16_t mine_id) const;

    // Marks a local detection as already shared via VISION_OBS broadcast.
    void markObsShared(uint16_t mine_id);

    // Index-ordered access for swarm pump sweeps.
    bool getMineByIndex(uint16_t index, Types::MineRecord& out) const;

    // Pure scoring helper (host-testable): inverse-distance-squared weight.
    static float vision_obs_weight(float confidence, float distance_m);

    // Pure vote resolver (host-testable).
    static bool resolve_votes(
        const Types::VisionMarkerType* types,
        const float* weights,
        uint8_t n,
        Types::MarkerConsensus& out);

    void update(uint32_t now_ms);

    uint16_t getMineCount() const;
    uint16_t getConfirmedCount() const;
    uint16_t getCandidateCount() const;
    uint16_t getRejectedCount() const;

    uint16_t getConfirmedMines(Types::MineRecord* out, uint16_t max_mines) const;
    uint16_t getCandidateMines(Types::MineRecord* out, uint16_t max_mines) const;

    bool getMineById(uint16_t mine_id, Types::MineRecord& out) const;

    uint32_t getMapVersion() const { return map_version_; }

    bool isPointNearConfirmedMine(float x, float y, float clearance_radius_m) const;
    bool isMineNearPath(float x, float y, float clearance_radius_m) const;

    bool isClaimReady(uint16_t mine_id) const;
    bool claimMine(uint16_t mine_id, uint8_t drone_id, uint32_t now_ms);
    void releaseClaim(uint16_t mine_id);
    bool isClaimedByOther(uint16_t mine_id, uint8_t self_drone_id) const;
    void updateClaims(uint32_t now_ms);

    bool exportOccupancyGrid(
        uint8_t* grid,
        uint16_t cols,
        uint16_t rows,
        float resolution_m,
        float inflate_radius_m
    ) const;

    uint16_t getLastTelemetryEventId() const { return last_telemetry_event_id_; }
    bool telemetryEventValid() const { return telemetry_event_valid_; }
    void clearTelemetryEvent() { telemetry_event_valid_ = false; }

private:
    int findNearestMine(float x, float y, float max_radius_m) const;
    int allocateMineSlot();
    uint16_t generateNextMineId();
    void decayStaleCandidates(uint32_t now_ms);
    void checkConfirmation(Types::MineRecord& mine);
    void setTelemetryEvent(uint16_t event_id);

private:
    Types::MineRecord mines_[Types::MAX_MINES] = {};
    uint16_t mine_count_ = 0;
    uint16_t next_mine_id_ = 1;
    uint32_t map_version_ = Config::MINE_MAP_VERSION_START;
    uint8_t self_drone_id_ = 1;
    uint32_t last_decay_ms_ = 0;

    uint16_t last_telemetry_event_id_ = TE_MINE_MAP_INITIALIZED;
    bool telemetry_event_valid_ = true;
};


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void mine_map_init();
void mine_map_update(uint32_t now_ms);
void mine_map_add_candidate(const Types::VisionCandidate& candidate, uint32_t now_ms);
uint16_t mine_map_get_confirmed_mines(Types::MineRecord* out_mines, uint16_t max_count);
bool mine_map_is_point_clear(float x, float y, float clearance_m);
MineMap& mine_map_get_instance();

} // namespace RobofestDrone
