#include "mine_map.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace RobofestDrone {

namespace {
    static MineMap s_global_mine_map;
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

MineMap::MineMap() {
    reset();
}

void MineMap::init() {
    reset();
}

void MineMap::reset() {
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        mines_[i] = Types::MineRecord();
    }

    mine_count_ = 0;
    next_mine_id_ = 1;
    map_version_ = Config::MINE_MAP_VERSION_START;
    self_drone_id_ = 1;
    last_decay_ms_ = 0;

    last_telemetry_event_id_ = TE_MINE_MAP_INITIALIZED;
    telemetry_event_valid_ = true;
}

void MineMap::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}


// ============================================================================
// ID MANAGEMENT & SLOT ALLOCATION
// ============================================================================

uint16_t MineMap::generateNextMineId() {
    uint16_t id = next_mine_id_++;
    if (id == 0 || id == 65535) {
        id = 1;
        next_mine_id_ = 2;
    }

    // Ensure unique ID by skipping any currently active mine IDs
    bool collision = true;
    while (collision) {
        collision = false;
        for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
            if (mines_[i].mine_id == id && mines_[i].status != Types::MineStatus::REJECTED) {
                id++;
                if (id == 0 || id == 65535) id = 1;
                collision = true;
                break;
            }
        }
    }

    return id;
}

int MineMap::allocateMineSlot() {
    // 1. First priority: find an empty unused slot
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id == 0) {
            return static_cast<int>(i);
        }
    }

    // 2. Second priority: recycle a REJECTED mine slot
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].status == Types::MineStatus::REJECTED) {
            return static_cast<int>(i);
        }
    }

    // 3. Third priority: replace the lowest-confidence CANDIDATE
    int lowest_idx = -1;
    float lowest_confidence = 1000.0f;
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].status == Types::MineStatus::CANDIDATE) {
            if (mines_[i].confidence < lowest_confidence) {
                lowest_confidence = mines_[i].confidence;
                lowest_idx = static_cast<int>(i);
            }
        }
    }

    return lowest_idx;
}


// ============================================================================
// DEDUPLICATION SEARCH
// ============================================================================

int MineMap::findNearestMine(float x, float y, float max_radius_m) const {
    int best_match = -1;
    float min_dist_sq = max_radius_m * max_radius_m;

    // Search CONFIRMED mines first
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status == Types::MineStatus::CONFIRMED) {
            float dx = mines_[i].x - x;
            float dy = mines_[i].y - y;
            float dist_sq = dx * dx + dy * dy;

            if (dist_sq <= min_dist_sq) {
                min_dist_sq = dist_sq;
                best_match = static_cast<int>(i);
            }
        }
    }

    if (best_match >= 0) {
        return best_match;
    }

    // Then search CANDIDATE mines
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status == Types::MineStatus::CANDIDATE) {
            float dx = mines_[i].x - x;
            float dy = mines_[i].y - y;
            float dist_sq = dx * dx + dy * dy;

            if (dist_sq <= min_dist_sq) {
                min_dist_sq = dist_sq;
                best_match = static_cast<int>(i);
            }
        }
    }

    return best_match;
}


// ============================================================================
// DETECTION ADDITION & FUSION
// ============================================================================

void MineMap::addDetection(
    float x,
    float y,
    float confidence,
    Types::VisionMarkerType marker_type,
    uint8_t source_drone_id,
    uint32_t now_ms
) {
    // 1. Input sanity validation
    if (std::isnan(x) || std::isnan(y) || std::isnan(confidence) || confidence < 0.0f || confidence > 100.0f) {
        setTelemetryEvent(TE_MINE_REJECTED_INVALID_INPUT);
        return;
    }

    if (Config::MINE_REJECT_OUTSIDE_FIELD) {
        if (x < (Config::FIELD_X_MIN - Config::MINE_MAP_BOUNDARY_MARGIN_M) ||
            x > (Config::FIELD_X_MAX + Config::MINE_MAP_BOUNDARY_MARGIN_M) ||
            y < (Config::FIELD_Y_MIN - Config::MINE_MAP_BOUNDARY_MARGIN_M) ||
            y > (Config::FIELD_Y_MAX + Config::MINE_MAP_BOUNDARY_MARGIN_M)) {
            setTelemetryEvent(TE_MINE_REJECTED_OUTSIDE_FIELD);
            return;
        }
    }

    // 2. Select deduplication radius
    float dedup_radius = (source_drone_id == self_drone_id_) ?
                         Config::SAME_DRONE_DEDUP_RADIUS_M : Config::CROSS_DRONE_DEDUP_RADIUS_M;

    // 3. Deduplication search
    int match_idx = findNearestMine(x, y, dedup_radius);

    if (match_idx >= 0) {
        // Fuse detection into existing mine record
        Types::MineRecord& m = mines_[match_idx];
        m.persistence_count++;
        m.last_seen_time = now_ms;
        m.update_count++;

        // Confidence weighted fusion
        m.confidence = m.confidence * (1.0f - Config::MINE_FUSION_CONFIDENCE_GAIN) +
                       confidence * Config::MINE_FUSION_CONFIDENCE_GAIN;
        if (m.confidence > 100.0f) m.confidence = 100.0f;
        if (m.confidence < 0.0f) m.confidence = 0.0f;

        // Position fusion (dampen movement of confirmed mines on weak detections)
        float pos_gain = Config::MINE_FUSION_POSITION_GAIN;
        if (m.status == Types::MineStatus::CONFIRMED && confidence < (m.confidence - 20.0f)) {
            pos_gain *= 0.5f;
        }

        m.x = m.x * (1.0f - pos_gain) + x * pos_gain;
        m.y = m.y * (1.0f - pos_gain) + y * pos_gain;

        if (m.marker_type == Types::VisionMarkerType::UNKNOWN && marker_type != Types::VisionMarkerType::UNKNOWN) {
            m.marker_type = marker_type;
        }

        checkConfirmation(m);
        setTelemetryEvent(TE_MINE_CANDIDATE_FUSED);
    } else {
        // Allocate slot for new mine candidate
        int slot_idx = allocateMineSlot();
        if (slot_idx < 0) {
            setTelemetryEvent(TE_MINE_MAP_FULL);
            return;
        }

        Types::MineRecord& m = mines_[slot_idx];
        m.mine_id = generateNextMineId();
        m.x = x;
        m.y = y;
        m.confidence = confidence;
        m.persistence_count = 1;
        m.first_seen_time = now_ms;
        m.last_seen_time = now_ms;
        m.source_drone_id = source_drone_id;
        m.marker_type = marker_type;
        m.status = Types::MineStatus::CANDIDATE;
        m.map_version = map_version_;
        m.claimed = false;
        m.claim_owner_id = 0;
        m.claim_time_ms = 0;
        m.claim_expiry_ms = 0;
        m.rejection_reason = 0;
        m.update_count = 1;

        checkConfirmation(m);
        setTelemetryEvent(TE_MINE_CANDIDATE_CREATED);
    }
}

void MineMap::addDetectionFromCandidate(
    const Types::VisionCandidate& candidate,
    uint8_t source_drone_id,
    uint32_t now_ms
) {
    if ((now_ms >= candidate.timestamp_ms) && ((now_ms - candidate.timestamp_ms) > Config::MINE_DETECTION_MAX_AGE_MS)) {
        return;
    }

    addDetection(candidate.world_x, candidate.world_y, candidate.confidence, candidate.marker_type, source_drone_id, now_ms);
}


// ============================================================================
// CROSS-DRONE VISION FUSION + CLASSIFICATION CONSENSUS
// (items 11/12, REQ-DER-111/112)
// ============================================================================

float MineMap::vision_obs_weight(float confidence, float distance_m) {
    if (confidence <= 0.0f || distance_m < 0.0f || !std::isfinite(distance_m)) {
        return 0.0f;
    }
    const float ref2 = Config::VISION_FUSION_REF_DISTANCE_M *
                       Config::VISION_FUSION_REF_DISTANCE_M;
    const float conf = std::min(confidence, 100.0f) / 100.0f;
    return conf / (1.0f + (distance_m * distance_m) / ref2);
}

bool MineMap::resolve_votes(
    const Types::VisionMarkerType* types,
    const float* weights,
    uint8_t n,
    Types::MarkerConsensus& out) {

    out = Types::MarkerConsensus();
    if (types == nullptr || weights == nullptr || n == 0) return false;

    // Weighted votes per marker type (UNKNOWN votes dilute but never win).
    float weight_by_type[16] = {};
    uint8_t count_by_type[16] = {};
    float total_weight = 0.0f;

    for (uint8_t i = 0; i < n; ++i) {
        const uint8_t t = static_cast<uint8_t>(types[i]);
        if (t >= 16) continue;
        weight_by_type[t] += weights[i];
        count_by_type[t]++;
        total_weight += weights[i];
    }
    if (total_weight <= 0.0f) return false;

    uint8_t best = static_cast<uint8_t>(Types::VisionMarkerType::UNKNOWN);
    float best_w = 0.0f;
    for (uint8_t t = 1; t < 16; ++t) { // skip UNKNOWN(0) as a winner
        if (weight_by_type[t] > best_w) {
            best_w = weight_by_type[t];
            best = t;
        }
    }
    if (best == 0u || best_w <= 0.0f) {
        out.total_votes = n;
        out.weighted_agreement = 0.0f;
        out.ambiguous = true;
        return true;
    }

    out.winning_type = static_cast<Types::VisionMarkerType>(best);
    out.winning_votes = count_by_type[best];
    out.total_votes = n;
    out.weighted_agreement = best_w / total_weight;

    // Ambiguity policy (item 12): split votes OR too little accumulated
    // evidence both mark the classification untrusted.
    out.ambiguous =
        (out.weighted_agreement < Config::MARKER_CONSENSUS_AGREEMENT_MIN) ||
        (total_weight < Config::MARKER_CONSENSUS_MIN_WEIGHT);
    return true;
}

void MineMap::addVisionObservation(
    const Types::VisionObsPayload& obs,
    uint8_t source_drone_id,
    uint32_t now_ms) {

    if (!std::isfinite(obs.x) || !std::isfinite(obs.y) ||
        obs.confidence < 0.0f || obs.confidence > 100.0f ||
        obs.observer_distance_m < 0.0f) {
        setTelemetryEvent(TE_MINE_REJECTED_INVALID_INPUT);
        return;
    }

    // Position fusion through the standard dedup path with the observer's
    // confidence; distance weighting is applied on top below.
    addDetection(obs.x, obs.y, obs.confidence, obs.marker_type,
                 source_drone_id, now_ms);

    const int idx = findNearestMine(
        obs.x, obs.y,
        (source_drone_id == self_drone_id_)
            ? Config::SAME_DRONE_DEDUP_RADIUS_M
            : Config::CROSS_DRONE_DEDUP_RADIUS_M);
    if (idx < 0) return;

    Types::MineRecord& m = mines_[idx];

    // Distance-weighted position refinement (item 11): closer observers pull
    // the fused position harder than distant ones.
    const float w = vision_obs_weight(obs.confidence, obs.observer_distance_m);
    if (w > 0.0f) {
        const uint8_t slot = (m.obs_n < Types::MINE_OBS_HISTORY)
            ? m.obs_n
            : static_cast<uint8_t>(Types::MINE_OBS_HISTORY - 1);
        m.obs_types[slot] = obs.marker_type;
        m.obs_weights[slot] = w;
        m.obs_pos_x[slot] = obs.x;
        m.obs_pos_y[slot] = obs.y;
        if (m.obs_n < Types::MINE_OBS_HISTORY) m.obs_n++;

        // Pull fused world position toward high-weight observations.
        const float pull = std::min(0.25f, w);
        m.x = m.x * (1.0f - pull) + obs.x * pull;
        m.y = m.y * (1.0f - pull) + obs.y * pull;
    }
}

Types::MarkerConsensus MineMap::getMarkerConsensus(uint16_t mine_id) const {
    Types::MarkerConsensus out;
    for (uint16_t i = 0; i < mine_count_; ++i) {
        if (mines_[i].mine_id != mine_id) continue;
        resolve_votes(mines_[i].obs_types, mines_[i].obs_weights,
                      mines_[i].obs_n, out);
        break;
    }
    return out;
}

bool MineMap::isMarkerAmbiguous(uint16_t mine_id) const {
    for (uint16_t i = 0; i < mine_count_; ++i) {
        if (mines_[i].mine_id != mine_id) continue;
        Types::MarkerConsensus c;
        if (!resolve_votes(mines_[i].obs_types, mines_[i].obs_weights,
                           mines_[i].obs_n, c)) {
            return false; // no data: not ambiguous, just unknown
        }
        return c.ambiguous;
    }
    return false;
}

void MineMap::markObsShared(uint16_t mine_id) {
    for (uint16_t i = 0; i < mine_count_; ++i) {
        if (mines_[i].mine_id == mine_id) {
            mines_[i].obs_shared = true;
            return;
        }
    }
}

bool MineMap::getMineByIndex(uint16_t index, Types::MineRecord& out) const {
    if (index >= mine_count_) return false;
    out = mines_[index];
    return true;
}


// ============================================================================
// CONFIRMATION & DECAY
// ============================================================================

void MineMap::checkConfirmation(Types::MineRecord& mine) {
    if (mine.status == Types::MineStatus::CANDIDATE) {
        if (mine.persistence_count >= Config::MINE_CONFIRM_PERSISTENCE_MIN &&
            mine.confidence >= Config::MINE_CONFIRM_CONFIDENCE_MIN) {
            mine.status = Types::MineStatus::CONFIRMED;
            map_version_++;
            mine.map_version = map_version_;
            setTelemetryEvent(TE_MINE_CONFIRMED);
        }
    }
}

void MineMap::decayStaleCandidates(uint32_t now_ms) {
    if ((now_ms - last_decay_ms_) < Config::CANDIDATE_DECAY_INTERVAL_MS) {
        return;
    }

    last_decay_ms_ = now_ms;
    bool version_changed = false;

    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status == Types::MineStatus::CANDIDATE) {
            if ((now_ms - mines_[i].last_seen_time) > Config::MINE_STALE_TIMEOUT_MS) {
                mines_[i].status = Types::MineStatus::REJECTED;
                mines_[i].rejection_reason = 1; // Stale timeout
                version_changed = true;
                setTelemetryEvent(TE_MINE_REJECTED_STALE);
            } else if ((now_ms - mines_[i].last_seen_time) > Config::CANDIDATE_DECAY_INTERVAL_MS) {
                mines_[i].confidence *= Config::CANDIDATE_CONFIDENCE_DECAY_FACTOR;
                if (mines_[i].confidence < Config::CANDIDATE_MIN_CONFIDENCE_AFTER_DECAY) {
                    mines_[i].status = Types::MineStatus::REJECTED;
                    mines_[i].rejection_reason = 2; // Confidence decayed too low
                    version_changed = true;
                    setTelemetryEvent(TE_MINE_REJECTED_LOW_CONFIDENCE);
                }
            }
        }
    }

    if (version_changed) {
        map_version_++;
        setTelemetryEvent(TE_MINE_MAP_VERSION_CHANGED);
    }

    setTelemetryEvent(TE_MINE_DECAY_RUN);
}


// ============================================================================
// SWARM CLAIM MANAGEMENT
// ============================================================================

bool MineMap::isClaimReady(uint16_t mine_id) const {
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id == mine_id) {
            if (mines_[i].status == Types::MineStatus::REJECTED) {
                return false;
            }
            if (mines_[i].confidence >= Config::CLAIM_READY_CONFIDENCE_MIN &&
                mines_[i].persistence_count >= Config::CLAIM_READY_PERSISTENCE_MIN) {
                return !mines_[i].claimed;
            }
            return false;
        }
    }
    return false;
}

bool MineMap::claimMine(uint16_t mine_id, uint8_t drone_id, uint32_t now_ms) {
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id == mine_id) {
            if (mines_[i].status == Types::MineStatus::REJECTED) {
                return false;
            }
            if (mines_[i].claimed && mines_[i].claim_owner_id != drone_id && now_ms < mines_[i].claim_expiry_ms) {
                return false; // Already claimed by another peer
            }

            mines_[i].claimed = true;
            mines_[i].claim_owner_id = drone_id;
            mines_[i].claim_time_ms = now_ms;
            mines_[i].claim_expiry_ms = now_ms + Config::CLAIM_TIMEOUT_MS;
            setTelemetryEvent(TE_MINE_CLAIM_ACCEPTED);
            return true;
        }
    }
    return false;
}

void MineMap::releaseClaim(uint16_t mine_id) {
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id == mine_id) {
            mines_[i].claimed = false;
            mines_[i].claim_owner_id = 0;
            mines_[i].claim_time_ms = 0;
            mines_[i].claim_expiry_ms = 0;
            setTelemetryEvent(TE_MINE_CLAIM_RELEASED);
            return;
        }
    }
}

bool MineMap::isClaimedByOther(uint16_t mine_id, uint8_t self_drone_id) const {
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id == mine_id) {
            return mines_[i].claimed && (mines_[i].claim_owner_id != self_drone_id);
        }
    }
    return false;
}

void MineMap::updateClaims(uint32_t now_ms) {
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].claimed && now_ms >= mines_[i].claim_expiry_ms) {
            mines_[i].claimed = false;
            mines_[i].claim_owner_id = 0;
            mines_[i].claim_time_ms = 0;
            mines_[i].claim_expiry_ms = 0;
            setTelemetryEvent(TE_MINE_CLAIM_EXPIRED);
        }
    }
}


// ============================================================================
// MAIN UPDATE (50 Hz NON-BLOCKING)
// ============================================================================

void MineMap::update(uint32_t now_ms) {
    decayStaleCandidates(now_ms);
    updateClaims(now_ms);
}


// ============================================================================
// QUERIES & OCCUPANCY GRID EXPORT
// ============================================================================

uint16_t MineMap::getMineCount() const {
    uint16_t count = 0;
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status != Types::MineStatus::REJECTED) {
            count++;
        }
    }
    return count;
}

uint16_t MineMap::getConfirmedCount() const {
    uint16_t count = 0;
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status == Types::MineStatus::CONFIRMED) {
            count++;
        }
    }
    return count;
}

uint16_t MineMap::getCandidateCount() const {
    uint16_t count = 0;
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status == Types::MineStatus::CANDIDATE) {
            count++;
        }
    }
    return count;
}

uint16_t MineMap::getRejectedCount() const {
    uint16_t count = 0;
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status == Types::MineStatus::REJECTED) {
            count++;
        }
    }
    return count;
}

uint16_t MineMap::getConfirmedMines(Types::MineRecord* out, uint16_t max_mines) const {
    if (out == nullptr || max_mines == 0) return 0;
    uint16_t count = 0;
    for (uint16_t i = 0; i < Types::MAX_MINES && count < max_mines; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status == Types::MineStatus::CONFIRMED) {
            out[count++] = mines_[i];
        }
    }
    return count;
}

uint16_t MineMap::getCandidateMines(Types::MineRecord* out, uint16_t max_mines) const {
    if (out == nullptr || max_mines == 0) return 0;
    uint16_t count = 0;
    for (uint16_t i = 0; i < Types::MAX_MINES && count < max_mines; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status == Types::MineStatus::CANDIDATE) {
            out[count++] = mines_[i];
        }
    }
    return count;
}

bool MineMap::getMineById(uint16_t mine_id, Types::MineRecord& out) const {
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id == mine_id) {
            out = mines_[i];
            return true;
        }
    }
    return false;
}

bool MineMap::isPointNearConfirmedMine(float x, float y, float clearance_radius_m) const {
    float clearance_sq = clearance_radius_m * clearance_radius_m;
    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status == Types::MineStatus::CONFIRMED) {
            float dx = mines_[i].x - x;
            float dy = mines_[i].y - y;
            if ((dx * dx + dy * dy) <= clearance_sq) {
                return true;
            }
        }
    }
    return false;
}

bool MineMap::isMineNearPath(float x, float y, float clearance_radius_m) const {
    return isPointNearConfirmedMine(x, y, clearance_radius_m);
}

bool MineMap::exportOccupancyGrid(
    uint8_t* grid,
    uint16_t cols,
    uint16_t rows,
    float resolution_m,
    float inflate_radius_m
) const {
    if (grid == nullptr || cols == 0 || rows == 0 || resolution_m <= 0.0f) {
        return false;
    }

    std::memset(grid, 0, cols * rows);

    float radius = (inflate_radius_m > 0.0f) ? inflate_radius_m : Config::MINE_CLEARANCE_RADIUS_M;
    float radius_sq = radius * radius;

    for (uint16_t i = 0; i < Types::MAX_MINES; ++i) {
        if (mines_[i].mine_id != 0 && mines_[i].status == Types::MineStatus::CONFIRMED) {
            float mx = mines_[i].x;
            float my = mines_[i].y;

            int min_c = static_cast<int>((mx - radius) / resolution_m);
            int max_c = static_cast<int>((mx + radius) / resolution_m);
            int min_r = static_cast<int>((my - radius) / resolution_m);
            int max_r = static_cast<int>((my + radius) / resolution_m);

            if (min_c < 0) min_c = 0;
            if (max_c >= cols) max_c = cols - 1;
            if (min_r < 0) min_r = 0;
            if (max_r >= rows) max_r = rows - 1;

            for (int r = min_r; r <= max_r; ++r) {
                float cell_y = (static_cast<float>(r) + 0.5f) * resolution_m;
                float dy = cell_y - my;
                for (int c = min_c; c <= max_c; ++c) {
                    float cell_x = (static_cast<float>(c) + 0.5f) * resolution_m;
                    float dx = cell_x - mx;

                    if ((dx * dx + dy * dy) <= radius_sq) {
                        grid[r * cols + c] = 1;
                    }
                }
            }
        }
    }

    return true;
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void mine_map_init() {
    s_global_mine_map.init();
}

void mine_map_update(uint32_t now_ms) {
    s_global_mine_map.update(now_ms);
}

void mine_map_add_candidate(const Types::VisionCandidate& candidate, uint32_t now_ms) {
    s_global_mine_map.addDetectionFromCandidate(candidate, 1, now_ms);
}

uint16_t mine_map_get_confirmed_mines(Types::MineRecord* out_mines, uint16_t max_count) {
    return s_global_mine_map.getConfirmedMines(out_mines, max_count);
}

bool mine_map_is_point_clear(float x, float y, float clearance_m) {
    return !s_global_mine_map.isPointNearConfirmedMine(x, y, clearance_m);
}

MineMap& mine_map_get_instance() {
    return s_global_mine_map;
}

} // namespace RobofestDrone
