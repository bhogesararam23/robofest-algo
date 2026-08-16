"""
Metrics Calculation Module for Robofest Gujarat 6.0 SITL Simulation.
Calculates activation, mapping precision/recall, path 1.0m clearance violations, and safety scores.
"""

import math
from typing import Dict, List, Any
try:
    from sim import config
except ImportError:
    import config


def point_to_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    """
    Computes exact Euclidean distance from point (px, py) to line segment (ax, ay)-(bx, by).
    """
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    seg_len_sq = abx * abx + aby * aby

    if seg_len_sq <= 1e-6:
        return math.hypot(px - ax, py - ay)

    t = max(0.0, min(1.0, (apx * abx + apy * aby) / seg_len_sq))
    proj_x = ax + t * abx
    proj_y = ay + t * aby
    return math.hypot(px - proj_x, py - proj_y)


def calculate_metrics(run_data: Dict[str, Any], true_mines: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Analyzes simulation run data against ground truth and generates structured metrics.
    """
    events = run_data.get("events", [])
    drones = run_data.get("drones", [])
    human = run_data.get("human", {})
    final_path = run_data.get("final_path", [])

    # 1. Activation & Command Recognition Metrics
    start_accepted = any(e["event_type"] == "start_command_accepted" for e in events)
    takeoff_success = any(e["event_type"] == "takeoff_complete" for e in events)
    takeoff_ev = next((e for e in events if e["event_type"] == "takeoff_complete"), None)
    time_to_takeoff = takeoff_ev["timestamp_s"] if takeoff_ev else None

    # 2. Swarm Coordination Metrics
    formation_ev = next((e for e in events if e["event_type"] == "formation_complete"), None)
    formation_success = (formation_ev is not None)
    formation_time = formation_ev["timestamp_s"] if formation_ev else None

    peer_lost_count = sum(1 for e in events if e["event_type"] == "peer_lost")
    role_failover_count = sum(1 for e in events if e["event_type"] == "role_failover")
    duplicate_suppression_count = sum(1 for e in events if e["event_type"] == "duplicate_mine_suppressed")
    claim_conflicts = sum(1 for e in events if e["event_type"] == "claim_conflict")

    # 3. Mine Mapping Evaluation against Ground Truth
    fused_mines: List[Dict[str, Any]] = run_data.get("fused_mines", [])
    true_mine_count = len(true_mines)

    matched_true_ids = set()
    confirmed_true_count = 0
    confirmed_false_count = 0
    pos_errors = []

    for fm in fused_mines:
        fx = fm["x"]
        fy = fm["y"]
        is_conf = (fm.get("status") == "CONFIRMED")

        # Find closest true mine
        best_true_id = None
        min_dist = 100.0
        for tm in true_mines:
            d = math.hypot(tm["x"] - fx, tm["y"] - fy)
            if d < min_dist:
                min_dist = d
                best_true_id = tm["mine_id"]

        if min_dist <= 0.60 and best_true_id is not None:
            matched_true_ids.add(best_true_id)
            pos_errors.append(min_dist)
            if is_conf:
                confirmed_true_count += 1
        else:
            if is_conf:
                confirmed_false_count += 1

    detected_true_count = len(matched_true_ids)
    false_positive_count = max(0, len(fused_mines) - detected_true_count)

    precision = detected_true_count / max(1, len(fused_mines))
    recall = detected_true_count / max(1, true_mine_count)
    f1 = (2 * precision * recall) / max(1e-5, precision + recall)
    rmse = math.sqrt(sum(e * e for e in pos_errors) / max(1, len(pos_errors))) if pos_errors else 0.0

    # 4. Path 1.0m Clearance Verification against True Mines
    path_violations = 0
    min_clearance_to_true_mines = 100.0

    if final_path and len(final_path) >= 2:
        for i in range(len(final_path) - 1):
            ax = final_path[i]["x"]
            ay = final_path[i]["y"]
            bx = final_path[i + 1]["x"]
            by = final_path[i + 1]["y"]

            for tm in true_mines:
                d = point_to_segment_distance(tm["x"], tm["y"], ax, ay, bx, by)
                if d < min_clearance_to_true_mines:
                    min_clearance_to_true_mines = d
                if d < config.MINE_CLEARANCE_RADIUS_M:
                    path_violations += 1

    path_success = (final_path is not None and len(final_path) >= 2 and path_violations == 0)

    # 5. Human Guidance Metrics
    crossing_success = human.get("reached_exit", False)
    crossing_time_s = human.get("crossing_time_s", 0.0)
    human_off_path_count = human.get("off_path_count", 0)
    human_max_deviation = human.get("max_deviation_m", 0.0)

    # 6. Safety Metrics
    collision_count = sum(1 for e in events if e["event_type"] == "collision_detected")
    unsafe_proximity_count = sum(1 for e in events if e["event_type"] == "unsafe_proximity_detected")
    surface_contact_count = sum(1 for e in events if e["event_type"] == "surface_contact_detected")
    mission_timeout = any(e["event_type"] == "mission_timeout" for e in events)

    return {
        "activation": {
            "start_command_accepted": start_accepted,
            "time_to_takeoff_s": time_to_takeoff,
            "takeoff_success": takeoff_success
        },
        "command_recognition": {
            "commands_issued": 1,
            "commands_accepted": 1 if start_accepted else 0,
            "commands_rejected": 0,
            "false_triggers": 0
        },
        "swarm": {
            "formation_success": formation_success,
            "formation_time_s": formation_time,
            "peer_lost_count": peer_lost_count,
            "role_failover_count": role_failover_count,
            "claim_conflict_count": claim_conflicts,
            "duplicate_suppression_count": duplicate_suppression_count
        },
        "mapping": {
            "true_mine_count": true_mine_count,
            "detected_true_mines": detected_true_count,
            "false_positive_mines": false_positive_count,
            "confirmed_true_mines": confirmed_true_count,
            "confirmed_false_mines": confirmed_false_count,
            "mine_position_rmse_m": round(rmse, 3),
            "precision": round(precision, 3),
            "recall": round(recall, 3),
            "f1_score": round(f1, 3)
        },
        "path": {
            "path_success": path_success,
            "path_clearance_violation_count": path_violations,
            "minimum_clearance_to_true_mines_m": round(min_clearance_to_true_mines, 3),
            "total_waypoints": len(final_path)
        },
        "human_crossing": {
            "crossing_success": crossing_success,
            "crossing_time_s": round(crossing_time_s, 2),
            "off_path_count": human_off_path_count,
            "max_deviation_m": round(human_max_deviation, 3)
        },
        "safety": {
            "collision_count": collision_count,
            "unsafe_proximity_count": unsafe_proximity_count,
            "surface_contact_count": surface_contact_count,
            "mission_timeout_count": 1 if mission_timeout else 0
        }
    }
