"""
Preset Scenarios for Robofest Gujarat 6.0 SITL Harness.
Defines varied mission environments, mine densities, sensor noise stress cases, and fault injections.
"""

from typing import Dict, Any

SCENARIO_PRESETS: Dict[str, Dict[str, Any]] = {
    "easy_map": {
        "name": "easy_map",
        "mine_count": 20,
        "on_ground_ratio": 0.85,
        "buried_ratio": 0.15,
        "obstacle_count": 0,
        "false_positive_multiplier": 0.5,
        "true_positive_multiplier": 1.1,
        "drift_multiplier": 0.8,
        "radio_loss_multiplier": 0.5,
        "human_behavior": "obedient_human",
        "peer_failure_enabled": False,
        "description": "Low mine density (20 mines), high contrast markers, minimal drift noise."
    },
    "nominal_map": {
        "name": "nominal_map",
        "mine_count": 40,
        "on_ground_ratio": 0.75,
        "buried_ratio": 0.25,
        "obstacle_count": 0,
        "false_positive_multiplier": 1.0,
        "true_positive_multiplier": 1.0,
        "drift_multiplier": 1.0,
        "radio_loss_multiplier": 1.0,
        "human_behavior": "obedient_human",
        "peer_failure_enabled": False,
        "description": "Standard Robofest competition baseline: 40 mines, 75% on-ground, nominal sensor noise."
    },
    "dense_map": {
        "name": "dense_map",
        "mine_count": 60,
        "on_ground_ratio": 0.75,
        "buried_ratio": 0.25,
        "obstacle_count": 0,
        "false_positive_multiplier": 1.0,
        "true_positive_multiplier": 1.0,
        "drift_multiplier": 1.0,
        "radio_loss_multiplier": 1.0,
        "human_behavior": "nervous_human",
        "peer_failure_enabled": False,
        "description": "High mine density (60 mines) testing narrow corridor A* planning and 1.0m clearance."
    },
    "obstacle_map": {
        "name": "obstacle_map",
        "mine_count": 40,
        "on_ground_ratio": 0.75,
        "buried_ratio": 0.25,
        "obstacle_count": 4,
        "false_positive_multiplier": 1.0,
        "true_positive_multiplier": 1.0,
        "drift_multiplier": 1.0,
        "radio_loss_multiplier": 1.0,
        "human_behavior": "obedient_human",
        "peer_failure_enabled": False,
        "description": "Arena with 4 stationary obstacles (poles/trees) requiring multi-hazard avoidance."
    },
    "high_false_positive_map": {
        "name": "high_false_positive_map",
        "mine_count": 40,
        "on_ground_ratio": 0.75,
        "buried_ratio": 0.25,
        "obstacle_count": 0,
        "false_positive_multiplier": 2.5,
        "true_positive_multiplier": 1.0,
        "drift_multiplier": 1.0,
        "radio_loss_multiplier": 1.0,
        "human_behavior": "obedient_human",
        "peer_failure_enabled": False,
        "description": "High surface glare and ground texture artifacts testing deduplication and persistence filtering."
    },
    "low_visibility_map": {
        "name": "low_visibility_map",
        "mine_count": 40,
        "on_ground_ratio": 0.50,
        "buried_ratio": 0.50,
        "obstacle_count": 0,
        "false_positive_multiplier": 1.0,
        "true_positive_multiplier": 0.75,
        "drift_multiplier": 1.0,
        "radio_loss_multiplier": 1.0,
        "human_behavior": "obedient_human",
        "peer_failure_enabled": False,
        "description": "Challenging lighting with 50% buried mines testing surface ribbon detection limits."
    },
    "drift_stress_map": {
        "name": "drift_stress_map",
        "mine_count": 40,
        "on_ground_ratio": 0.75,
        "buried_ratio": 0.25,
        "obstacle_count": 0,
        "false_positive_multiplier": 1.0,
        "true_positive_multiplier": 1.0,
        "drift_multiplier": 2.5,
        "radio_loss_multiplier": 1.0,
        "human_behavior": "obedient_human",
        "peer_failure_enabled": False,
        "description": "High optical flow dead-reckoning random walk and bias drift testing geofence margin scaling."
    },
    "peer_failure_map": {
        "name": "peer_failure_map",
        "mine_count": 40,
        "on_ground_ratio": 0.75,
        "buried_ratio": 0.25,
        "obstacle_count": 0,
        "false_positive_multiplier": 1.0,
        "true_positive_multiplier": 1.0,
        "drift_multiplier": 1.0,
        "radio_loss_multiplier": 2.0,
        "human_behavior": "obedient_human",
        "peer_failure_enabled": True,
        "description": "Simulates complete loss of Drone 2 mid-flight, validating autonomous lane expansion & role failover."
    },
    "human_deviation_map": {
        "name": "human_deviation_map",
        "mine_count": 40,
        "on_ground_ratio": 0.75,
        "buried_ratio": 0.25,
        "obstacle_count": 0,
        "false_positive_multiplier": 1.0,
        "true_positive_multiplier": 1.0,
        "drift_multiplier": 1.0,
        "radio_loss_multiplier": 1.0,
        "human_behavior": "deviating_human",
        "peer_failure_enabled": False,
        "description": "Person-at-risk deviates off safe corridor, testing visual guidance corrections and dynamic re-routing."
    },

    # --- Phase 6 additions (items 11-15 integration coverage) ---

    "night_low_light_map": {
        "name": "night_low_light_map",
        "mine_count": 40,
        "on_ground_ratio": 0.75,
        "buried_ratio": 0.25,
        "obstacle_count": 2,
        "false_positive_multiplier": 1.8,
        "true_positive_multiplier": 0.6,
        "drift_multiplier": 1.3,
        "radio_loss_multiplier": 1.0,
        "human_behavior": "slow_human",
        "peer_failure_enabled": False,
        "description": "Night/low-light proxy: depressed true-positive rate, elevated false positives and drift; exercises the firmware night-mode relaxation thresholds (item 15)."
    },
    "swarm_fusion_stress_map": {
        "name": "swarm_fusion_stress_map",
        "mine_count": 55,
        "on_ground_ratio": 0.75,
        "buried_ratio": 0.25,
        "obstacle_count": 0,
        "false_positive_multiplier": 2.2,
        "true_positive_multiplier": 0.9,
        "drift_multiplier": 1.6,
        "radio_loss_multiplier": 2.5,
        "human_behavior": "nervous_human",
        "peer_failure_enabled": True,
        "description": "Cross-drone fusion stress: noisy asymmetric detections plus a mid-flight peer loss, validating VISION_OBS distance-weighted fusion and consensus voting under degraded comms (items 11/12)."
    },
    "dynamic_obstacle_map": {
        "name": "dynamic_obstacle_map",
        "mine_count": 40,
        "on_ground_ratio": 0.75,
        "buried_ratio": 0.25,
        "obstacle_count": 8,
        "false_positive_multiplier": 1.0,
        "true_positive_multiplier": 1.0,
        "drift_multiplier": 1.1,
        "radio_loss_multiplier": 1.0,
        "human_behavior": "fast_human",
        "peer_failure_enabled": False,
        "description": "Dense obstacle field with a fast-moving person-at-risk: exercises vision-based TTC avoidance margins (item 14) and moving-target guidance."
    }
}


def get_scenario(name: str) -> Dict[str, Any]:
    """
    Returns scenario preset dictionary by name or fallback to nominal_map.
    """
    return SCENARIO_PRESETS.get(name, SCENARIO_PRESETS["nominal_map"])
