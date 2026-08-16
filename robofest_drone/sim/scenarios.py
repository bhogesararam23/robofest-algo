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
    }
}


def get_scenario(name: str) -> Dict[str, Any]:
    """
    Returns scenario preset dictionary by name or fallback to nominal_map.
    """
    return SCENARIO_PRESETS.get(name, SCENARIO_PRESETS["nominal_map"])
