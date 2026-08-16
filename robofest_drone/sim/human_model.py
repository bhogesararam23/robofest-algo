"""
Human Model Module for Robofest Gujarat 6.0 SITL Harness.
Simulates the person-at-risk crossing the field following autonomous aerial guidance.
"""

import math
import random
from typing import Dict, List, Any, Optional
try:
    from sim import config
except ImportError:
    import config


class HumanModel:
    def __init__(self, human_id: int = 1, behavior: str = "obedient_human", seed: Optional[int] = None):
        self.human_id = human_id
        self.behavior = behavior
        if seed is not None:
            random.seed(seed)

        # Start in Start Zone (7.5, 0.5)
        self.x = 7.5
        self.y = 0.5
        self.state = "WAITING"  # 'WAITING', 'WALKING', 'STOPPED', 'REACHED_EXIT'
        self.reached_exit = False
        self.start_crossing_time_s: Optional[float] = None
        self.crossing_time_s = 0.0
        self.max_deviation_m = 0.0
        self.off_path_count = 0
        self.target_wp_idx = 0

        # Behavior tuning
        if behavior == "slow_human":
            self.speed_mps = 0.40
            self.deviation_tendency = 0.05
        elif behavior == "fast_human":
            self.speed_mps = 1.20
            self.deviation_tendency = 0.15
        elif behavior == "deviating_human":
            self.speed_mps = 0.75
            self.deviation_tendency = 0.85
        elif behavior == "nervous_human":
            self.speed_mps = 0.50
            self.deviation_tendency = 0.20
        else:  # 'obedient_human'
            self.speed_mps = config.HUMAN_NOMINAL_SPEED_MPS
            self.deviation_tendency = 0.02

    def update(self, active_path: List[Dict[str, float]], guidance_cmd: str, timestamp_s: float, dt_s: float):
        """
        Updates human movement along the safe path.
        """
        if self.reached_exit or not active_path:
            return

        if self.start_crossing_time_s is None and guidance_cmd in ["FORWARD", "SAFE_PATH"]:
            self.start_crossing_time_s = timestamp_s
            self.state = "WALKING"

        if guidance_cmd == "STOP":
            self.state = "STOPPED"
            return
        elif guidance_cmd in ["FORWARD", "SAFE_PATH"]:
            self.state = "WALKING"

        if self.state != "WALKING":
            return

        # Target next waypoint along path
        if self.target_wp_idx < len(active_path):
            target_wp = active_path[self.target_wp_idx]
            dx = target_wp["x"] - self.x
            dy = target_wp["y"] - self.y
            dist = math.hypot(dx, dy)

            if dist < 0.4:
                self.target_wp_idx += 1
            else:
                # Add slight walking noise / deviation
                noise_x = random.gauss(0, self.deviation_tendency * 0.05)
                move_step = self.speed_mps * dt_s

                self.x += (dx / dist) * move_step + noise_x
                self.y += (dy / dist) * move_step

                # Calculate deviation from straight segment to waypoint
                dev = abs(dx)
                if dev > self.max_deviation_m:
                    self.max_deviation_m = dev
                if dev > 0.80:
                    self.off_path_count += 1

        # Check Exit Zone arrival (Y >= 59.0 m)
        if self.y >= config.EXIT_ZONE_Y_MIN:
            self.y = min(config.FIELD_LENGTH_M - 0.2, self.y)
            self.state = "REACHED_EXIT"
            self.reached_exit = True
            if self.start_crossing_time_s is not None:
                self.crossing_time_s = timestamp_s - self.start_crossing_time_s

    def get_state(self) -> Dict[str, Any]:
        return {
            "human_id": self.human_id,
            "behavior": self.behavior,
            "state": self.state,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "speed_mps": round(self.speed_mps, 2),
            "reached_exit": self.reached_exit,
            "crossing_time_s": round(self.crossing_time_s, 2),
            "max_deviation_m": round(self.max_deviation_m, 3),
            "off_path_count": self.off_path_count
        }
