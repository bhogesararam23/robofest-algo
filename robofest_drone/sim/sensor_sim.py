"""
Sensor Simulation Module for Robofest Gujarat 6.0 SITL Harness.
Generates realistic, noisy sensor observations without leaking ground-truth global pose to drone autonomy.
"""

import math
import random
from typing import Dict, List, Any, Optional, Tuple
try:
    from sim import config
except ImportError:
    import config


class SensorSimulator:
    def __init__(self, seed: Optional[int] = None):
        if seed is not None:
            random.seed(seed)

        self.flow_bias_x = 0.0
        self.flow_bias_y = 0.0
        self.in_dropout = False
        self.dropout_timer_s = 0.0
        self.next_detection_id = 1

    def generate_flow_and_tof(self,
                              true_vx: float,
                              true_vy: float,
                              true_alt_m: float,
                              dt_s: float,
                              drift_multiplier: float = 1.0) -> Tuple[float, float, float, float, bool]:
        """
        Simulates optical flow velocity (vx, vy), ToF altitude, quality, and validity.
        """
        # Check low-texture dropout
        if self.in_dropout:
            self.dropout_timer_s -= dt_s
            if self.dropout_timer_s <= 0.0:
                self.in_dropout = False
        else:
            if random.random() < (config.LOW_TEXTURE_PROBABILITY * dt_s * 20.0):
                self.in_dropout = True
                self.dropout_timer_s = config.FLOW_DROPOUT_DURATION_S

        if self.in_dropout or true_alt_m < 0.15:
            # Low quality or dropped out
            flow_vx = 0.0
            flow_vy = 0.0
            quality = 0.10
            flow_valid = False
        else:
            # Accumulate slow bias drift
            self.flow_bias_x += random.gauss(0, config.FLOW_BIAS_DRIFT_MPS * dt_s * drift_multiplier)
            self.flow_bias_y += random.gauss(0, config.FLOW_BIAS_DRIFT_MPS * dt_s * drift_multiplier)

            noise_x = random.gauss(0, config.FLOW_NOISE_STD_MPS * drift_multiplier)
            noise_y = random.gauss(0, config.FLOW_NOISE_STD_MPS * drift_multiplier)

            flow_vx = true_vx + self.flow_bias_x + noise_x
            flow_vy = true_vy + self.flow_bias_y + noise_y
            quality = max(0.4, min(1.0, 0.90 + random.gauss(0, 0.05)))
            flow_valid = True

        # Time-of-Flight altitude
        tof_noise = random.gauss(0, config.TOF_NOISE_STD_M)
        measured_alt_m = max(0.0, true_alt_m + tof_noise)

        return flow_vx, flow_vy, measured_alt_m, quality, flow_valid

    def generate_vision_detections(self,
                                   drone_id: int,
                                   true_drone_x: float,
                                   true_drone_y: float,
                                   true_drone_alt: float,
                                   true_mines: List[Dict[str, Any]],
                                   timestamp_s: float,
                                   tp_multiplier: float = 1.0,
                                   fp_multiplier: float = 1.0) -> List[Dict[str, Any]]:
        """
        Generates noisy vision candidates based on downward camera FOV footprint.
        """
        detections: List[Dict[str, Any]] = []

        # Downward footprint radius at current altitude
        # FOV ~60 deg horizontal -> footprint half-width = alt * tan(30 deg)
        half_w = max(0.5, true_drone_alt * math.tan(math.radians(config.CAMERA_H_FOV_DEG * 0.5)))
        half_l = max(0.4, true_drone_alt * math.tan(math.radians(config.CAMERA_V_FOV_DEG * 0.5)))

        # 1. True Positive Evaluations for Mines in Footprint
        for mine in true_mines:
            dx = abs(mine["x"] - true_drone_x)
            dy = abs(mine["y"] - true_drone_y)

            if dx <= half_w and dy <= half_l:
                # Inside camera view
                is_on_ground = (mine["marker_type"] == "ON_GROUND_MINE")
                base_prob = config.ON_GROUND_TRUE_POSITIVE_RATE if is_on_ground else config.BURIED_TRUE_POSITIVE_RATE
                detection_prob = min(0.99, base_prob * tp_multiplier)

                if random.random() < detection_prob:
                    # Detected! Add measurement noise
                    meas_noise_x = random.gauss(0, 0.08)
                    meas_noise_y = random.gauss(0, 0.08)
                    conf_noise = random.gauss(0, config.CONFIDENCE_NOISE_STD)

                    base_conf = 85.0 if is_on_ground else 75.0
                    conf = max(45.0, min(100.0, base_conf + conf_noise))

                    detections.append({
                        "detection_id": self.next_detection_id,
                        "drone_id": drone_id,
                        "timestamp_s": round(timestamp_s, 3),
                        "observed_x": round(mine["x"] + meas_noise_x, 3),
                        "observed_y": round(mine["y"] + meas_noise_y, 3),
                        "confidence": round(conf, 1),
                        "marker_type": mine["marker_type"],
                        "source_ground_truth_mine_id": mine["mine_id"]
                    })
                    self.next_detection_id += 1

        # 2. False Positive Generation (glare, false ground textures)
        if random.random() < (config.FALSE_POSITIVE_RATE_PER_FRAME * fp_multiplier):
            fp_x = true_drone_x + random.uniform(-half_w, half_w)
            fp_y = true_drone_y + random.uniform(-half_l, half_l)
            fp_conf = random.uniform(45.0, 72.0)
            marker_type = "ON_GROUND_MINE" if random.random() > 0.5 else "BURIED_SURFACE_MARKER"

            detections.append({
                "detection_id": self.next_detection_id,
                "drone_id": drone_id,
                "timestamp_s": round(timestamp_s, 3),
                "observed_x": round(fp_x, 3),
                "observed_y": round(fp_y, 3),
                "confidence": round(fp_conf, 1),
                "marker_type": marker_type,
                "source_ground_truth_mine_id": None  # False positive!
            })
            self.next_detection_id += 1

        return detections

    def generate_human_detection(self,
                                 true_drone_x: float,
                                 true_drone_y: float,
                                 true_human_x: float,
                                 true_human_y: float) -> Tuple[bool, float, float, float]:
        """
        Simulates human detection (e.g. thermal or camera tracking).
        """
        dist = math.hypot(true_human_x - true_drone_x, true_human_y - true_drone_y)
        if dist <= config.HUMAN_DETECTION_RANGE_M:
            if random.random() < config.HUMAN_TRUE_POSITIVE_RATE:
                noise_x = random.gauss(0, 0.10)
                noise_y = random.gauss(0, 0.10)
                conf = max(0.5, min(1.0, 0.90 + random.gauss(0, config.HUMAN_CONFIDENCE_NOISE_STD)))
                return True, true_human_x + noise_x, true_human_y + noise_y, conf

        return False, 0.0, 0.0, 0.0
