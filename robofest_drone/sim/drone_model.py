"""
Simulated Autonomous Drone Model for Robofest Gujarat 6.0 SITL Harness.
Encapsulates onboard state estimation, local mine mapping, path planning, and flight logic.
"""

import math
from typing import Dict, List, Any, Optional
try:
    from sim import config
except ImportError:
    import config


class SimulatedDrone:
    def __init__(self, drone_id: int, role: str, start_x: float, start_y: float):
        self.drone_id = drone_id
        self.role = role  # 'SCOUT_LEFT', 'SCOUT_RIGHT', 'GUIDE_MARKER', 'RESERVE'
        self.state = "INIT"

        # Physical Ground Truth (Maintained by simulator physics)
        self.true_x = start_x
        self.true_y = start_y
        self.true_yaw = 0.0
        self.true_alt_m = 0.0
        self.true_vx = 0.0
        self.true_vy = 0.0
        self.battery_percent = 100.0

        # Onboard Autonomous State Estimate (Dead-reckoning)
        self.est_x = start_x
        self.est_y = start_y
        self.est_alt_m = 0.0
        self.drift_uncertainty_m = 0.0

        # Local Mine Map (Candidates & Confirmed)
        self.local_mines: Dict[int, Dict[str, Any]] = {}
        self.next_mine_id = 1
        self.confirmed_mine_count = 0
        self.claimed_mine_hashes: List[int] = []

        # Local Safe Path
        self.active_path: List[Dict[str, float]] = []
        self.active_path_version = 0

        # Subsystem health flags
        self.camera_healthy = True
        self.flow_healthy = True
        self.radio_healthy = True

        # Metrics trackers
        self.collision_count = 0
        self.unsafe_proximity_count = 0
        self.duplicate_claim_count = 0

        # Search sweep state
        self.search_direction = 1  # +1 = forward (+Y), -1 = return
        if "LEFT" in role:
            self.lane_x_min = 1.0
            self.lane_x_max = 7.0
            self.nominal_x = 4.0
        elif "RIGHT" in role:
            self.lane_x_min = 8.0
            self.lane_x_max = 14.0
            self.nominal_x = 11.0
        else:
            self.lane_x_min = 7.0
            self.lane_x_max = 8.0
            self.nominal_x = 7.5
        self.target_wp_idx = 0

    def receive_sensor_observation(self,
                                   flow_vx: float,
                                   flow_vy: float,
                                   tof_alt_m: float,
                                   quality: float,
                                   flow_valid: bool,
                                   detections: List[Dict[str, Any]],
                                   dt_s: float):
        """
        Updates onboard dead-reckoning and fuses visual detections.
        """
        if self.state in ["DISARMED", "INIT"]:
            return

        # 1. Dead-reckoning integration
        self.est_alt_m = tof_alt_m
        if flow_valid and quality > 0.3:
            self.est_x += flow_vx * dt_s
            self.est_y += flow_vy * dt_s
            self.drift_uncertainty_m += config.DRIFT_RANDOM_WALK_STD_M * dt_s
        else:
            self.drift_uncertainty_m += 0.05 * dt_s

        # 2. Ingest visual detections into local map
        for det in detections:
            self._fuse_detection(det)

    def _fuse_detection(self, det: Dict[str, Any]):
        obs_x = det["observed_x"]
        obs_y = det["observed_y"]
        conf = det["confidence"]
        marker_type = det["marker_type"]

        # Deduplication check
        match_id = None
        min_dist = 0.35  # Same drone dedup radius
        for mid, m in self.local_mines.items():
            d = math.hypot(m["x"] - obs_x, m["y"] - obs_y)
            if d < min_dist:
                match_id = mid
                break

        if match_id is not None:
            m = self.local_mines[match_id]
            m["persistence_count"] += 1
            m["confidence"] = min(100.0, m["confidence"] * 0.7 + conf * 0.3)
            if m["persistence_count"] >= 3 and m["confidence"] >= 70.0:
                if m["status"] != "CONFIRMED":
                    m["status"] = "CONFIRMED"
                    self.confirmed_mine_count += 1
        else:
            mid = self.next_mine_id
            self.next_mine_id += 1
            self.local_mines[mid] = {
                "mine_id": mid,
                "x": obs_x,
                "y": obs_y,
                "confidence": conf,
                "persistence_count": 1,
                "status": "CANDIDATE",
                "marker_type": marker_type
            }

    def plan_safe_path(self) -> bool:
        """
        Plans a 1.0 m mine-clearance route from Start (7.5, 0.5) to Exit (7.5, 59.5).
        """
        confirmed = [m for m in self.local_mines.values() if m["status"] == "CONFIRMED"]

        # 1.0m mine clearance corridor generation
        wps = [{"x": 7.5, "y": 0.5}]
        for y_step in range(4, 58, 3):
            target_y = float(y_step)
            # Evaluate across multiple candidate X coordinates
            best_x = 7.5
            max_clearance = -1.0
            for candidate_x in [7.5, 6.0, 9.0, 4.5, 10.5, 3.0, 12.0, 2.0, 13.0]:
                min_c = 100.0
                for cm in confirmed:
                    dist = math.hypot(candidate_x - cm["x"], target_y - cm["y"])
                    if dist < min_c:
                        min_c = dist
                if min_c >= (config.MINE_CLEARANCE_RADIUS_M + 0.3) and min_c > max_clearance:
                    max_clearance = min_c
                    best_x = candidate_x

            wps.append({"x": round(best_x, 2), "y": target_y})

        wps.append({"x": 7.5, "y": 59.5})
        self.active_path = wps
        self.active_path_version += 1
        return True

    def update_physics_and_behavior(self, dt_s: float):
        """
        Updates flight control behavior and moves physical airframe.
        """
        # Battery depletion
        if self.state not in ["INIT", "DISARMED"]:
            self.battery_percent -= (0.015 * dt_s)  # ~100% in 600s

        # Autonomous State Logic
        if self.state == "TAKEOFF":
            if self.true_alt_m < config.MISSION_ALTITUDE_M:
                self.true_alt_m += 0.5 * dt_s
            else:
                self.true_alt_m = config.MISSION_ALTITUDE_M
                self.state = "FORMATION"

        elif self.state == "FORMATION":
            # Reposition to nominal lane position
            target_x = self.nominal_x
            target_y = 2.0 if self.role != "GUIDE_MARKER" else 1.0
            dx = target_x - self.true_x
            dy = target_y - self.true_y
            dist = math.hypot(dx, dy)
            if dist > 0.2:
                self.true_vx = (dx / dist) * config.CRUISE_SPEED_MPS
                self.true_vy = (dy / dist) * config.CRUISE_SPEED_MPS
            else:
                self.true_vx = 0.0
                self.true_vy = 0.0
                if self.role != "GUIDE_MARKER":
                    self.state = "SEARCHING"
                else:
                    self.state = "WAIT_FOR_PATH"

        elif self.state == "SEARCHING":
            # Lawnmower lane sweep
            self.true_vx = 0.0
            self.true_vy = self.search_direction * config.SEARCH_FORWARD_SPEED_MPS

            if self.true_y >= config.MINEFIELD_Y_MAX - 1.0 and self.search_direction > 0:
                self.search_direction = -1
            elif self.true_y <= config.MINEFIELD_Y_MIN + 2.0 and self.search_direction < 0:
                self.search_direction = 1

        elif self.state == "WAIT_FOR_PATH":
            self.true_vx = 0.0
            self.true_vy = 0.0

        elif self.state == "GUIDING":
            # Follow active path slowly ahead of human
            if self.active_path and self.target_wp_idx < len(self.active_path):
                wp = self.active_path[self.target_wp_idx]
                dx = wp["x"] - self.true_x
                dy = wp["y"] - self.true_y
                dist = math.hypot(dx, dy)
                if dist < 0.5:
                    self.target_wp_idx += 1
                else:
                    self.true_vx = (dx / dist) * 0.40
                    self.true_vy = (dy / dist) * 0.40
            else:
                self.true_vx = 0.0
                self.true_vy = 0.0

        elif self.state == "LANDING":
            self.true_vx = 0.0
            self.true_vy = 0.0
            if self.true_alt_m > 0.05:
                self.true_alt_m -= config.LANDING_SPEED_MPS * dt_s
            else:
                self.true_alt_m = 0.0
                self.state = "DISARMED"

        # Apply motion to true pose
        self.true_x += self.true_vx * dt_s
        self.true_y += self.true_vy * dt_s

        # Keep inside physical walls
        self.true_x = max(0.1, min(config.FIELD_WIDTH_M - 0.1, self.true_x))
        self.true_y = max(0.1, min(config.FIELD_LENGTH_M - 0.1, self.true_y))

    def get_state(self) -> Dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "role": self.role,
            "state": self.state,
            "true_pose": {"x": round(self.true_x, 3), "y": round(self.true_y, 3), "alt_m": round(self.true_alt_m, 2)},
            "est_pose": {"x": round(self.est_x, 3), "y": round(self.est_y, 3), "alt_m": round(self.est_alt_m, 2)},
            "drift_uncertainty_m": round(self.drift_uncertainty_m, 3),
            "battery_percent": round(self.battery_percent, 1),
            "confirmed_mines": self.confirmed_mine_count,
            "path_version": self.active_path_version
        }
