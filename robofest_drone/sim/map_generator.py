"""
Field & Minefield Map Generator for Robofest Gujarat 6.0 SITL Simulation.
Generates deterministic synthetic arenas with ground-truth mine and obstacle distributions.
"""

import math
import random
from typing import Dict, List, Any, Optional
try:
    from sim import config
except ImportError:
    import config


class Mine:
    def __init__(self, mine_id: int, x: float, y: float, marker_type: str,
                 buried_depth_mm: float = 0.0, profile: str = "HIGH_CONTRAST"):
        self.mine_id = mine_id
        self.x = x
        self.y = y
        self.marker_type = marker_type  # 'ON_GROUND_MINE' or 'BURIED_SURFACE_MARKER'
        self.buried_depth_mm = buried_depth_mm
        self.visible_surface_marker_profile = profile
        self.ground_truth_confidence = 100.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mine_id": self.mine_id,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "marker_type": self.marker_type,
            "buried_depth_mm": self.buried_depth_mm,
            "visible_surface_marker_profile": self.visible_surface_marker_profile,
            "ground_truth_confidence": self.ground_truth_confidence
        }


class Obstacle:
    def __init__(self, obstacle_id: int, x: float, y: float, radius: float, obs_type: str = "POLE"):
        self.obstacle_id = obstacle_id
        self.x = x
        self.y = y
        self.radius = radius
        self.obs_type = obs_type  # 'POLE' or 'TREE'

    def to_dict(self) -> Dict[str, Any]:
        return {
            "obstacle_id": self.obstacle_id,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "radius": round(self.radius, 3),
            "type": self.obs_type
        }


class MapGenerator:
    def __init__(self, seed: Optional[int] = None):
        if seed is not None:
            random.seed(seed)

    def generate_field(self,
                       mine_count: int = config.MINE_COUNT_ESTIMATE,
                       on_ground_ratio: float = config.ON_GROUND_MINE_RATIO,
                       obstacle_count: int = 0,
                       min_spacing: float = config.MIN_MINE_SPACING_M) -> Dict[str, Any]:
        """
        Generates a complete arena map ensuring safe clearances between spawned mines.
        """
        mines: List[Mine] = []
        obstacles: List[Obstacle] = []

        # Safe margins inside minefield
        x_min = config.FIELD_X_MIN + 0.8
        x_max = config.FIELD_X_MAX - 0.8
        y_min = config.MINEFIELD_Y_MIN + 1.0
        y_max = config.MINEFIELD_Y_MAX - 1.0

        # Generate Obstacles first
        for i in range(obstacle_count):
            ox = random.uniform(x_min + 1.0, x_max - 1.0)
            oy = random.uniform(y_min + 2.0, y_max - 2.0)
            radius = random.uniform(0.15, 0.35)
            obs_type = "TREE" if random.random() > 0.5 else "POLE"
            obstacles.append(Obstacle(i + 1, ox, oy, radius, obs_type))

        # Generate Mines with minimum clearance spacing
        attempts = 0
        max_attempts = mine_count * 200

        while len(mines) < mine_count and attempts < max_attempts:
            attempts += 1
            mx = random.uniform(x_min, x_max)
            my = random.uniform(y_min, y_max)

            # Check distance to existing mines
            too_close = False
            for existing in mines:
                dist = math.hypot(existing.x - mx, existing.y - my)
                if dist < min_spacing:
                    too_close = True
                    break

            if too_close:
                continue

            # Check distance to obstacles
            for obs in obstacles:
                dist = math.hypot(obs.x - mx, obs.y - my)
                if dist < (obs.radius + 0.4):
                    too_close = True
                    break

            if too_close:
                continue

            # Determine mine type
            is_on_ground = (random.random() < on_ground_ratio)
            marker_type = "ON_GROUND_MINE" if is_on_ground else "BURIED_SURFACE_MARKER"
            depth = 0.0 if is_on_ground else random.uniform(30.0, 50.0)
            profile = "HIGH_CONTRAST" if is_on_ground else "RIBBON_SURFACE"

            mine_id = len(mines) + 1
            mines.append(Mine(mine_id, mx, my, marker_type, depth, profile))

        return {
            "field_dimensions": {
                "width_m": config.FIELD_WIDTH_M,
                "length_m": config.FIELD_LENGTH_M,
                "x_range": [config.FIELD_X_MIN, config.FIELD_X_MAX],
                "y_range": [config.FIELD_Y_MIN, config.FIELD_Y_MAX]
            },
            "zones": {
                "start_zone": {"y_min": config.START_ZONE_Y_MIN, "y_max": config.START_ZONE_Y_MAX},
                "minefield_zone": {"y_min": config.MINEFIELD_Y_MIN, "y_max": config.MINEFIELD_Y_MAX},
                "exit_zone": {"y_min": config.EXIT_ZONE_Y_MIN, "y_max": config.EXIT_ZONE_Y_MAX}
            },
            "human_positions": {
                "start_pos": {"x": 7.5, "y": 0.5},
                "target_exit_pos": {"x": 7.5, "y": 59.5}
            },
            "mines": [m.to_dict() for m in mines],
            "obstacles": [o.to_dict() for o in obstacles],
            "total_mines_placed": len(mines),
            "total_obstacles_placed": len(obstacles)
        }
