"""Deterministic mine-map fusion and decay, independent of ROS for testing."""
from dataclasses import dataclass
from math import cos, sin, hypot
from typing import Iterable, List, Optional, Tuple

@dataclass
class Detection:
    id: int = 0
    x: float = 0.0
    y: float = 0.0
    confidence: float = 0.0
    type: str = "surface"
    last_seen: float = 0.0

class MineMapStore:
    def __init__(self, dedup_radius: float = 0.5, decay_after: float = 5.0,
                 decay_amount: float = 0.1, remove_below: float = 0.3):
        self.dedup_radius = dedup_radius
        self.decay_after = decay_after
        self.decay_amount = decay_amount
        self.remove_below = remove_below
        self.mines: List[Detection] = []
        self.next_id = 1
        self.version = 1

    @staticmethod
    def transform_relative(x: float, y: float, pose_x: float, pose_y: float, yaw: float) -> Tuple[float, float]:
        return pose_x + cos(yaw) * x - sin(yaw) * y, pose_y + sin(yaw) * x + cos(yaw) * y

    def add_global(self, detection: Detection, now: float) -> Detection:
        nearest: Optional[Detection] = None
        nearest_distance = self.dedup_radius
        for existing in self.mines:
            distance = hypot(existing.x - detection.x, existing.y - detection.y)
            if distance < nearest_distance:
                nearest, nearest_distance = existing, distance
        confidence = max(0.0, min(1.0, float(detection.confidence)))
        if nearest is not None:
            old_weight = max(0.01, nearest.confidence)
            total = old_weight + max(0.01, confidence)
            nearest.x = (nearest.x * old_weight + detection.x * confidence) / total
            nearest.y = (nearest.y * old_weight + detection.y * confidence) / total
            nearest.confidence = min(1.0, nearest.confidence + 0.1 * confidence)
            nearest.type = detection.type or nearest.type
            nearest.last_seen = now
            self.version += 1
            return nearest
        added = Detection(self.next_id, detection.x, detection.y, confidence, detection.type or "surface", now)
        self.next_id = 1 if self.next_id >= 255 else self.next_id + 1
        self.mines.append(added)
        self.version += 1
        return added

    def add_relative(self, detection: Detection, pose: Tuple[float, float, float], now: float) -> Detection:
        x, y = self.transform_relative(detection.x, detection.y, *pose)
        return self.add_global(Detection(detection.id, x, y, detection.confidence, detection.type), now)

    def merge(self, detections: Iterable[Detection], now: float) -> None:
        for detection in detections:
            self.add_global(detection, now)

    def decay(self, now: float) -> None:
        kept = []
        changed = False
        for mine in self.mines:
            if now - mine.last_seen > self.decay_after:
                mine.confidence -= self.decay_amount
                changed = True
            if mine.confidence >= self.remove_below:
                kept.append(mine)
            else:
                changed = True
        if changed:
            self.mines = kept
            self.version += 1
