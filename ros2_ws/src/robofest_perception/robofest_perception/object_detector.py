"""Deterministic two-label detector for the drone's downward camera.

The detector intentionally exposes only two labels: ``mine`` for configured
mine-marker colours with a plausible compact shape, and ``object`` for any
other sufficiently large foreground region. Ordinary objects are not named.
"""

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class DetectorConfig:
    """Image-space thresholds; values use OpenCV's 8-bit HSV scale."""

    mine_hsv_ranges: Tuple[Tuple[int, int, int, int, int, int], ...] = (
        (0, 100, 100, 10, 255, 255),
        (170, 100, 100, 180, 255, 255),
        (20, 80, 80, 40, 255, 255),
    )
    min_mine_area_px: float = 50.0
    max_mine_area_px: float = 2500.0
    min_object_area_px: float = 120.0
    min_mine_circularity: float = 0.35
    object_contrast_threshold: float = 18.0
    background_border_px: int = 8
    morphology_kernel_px: int = 3


@dataclass(frozen=True)
class Detection:
    """One image-space detection, before world-coordinate projection."""

    label: str
    confidence: float
    pixel_x: float
    pixel_y: float
    width_px: float
    height_px: float
    area_px: float


def _clip01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _valid_hsv_ranges(
    ranges: Iterable[Sequence[int]],
) -> Tuple[Tuple[int, int, int, int, int, int], ...]:
    result = []
    for values in ranges:
        if len(values) != 6:
            raise ValueError("each HSV range must contain six values")
        h0, s0, v0, h1, s1, v1 = (int(value) for value in values)
        if not (0 <= h0 <= 180 and 0 <= h1 <= 180):
            raise ValueError("HSV hue values must be between 0 and 180")
        if not all(0 <= value <= 255 for value in (s0, v0, s1, v1)):
            raise ValueError("HSV saturation/value values must be between 0 and 255")
        result.append((h0, s0, v0, h1, s1, v1))
    if not result:
        raise ValueError("at least one mine HSV range is required")
    return tuple(result)


class ObjectDetector:
    """Detect mine markers and unclassified foreground objects."""

    def __init__(self, config: DetectorConfig = DetectorConfig()):
        self.config = config
        _valid_hsv_ranges(config.mine_hsv_ranges)

    def detect(self, image) -> List[Detection]:
        """Return image-space detections for a BGR OpenCV image."""
        import cv2
        import numpy as np

        if image is None or not hasattr(image, "shape") or len(image.shape) != 3:
            raise ValueError("image must be a BGR image with three dimensions")
        if image.shape[2] != 3 or image.shape[0] < 3 or image.shape[1] < 3:
            raise ValueError("image must have at least 3x3 BGR pixels")

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mine_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for h0, s0, v0, h1, s1, v1 in self.config.mine_hsv_ranges:
            lower = np.array((h0, s0, v0), dtype=np.uint8)
            upper = np.array((h1, s1, v1), dtype=np.uint8)
            mine_mask = cv2.bitwise_or(mine_mask, cv2.inRange(hsv, lower, upper))

        kernel_size = max(1, int(self.config.morphology_kernel_px))
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mine_mask = cv2.morphologyEx(mine_mask, cv2.MORPH_OPEN, kernel)
        mine_mask = cv2.morphologyEx(mine_mask, cv2.MORPH_CLOSE, kernel)

        detections = self._contour_detections(
            cv2, mine_mask, self.config.min_mine_area_px,
            self.config.max_mine_area_px, self.config.min_mine_circularity,
            "mine", image.shape[:2],
        )
        foreground = self._foreground_mask(cv2, np, image, mine_mask, kernel)
        detections.extend(self._contour_detections(
            cv2, foreground, self.config.min_object_area_px, float("inf"),
            0.0, "object", image.shape[:2],
        ))
        return sorted(detections, key=lambda item: (item.pixel_y, item.pixel_x))

    def _foreground_mask(self, cv2, np, image, mine_mask, kernel):
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB).astype(np.int16)
        height, width = lab.shape[:2]
        border = max(1, min(int(self.config.background_border_px), height // 2, width // 2))
        samples = np.concatenate((
            lab[:border, :, :].reshape(-1, 3),
            lab[-border:, :, :].reshape(-1, 3),
            lab[:, :border, :].reshape(-1, 3),
            lab[:, -border:, :].reshape(-1, 3),
        ), axis=0)
        background = np.median(samples, axis=0)
        distance = np.linalg.norm(lab - background, axis=2)
        foreground = (distance >= float(self.config.object_contrast_threshold)).astype(np.uint8) * 255
        foreground[mine_mask > 0] = 0
        foreground = cv2.morphologyEx(foreground, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(foreground, cv2.MORPH_CLOSE, kernel)

    @staticmethod
    def _contour_detections(cv2, mask, min_area, max_area, min_circularity,
                            label, shape) -> List[Detection]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result = []
        height, width = shape
        image_area = float(height * width)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < float(min_area) or area > float(max_area):
                continue
            perimeter = float(cv2.arcLength(contour, True))
            circularity = (4.0 * 3.141592653589793 * area / (perimeter * perimeter)) if perimeter else 0.0
            if circularity < float(min_circularity):
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0.0:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            pixel_x = float(moments["m10"] / moments["m00"])
            pixel_y = float(moments["m01"] / moments["m00"])
            if label == "mine":
                confidence = _clip01(0.55 + 0.35 * min(1.0, circularity) + 0.10 * min(1.0, area / 400.0))
            else:
                confidence = _clip01(0.45 + 0.55 * min(1.0, area / max(1.0, image_area * 0.08)))
            result.append(Detection(label, confidence, pixel_x, pixel_y,
                                    float(box_width), float(box_height), area))
        return result
