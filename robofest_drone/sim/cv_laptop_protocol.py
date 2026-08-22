#!/usr/bin/env python3
"""
Robofest Gujarat 6.0 Minefield Swarm Drone
Laptop Webcam Computer Vision (CV) Calibration & Perception Protocol

Interactive multi-color / multi-shape calibration tool mirroring the onboard
VisionPipeline. Calibrate every marker color AND its shape gates here under
real lighting conditions, then press [E] to export a firmware-ready
vision_profiles_generated.h that overrides the built-in profile table.

HSV SCALE NOTE: This tool and the onboard firmware both use OpenCV-compatible
8-bit ranges - H 0-180, S 0-255, V 0-255. The firmware's inline RGB->HSV
conversion produces the same 0-180 hue scale, so NO conversion is needed.
"""

import sys
import os
import json
import time
import math
import copy
import argparse
import datetime

import cv2
import numpy as np

CONFIG_FILENAME = "cv_hsv_config.json"
GENERATED_HEADER_RELPATH = os.path.join("..", "config", "vision_profiles_generated.h")

# Mirror of Types::VisionMarkerType in src/types.h (values are wire format).
MARKER_TYPE_IDS = {
    "ON_GROUND_MINE": 1,
    "BURIED_SURFACE_MARKER": 2,
    "MARKER_ORANGE": 3,
    "MARKER_GREEN": 4,
    "MARKER_CYAN": 5,
    "MARKER_BLUE": 6,
    "MARKER_PURPLE": 7,
    "MARKER_PINK": 8,
    "MARKER_WHITE": 9,
    "MARKER_BLACK": 10,
}

# Mirror of Config::VISION_SHAPE_* gate presets in config/vision_profiles.h.
SHAPE_PRESETS = {
    "ANY":      dict(aspect_min=1.00, aspect_max=10.00, extent_min=0.00, extent_max=1.05, solidity_min=0.00, corners_min=0, corners_max=255),
    "CIRCLE":   dict(aspect_min=1.00, aspect_max=1.35,  extent_min=0.62, extent_max=1.05, solidity_min=0.88, corners_min=0, corners_max=255),
    "SQUARE":   dict(aspect_min=1.00, aspect_max=1.25,  extent_min=0.78, extent_max=1.05, solidity_min=0.92, corners_min=3, corners_max=5),
    "TRIANGLE": dict(aspect_min=1.00, aspect_max=2.20,  extent_min=0.40, extent_max=0.90, solidity_min=0.88, corners_min=2, corners_max=4),
    "PENTAGON": dict(aspect_min=1.00, aspect_max=1.50,  extent_min=0.55, extent_max=1.00, solidity_min=0.90, corners_min=4, corners_max=6),
    "STAR":     dict(aspect_min=1.00, aspect_max=1.60,  extent_min=0.25, extent_max=0.65, solidity_min=0.45, corners_min=7, corners_max=12),
}
SHAPE_PRESET_ORDER = ["ANY", "CIRCLE", "SQUARE", "TRIANGLE", "PENTAGON", "STAR"]

# Mirror of firmware confidence blend weights (thresholds.h).
CONF_WEIGHT_CIRCULARITY = 24.0
CONF_WEIGHT_SHAPE_MATCH = 36.0
CONF_WEIGHT_AREA = 40.0
CONF_CIRC_PERFECT_AT = 0.85
CONF_GATE_SOFT_MARGIN_RATIO = 0.25

# Mirror of exposure/lighting constants (thresholds.h).
EXPOSURE_TARGET_MEAN_V = 135.0
EXPOSURE_GAIN_MIN = 0.70
EXPOSURE_GAIN_MAX = 1.50
GRAY_WORLD_STRENGTH = 0.5
LIGHTING_V_SUNNY_MIN = 115

# Firmware full-frame resolution used for area scaling on export.
FW_FRAME_W, FW_FRAME_H = 320, 240


def _band(h_low, h_high, s_low, s_high, v_low, v_high):
    return {"h_low": h_low, "h_high": h_high, "s_low": s_low, "s_high": s_high,
            "v_low": v_low, "v_high": v_high}


def make_default_profiles():
    """Factory defaults mirroring the built-in table in vision_profiles.h."""
    defs = [
        ("ON_GROUND_MINE",         [170, 10, 100, 255, 100, 255], True,  [165, 15, 70, 255, 60, 190],  0.70, 0.0, "CIRCLE"),
        ("BURIED_SURFACE_MARKER",  [20, 42, 110, 255, 110, 255],  True,  [18, 45, 85, 255, 75, 210],   0.70, 5.0, "CIRCLE"),
        ("MARKER_ORANGE",          [9, 24, 130, 255, 120, 255],   True,  [8, 26, 95, 255, 80, 215],    0.55, 0.0, "ANY"),
        ("MARKER_GREEN",           [35, 86, 80, 255, 80, 255],    True,  [33, 88, 60, 255, 55, 220],   0.55, 0.0, "ANY"),
        ("MARKER_CYAN",            [87, 99, 90, 255, 90, 255],    True,  [85, 101, 65, 255, 65, 225],  0.55, 0.0, "ANY"),
        ("MARKER_BLUE",            [100, 131, 85, 255, 70, 255],  True,  [98, 133, 60, 255, 50, 230],  0.55, 0.0, "ANY"),
        ("MARKER_PURPLE",          [132, 159, 70, 255, 60, 255],  True,  [130, 161, 50, 255, 45, 235], 0.55, 0.0, "ANY"),
        ("MARKER_PINK",            [160, 175, 60, 255, 110, 255], True,  [158, 177, 40, 255, 80, 235], 0.55, 0.0, "ANY"),
        ("MARKER_WHITE",           [0, 180, 0, 60, 150, 255],     True,  [0, 180, 0, 80, 120, 235],    0.55, 0.0, "ANY"),
        ("MARKER_BLACK",           [0, 180, 0, 140, 0, 55],       True,  [0, 180, 0, 150, 0, 70],      0.55, 0.0, "ANY"),
    ]
    # Areas are expressed at webcam resolution; export scales them down to the
    # firmware 320x240 full-frame pixel domain.
    out = []
    for name, prim, has_alt, alt, circ, bias, preset in defs:
        out.append({
            "name": name,
            "type_id": MARKER_TYPE_IDS[name],
            "enabled": True,
            "primary": _band(*prim),
            "has_alt": has_alt,
            "alt": _band(*alt),
            "min_area": 100.0,
            "max_area": 10000.0,
            "circularity_min": circ,
            "confidence_bias": bias,
            "expected_area": 1600.0,
            "shape_preset": preset,
            "gates": dict(SHAPE_PRESETS[preset]),
        })
    return out


# ============================================================================
# FIRMWARE-MIRRORED SCORING MATH (keep in sync with vision_pipeline.cpp)
# ============================================================================

def range_score(value, lo, hi):
    if hi < lo:
        return 1.0
    if lo <= value <= hi:
        return 1.0
    span = hi - lo
    margin = max(span * CONF_GATE_SOFT_MARGIN_RATIO, 0.05)
    dist = (lo - value) if value < lo else (value - hi)
    return max(0.0, 1.0 - dist / margin)


def floor_score(value, floor_v):
    if value >= floor_v:
        return 1.0
    margin = max(floor_v * CONF_GATE_SOFT_MARGIN_RATIO, 0.05)
    return max(0.0, 1.0 - (floor_v - value) / margin)


def corner_score(corners, cmin, cmax):
    if cmin == 0 or cmax == 0 or cmin > cmax:
        return 1.0
    if cmin <= corners <= cmax:
        return 1.0
    dist = float(cmin - corners) if corners < cmin else float(corners - cmax)
    return max(0.0, 1.0 - dist / 3.0)


def shape_match(aspect, extent, solidity, corners, gates):
    return (0.15 * range_score(aspect, gates["aspect_min"], gates["aspect_max"]) +
            0.35 * range_score(extent, gates["extent_min"], gates["extent_max"]) +
            0.30 * floor_score(solidity, gates["solidity_min"]) +
            0.20 * corner_score(corners, gates["corners_min"], gates["corners_max"]))


def blob_confidence(circularity, area, aspect, extent, solidity, corners, prof):
    circ_term = CONF_WEIGHT_CIRCULARITY * min(1.0, circularity / CONF_CIRC_PERFECT_AT)
    area_term = CONF_WEIGHT_AREA * min(1.0, area / max(prof["expected_area"], 1.0))
    sm = shape_match(aspect, extent, solidity, corners, prof["gates"])
    conf = circ_term + CONF_WEIGHT_SHAPE_MATCH * sm + area_term + prof["confidence_bias"]
    return max(0.0, min(100.0, conf)), sm


def hsv_in_band(h, s, v, band):
    h_lo, h_hi = band["h_low"], band["h_high"]
    h_ok = (h_lo <= h <= h_hi) if h_lo <= h_hi else (h >= h_lo or h <= h_hi)
    return h_ok and band["s_low"] <= s <= band["s_high"] and band["v_low"] <= v <= band["v_high"]


def build_mask(hsv, band):
    h_lo, h_hi = band["h_low"], band["h_high"]
    s_lo, s_hi = band["s_low"], band["s_high"]
    v_lo, v_hi = band["v_low"], band["v_high"]
    if h_lo <= h_hi:
        lower = np.array([h_lo, s_lo, v_lo], dtype=np.uint8)
        upper = np.array([h_hi, s_hi, v_hi], dtype=np.uint8)
        return cv2.inRange(hsv, lower, upper)
    lower1 = np.array([0, s_lo, v_lo], dtype=np.uint8)
    upper1 = np.array([h_hi, s_hi, v_hi], dtype=np.uint8)
    lower2 = np.array([h_lo, s_lo, v_lo], dtype=np.uint8)
    upper2 = np.array([180, s_hi, v_hi], dtype=np.uint8)
    return cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))


PALETTE = [
    (0, 255, 255), (0, 191, 255), (0, 255, 0), (255, 160, 0), (255, 0, 255),
    (255, 0, 0), (128, 0, 255), (203, 192, 255), (255, 255, 255), (80, 80, 80),
]


class CvLaptopProtocol:
    def __init__(self, camera_index=0, width=640, height=480):
        self.camera_index = camera_index
        self.width = width
        self.height = height

        self.profiles = make_default_profiles()
        self.active_idx = 0
        self.edit_alt = False          # tune primary vs alt (overcast) band
        self.multi_color = True        # scan ALL enabled profiles simultaneously
        self.norm_enabled = True       # exposure normalization prototype
        self.lighting_override = "AUTO"  # AUTO | SUNNY | OVERCAST

        self.window_main = "Robofest CV Protocol - Live Feed"
        self.window_mask = "HSV Mask Segment"
        self.window_controls = "Profile Trackbar Controls"

        self.load_config()

    # ------------------------------------------------------------------
    # Config persistence (JSON round-trip)
    # ------------------------------------------------------------------

    def load_config(self):
        if not os.path.exists(CONFIG_FILENAME):
            return
        try:
            with open(CONFIG_FILENAME, "r") as f:
                data = json.load(f)
        except Exception as e:
            print(f"[CV_PROTOCOL][WARN] Failed to load config: {e}")
            return

        loaded = data.get("profiles", [])
        by_name = {p["name"]: p for p in loaded if isinstance(p, dict) and "name" in p}
        merged = []
        for default_p in self.profiles:
            if default_p["name"] in by_name:
                p = copy.deepcopy(default_p)
                p.update(by_name[default_p["name"]])
                if "gates" not in p or not isinstance(p.get("gates"), dict):
                    p["gates"] = dict(SHAPE_PRESETS[p.get("shape_preset", "ANY")])
                merged.append(p)
            else:
                merged.append(default_p)
        self.profiles = merged

        cam = data.get("camera", {})
        self.width = int(cam.get("w", self.width))
        self.height = int(cam.get("h", self.height))

        settings = data.get("settings", {})
        self.norm_enabled = bool(settings.get("norm_enabled", True))
        self.lighting_override = settings.get("lighting_override", "AUTO")
        self.multi_color = bool(settings.get("multi_color", True))
        print(f"[CV_PROTOCOL] Loaded '{CONFIG_FILENAME}' ({len(self.profiles)} profiles).")

    def save_config(self):
        try:
            payload = {
                "version": 2,
                "camera": {"w": self.width, "h": self.height},
                "settings": {
                    "norm_enabled": self.norm_enabled,
                    "lighting_override": self.lighting_override,
                    "multi_color": self.multi_color,
                },
                "profiles": self.profiles,
            }
            with open(CONFIG_FILENAME, "w") as f:
                json.dump(payload, f, indent=2)
            print(f"[CV_PROTOCOL] Saved calibration to '{CONFIG_FILENAME}'.")
            return True
        except Exception as e:
            print(f"[CV_PROTOCOL][ERROR] Save failed: {e}")
            return False

    # ------------------------------------------------------------------
    # Firmware header export ("upload to firmware" path)
    # ------------------------------------------------------------------

    def export_header(self, out_path=None):
        if out_path is None:
            out_path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                     GENERATED_HEADER_RELPATH))
        area_scale = (FW_FRAME_W * FW_FRAME_H) / float(max(self.width * self.height, 1))

        rows = []
        for p in self.profiles:
            g = p["gates"]
            pb, ab = p["primary"], p["alt"]

            def ub(v):
                return f"{int(round(v))}u"

            row = (
                "/* {name} */ "
                "{{{tid}u, {en}, "
                "{{{h0},{h1},{s0},{s1},{v0},{v1}}}, {ha}, "
                "{{{a0},{a1},{a2},{a3},{a4},{a5}}}, "
                "{min_a:.2f}f, {max_a:.2f}f, {circ:.2f}f, {bias:.2f}f, {exp}u, "
                "{{{amn:.2f}f, {amx:.2f}f, {emn:.2f}f, {emx:.2f}f, {sol:.2f}f, {cmn}u, {cmx}u}}"
                "}}"
            ).format(
                tid=p["type_id"],
                en="true" if p["enabled"] else "false",
                h0=ub(pb["h_low"]), h1=ub(pb["h_high"]),
                s0=ub(pb["s_low"]), s1=ub(pb["s_high"]),
                v0=ub(pb["v_low"]), v1=ub(pb["v_high"]),
                ha="true" if p["has_alt"] else "false",
                a0=ub(ab["h_low"]), a1=ub(ab["h_high"]),
                a2=ub(ab["s_low"]), a3=ub(ab["s_high"]),
                a4=ub(ab["v_low"]), a5=ub(ab["v_high"]),
                min_a=max(p["min_area"] * area_scale, 0.0),
                max_a=p["max_area"] * area_scale,
                circ=p["circularity_min"],
                bias=p["confidence_bias"],
                exp=int(round(p["expected_area"] * area_scale)),
                amn=g["aspect_min"], amx=g["aspect_max"],
                emn=g["extent_min"], emx=g["extent_max"],
                sol=g["solidity_min"],
                cmn=int(g["corners_min"]), cmx=int(g["corners_max"]),
                name=p["name"],
            )
            rows.append("    " + row)

        body = ", \\\n".join(rows)

        header = (
            "#pragma once\n"
            "\n"
            "// ============================================================================\n"
            "// AUTO-GENERATED FILE - DO NOT EDIT BY HAND\n"
            f"// Generated by sim/cv_laptop_protocol.py on {datetime.datetime.now().isoformat(timespec='seconds')}\n"
            "// Source calibration: cv_hsv_config.json\n"
            "//\n"
            "// Consumed by config/vision_profiles.h via ROBOFEST_VISION_PROFILE_TABLE_DATA.\n"
            "// Delete this file to fall back to the built-in default profile table.\n"
            "// ============================================================================\n"
            "\n"
            "#define ROBOFEST_VISION_PROFILE_TABLE_DATA { \\\n"
            f"{body} \\\n"
            "}\n"
        )

        try:
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            with open(out_path, "w") as f:
                f.write(header)
            print("=" * 64)
            print(f"[EXPORT] Firmware profile table written to:")
            print(f"         {out_path}")
            print(f"         {len(self.profiles)} profiles | area scale x{area_scale:.4f}")
            print("         Rebuild the PlatformIO project to flash it.")
            print("=" * 64)
            return True
        except Exception as e:
            print(f"[EXPORT][ERROR] Failed writing header: {e}")
            return False

    # ------------------------------------------------------------------
    # Trackbars (active profile + active variant)
    # ------------------------------------------------------------------

    def active_profile(self):
        return self.profiles[self.active_idx]

    def active_band(self):
        p = self.active_profile()
        if self.edit_alt and p["has_alt"]:
            return p["alt"]
        return p["primary"]

    def setup_trackbars(self):
        win = self.window_controls
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 480, 520)
        cb = self._nothing
        cv2.createTrackbar("H Low", win, 0, 180, cb)
        cv2.createTrackbar("H High", win, 0, 180, cb)
        cv2.createTrackbar("S Low", win, 0, 255, cb)
        cv2.createTrackbar("S High", win, 0, 255, cb)
        cv2.createTrackbar("V Low", win, 0, 255, cb)
        cv2.createTrackbar("V High", win, 0, 255, cb)
        cv2.createTrackbar("Min Area x10", win, 0, 300, cb)
        cv2.createTrackbar("Circularity %", win, 0, 100, cb)
        cv2.createTrackbar("Extent Min %", win, 0, 105, cb)
        cv2.createTrackbar("Extent Max %", win, 0, 105, cb)
        cv2.createTrackbar("Solidity Min %", win, 0, 100, cb)
        cv2.createTrackbar("Corners Min", win, 0, 16, cb)
        cv2.createTrackbar("Corners Max", win, 0, 16, cb)
        self.update_trackbar_positions()

    def _nothing(self, _):
        pass

    def update_trackbar_positions(self):
        win = self.window_controls
        b = self.active_band()
        p = self.active_profile()
        g = p["gates"]
        cv2.setTrackbarPos("H Low", win, b["h_low"])
        cv2.setTrackbarPos("H High", win, b["h_high"])
        cv2.setTrackbarPos("S Low", win, b["s_low"])
        cv2.setTrackbarPos("S High", win, b["s_high"])
        cv2.setTrackbarPos("V Low", win, b["v_low"])
        cv2.setTrackbarPos("V High", win, b["v_high"])
        cv2.setTrackbarPos("Min Area x10", win, int(p["min_area"]) // 10)
        cv2.setTrackbarPos("Circularity %", win, int(round(p["circularity_min"] * 100)))
        cv2.setTrackbarPos("Extent Min %", win, int(round(g["extent_min"] * 100)))
        cv2.setTrackbarPos("Extent Max %", win, int(round(g["extent_max"] * 100)))
        cv2.setTrackbarPos("Solidity Min %", win, int(round(g["solidity_min"] * 100)))
        cv2.setTrackbarPos("Corners Min", win, min(int(g["corners_min"]), 16))
        cv2.setTrackbarPos("Corners Max", win, min(int(g["corners_max"]), 16))

    def read_trackbars(self):
        win = self.window_controls
        b = self.active_band()
        p = self.active_profile()
        g = p["gates"]
        b["h_low"] = cv2.getTrackbarPos("H Low", win)
        b["h_high"] = cv2.getTrackbarPos("H High", win)
        b["s_low"] = cv2.getTrackbarPos("S Low", win)
        b["s_high"] = cv2.getTrackbarPos("S High", win)
        b["v_low"] = cv2.getTrackbarPos("V Low", win)
        b["v_high"] = cv2.getTrackbarPos("V High", win)
        p["min_area"] = max(10.0, cv2.getTrackbarPos("Min Area x10", win) * 10.0)
        p["circularity_min"] = cv2.getTrackbarPos("Circularity %", win) / 100.0
        g["extent_min"] = cv2.getTrackbarPos("Extent Min %", win) / 100.0
        g["extent_max"] = max(cv2.getTrackbarPos("Extent Max %", win) / 100.0, g["extent_min"])
        g["solidity_min"] = cv2.getTrackbarPos("Solidity Min %", win) / 100.0
        g["corners_min"] = cv2.getTrackbarPos("Corners Min", win)
        g["corners_max"] = max(cv2.getTrackbarPos("Corners Max", win), g["corners_min"])

    # ------------------------------------------------------------------
    # Exposure normalization prototype (mirrors firmware Step 5 logic)
    # ------------------------------------------------------------------

    def measure_exposure(self, bgr):
        small = bgr[::4, ::4]
        hsv_s = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        mean_v = float(np.mean(hsv_s[:, :, 2]))
        mean_b = float(np.mean(small[:, :, 0]))
        mean_g = float(np.mean(small[:, :, 1]))
        mean_r = float(np.mean(small[:, :, 2]))
        return mean_v, (mean_b, mean_g, mean_r)

    def apply_exposure_norm(self, bgr):
        mean_v, (mb, mg, mr) = self.measure_exposure(bgr)
        if mean_v < 10.0:
            return bgr, 1.0, mean_v
        base = max(EXPOSURE_GAIN_MIN, min(EXPOSURE_GAIN_MAX, EXPOSURE_TARGET_MEAN_V / mean_v))
        avg = (mb + mg + mr) / 3.0

        def gw(m):
            if m < 5.0:
                return base
            g = base * (1.0 + GRAY_WORLD_STRENGTH * (avg / m - 1.0))
            return max(EXPOSURE_GAIN_MIN, min(EXPOSURE_GAIN_MAX, g))

        gains = np.array([gw(mb), gw(mg), gw(mr)], dtype=np.float32)
        out = bgr.astype(np.float32) * gains.reshape(1, 1, 3)
        return np.clip(out, 0, 255).astype(np.uint8), base, mean_v

    # ------------------------------------------------------------------
    # Lighting mode resolution (mirrors firmware Step 6 selection rule)
    # ------------------------------------------------------------------

    def resolve_lighting_mode(self, mean_v):
        if self.lighting_override != "AUTO":
            return self.lighting_override
        return "SUNNY" if mean_v >= LIGHTING_V_SUNNY_MIN else "OVERCAST"

    # ------------------------------------------------------------------
    # Per-profile contour analysis (mirrors firmware descriptors/confidence)
    # ------------------------------------------------------------------

    def analyze_profile(self, mask, prof):
        kernel = np.ones((3, 3), np.uint8)
        clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        clean = cv2.dilate(clean, kernel, iterations=1)

        contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < prof["min_area"] or area > prof["max_area"]:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0:
                continue
            circularity = (4.0 * math.pi * area) / (perimeter * perimeter)
            circularity = min(1.0, circularity)
            if circularity < prof["circularity_min"]:
                continue

            x, y, w_box, h_box = cv2.boundingRect(cnt)
            short_side = max(min(w_box, h_box), 1)
            aspect = max(w_box, h_box) / short_side
            extent = area / float(max(w_box * h_box, 1))

            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = min(1.0, area / hull_area) if hull_area > 1.0 else 1.0

            bbox_diag = math.hypot(w_box, h_box)
            approx = cv2.approxPolyDP(hull, 0.035 * bbox_diag, True)
            corners = len(approx)

            conf, sm = blob_confidence(
                circularity, area, aspect, extent, solidity, corners, prof)

            M = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else x + w_box // 2
            cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else y + h_box // 2

            candidates.append({
                "cnt": cnt, "x": x, "y": y, "w": w_box, "h": h_box,
                "cx": cx, "cy": cy, "area": area,
                "circularity": circularity, "aspect": aspect,
                "extent": extent, "solidity": solidity,
                "corners": corners, "conf": conf, "sm": sm,
            })
        candidates.sort(key=lambda c: c["conf"], reverse=True)
        return clean, candidates

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        print("=" * 64)
        print("  ROBOFEST GUJARAT 6.0 - LAPTOP CV PROTOCOL (MULTI-COLOR/SHAPE)")
        print("=" * 64)
        print(f"[CV_PROTOCOL] Opening camera index {self.camera_index}...")

        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW) \
            if sys.platform.startswith("win") else cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            print("[CV_PROTOCOL][WARN] Camera index failed, trying fallback index 1...")
            cap = cv2.VideoCapture(1)
            if not cap.isOpened():
                print("[CV_PROTOCOL][CRITICAL] No available webcam found!")
                return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.setup_trackbars()
        cv2.namedWindow(self.window_main, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.window_mask, cv2.WINDOW_NORMAL)

        print("-" * 64)
        print("KEYBOARD CONTROLS:")
        print("  [1..9,0] Select profile   [ [/] ] Prev/Next profile   [P] Next profile")
        print("  [A] Toggle profile enabled        [L] Edit ALT (overcast) band")
        print("  [M] Toggle multi-color scan       [N] Toggle exposure normalization")
        print("  [K] Lighting: AUTO/SUNNY/OVERCAST [C] Cycle shape preset (active)")
        print("  [R] Reset active profile          [S] Save JSON")
        print("  [E] EXPORT firmware header        [Q/ESC] Exit")
        print("-" * 64)

        fps_avg = 0.0
        last_mean_v = 0.0
        last_gain = 1.0
        lighting_mode = "SUNNY"

        while True:
            t_start = time.time()
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[CV_PROTOCOL][WARN] Frame grab empty. Retrying...")
                time.sleep(0.03)
                continue

            self.read_trackbars()

            work = frame
            if self.norm_enabled:
                work, last_gain, last_mean_v = self.apply_exposure_norm(frame)
            else:
                _, last_mean_v = self.measure_exposure(frame)
                last_gain = 1.0

            lighting_mode = self.resolve_lighting_mode(last_mean_v)
            hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)

            use_alt_globally = (lighting_mode == "OVERCAST")

            # Decide which profiles participate
            active_list = []
            if self.multi_color:
                active_list = [(i, p) for i, p in enumerate(self.profiles) if p["enabled"]]
            else:
                active_list = [(self.active_idx, self.active_profile())]

            label_overlay = frame.copy()
            union_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            total_valid = 0

            for slot, (pi, prof) in enumerate(active_list):
                band_key = None
                if prof["has_alt"]:
                    band_key = "alt" if (use_alt_globally or (not self.multi_color and self.edit_alt)) else "primary"
                else:
                    band_key = "primary"
                band = prof[band_key]

                mask = build_mask(hsv, band)
                color_val = min(slot + 1, 255)
                union_mask[mask > 0] = color_val

                clean, candidates = self.analyze_profile(mask, prof)
                color = PALETTE[slot % len(PALETTE)]
                valid_count = 0

                for idx, c in enumerate(candidates):
                    valid_count += 1
                    cv2.rectangle(label_overlay, (c["x"], c["y"]),
                                  (c["x"] + c["w"], c["y"] + c["h"]), color, 2)
                    cv2.drawMarker(label_overlay, (c["cx"], c["cy"]), color,
                                   cv2.MARKER_CROSS, 12, 2)
                    label_text = (f"{prof['name'][:12]} {c['conf']:.0f}% "
                                  f"c{c['corners']} sol{c['solidity']:.2f}")
                    cv2.putText(label_overlay, label_text, (c["x"], max(15, c["y"] - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
                total_valid += valid_count

            dt_ms = (time.time() - t_start) * 1000.0
            fps_curr = 1000.0 / dt_ms if dt_ms > 0 else 0
            fps_avg = 0.9 * fps_avg + 0.1 * fps_curr if fps_avg > 0 else fps_curr

            ap = self.active_profile()
            ab_ = self.active_band()
            enabled_names = ",".join(p["name"].replace("MARKER_", "") for p in self.profiles if p["enabled"])

            banner = np.zeros((96, frame.shape[1], 3), dtype=np.uint8)
            cv2.rectangle(banner, (0, 0), (frame.shape[1], 96), (20, 20, 20), -1)

            mode_str = f"MULTI({len(active_list)})" if self.multi_color else ap["name"]
            line1 = (f"PROFILE[{self.active_idx}] {ap['name']} | band:{'ALT' if self.edit_alt else 'PRI'} "
                     f"| scan:{mode_str} | norm:{'ON' if self.norm_enabled else 'OFF'} "
                     f"| light:{self.lighting_override}->{lighting_mode}")
            line2 = (f"meanV:{last_mean_v:.0f} gain:{last_gain:.2f} | valid blobs:{total_valid} "
                     f"| enabled: {enabled_names}")
            line3 = (f"ACTIVE HSV L[{ab_['h_low']},{ab_['s_low']},{ab_['v_low']}] "
                     f"H[{ab_['h_high']},{ab_['s_high']},{ab_['v_high']}] "
                     f"circ>{ap['circularity_min']:.2f} ext[{ap['gates']['extent_min']:.2f},"
                     f"{ap['gates']['extent_max']:.2f}] sol>{ap['gates']['solidity_min']:.2f} "
                     f"corn[{ap['gates']['corners_min']},{ap['gates']['corners_max']}]")

            cv2.putText(banner, line1, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(banner, line2, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(banner, line3, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1, cv2.LINE_AA)
            cv2.putText(banner, f"FPS {fps_avg:.1f}", (frame.shape[1] - 110, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1, cv2.LINE_AA)

            display_final = np.vstack([banner, label_overlay])

            cv2.imshow(self.window_main, display_final)
            cv2.imshow(self.window_mask, union_mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key in (ord('p'), ord(']')):
                self.active_idx = (self.active_idx + 1) % len(self.profiles)
                self.update_trackbar_positions()
                print(f"[CV_PROTOCOL] Active profile: {self.active_profile()['name']}")
            elif key == ord('['):
                self.active_idx = (self.active_idx - 1) % len(self.profiles)
                self.update_trackbar_positions()
                print(f"[CV_PROTOCOL] Active profile: {self.active_profile()['name']}")
            elif ord('1') <= key <= ord('9'):
                idx = key - ord('1')
                if idx < len(self.profiles):
                    self.active_idx = idx
                    self.update_trackbar_positions()
                    print(f"[CV_PROTOCOL] Active profile: {self.active_profile()['name']}")
            elif key == ord('0'):
                if 9 < len(self.profiles):
                    self.active_idx = 9
                    self.update_trackbar_positions()
                    print(f"[CV_PROTOCOL] Active profile: {self.active_profile()['name']}")
            elif key in (ord('a'), ord('A')):
                ap_en = self.active_profile()
                ap_en["enabled"] = not ap_en["enabled"]
                state = "ENABLED" if ap_en["enabled"] else "DISABLED"
                print(f"[CV_PROTOCOL] {ap_en['name']} {state}")
            elif key in (ord('l'), ord('L')):
                if not self.active_profile()["has_alt"]:
                    print("[CV_PROTOCOL] Active profile has no ALT band.")
                else:
                    self.edit_alt = not self.edit_alt
                    self.update_trackbar_positions()
                    print(f"[CV_PROTOCOL] Editing band: {'ALT (overcast)' if self.edit_alt else 'PRIMARY (sunny)'}")
            elif key in (ord('m'), ord('M')):
                self.multi_color = not self.multi_color
                print(f"[CV_PROTOCOL] Multi-color scan: {self.multi_color}")
            elif key in (ord('n'), ord('N')):
                self.norm_enabled = not self.norm_enabled
                print(f"[CV_PROTOCOL] Exposure normalization: {self.norm_enabled}")
            elif key in (ord('k'), ord('K')):
                order = ["AUTO", "SUNNY", "OVERCAST"]
                self.lighting_override = order[(order.index(self.lighting_override) + 1) % 3]
                print(f"[CV_PROTOCOL] Lighting override: {self.lighting_override}")
            elif key in (ord('c'), ord('C')):
                ap_c = self.active_profile()
                cur = SHAPE_PRESET_ORDER.index(ap_c["shape_preset"]) \
                    if ap_c["shape_preset"] in SHAPE_PRESET_ORDER else 0
                nxt = SHAPE_PRESET_ORDER[(cur + 1) % len(SHAPE_PRESET_ORDER)]
                ap_c["shape_preset"] = nxt
                ap_c["gates"] = dict(SHAPE_PRESETS[nxt])
                self.update_trackbar_positions()
                print(f"[CV_PROTOCOL] Shape preset: {nxt}")
            elif key in (ord('r'), ord('R')):
                name = self.active_profile()["name"]
                for d in make_default_profiles():
                    if d["name"] == name:
                        self.profiles[self.active_idx] = d
                        break
                self.update_trackbar_positions()
                print(f"[CV_PROTOCOL] Reset '{name}' to factory defaults.")
            elif key in (ord('s'), ord('S')):
                self.save_config()
            elif key in (ord('e'), ord('E')):
                self.save_config()
                self.export_header()

        cap.release()
        cv2.destroyAllWindows()
        print("[CV_PROTOCOL] Camera released and windows closed clean.")
        return True


def main():
    global CONFIG_FILENAME
    parser = argparse.ArgumentParser(description="Robofest laptop CV calibration protocol")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--export-only", metavar="JSON_PATH", default=None,
                        help="Load calibration JSON and write the firmware header without camera")
    args = parser.parse_args()

    if args.export_only is not None:
        CONFIG_FILENAME = args.export_only
        protocol = CvLaptopProtocol(camera_index=args.camera,
                                    width=args.width, height=args.height)
        ok = protocol.export_header()
        sys.exit(0 if ok else 1)

    protocol = CvLaptopProtocol(camera_index=args.camera,
                                width=args.width, height=args.height)
    protocol.run()


if __name__ == "__main__":
    main()
