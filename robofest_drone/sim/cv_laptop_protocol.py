#!/usr/bin/env python3
"""
Robofest Gujarat 6.0 Minefield Swarm Drone
Laptop Webcam Computer Vision (CV) Calibration & Perception Protocol

This interactive tool connects to the laptop's live webcam, executes the onboard 
VisionPipeline HSV color segmentation & candidate blob detection, provides 
real-time interactive trackbars for HSV threshold tuning, and displays an on-screen HUD.
"""

import sys
import time
import json
import os
import cv2
import numpy as np

# Default Firmware Constants (Mapped to OpenCV HSV range: H 0-180, S 0-255, V 0-255)
# Note: Onboard C++ code uses H 0-255 scale; OpenCV uses H 0-180 scale.
# OpenCV H = C++ H * (180/255)
DEFAULT_GROUND_HSV_LOW = np.array([0, 100, 100], dtype=np.uint8)
DEFAULT_GROUND_HSV_HIGH = np.array([11, 255, 255], dtype=np.uint8)

DEFAULT_BURIED_HSV_LOW = np.array([17, 120, 120], dtype=np.uint8)
DEFAULT_BURIED_HSV_HIGH = np.array([28, 255, 255], dtype=np.uint8)

CONFIG_FILENAME = "cv_hsv_config.json"

class CvLaptopProtocol:
    def __init__(self, camera_index=0, width=640, height=480):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        
        self.active_profile = "ON_GROUND_MINE"  # or "BURIED_SURFACE_MARKER"
        
        # Load saved thresholds if available
        self.profiles = {
            "ON_GROUND_MINE": {
                "h_low": int(DEFAULT_GROUND_HSV_LOW[0]), "h_high": int(DEFAULT_GROUND_HSV_HIGH[0]),
                "s_low": int(DEFAULT_GROUND_HSV_LOW[1]), "s_high": int(DEFAULT_GROUND_HSV_HIGH[1]),
                "v_low": int(DEFAULT_GROUND_HSV_LOW[2]), "v_high": int(DEFAULT_GROUND_HSV_HIGH[2]),
                "min_area": 100, "max_area": 10000, "circularity_min": 65
            },
            "BURIED_SURFACE_MARKER": {
                "h_low": int(DEFAULT_BURIED_HSV_LOW[0]), "h_high": int(DEFAULT_BURIED_HSV_HIGH[0]),
                "s_low": int(DEFAULT_BURIED_HSV_LOW[1]), "s_high": int(DEFAULT_BURIED_HSV_HIGH[1]),
                "v_low": int(DEFAULT_BURIED_HSV_LOW[2]), "v_high": int(DEFAULT_BURIED_HSV_HIGH[2]),
                "min_area": 80, "max_area": 8000, "circularity_min": 50
            }
        }
        self.load_config()

        self.window_main = "Robofest CV Protocol - Live Feed"
        self.window_mask = "HSV Mask Segment"
        self.window_controls = "HSV Trackbar Controls"

    def load_config(self):
        if os.path.exists(CONFIG_FILENAME):
            try:
                with open(CONFIG_FILENAME, "r") as f:
                    data = json.load(f)
                    self.profiles.update(data.get("profiles", {}))
                print(f"[CV_PROTOCOL] Loaded custom HSV thresholds from '{CONFIG_FILENAME}'.")
            except Exception as e:
                print(f"[CV_PROTOCOL][WARN] Failed to load config: {e}")

    def save_config(self):
        try:
            with open(CONFIG_FILENAME, "w") as f:
                json.dump({"profiles": self.profiles}, f, indent=2)
            print(f"[CV_PROTOCOL] Successfully saved calibrated HSV thresholds to '{CONFIG_FILENAME}'.")
            return True
        except Exception as e:
            print(f"[CV_PROTOCOL][ERROR] Save failed: {e}")
            return False

    def setup_trackbars(self):
        cv2.namedWindow(self.window_controls, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_controls, 450, 350)

        p = self.profiles[self.active_profile]

        cv2.createTrackbar("H Low", self.window_controls, p["h_low"], 180, self._nothing)
        cv2.createTrackbar("H High", self.window_controls, p["h_high"], 180, self._nothing)
        cv2.createTrackbar("S Low", self.window_controls, p["s_low"], 255, self._nothing)
        cv2.createTrackbar("S High", self.window_controls, p["s_high"], 255, self._nothing)
        cv2.createTrackbar("V Low", self.window_controls, p["v_low"], 255, self._nothing)
        cv2.createTrackbar("V High", self.window_controls, p["v_high"], 255, self._nothing)
        cv2.createTrackbar("Min Area", self.window_controls, p["min_area"], 500, self._nothing)
        cv2.createTrackbar("Circularity %", self.window_controls, p["circularity_min"], 100, self._nothing)

    def read_trackbars(self):
        p = self.profiles[self.active_profile]
        p["h_low"] = cv2.getTrackbarPos("H Low", self.window_controls)
        p["h_high"] = cv2.getTrackbarPos("H High", self.window_controls)
        p["s_low"] = cv2.getTrackbarPos("S Low", self.window_controls)
        p["s_high"] = cv2.getTrackbarPos("S High", self.window_controls)
        p["v_low"] = cv2.getTrackbarPos("V Low", self.window_controls)
        p["v_high"] = cv2.getTrackbarPos("V High", self.window_controls)
        p["min_area"] = max(10, cv2.getTrackbarPos("Min Area", self.window_controls) * 10)
        p["circularity_min"] = cv2.getTrackbarPos("Circularity %", self.window_controls)

    def update_trackbar_positions(self):
        p = self.profiles[self.active_profile]
        cv2.setTrackbarPos("H Low", self.window_controls, p["h_low"])
        cv2.setTrackbarPos("H High", self.window_controls, p["h_high"])
        cv2.setTrackbarPos("S Low", self.window_controls, p["s_low"])
        cv2.setTrackbarPos("S High", self.window_controls, p["s_high"])
        cv2.setTrackbarPos("V Low", self.window_controls, p["v_low"])
        cv2.setTrackbarPos("V High", self.window_controls, p["v_high"])
        cv2.setTrackbarPos("Min Area", self.window_controls, p["min_area"] // 10)
        cv2.setTrackbarPos("Circularity %", self.window_controls, p["circularity_min"])

    def _nothing(self, x):
        pass

    def run(self):
        print("================================================================")
        print("  ROBOFEST GUJARAT 6.0 - LAPTOP WEBCAM CV PROTOCOL INITIALIZING")
        print("================================================================")
        print(f"[CV_PROTOCOL] Opening camera index {self.camera_index}...")

        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW) if sys.platform.startswith("win") else cv2.VideoCapture(self.camera_index)
        
        if not cap.isOpened():
            print(f"[CV_PROTOCOL][ERROR] Unable to open camera index {self.camera_index}.")
            print("Trying fallback camera index 1...")
            cap = cv2.VideoCapture(1)
            if not cap.isOpened():
                print("[CV_PROTOCOL][CRITICAL] No available webcam found!")
                return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.setup_trackbars()

        cv2.namedWindow(self.window_main, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.window_mask, cv2.WINDOW_NORMAL)

        print("[CV_PROTOCOL] Camera stream started successfully.")
        print("----------------------------------------------------------------")
        print("KEYBOARD CONTROLS:")
        print("  [P] Switch Profile (ON_GROUND_MINE <-> BURIED_SURFACE_MARKER)")
        print("  [S] Save calibrated HSV thresholds to config JSON")
        print("  [R] Reset active profile to factory firmware defaults")
        print("  [Q] or [ESC] Exit CV Protocol")
        print("----------------------------------------------------------------")

        fps_avg = 0.0
        frame_counter = 0

        while True:
            t_start = time.time()
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[CV_PROTOCOL][WARN] Frame grab empty. Retrying...")
                time.sleep(0.03)
                continue

            frame_counter += 1
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            self.read_trackbars()
            p = self.profiles[self.active_profile]

            # HSV Mask Segmentation
            h_low, h_high = p["h_low"], p["h_high"]
            s_low, s_high = p["s_low"], p["s_high"]
            v_low, v_high = p["v_low"], p["v_high"]

            if h_low <= h_high:
                lower = np.array([h_low, s_low, v_low], dtype=np.uint8)
                upper = np.array([h_high, s_high, v_high], dtype=np.uint8)
                mask = cv2.inRange(hsv, lower, upper)
            else:
                # Wraparound for low/high red hues
                lower1 = np.array([0, s_low, v_low], dtype=np.uint8)
                upper1 = np.array([h_high, s_high, v_high], dtype=np.uint8)
                mask1 = cv2.inRange(hsv, lower1, upper1)

                lower2 = np.array([h_low, s_low, v_low], dtype=np.uint8)
                upper2 = np.array([180, s_high, v_high], dtype=np.uint8)
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask1, mask2)

            # Morphological Cleanup (Erosion & Dilation)
            kernel = np.ones((3, 3), np.uint8)
            mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask_clean = cv2.dilate(mask_clean, kernel, iterations=1)

            # Contour Extraction & Candidate Analysis
            contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            candidates = []
            min_area = p["min_area"]
            min_circ = p["circularity_min"] / 100.0

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < min_area or area > 50000:
                    continue

                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue

                circularity = (4.0 * np.pi * area) / (perimeter * perimeter)
                x, y, w, h_box = cv2.boundingRect(cnt)
                M = cv2.moments(cnt)
                cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else x + w // 2
                cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else y + h_box // 2

                # Calculate confidence score
                circ_score = min(1.0, circularity / 0.85) * 50.0
                area_score = min(1.0, area / 1500.0) * 30.0
                dist_to_center = np.hypot(cx - frame.shape[1] / 2, cy - frame.shape[0] / 2)
                center_score = max(0.0, 1.0 - (dist_to_center / (frame.shape[1] / 2))) * 20.0
                
                confidence = circ_score + area_score + center_score

                valid = circularity >= min_circ
                candidates.append({
                    "cnt": cnt, "x": x, "y": y, "w": w, "h": h_box,
                    "cx": cx, "cy": cy, "area": area, "circularity": circularity,
                    "confidence": confidence, "valid": valid
                })

            # Sort by confidence descending
            candidates.sort(key=lambda c: c["confidence"], reverse=True)

            # Render Overlay HUD
            display_frame = frame.copy()

            valid_count = 0
            for idx, c in enumerate(candidates):
                color = (0, 255, 0) if c["valid"] else (0, 165, 255) if c["confidence"] >= 45 else (0, 0, 255)
                label_text = f"C{idx+1}: {c['confidence']:.1f}% (circ:{c['circularity']:.2f})"
                
                if c["valid"]:
                    valid_count += 1

                # Draw bounding box & centroid crosshair
                cv2.rectangle(display_frame, (c["x"], c["y"]), (c["x"] + c["w"], c["y"] + c["h"]), color, 2)
                cv2.drawMarker(display_frame, (c["cx"], c["cy"]), color, cv2.MARKER_CROSS, 12, 2)
                cv2.putText(display_frame, label_text, (c["x"], max(15, c["y"] - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

            # HUD Header Banner
            dt_ms = (time.time() - t_start) * 1000.0
            fps_curr = 1000.0 / dt_ms if dt_ms > 0 else 0
            fps_avg = 0.9 * fps_avg + 0.1 * fps_curr if fps_avg > 0 else fps_curr

            banner_bg = np.zeros((70, display_frame.shape[1], 3), dtype=np.uint8)
            cv2.rectangle(banner_bg, (0, 0), (display_frame.shape[1], 70), (20, 20, 20), -1)

            profile_color = (0, 255, 255) if self.active_profile == "ON_GROUND_MINE" else (255, 191, 0)
            cv2.putText(banner_bg, f"ROBOFEST CV PROTOCOL | PROFILE: {self.active_profile}", (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, profile_color, 2, cv2.LINE_AA)
            
            cv2.putText(banner_bg, f"FPS: {fps_avg:.1f} ({dt_ms:.1f} ms) | Candidates: {valid_count}/{len(candidates)} valid", (10, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

            cv2.putText(banner_bg, f"HSV Low:[{h_low},{s_low},{v_low}] High:[{h_high},{s_high},{v_high}] MinArea:{min_area} Circ:{min_circ:.2f}", (10, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1, cv2.LINE_AA)

            display_final = np.vstack([banner_bg, display_frame])

            # Render Window Displays
            cv2.imshow(self.window_main, display_final)
            cv2.imshow(self.window_mask, mask_clean)

            # Process Keyboard Controls
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # ESC
                print("[CV_PROTOCOL] Exiting CV protocol runtime.")
                break
            elif key == ord('p') or key == ord('P'):
                self.active_profile = "BURIED_SURFACE_MARKER" if self.active_profile == "ON_GROUND_MINE" else "ON_GROUND_MINE"
                self.update_trackbar_positions()
                print(f"[CV_PROTOCOL] Switched active profile to: {self.active_profile}")
            elif key == ord('s') or key == ord('S'):
                self.save_config()
            elif key == ord('r') or key == ord('R'):
                if self.active_profile == "ON_GROUND_MINE":
                    self.profiles["ON_GROUND_MINE"]["h_low"] = int(DEFAULT_GROUND_HSV_LOW[0])
                    self.profiles["ON_GROUND_MINE"]["h_high"] = int(DEFAULT_GROUND_HSV_HIGH[0])
                    self.profiles["ON_GROUND_MINE"]["s_low"] = int(DEFAULT_GROUND_HSV_LOW[1])
                    self.profiles["ON_GROUND_MINE"]["s_high"] = int(DEFAULT_GROUND_HSV_HIGH[1])
                    self.profiles["ON_GROUND_MINE"]["v_low"] = int(DEFAULT_GROUND_HSV_LOW[2])
                    self.profiles["ON_GROUND_MINE"]["v_high"] = int(DEFAULT_GROUND_HSV_HIGH[2])
                else:
                    self.profiles["BURIED_SURFACE_MARKER"]["h_low"] = int(DEFAULT_BURIED_HSV_LOW[0])
                    self.profiles["BURIED_SURFACE_MARKER"]["h_high"] = int(DEFAULT_BURIED_HSV_HIGH[0])
                    self.profiles["BURIED_SURFACE_MARKER"]["s_low"] = int(DEFAULT_BURIED_HSV_LOW[1])
                    self.profiles["BURIED_SURFACE_MARKER"]["s_high"] = int(DEFAULT_BURIED_HSV_HIGH[1])
                    self.profiles["BURIED_SURFACE_MARKER"]["v_low"] = int(DEFAULT_BURIED_HSV_LOW[2])
                    self.profiles["BURIED_SURFACE_MARKER"]["v_high"] = int(DEFAULT_BURIED_HSV_HIGH[2])
                self.update_trackbar_positions()
                print(f"[CV_PROTOCOL] Reset profile '{self.active_profile}' to default firmware values.")

        cap.release()
        cv2.destroyAllWindows()
        print("[CV_PROTOCOL] Camera released and windows closed clean.")
        return True

if __name__ == "__main__":
    protocol = CvLaptopProtocol(camera_index=0)
    protocol.run()
