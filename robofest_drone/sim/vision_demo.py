"""
Robofest Vision Pipeline Demo — Desktop OpenCV Version (Single & Dual Camera)
=============================================================================
Full-Spectrum Color & Shape Detection with Real-time Bounding Boxes & HUD:
  - Colors: Red, Orange, Yellow, Green, Cyan, Blue, Purple, Magenta, Black, White
  - Shapes: Circle, Square, Rectangle, Triangle, Pentagon, Hexagon, Star
  - Exposure: Automatic brightness adaptation (works under any indoor room lighting)
  - Detection: Low-threshold area filter (150 px) detects small and large objects instantly
  - Multi-Camera: Run Laptop Webcam + Phone DroidCam simultaneously side-by-side!

Controls:
  Q / ESC - Quit
  T       - Toggle HSV trackbar tuner window
  M       - Toggle binary mask debug view
  B       - Toggle Gaussian blur on/off
  O       - Toggle morphology on/off
  N       - Toggle brightness normalization on/off
"""

import sys
import os
import argparse
import threading
import time
import math
import cv2
import numpy as np


def resolve_video_source(source):
    if source is None:
        return 0
    if isinstance(source, int):
        return source
    src_str = str(source).strip()
    if src_str.isdigit() or (src_str.startswith("-") and src_str[1:].isdigit()):
        return int(src_str)
    if src_str.startswith("http://") or src_str.startswith("https://"):
        if src_str.count("/") <= 2 or src_str.endswith(":4747"):
            src_str = src_str.rstrip("/") + "/video"
    return src_str


def open_video_capture(source):
    if isinstance(source, int):
        cap = cv2.VideoCapture(source)
        if not cap.isOpened() or not cap.read()[0]:
            cap.release()
            try:
                cap = cv2.VideoCapture(source, cv2.CAP_DSHOW)
            except Exception:
                cap = cv2.VideoCapture(source)
    else:
        cap = cv2.VideoCapture(source)
    return cap


class CameraStream:
    """Threaded camera capture to prevent stream lag."""
    def __init__(self, src, name="Camera"):
        self.src = resolve_video_source(src)
        self.name = name
        self.cap = open_video_capture(self.src)
        self.ret = False
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()
        self.fps = 0.0
        self._prev_time = time.time()

        if self.cap.isOpened():
            self.ret, self.frame = self.cap.read()
            self.thread = threading.Thread(target=self._update, daemon=True)
            self.thread.start()

    def _update(self):
        while not self.stopped:
            if not self.cap.isOpened():
                break
            ret, frame = self.cap.read()
            now = time.time()
            dt = now - self._prev_time
            self._prev_time = now
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

            with self.lock:
                self.ret = ret
                if ret and frame is not None:
                    self.frame = frame
            time.sleep(0.005)

    def read(self):
        with self.lock:
            if self.frame is None:
                return self.ret, None
            return self.ret, self.frame.copy()

    def is_opened(self):
        return self.cap.isOpened()

    def release(self):
        self.stopped = True
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=0.4)
        self.cap.release()


# ---------------------------------------------------------------------------
# COLOR PROFILES - Full Spectrum (Hue 0-180, S 0-255, V 0-255)
# ---------------------------------------------------------------------------
COLOR_BANDS = {
    "Red (Mine)":      [(0, 10, 70, 255, 50, 255), (170, 180, 70, 255, 50, 255)],
    "Orange":          [(11, 22, 80, 255, 60, 255)],
    "Yellow (Buried)": [(23, 35, 70, 255, 60, 255)],
    "Green":           [(36, 85, 50, 255, 40, 255)],
    "Cyan":            [(86, 100, 50, 255, 40, 255)],
    "Blue":            [(101, 130, 50, 255, 40, 255)],
    "Purple":          [(131, 150, 50, 255, 40, 255)],
    "Magenta":         [(151, 169, 50, 255, 40, 255)],
}

DRAW_COLORS = {
    "Red (Mine)":      (0, 0, 255),
    "Orange":          (0, 140, 255),
    "Yellow (Buried)": (0, 255, 255),
    "Green":           (0, 255, 0),
    "Cyan":            (255, 255, 0),
    "Blue":            (255, 100, 0),
    "Purple":          (240, 32, 160),
    "Magenta":         (180, 0, 255),
}

# Detection parameters (tuned for responsive indoor detection)
MIN_AREA             = 150       # Pixels (detects small markers / objects easily)
MAX_AREA             = 300000
CORNER_EPSILON_RATIO = 0.045
BLUR_KSIZE           = (5, 5)
MORPH_KERNEL         = np.ones((3, 3), np.uint8)


def classify_shape(corners, aspect, extent, solidity, circularity):
    """Classify 2D geometric shape from contour properties."""
    if circularity >= 0.82 and solidity >= 0.85:
        return "circle"
    if solidity < 0.70 and corners >= 4:
        return "star"
    if corners == 3:
        return "triangle"
    if corners == 4:
        return "square" if aspect <= 1.25 else "rect"
    if corners == 5:
        return "pentagon"
    if corners == 6:
        return "hexagon"
    if corners > 6:
        return "circle" if circularity >= 0.65 else f"{corners}-gon"
    return "blob"


def normalize_frame(bgr):
    """Normalize brightness across varying indoor lighting."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    return cv2.cvtColor(cv2.merge((cl, a, b)), cv2.COLOR_LAB2BGR)


def nothing(x):
    pass


def launch_tuner():
    cv2.namedWindow("HSV Tuner")
    cv2.createTrackbar("L-H", "HSV Tuner", 0, 179, nothing)
    cv2.createTrackbar("L-S", "HSV Tuner", 70, 255, nothing)
    cv2.createTrackbar("L-V", "HSV Tuner", 50, 255, nothing)
    cv2.createTrackbar("U-H", "HSV Tuner", 15, 179, nothing)
    cv2.createTrackbar("U-S", "HSV Tuner", 255, 255, nothing)
    cv2.createTrackbar("U-V", "HSV Tuner", 255, 255, nothing)
    print("[TUNER] Trackbar window opened - adjust sliders to isolate colors.")


def process_frame(frame, blur_enabled, morph_enabled, norm_enabled, label="CAMERA", target_h=480):
    """Run perception pipeline on a single frame."""
    if frame is None:
        target_w = int(target_h * 4 / 3)
        blank = np.zeros((target_h, target_w, 3), dtype=np.uint8)
        cv2.putText(blank, f"{label}: CONNECTING...", (30, target_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return blank, np.zeros((target_h, target_w), dtype=np.uint8), None, 0

    # Resize to standardized height
    h, w = frame.shape[:2]
    scale = target_h / float(h)
    target_w = int(w * scale)
    frame_resized = cv2.resize(frame, (target_w, target_h))

    # Brightness normalization
    processed = normalize_frame(frame_resized) if norm_enabled else frame_resized

    # Optional Gaussian blur
    if blur_enabled:
        processed = cv2.GaussianBlur(processed, BLUR_KSIZE, 0)

    # BGR -> HSV
    hsv = cv2.cvtColor(processed, cv2.COLOR_BGR2HSV)
    combined_mask = np.zeros(frame_resized.shape[:2], dtype=np.uint8)
    display = frame_resized.copy()

    total_detections = 0

    for color_name, bands in COLOR_BANDS.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for h0, h1, s0, s1, v0, v1 in bands:
            m = cv2.inRange(hsv, np.array([h0, s0, v0]), np.array([h1, s1, v1]))
            mask = cv2.bitwise_or(mask, m)

        # Morphology cleanup
        if morph_enabled:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

        combined_mask = cv2.bitwise_or(combined_mask, mask)

        # Contour extraction
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA or area > MAX_AREA:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0:
                continue

            circularity = min(1.0, (4.0 * math.pi * area) / (perimeter * perimeter))
            bx, by, bw, bh = cv2.boundingRect(cnt)
            aspect = max(bw, bh) / float(max(min(bw, bh), 1))
            extent = area / float(max(bw * bh, 1))

            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = min(1.0, area / hull_area) if hull_area > 1.0 else 1.0

            approx = cv2.approxPolyDP(hull, CORNER_EPSILON_RATIO * math.hypot(bw, bh), True)
            corners = len(approx)

            # Centroid
            M = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else bx + bw // 2
            cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else by + bh // 2

            shape_name = classify_shape(corners, aspect, extent, solidity, circularity)
            draw_color = DRAW_COLORS.get(color_name, (255, 255, 255))

            # Bounding box and contour outline
            cv2.drawContours(display, [cnt], -1, draw_color, 2)
            cv2.rectangle(display, (bx, by), (bx + bw, by + bh), draw_color, 1)
            cv2.circle(display, (cx, cy), 4, (0, 255, 255), -1)

            # Label banner
            label_text = f"{color_name} {shape_name} ({int(area)}px)"
            cv2.putText(display, label_text, (bx, max(18, by - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 2)

            # Crosshair to center
            frame_cx = target_w // 2
            cv2.line(display, (cx, cy), (frame_cx, cy), draw_color, 1)
            total_detections += 1

    # Frame header banner
    cv2.rectangle(display, (0, 0), (target_w, 28), (25, 25, 25), -1)
    cv2.putText(display, f"{label} | Detections: {total_detections}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)

    return display, combined_mask, hsv, total_detections


def main():
    parser = argparse.ArgumentParser(description="Robofest Vision Pipeline Demo (Single & Dual Camera)")
    parser.add_argument("sources", nargs="*", default=[],
                        help="Camera index (e.g. 0) or dual sources (e.g. 0 http://192.168.1.45:4747/video)")
    parser.add_argument("--cam1", "-c1", default=None,
                        help="Camera 1 source (index or URL)")
    parser.add_argument("--cam2", "-c2", default=None,
                        help="Camera 2 source (index or URL)")
    parser.add_argument("--dual", action="store_true",
                        help="Force dual camera mode")
    args = parser.parse_args()

    cam1_src = args.cam1 if args.cam1 is not None else (args.sources[0] if len(args.sources) > 0 else 0)
    cam2_src = args.cam2 if args.cam2 is not None else (args.sources[1] if len(args.sources) > 1 else None)
    if args.dual and cam2_src is None:
        cam2_src = 1

    is_dual = cam2_src is not None

    print("=" * 64)
    print("  ROBOFEST VISION PIPELINE DEMO")
    print(f"  Mode: {'DUAL CAMERA (Side-by-Side)' if is_dual else 'SINGLE CAMERA'}")
    print(f"  Camera 1: {cam1_src}")
    if is_dual:
        print(f"  Camera 2: {cam2_src}")
    print("=" * 64)
    print("  Q = Quit | T = Tuner | M = Mask | B = Blur | O = Morph | N = Norm")
    print("=" * 64)

    stream1 = CameraStream(cam1_src, name="CAM 1")
    if not stream1.is_opened():
        print(f"[ERROR] Could not open Camera 1: {cam1_src}")
        return

    stream2 = None
    if is_dual:
        stream2 = CameraStream(cam2_src, name="CAM 2")
        if not stream2.is_opened():
            print(f"[WARN] Could not open Camera 2 ({cam2_src}). Falling back to single camera.")
            is_dual = False
            stream2 = None

    show_mask = False
    blur_enabled = True
    morph_enabled = True
    norm_enabled = True
    tuner_open = False

    while True:
        ret1, frame1 = stream1.read()
        ret2, frame2 = (stream2.read()) if is_dual and stream2 else (False, None)

        cam1_label = f"CAM 1 [{stream1.fps:.1f} FPS]"
        disp1, mask1, hsv1, det1 = process_frame(frame1, blur_enabled, morph_enabled, norm_enabled,
                                                 label=cam1_label, target_h=480)

        if is_dual:
            cam2_label = f"CAM 2 [{stream2.fps:.1f} FPS]"
            disp2, mask2, hsv2, det2 = process_frame(frame2, blur_enabled, morph_enabled, norm_enabled,
                                                     label=cam2_label, target_h=480)

            divider = np.zeros((480, 4, 3), dtype=np.uint8)
            divider[:] = (0, 255, 255)
            combined_display = np.hstack([disp1, divider, disp2])

            mask_divider = np.zeros((480, 4), dtype=np.uint8)
            mask_divider[:] = 255
            combined_mask = np.hstack([mask1, mask_divider, mask2])
            active_hsv = hsv1
        else:
            combined_display = disp1
            combined_mask = mask1
            active_hsv = hsv1

        # Tuner overlay
        if tuner_open and active_hsv is not None:
            try:
                l_h = cv2.getTrackbarPos("L-H", "HSV Tuner")
                l_s = cv2.getTrackbarPos("L-S", "HSV Tuner")
                l_v = cv2.getTrackbarPos("L-V", "HSV Tuner")
                u_h = cv2.getTrackbarPos("U-H", "HSV Tuner")
                u_s = cv2.getTrackbarPos("U-S", "HSV Tuner")
                u_v = cv2.getTrackbarPos("U-V", "HSV Tuner")
                tuner_mask = cv2.inRange(active_hsv, np.array([l_h, l_s, l_v]), np.array([u_h, u_s, u_v]))
                if morph_enabled:
                    tuner_mask = cv2.morphologyEx(tuner_mask, cv2.MORPH_OPEN, MORPH_KERNEL)
                    tuner_mask = cv2.morphologyEx(tuner_mask, cv2.MORPH_CLOSE, MORPH_KERNEL)
                cv2.imshow("Tuner Mask", tuner_mask)
            except cv2.error:
                pass

        # Bottom HUD flags
        status_flags = []
        if blur_enabled:  status_flags.append("BLUR")
        if morph_enabled: status_flags.append("MORPH")
        if norm_enabled:  status_flags.append("NORM")
        cv2.putText(combined_display, " | ".join(status_flags), (10, combined_display.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        cv2.imshow("Robofest Vision Demo", combined_display)

        if show_mask:
            mask_color = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
            cv2.imshow("Mask Debug", mask_color)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # Q or ESC
            break
        elif key == ord('m'):
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow("Mask Debug")
            print(f"[DEMO] Mask view {'ON' if show_mask else 'OFF'}")
        elif key == ord('b'):
            blur_enabled = not blur_enabled
            print(f"[DEMO] Blur {'ON' if blur_enabled else 'OFF'}")
        elif key == ord('o'):
            morph_enabled = not morph_enabled
            print(f"[DEMO] Morphology {'ON' if morph_enabled else 'OFF'}")
        elif key == ord('n'):
            norm_enabled = not norm_enabled
            print(f"[DEMO] Normalization {'ON' if norm_enabled else 'OFF'}")
        elif key == ord('t'):
            tuner_open = not tuner_open
            if tuner_open:
                launch_tuner()
            else:
                try:
                    cv2.destroyWindow("HSV Tuner")
                    cv2.destroyWindow("Tuner Mask")
                except cv2.error:
                    pass

    stream1.release()
    if stream2:
        stream2.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
