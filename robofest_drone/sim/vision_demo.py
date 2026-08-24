"""
Robofest Vision Pipeline Demo — Desktop OpenCV Version
=======================================================
Mirrors the embedded C++ vision_pipeline logic with a live webcam feed:
  1. BGR -> HSV conversion
  2. Gaussian blur (noise smoothing)
  3. Multi-color HSV thresholding (with red hue wraparound)
  4. Morphological cleanup (erode + dilate = MORPH_OPEN + MORPH_CLOSE)
  5. Contour extraction + area/circularity filtering
  6. Bounding box + center point overlay

Controls:
  Q - Quit
  T - Toggle HSV trackbar tuner window
  M - Toggle mask debug view
  B - Toggle blur on/off
  O - Toggle morphology on/off

Uses the same HSV ranges from config/thresholds.h as starting points.
"""

import cv2
import numpy as np
import time

# ---------------------------------------------------------------------------
# COLOR PROFILES - matches thresholds.h defaults + guide starting points
# Red uses two ranges to handle the hue wraparound at 0/180
# ---------------------------------------------------------------------------
COLOR_RANGES = {
    "Mine (Red)":  [((0, 100, 100), (15, 255, 255)),
                    ((170, 100, 100), (180, 255, 255))],
    "Buried (Yellow-Orange)": [((25, 120, 120), (40, 255, 255))],
    "Green":       [((36, 60, 60),  (89, 255, 255))],
    "Blue":        [((100, 150, 50), (130, 255, 255))],
}

DRAW_COLORS = {
    "Mine (Red)":           (0, 0, 255),
    "Buried (Yellow-Orange)": (0, 200, 255),
    "Green":                (0, 255, 0),
    "Blue":                 (255, 100, 0),
}

# Pipeline parameters (matching embedded config)
MIN_AREA        = 500       # BLOB_AREA_MIN_PX equivalent (desktop scale)
CIRCULARITY_MIN = 0.25      # More lenient than embedded (0.70) for demo variety
BLUR_KSIZE      = (5, 5)
MORPH_KERNEL    = np.ones((5, 5), np.uint8)

# ---------------------------------------------------------------------------
# HSV TRACKBAR TUNER (matches calibration/hsv_tuner logic)
# ---------------------------------------------------------------------------
def nothing(x):
    pass

def launch_tuner():
    cv2.namedWindow("HSV Tuner")
    cv2.createTrackbar("L-H", "HSV Tuner", 0, 179, nothing)
    cv2.createTrackbar("L-S", "HSV Tuner", 100, 255, nothing)
    cv2.createTrackbar("L-V", "HSV Tuner", 100, 255, nothing)
    cv2.createTrackbar("U-H", "HSV Tuner", 15, 179, nothing)
    cv2.createTrackbar("U-S", "HSV Tuner", 255, 255, nothing)
    cv2.createTrackbar("U-V", "HSV Tuner", 255, 255, nothing)
    print("[TUNER] Trackbar window opened - adjust sliders to find your HSV range.")
    print("[TUNER] Press T again to close tuner and print values.")

# ---------------------------------------------------------------------------
# MAIN DEMO LOOP
# ---------------------------------------------------------------------------
def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Cannot open webcam. Check your camera connection.")
        return

    # Lock exposure if supported (mirrors HAL camera exposure locking)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual mode
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)           # Lock white balance

    print("=" * 60)
    print("  ROBOFEST VISION PIPELINE DEMO")
    print("=" * 60)
    print("  Q = Quit | T = Tuner | M = Mask | B = Blur | O = Morphology")
    print("=" * 60)

    show_mask = False
    blur_enabled = True
    morph_enabled = True
    tuner_open = False
    prev_time = time.time()
    fps = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # FPS calculation
        now = time.time()
        dt = now - prev_time
        prev_time = now
        fps = 0.9 * fps + 0.1 * (1.0 / dt) if dt > 0 else fps

        display = frame.copy()

        # Step 1: Optional Gaussian blur (matches VISION_BLUR_ENABLED)
        if blur_enabled:
            blurred = cv2.GaussianBlur(frame, BLUR_KSIZE, 0)
        else:
            blurred = frame

        # Step 2: BGR -> HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)

        for color_name, ranges in COLOR_RANGES.items():
            # Step 3: HSV thresholding (with red wraparound via OR)
            mask = None
            for lower, upper in ranges:
                m = cv2.inRange(hsv, np.array(lower), np.array(upper))
                mask = m if mask is None else cv2.bitwise_or(mask, m)

            # Step 4: Morphological cleanup (matches VISION_MORPHOLOGY_ENABLED)
            if morph_enabled:
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

            combined_mask = cv2.bitwise_or(combined_mask, mask)

            # Step 5: Contour extraction + filtering
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            # Filter and sort by area
            valid_contours = []
            for c in contours:
                area = cv2.contourArea(c)
                if area < MIN_AREA:
                    continue
                perimeter = cv2.arcLength(c, True)
                if perimeter < 1.0:
                    continue
                circularity = (4.0 * np.pi * area) / (perimeter * perimeter)
                if circularity < CIRCULARITY_MIN:
                    continue
                valid_contours.append((c, area, circularity))

            if not valid_contours:
                continue

            # Pick largest valid contour
            valid_contours.sort(key=lambda x: x[1], reverse=True)
            largest, area, circ = valid_contours[0]

            # Step 6: Draw bounding box + centroid
            x, y, w, h = cv2.boundingRect(largest)
            cx, cy = x + w // 2, y + h // 2
            draw_color = DRAW_COLORS.get(color_name, (255, 255, 255))

            cv2.rectangle(display, (x, y), (x + w, y + h), draw_color, 2)
            cv2.circle(display, (cx, cy), 5, draw_color, -1)
            cv2.putText(display,
                        f"{color_name} A:{area:.0f} C:{circ:.2f}",
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, draw_color, 2)

            # Crosshair lines to frame center (steering reference)
            frame_cx = frame.shape[1] // 2
            cv2.line(display, (cx, cy), (frame_cx, cy), draw_color, 1)

        # Tuner overlay if open
        if tuner_open:
            try:
                l_h = cv2.getTrackbarPos("L-H", "HSV Tuner")
                l_s = cv2.getTrackbarPos("L-S", "HSV Tuner")
                l_v = cv2.getTrackbarPos("L-V", "HSV Tuner")
                u_h = cv2.getTrackbarPos("U-H", "HSV Tuner")
                u_s = cv2.getTrackbarPos("U-S", "HSV Tuner")
                u_v = cv2.getTrackbarPos("U-V", "HSV Tuner")
                tuner_mask = cv2.inRange(hsv,
                                         np.array([l_h, l_s, l_v]),
                                         np.array([u_h, u_s, u_v]))
                if morph_enabled:
                    tuner_mask = cv2.morphologyEx(tuner_mask, cv2.MORPH_OPEN, MORPH_KERNEL)
                    tuner_mask = cv2.morphologyEx(tuner_mask, cv2.MORPH_CLOSE, MORPH_KERNEL)
                cv2.imshow("Tuner Mask", tuner_mask)
                cv2.putText(display,
                            f"TUNER H:[{l_h},{u_h}] S:[{l_s},{u_s}] V:[{l_v},{u_v}]",
                            (10, frame.shape[0] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            except cv2.error:
                pass

        # HUD overlay
        h_frame, w_frame = display.shape[:2]
        cv2.line(display, (w_frame // 2, 0), (w_frame // 2, h_frame),
                 (50, 50, 50), 1)
        cv2.putText(display, f"FPS: {fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        status_y = 50
        flags = []
        if blur_enabled:  flags.append("BLUR")
        if morph_enabled: flags.append("MORPH")
        cv2.putText(display, " | ".join(flags), (10, status_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Robofest Vision Demo", display)

        if show_mask:
            mask_color = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
            cv2.imshow("Mask Debug", mask_color)

        # Key handling
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            if tuner_open:
                try:
                    l_h = cv2.getTrackbarPos("L-H", "HSV Tuner")
                    l_s = cv2.getTrackbarPos("L-S", "HSV Tuner")
                    l_v = cv2.getTrackbarPos("L-V", "HSV Tuner")
                    u_h = cv2.getTrackbarPos("U-H", "HSV Tuner")
                    u_s = cv2.getTrackbarPos("U-S", "HSV Tuner")
                    u_v = cv2.getTrackbarPos("U-V", "HSV Tuner")
                    print(f"\n[TUNER] Final HSV values:")
                    print(f"  lower = [{l_h}, {l_s}, {l_v}]")
                    print(f"  upper = [{u_h}, {u_s}, {u_v}]")
                    print(f"  For thresholds.h: HsvColor LOW = {{{l_h}, {l_s}, {l_v}}};")
                    print(f"                    HsvColor HIGH = {{{u_h}, {u_s}, {u_v}}};")
                except cv2.error:
                    pass
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
        elif key == ord('t'):
            tuner_open = not tuner_open
            if tuner_open:
                launch_tuner()
            else:
                try:
                    l_h = cv2.getTrackbarPos("L-H", "HSV Tuner")
                    l_s = cv2.getTrackbarPos("L-S", "HSV Tuner")
                    l_v = cv2.getTrackbarPos("L-V", "HSV Tuner")
                    u_h = cv2.getTrackbarPos("U-H", "HSV Tuner")
                    u_s = cv2.getTrackbarPos("U-S", "HSV Tuner")
                    u_v = cv2.getTrackbarPos("U-V", "HSV Tuner")
                    print(f"\n[TUNER] Captured HSV values:")
                    print(f"  lower = [{l_h}, {l_s}, {l_v}]")
                    print(f"  upper = [{u_h}, {u_s}, {u_v}]")
                    print(f"  For thresholds.h: HsvColor LOW = {{{l_h}, {l_s}, {l_v}}};")
                    print(f"                    HsvColor HIGH = {{{u_h}, {u_s}, {u_v}}};")
                except cv2.error:
                    pass
                cv2.destroyWindow("HSV Tuner")
                cv2.destroyWindow("Tuner Mask")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
