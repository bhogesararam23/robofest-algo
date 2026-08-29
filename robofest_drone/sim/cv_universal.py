#!/usr/bin/env python3
"""
Robofest Universal Color + Shape Detector
==========================================
Covers the ENTIRE hue wheel (plus black/white/gray) and an open-ended set of
polygon shapes, driven by corner count instead of per-shape hand-written gates.

Design (see CV_UNIVERSAL_GUIDE):
  - Color: hue wheel bucketed into named segments with no gaps/overlaps;
    black/white/gray classified by S/V before the hue table.
  - Shape: corner count decides polygon names directly, with circularity and
    solidity tie-breakers for circle-vs-many-sided-polygon and concave/star.

Modes:
  python cv_universal.py --selftest                 # synthetic ground-truth check
  python cv_universal.py --image photo.png --out labeled.png
  python cv_universal.py --camera        # live webcam (index 0)
  python cv_universal.py --camera 1      # different camera

Press Q or ESC in the live window to quit.
"""

import sys
import math
import argparse

import cv2
import numpy as np

# ============================================================================
# STEP 1: HUE WHEEL COVERAGE (OpenCV H runs 0-180). Red is split across both
# ends because it wraps around the 0/180 seam. Every hi == next lo - 1.
# ============================================================================

HUE_BUCKETS = [
    ("red",     0,   8),
    ("orange",  9,   20),
    ("yellow",  21,  33),
    ("green",   34,  78),
    ("cyan",    79,  100),
    ("blue",    101, 130),
    ("purple",  131, 150),
    ("magenta", 151, 172),
    ("red",     173, 180),
]


def _assert_hue_buckets_contiguous():
    ordered = sorted(HUE_BUCKETS, key=lambda r: r[1])
    for (n1, _l1, h1), (n2, l2, _h2) in zip(ordered, ordered[1:]):
        if h1 + 1 != l2:
            raise RuntimeError(f"HUE_BUCKETS gap/overlap between {n1} and {n2}")


_assert_hue_buckets_contiguous()


# ============================================================================
# STEP 2: ONE MASK PER COLOR BUCKET so contours from different colors never
# merge. Boundaries mirror HUE_BUCKETS exactly. All color classification goes
# through COLOR_BANDS — no standalone classify_color() function, because
# hardcoding separate thresholds inevitably drifts out of sync.
# ============================================================================

COLOR_BANDS = {
    "red":     [(0, 8, 90, 255, 50, 255), (173, 180, 90, 255, 50, 255)],
    "orange":  [(9, 20, 90, 255, 50, 255)],
    "yellow":  [(21, 33, 90, 255, 50, 255)],
    "green":   [(34, 78, 60, 255, 40, 255)],
    "cyan":    [(79, 100, 60, 255, 40, 255)],
    "blue":    [(101, 130, 60, 255, 40, 255)],
    "purple":  [(131, 150, 60, 255, 40, 255)],
    "magenta": [(151, 172, 60, 255, 40, 255)],
    "black":   [(0, 180, 0, 255, 0, 50)],
    "white":   [(0, 180, 0, 40, 200, 255)],
    "gray":    [(0, 180, 0, 40, 51, 199)],
}

# Generic obstacle mask: any hue with S/V above ground threshold.
# Subtracted by named color masks before contour analysis so pixels
# already claimed by a specific color don't produce duplicate blobs.
OBSTACLE_BAND = [(0, 180, 40, 255, 40, 255)]


def build_mask(hsv, bands):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for h0, h1, s0, s1, v0, v1 in bands:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, (h0, s0, v0), (h1, s1, v1)))
    return mask


# ============================================================================
# STEP 4: PER-CONTOUR DESCRIPTORS. Color/shape agnostic.
# CORNER_EPSILON_RATIO validated on synthetic renders: 0.035 left spurious
# anti-aliasing vertices on rasterized pentagons (read as hexagons); 0.045
# collapses them to true vertex counts without over-merging triangles/squares.
# Keep within the validated 0.03-0.05 window.
# ============================================================================

CORNER_EPSILON_RATIO = 0.045


def analyze(mask, min_area, max_area):
    kernel = np.ones((5, 5), np.uint8)
    clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel, iterations=1)
    clean = cv2.dilate(clean, kernel, iterations=1)
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    out = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 0:
            continue
        circularity = min(1.0, (4.0 * math.pi * area) / (perimeter * perimeter))

        x, y, w, h = cv2.boundingRect(cnt)
        short_side = max(min(w, h), 1)
        aspect = max(w, h) / short_side
        extent = area / float(max(w * h, 1))

        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = min(1.0, area / hull_area) if hull_area > 1.0 else 1.0

        approx = cv2.approxPolyDP(hull, CORNER_EPSILON_RATIO * math.hypot(w, h), True)
        corners = len(approx)

        M = cv2.moments(cnt)
        cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else x + w // 2
        cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else y + h // 2

        out.append(dict(x=x, y=y, w=w, h=h, cx=cx, cy=cy, area=area,
                        aspect=round(aspect, 2), extent=round(extent, 2),
                        solidity=round(solidity, 2),
                        circularity=round(circularity, 2),
                        corners=corners))
    return out


# ============================================================================
# STEP 5: SHAPE CLASSIFIER. Corner count drives the name; two tie-breakers run
# FIRST because they present corner counts that would otherwise collide with a
# polygon name: near-circular blobs (many corners from DP) and star/concave
# shapes (convex hull hides their inward points).
# ============================================================================

def classify_shape(corners, aspect, extent, solidity, circularity):
    # Tie-breaker 1: circles report many corners even though no real edges exist.
    if circularity >= 0.88 and solidity >= 0.85:
        return "circle"

    # Tie-breaker 2: concave shapes lose inward vertices on their hull, so they
    # show moderate corner counts like pentagon/hexagon. Low solidity is the tell.
    if solidity < 0.70 and corners >= 4:
        return "star"

    names = {3: "triangle", 5: "pentagon", 6: "hexagon", 7: "heptagon", 8: "octagon"}
    if corners == 4:
        return "square" if aspect <= 1.25 else "rectangle"
    if corners in names:
        return names[corners]
    if corners > 8:
        return "circle" if circularity >= 0.7 else f"{corners}-gon"
    return "unknown"


# ============================================================================
# LABEL RENDERING HELPERS
# ============================================================================

DRAW_BGR = {
    "red": (0, 0, 255),
    "orange": (0, 140, 255),
    "yellow": (0, 255, 255),
    "green": (0, 255, 0),
    "cyan": (255, 255, 0),
    "blue": (255, 0, 0),
    "purple": (240, 32, 160),
    "magenta": (180, 0, 255),
    "black": (20, 20, 20),
    "white": (245, 245, 245),
    "gray": (150, 150, 150),
}

SHAPE_ORDER = ["triangle", "square", "rectangle", "pentagon",
               "hexagon", "heptagon", "octagon", "circle", "star"]


def draw_label(overlay, d, label, color_name, is_mine=False):
    if color_name == "obstacle":
        bgr = (128, 128, 128)
    else:
        bgr = DRAW_BGR.get(color_name, (255, 255, 255))
    outline = (255, 255, 255) if color_name in ("black",) else (0, 0, 0)
    p1 = (d["x"], d["y"])
    p2 = (d["x"] + d["w"], d["y"] + d["h"])
    pos = (d["x"], max(15, d["y"] - 8))
    thickness = 3 if is_mine else 1
    cv2.rectangle(overlay, p1, p2, bgr, thickness)
    cv2.putText(overlay, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                outline, 2, cv2.LINE_AA)
    cv2.putText(overlay, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                bgr, 1, cv2.LINE_AA)


# ============================================================================
# EXPOSURE NORMALIZATION (ported from cv_laptop_protocol.py). Auto exposure /
# white balance drifts hue frame to frame; this clamps that drift. Default ON
# in capture modes; disable with --no-norm.
# ============================================================================

EXPOSURE_TARGET_MEAN_V = 135.0
EXPOSURE_GAIN_MIN = 0.70
EXPOSURE_GAIN_MAX = 1.50
GRAY_WORLD_STRENGTH = 0.5


def measure_exposure(bgr):
    small = bgr[::4, ::4]
    hsv_s = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    mean_v = float(np.mean(hsv_s[:, :, 2]))
    mean_b = float(np.mean(small[:, :, 0]))
    mean_g = float(np.mean(small[:, :, 1]))
    mean_r = float(np.mean(small[:, :, 2]))
    return mean_v, (mean_b, mean_g, mean_r)


def apply_exposure_norm(bgr):
    mean_v, (mb, mg, mr) = measure_exposure(bgr)
    if mean_v < 10.0:
        return bgr, 1.0
    base = max(EXPOSURE_GAIN_MIN,
               min(EXPOSURE_GAIN_MAX, EXPOSURE_TARGET_MEAN_V / mean_v))
    avg = (mb + mg + mr) / 3.0

    def gw(m):
        if m < 5.0:
            return base
        g = base * (1.0 + GRAY_WORLD_STRENGTH * (avg / m - 1.0))
        return max(EXPOSURE_GAIN_MIN, min(EXPOSURE_GAIN_MAX, g))

    gains = np.array([gw(mb), gw(mg), gw(mr)], dtype=np.float32)
    out = bgr.astype(np.float32) * gains.reshape(1, 1, 3)
    return np.clip(out, 0, 255).astype(np.uint8), base


# ============================================================================
# SHARED PER-FRAME PIPELINE (identical for static images and camera frames)
# ============================================================================

def process_frame(img, min_area, max_area):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    overlay = img.copy()
    results = []

    # Only red and yellow get full color+shape analysis (mine detection).
    MINE_COLORS = {"red", "yellow"}

    # 1. Build masks for mine colors and the combined obstacle pool
    mine_masks = {}
    obstacle_parts = []
    for color_name, bands in COLOR_BANDS.items():
        mask = build_mask(hsv, bands)
        if color_name in MINE_COLORS:
            mine_masks[color_name] = mask
        else:
            obstacle_parts.append(mask)

    # Combined obstacle mask: all non-mine bands bitwise-OR'd together
    obstacle_mask = obstacle_parts[0]
    for m in obstacle_parts[1:]:
        obstacle_mask = cv2.bitwise_or(obstacle_mask, m)

    # Subtract mine masks so red/yellow pixels don't also appear as obstacles
    for m in mine_masks.values():
        obstacle_mask = cv2.bitwise_and(obstacle_mask, cv2.bitwise_not(m))

    # 2. Analyze mine color masks — classify shape, relabel circles as mines
    for color_name, mask in mine_masks.items():
        for d in analyze(mask, min_area, max_area):
            shape = classify_shape(d["corners"], d["aspect"], d["extent"],
                                   d["solidity"], d["circularity"])
            is_mine = (shape == "circle")
            label = f"mine ({color_name})" if is_mine else f"{color_name} {shape}"
            draw_label(overlay, d, label, color_name, is_mine=is_mine)
            results.append({**d, "color": color_name, "shape": shape,
                            "label": label})

    # 3. Analyze combined obstacle mask — classify shape, label "obstacle {shape}"
    for d in analyze(obstacle_mask, min_area, max_area):
        shape = classify_shape(d["corners"], d["aspect"], d["extent"],
                               d["solidity"], d["circularity"])
        label = f"obstacle {shape}"
        draw_label(overlay, d, label, "obstacle", is_mine=False)
        results.append({**d, "color": "obstacle", "shape": shape,
                        "label": label})

    results = merge_overlapping_boxes(results)

    return overlay, results


def merge_overlapping_boxes(dets, iou_threshold=0.3):
    """Merge detection boxes with IoU above threshold. Keep the larger box."""
    if not dets:
        return dets
    dets = sorted(dets, key=lambda d: d["w"] * d["h"], reverse=True)
    merged = []
    used = set()
    for i, d in enumerate(dets):
        if i in used:
            continue
        box = [d["x"], d["y"], d["x"] + d["w"], d["y"] + d["h"]]
        for j in range(i + 1, len(dets)):
            if j in used:
                continue
            other = dets[j]
            obox = [other["x"], other["y"], other["x"] + other["w"], other["y"] + other["h"]]
            ix1 = max(box[0], obox[0])
            iy1 = max(box[1], obox[1])
            ix2 = min(box[2], obox[2])
            iy2 = min(box[3], obox[3])
            inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
            area_a = d["w"] * d["h"]
            area_b = other["w"] * other["h"]
            union = area_a + area_b - inter
            if union > 0 and inter / union >= iou_threshold:
                used.add(j)
        merged.append(d)
    return merged


def run(image_path, out_path, min_area, max_area):
    img = cv2.imread(image_path)
    if img is None:
        raise SystemExit(f"Could not read image: {image_path}")
    overlay, results = process_frame(img, min_area, max_area)
    cv2.imwrite(out_path, overlay)
    print(f"[CV_UNIVERSAL] {len(results)} objects -> {out_path}")
    for r in results:
        print(f"  {r['label']:<18} corners={r['corners']:<2} "
              f"circ={r['circularity']:.2f} sol={r['solidity']:.2f} "
              f"@ ({r['cx']},{r['cy']})")
    return results


def resolve_video_source(source):
    """Map a user-supplied --camera value to an OpenCV capture target.

    Supported forms (REQ item 1, laptop-side ingestion):
      "0", "1", ...          local USB webcam index
      "rtsp://..."           IP camera RTSP stream
      "http://.../video"     MJPEG over HTTP (IP cam or app)
      "droidcam://IP[:PORT]" DroidCam phone app (default port 4747 -> /video)
    """
    if isinstance(source, str) and source.startswith("droidcam://"):
        rest = source[len("droidcam://"):]
        if ":" not in rest:
            rest += ":4747"
        return f"http://{rest}/video"
    if isinstance(source, str) and source.isdigit():
        return int(source)
    return source


def open_video_capture(source):
    """Open any supported source with a Windows-friendly fallback chain."""
    cap = None
    if isinstance(source, int):
        cap = cv2.VideoCapture(source)
        if not cap.isOpened() or not cap.read()[0]:
            cap.release()
            try:
                cap = cv2.VideoCapture(source, cv2.CAP_DSHOW)
            except Exception:
                cap = cv2.VideoCapture(source)
    else:
        cap = cv2.VideoCapture(source)  # file / rtsp / http-mjpeg
    return cap


def run_camera(camera_source, min_area, max_area, width=640, height=480,
               normalize=True):
    import time
    camera_source = resolve_video_source(camera_source)

    cap = open_video_capture(camera_source)
    if isinstance(camera_source, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera source '{camera_source}'")

    # Tracking disabled until single-frame detection is clean.
    # Re-enable with SEEDED_MIN=3, MISSED_MAX=2 once Problems 1-4 are verified.

    print("[CV_UNIVERSAL] Live camera mode - press Q or ESC to quit.")
    fps_avg = 0.0
    while True:
        t_start = time.time()
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[CV_UNIVERSAL] Empty frame grab, retrying...")
            continue

        work = frame
        gain = 1.0
        if normalize:
            work, gain = apply_exposure_norm(frame)

        overlay, results = process_frame(work, min_area, max_area)

        dt_ms = (time.time() - t_start) * 1000.0
        fps_curr = 1000.0 / dt_ms if dt_ms > 0 else 0.0
        fps_avg = 0.9 * fps_avg + 0.1 * fps_curr if fps_avg > 0 else fps_curr

        hud = f"objects: {len(results)} | norm {'ON' if normalize else 'OFF'} " \
              f"(gain {gain:.2f}) | FPS {fps_avg:.1f}"
        cv2.putText(overlay, hud, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Universal Color+Shape Detector - Live", overlay)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


# ============================================================================
# SELFTEST: renders a ground-truth canvas (all colors x shape types), runs the
# REAL pipeline on it, and checks every label against what was drawn.
# ============================================================================

SELFTEST_COLORS = ["red", "orange", "yellow", "green", "cyan", "blue",
                   "purple", "magenta", "black", "white", "gray"]

SELFTEST_SHAPES = ["triangle", "square", "rectangle", "pentagon",
                   "hexagon", "circle", "star"]

_SELFTEST_MATCH_TOLERANCE_PX = 60.0


def _regular_polygon_points(cx, cy, radius, n_sides, rotation_deg=-90.0):
    pts = []
    for i in range(n_sides):
        ang = math.radians(rotation_deg + i * 360.0 / n_sides)
        pts.append([int(round(cx + radius * math.cos(ang))),
                    int(round(cy + radius * math.sin(ang)))])
    return np.array(pts, dtype=np.int32)


def _star_points(cx, cy, outer_r, inner_ratio=0.45, points=5):
    pts = []
    for i in range(points * 2):
        r = outer_r if i % 2 == 0 else outer_r * inner_ratio
        ang = math.radians(-90.0 + i * 180.0 / points)
        pts.append([int(round(cx + r * math.cos(ang))),
                    int(round(cy + r * math.sin(ang)))])
    return np.array(pts, dtype=np.int32)


def render_selftest_canvas(cols=4, rows=3, cell_w=320, cell_h=310, radius=78):
    """Draws one shape per color cycling through SELFTEST_SHAPES.

    Background is a desaturated blue-gray (S~=51) chosen to fall OUTSIDE every
    COLOR_BANDS entry: achromatic backgrounds would merge black/white/gray
    shapes into one giant background blob, and any saturated hue would share a
    bucket with its objects. S in [41..59] with V in [51..199] escapes all of
    them (colored bands need S>=60; neutral bands need S<=40).
    """
    canvas = np.full((rows * cell_h, cols * cell_w, 3), (160, 132, 128),
                     dtype=np.uint8)
    ground_truth = []

    for idx, color_name in enumerate(SELFTEST_COLORS):
        col = idx % cols
        row = idx // cols
        cx = int(col * cell_w + cell_w / 2)
        cy = int(row * cell_h + cell_h / 2)
        shape = SELFTEST_SHAPES[idx % len(SELFTEST_SHAPES)]
        bgr = DRAW_BGR[color_name]

        if shape == "triangle":
            pts = _regular_polygon_points(cx, cy, radius + 12, 3)
            cv2.fillPoly(canvas, [pts], bgr)
        elif shape == "square":
            half = radius
            cv2.rectangle(canvas, (cx - half, cy - half),
                          (cx + half, cy + half), bgr, -1)
        elif shape == "rectangle":
            # aspect ~2.1 comfortably above the 1.25 square/rectangle split
            cv2.rectangle(canvas, (cx - int(radius * 1.45), cy - int(radius * 0.7)),
                          (cx + int(radius * 1.45), cy + int(radius * 0.7)),
                          bgr, -1)
        elif shape in ("pentagon", "hexagon", "heptagon"):
            n = {"pentagon": 5, "hexagon": 6, "heptagon": 7}[shape]
            pts = _regular_polygon_points(cx, cy, radius + 6, n)
            cv2.fillPoly(canvas, [pts], bgr)
        elif shape == "circle":
            cv2.circle(canvas, (cx, cy), radius, bgr, -1)
        elif shape == "star":
            pts = _star_points(cx, cy, radius + 14)
            cv2.fillPoly(canvas, [pts], bgr)

        ground_truth.append({"color": color_name, "shape": shape,
                             "cx": cx, "cy": cy})

    return canvas, ground_truth


def selftest(verbose=True):
    canvas, ground_truth = render_selftest_canvas()
    overlay, results = process_frame(canvas, min_area=200, max_area=500000)

    matched_gt = set()
    matched_det = set()
    failures = []

    for gi, gt in enumerate(ground_truth):
        best_j = -1
        best_dist = _SELFTEST_MATCH_TOLERANCE_PX
        expected_label = (f"{gt['color']} {gt['shape']}"
                          if gt['color'] in ('red', 'yellow')
                          else f"obstacle {gt['shape']}")

        for j, det in enumerate(results):
            if j in matched_det:
                continue
            dist = math.hypot(det["cx"] - gt["cx"], det["cy"] - gt["cy"])
            if dist < best_dist:
                best_dist = dist
                best_j = j

        if best_j < 0:
            failures.append(f"MISS  {expected_label} at "
                            f"({gt['cx']},{gt['cy']}): not detected")
            continue

        got_label = results[best_j]["label"]
        if got_label == expected_label:
            matched_gt.add(gi)
            matched_det.add(best_j)
            if verbose:
                print(f"  PASS  {expected_label:<18} @ ({gt['cx']:>4},{gt['cy']:>3})")
        else:
            failures.append(
                f"WRONG {expected_label} @ ({gt['cx']},{gt['cy']}): "
                f"classified as '{got_label}' (corners={results[best_j]['corners']}, "
                f"circ={results[best_j]['circularity']}, "
                f"sol={results[best_j]['solidity']})")
            matched_det.add(best_j)  # consumed even if mislabeled

    extras = [f"{d['label']} @ ({d['cx']},{d['cy']})"
              for j, d in enumerate(results) if j not in matched_det]
    for e in extras:
        failures.append(f"EXTRA unexpected detection: {e}")

    total = len(ground_truth)
    passed = len(matched_gt)
    if verbose:
        print("-" * 64)
        for f_line in failures:
            print(f"  FAIL  {f_line}")
        print("-" * 64)
        status = "PASS" if not failures else "FAIL"
        print(f"[SELFTEST] {status}: {passed}/{total} correct, "
              f"{len(extras)} extra detections")

    cv2.imwrite("universal_selftest.png", overlay)
    if verbose and not failures:
        print("[SELFTEST] Annotated canvas saved to universal_selftest.png")

    return not failures


# ============================================================================
# CLI
# ============================================================================

def main():
    ap = argparse.ArgumentParser(
        description="Universal full-hue-wheel color + corner-count shape detector")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--image", help="Path to a static image file.")
    src.add_argument("--camera", type=str, nargs="?", const="0",
                     help="Live source: USB index (0,1), rtsp:// IP cam, http MJPEG URL, "
                          "or droidcam://IP[:PORT] (default port 4747).")
    ap.add_argument("--selftest", action="store_true",
                    help="Run the synthetic ground-truth validation and exit.")
    ap.add_argument("--out", default="universal_result.png",
                    help="Output path (--image mode only).")
    ap.add_argument("--min-area", type=float, default=600)
    ap.add_argument("--max-area", type=float, default=500000)
    ap.add_argument("--width", type=int, default=640,
                    help="Camera capture width (--camera mode only).")
    ap.add_argument("--height", type=int, default=480,
                    help="Camera capture height (--camera mode only).")
    ap.add_argument("--no-norm", action="store_true",
                    help="Disable exposure normalization (--camera mode only).")
    args = ap.parse_args()

    chosen = sum(bool(x) for x in (args.image, args.camera is not None,
                                   args.selftest))
    if chosen == 0:
        ap.error("one of --image / --camera / --selftest is required")
    if chosen > 1:
        ap.error("--image, --camera and --selftest are mutually exclusive")

    if args.selftest:
        ok = selftest()
        sys.exit(0 if ok else 1)
    if args.image:
        run(args.image, args.out, args.min_area, args.max_area)
    else:
        run_camera(args.camera, args.min_area, args.max_area,
                   args.width, args.height, normalize=not args.no_norm)


if __name__ == "__main__":
    main()
