"""
detection_test.py  —  WRO 2025  |  Pure OpenCV Detection (No Model, No Coral)
==============================================================================
Replaces the EdgeTPU model entirely with OpenCV HSV colour thresholding.
Expected FPS: 40-60 FPS on Pi (vs 8-9 FPS with the SSD MobileNetV2 FPN model).

NO EdgeTPU needed. NO Coral USB needed. NO pycoral needed.
Works over SSH. No display. Terminal output only.

STEP 1: Run hsv_calibrate.py first to tune HSV values for your venue lighting.
STEP 2: Paste the saved values into the CONFIG section below.
STEP 3: Run this file.

RUN:
    source /home/pi/coral-py39-env/bin/activate
    python3 /home/pi/WRO_2025_PI/detection_test.py

Ctrl+C to stop.
"""

import cv2
import numpy as np
import time
from collections import deque

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG  — paste your calibrated values from hsv_calibrate.py here
# ─────────────────────────────────────────────────────────────────────────────
CAM_INDEX = 1      # change to 0/1/2 to match your camera
CAP_W     = 320    # capture resolution — 320x240 is fast and enough
CAP_H     = 240

# ── RED HSV (two ranges — red wraps around H=0 and H=180 in OpenCV) ──────────
# Range 1: covers H 0-10 (warm red)
# Range 2: covers H 170-180 (cool red)
# TUNE THESE with hsv_calibrate.py at your venue
RED_LOWER_1   = np.array([0,   120,  60], dtype=np.uint8)
RED_UPPER_1   = np.array([10,  255, 255], dtype=np.uint8)
RED_LOWER_2   = np.array([170, 120,  60], dtype=np.uint8)
RED_UPPER_2   = np.array([180, 255, 255], dtype=np.uint8)

# ── GREEN HSV ────────────────────────────────────────────────────────────────
GREEN_LOWER   = np.array([40,  80,  40], dtype=np.uint8)
GREEN_UPPER   = np.array([90, 255, 200], dtype=np.uint8)

# ── PINK / MAGENTA HSV ───────────────────────────────────────────────────────
PINK_LOWER    = np.array([135,  70,  60], dtype=np.uint8)
PINK_UPPER    = np.array([170, 255, 255], dtype=np.uint8)

# ── Detection thresholds ──────────────────────────────────────────────────────
# Minimum contour area in pixels to count as a valid detection.
# At 320x240: pillar occupies ~500-3000 px². Pink wall: ~3000-15000 px².
# Lower = more sensitive but more noise. Raise if you get false positives.
RED_MIN_AREA   = 400
GREEN_MIN_AREA = 400
PINK_MIN_AREA  = 800

# ── Morphology settings ───────────────────────────────────────────────────────
# Morphological clean-up removes noise from the mask.
# MORPH_KERNEL_SIZE: 3 = fast+slight, 5 = slower+better noise removal
MORPH_KERNEL_SIZE = 3

# ── Temporal smoother ─────────────────────────────────────────────────────────
# N frames majority vote before confirming a detection.
# 1 = instant (no smoothing), 2-3 = stable, 4+ = laggy
SMOOTH_N = 2

# ── Terminal print rate ───────────────────────────────────────────────────────
PRINT_INTERVAL = 0.15   # seconds between terminal prints


# ─────────────────────────────────────────────────────────────────────────────
#  TEMPORAL SMOOTHER — independent per colour
# ─────────────────────────────────────────────────────────────────────────────
class FrameSmoother:
    def __init__(self, n=SMOOTH_N):
        self.n    = max(1, n)
        self._buf = {}

    def update(self, name: str, detected: bool) -> bool:
        if name not in self._buf:
            self._buf[name] = deque(maxlen=self.n)
        self._buf[name].append(detected)
        return sum(self._buf[name]) > (len(self._buf[name]) // 2)


# ─────────────────────────────────────────────────────────────────────────────
#  COLOUR DETECTION ENGINE
# ─────────────────────────────────────────────────────────────────────────────
class ColourDetector:
    """
    Pure OpenCV HSV detector for red, green, pink.
    All three colours detected independently — no cancellation.

    Returns detections as a list of dicts:
        { colour, cx, cy, area, x1, y1, x2, y2 }
    sorted by area descending (closest/largest first).
    """

    def __init__(self):
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
        )

    def _mask_to_detections(self, mask, colour_name, min_area):
        """Convert a binary mask to a list of detection dicts."""
        # Single morphology pass: removes salt-pepper noise
        clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)

        contours, _ = cv2.findContours(
            clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dets = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            x, y, w, h = cv2.boundingRect(c)
            dets.append({
                "colour": colour_name,
                "cx"    : cx,
                "cy"    : cy,
                "area"  : area,
                "x1"    : x,   "y1": y,
                "x2"    : x+w, "y2": y+h,
            })

        dets.sort(key=lambda d: d["area"], reverse=True)
        return dets

    def detect(self, frame_bgr):
        """
        Run detection on one frame.
        Returns dict: { "red": [...], "green": [...], "pink": [...] }
        Each list is sorted largest-first. Lists are independent.
        """
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # ── RED: OR two hue ranges ────────────────────────────────────────────
        # Red wraps around H=0 in OpenCV (0-179 scale).
        # Range 1 covers 0-10, Range 2 covers 170-180. Both OR-ed together.
        m_r1 = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1)
        m_r2 = cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
        mask_red = cv2.bitwise_or(m_r1, m_r2)

        # ── GREEN: single range ───────────────────────────────────────────────
        mask_green = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

        # ── PINK/MAGENTA: single range ────────────────────────────────────────
        mask_pink = cv2.inRange(hsv, PINK_LOWER, PINK_UPPER)

        return {
            "red"  : self._mask_to_detections(mask_red,   "red",   RED_MIN_AREA),
            "green": self._mask_to_detections(mask_green, "green", GREEN_MIN_AREA),
            "pink" : self._mask_to_detections(mask_pink,  "pink",  PINK_MIN_AREA),
        }


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("=" * 65)
    print("  WRO 2025  —  Pure OpenCV Detection (No Model / No Coral)")
    print("=" * 65)
    print(f"  Camera     : index {CAM_INDEX}  target {CAP_W}x{CAP_H}")
    print(f"  Smooth N   : {SMOOTH_N}")
    print(f"  Min areas  : RED={RED_MIN_AREA}  GREEN={GREEN_MIN_AREA}  PINK={PINK_MIN_AREA}")
    print()
    print("  TIP: Run hsv_calibrate.py first to tune HSV values!")
    print("  Ctrl+C to stop")
    print("=" * 65)
    print()

    # ── Camera ────────────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"[Camera] FAILED to open index {CAM_INDEX}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,   CAP_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  CAP_H)
    cap.set(cv2.CAP_PROP_FPS,           120)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # manual exposure
    cap.set(cv2.CAP_PROP_EXPOSURE,      -6)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)       # always get latest frame

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] Opened — requested:{CAP_W}x{CAP_H}  actual:{actual_w}x{actual_h}")
    print()

    # ── Detection engine + smoother ───────────────────────────────────────────
    detector = ColourDetector()
    smoother = FrameSmoother(n=SMOOTH_N)

    # ── Terminal header ───────────────────────────────────────────────────────
    # Timing columns: CAP=capture, HSV=colour space convert, DET=detection,
    # TOT=full loop time
    print(f"{'FPS':>5} {'AVG':>5} | "
          f"{'CAP':>6} {'HSV':>6} {'DET':>6} {'TOT':>6} | "
          f"{'COLOUR':<8} {'CX':>5} {'CY':>5} {'AREA':>6}")
    print("─" * 75)

    fps_history  = deque(maxlen=60)
    t_prev       = time.perf_counter()
    t_last_print = time.time()
    frame_count  = 0

    try:
        while True:
            t0 = time.perf_counter()

            # ── CAPTURE ───────────────────────────────────────────────────────
            # grab() discards stale buffer frames, retrieve() decodes only one.
            # This prevents reading a frame that's 100ms old.
            cap.grab()
            ret, frame_bgr = cap.retrieve()
            if not ret:
                continue

            frame_count += 1
            t_cap = (time.perf_counter() - t0) * 1000

            # ── COLOUR DETECTION ──────────────────────────────────────────────
            t1 = time.perf_counter()
            results = detector.detect(frame_bgr)
            t_det = (time.perf_counter() - t1) * 1000

            # ── TEMPORAL SMOOTHER — independent per colour ────────────────────
            # Each colour's smoother is completely independent.
            # Red being detected does NOT affect green's smoother.
            red_confirmed   = smoother.update("red",   len(results["red"])   > 0)
            green_confirmed = smoother.update("green", len(results["green"]) > 0)
            pink_confirmed  = smoother.update("pink",  len(results["pink"])  > 0)

            # Clear lists if smoother says not confirmed yet
            if not red_confirmed:
                results["red"]   = []
            if not green_confirmed:
                results["green"] = []
            if not pink_confirmed:
                results["pink"]  = []

            # ── FPS + TIMING ──────────────────────────────────────────────────
            t_now = time.perf_counter()
            t_tot = (t_now - t0) * 1000
            fps   = 1000.0 / max(t_tot, 0.1)
            fps_history.append(fps)
            avg_fps = sum(fps_history) / len(fps_history)

            # ── TERMINAL PRINT ────────────────────────────────────────────────
            now = time.time()
            if now - t_last_print >= PRINT_INTERVAL:
                t_last_print = now

                # Build rows — ALL detections from ALL colours independently
                rows = []
                # Priority order: PINK first (parking wall), then GREEN, then RED
                for colour in ("pink", "green", "red"):
                    for i, d in enumerate(results[colour]):
                        lbl = colour.upper() if i == 0 else f"{colour.upper()}#{i+1}"
                        rows.append((lbl, d["cx"], d["cy"], d["area"]))

                timing = (f"{fps:5.1f} {avg_fps:5.1f} | "
                          f"{t_cap:5.1f}ms {0:5.1f}ms "
                          f"{t_det:5.1f}ms {t_tot:5.1f}ms |")

                if not rows:
                    print(f"{timing} nothing detected")
                else:
                    for i, (lbl, cx_, cy_, area) in enumerate(rows):
                        pfx = timing if i == 0 else " " * len(timing)
                        print(f"{pfx} {lbl:<8} {cx_:5.0f} {cy_:5.0f} {area:6.0f}")

            t_prev = t_now

    except KeyboardInterrupt:
        print()
        print("─" * 65)
        print("[Test] Stopped.")
    finally:
        cap.release()
        if fps_history:
            print(f"[Test] Frames: {frame_count}")
            print(f"[Test] FPS  avg:{sum(fps_history)/len(fps_history):.1f}"
                  f"  min:{min(fps_history):.1f}"
                  f"  max:{max(fps_history):.1f}")
        print("[Test] Done.")


if __name__ == "__main__":
    main()
