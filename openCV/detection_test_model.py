"""
detection_opencv_clean.py  —  WRO 2025  |  OpenCV HSV  (Clean Version)
=======================================================================
Pure OpenCV HSV detection. Clean minimal camera window.
Detailed terminal output with priority.

The window shows only:
  - Bounding boxes + colour labels
  - Centroid dots
  - Priority badge (P1, P2, P3)
  - FPS top-left
  Nothing else — no clutter.

Terminal shows:
  - FPS + avg
  - Priority + colour + cx + cy + area + proximity score

RUN:
    export DISPLAY=:0
    export QT_QPA_PLATFORM=xcb
    source /home/pi/coral-py39-env/bin/activate
    python3 /home/pi/WRO_2025_PI/detection_opencv_clean.py

Controls:
    Q or ESC → quit
"""

import os
os.environ["DISPLAY"]         = ":0"
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import numpy as np
import time
from collections import deque

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────────────────
CAM_INDEX    = 1
CAP_W, CAP_H = 640, 360

# ── PASTE YOUR CALIBRATED VALUES HERE ────────────────────────────────────────
RED_LOWER_1   = np.array([0,   120,  60], dtype=np.uint8)
RED_UPPER_1   = np.array([10,  255, 255], dtype=np.uint8)
RED_LOWER_2   = np.array([170, 120,  60], dtype=np.uint8)
RED_UPPER_2   = np.array([180, 255, 255], dtype=np.uint8)

GREEN_LOWER   = np.array([40,  80,  40], dtype=np.uint8)
GREEN_UPPER   = np.array([90, 255, 200], dtype=np.uint8)

PINK_LOWER    = np.array([135,  70,  60], dtype=np.uint8)
PINK_UPPER    = np.array([175, 255, 255], dtype=np.uint8)

# Minimum contour area per colour (px²)
RED_MIN_AREA   = 400
GREEN_MIN_AREA = 400
PINK_MIN_AREA  = 800

SMOOTH_N       = 2
PRINT_INTERVAL = 0.12   # terminal print rate (seconds)

# Colours (BGR)
CLR = {
    "red"  : (0,   0,   220),
    "green": (0,   210,   0),
    "pink" : (220,   0, 220),
}
C_WHITE  = (255, 255, 255)
C_BLACK  = (0,     0,   0)
C_GREY   = (90,   90,  90)
C_YELLOW = (0,   210, 210)
C_CYAN   = (210, 210,   0)


# ─────────────────────────────────────────────────────────────────────────────
#  SMOOTHER
# ─────────────────────────────────────────────────────────────────────────────
class FrameSmoother:
    def __init__(self, n):
        self.n = max(1, n)
        self._buf = {}

    def update(self, name, detected):
        if name not in self._buf:
            self._buf[name] = deque(maxlen=self.n)
        self._buf[name].append(detected)
        return sum(self._buf[name]) > (len(self._buf[name]) // 2)


# ─────────────────────────────────────────────────────────────────────────────
#  DETECTION
# ─────────────────────────────────────────────────────────────────────────────
def get_best_blob(mask, min_area):
    """
    Returns the largest contour above min_area as a dict, or None.
    Dict: { cx, cy, area, x1, y1, x2, y2, contour }
    """
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    conts, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
    if not conts:
        return None

    best = max(conts, key=cv2.contourArea)
    area = cv2.contourArea(best)
    if area < min_area:
        return None

    M = cv2.moments(best)
    if M["m00"] == 0:
        return None

    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    x, y, w, h = cv2.boundingRect(best)
    return {
        "cx": cx, "cy": cy, "area": area,
        "x1": x,  "y1": y,
        "x2": x+w, "y2": y+h,
        "contour": best,
    }


def detect_all(frame_bgr):
    """Returns dict with best detection per colour (or None)."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # Red: OR two hue ranges
    m_r = cv2.bitwise_or(
        cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1),
        cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2),
    )
    m_g = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
    m_p = cv2.inRange(hsv, PINK_LOWER,  PINK_UPPER)

    return {
        "red"  : get_best_blob(m_r, RED_MIN_AREA),
        "green": get_best_blob(m_g, GREEN_MIN_AREA),
        "pink" : get_best_blob(m_p, PINK_MIN_AREA),
    }


# ─────────────────────────────────────────────────────────────────────────────
#  PRIORITY
#  Pink → always P1 (parking wall, highest priority)
#  Red / Green → ranked by proximity (largest area + lowest cy = closest)
# ─────────────────────────────────────────────────────────────────────────────
def assign_priority(dets, frame_h):
    """
    dets: dict { colour: det_dict | None }
    Returns flat list of dets with 'colour' and 'priority' added,
    sorted P1 first.
    """
    flat = []
    for colour, d in dets.items():
        if d is not None:
            d["colour"] = colour
            flat.append(d)

    if not flat:
        return []

    max_area = max(d["area"] for d in flat) or 1

    for d in flat:
        area_score = d["area"] / max_area         # 0-1, larger = closer
        cy_score   = d["cy"]   / frame_h          # 0-1, lower = closer
        prox       = 0.7 * area_score + 0.3 * cy_score
        if d["colour"] == "pink":
            prox += 2.0                            # pink always wins priority
        d["prox"] = prox

    flat.sort(key=lambda d: d["prox"], reverse=True)
    for i, d in enumerate(flat):
        d["priority"] = i + 1

    return flat


# ─────────────────────────────────────────────────────────────────────────────
#  DRAW — MINIMAL CLEAN WINDOW
# ─────────────────────────────────────────────────────────────────────────────
def draw_clean(canvas, dets_by_priority, fps, avg):
    H, W = canvas.shape[:2]

    # ── FPS (top-left only, small) ────────────────────────────────
    cv2.putText(canvas, f"FPS {fps:.1f}  avg {avg:.1f}",
                (6, 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, C_WHITE, 1, cv2.LINE_AA)

    # ── y=240 trigger line ────────────────────────────────────────
    cv2.line(canvas, (0, 240), (W, 240), (55, 55, 55), 1)

    # ── Each detection ────────────────────────────────────────────
    for d in dets_by_priority:
        col  = CLR[d["colour"]]
        cx   = int(d["cx"]); cy = int(d["cy"])
        x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
        prio = d["priority"]

        # Clean bounding box (thick = higher priority)
        thickness = max(1, 4 - prio)   # P1=3px, P2=2px, P3=1px
        cv2.rectangle(canvas, (x1, y1), (x2, y2), col, thickness)

        # Label: priority + colour
        label = f"P{prio} {d['colour'].upper()}"
        (tw, th), bl = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
        ly = max(y1 - th - bl - 3, 0)
        cv2.rectangle(canvas, (x1, ly), (x1+tw+5, y1), col, -1)
        cv2.putText(canvas, label, (x1+3, y1-bl-2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, C_BLACK, 1, cv2.LINE_AA)

        # Area label below box
        cv2.putText(canvas, f"A:{d['area']:.0f}",
                    (x1, y2 + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, col, 1, cv2.LINE_AA)

        # Centroid dot (clean, no crosshair clutter)
        cv2.circle(canvas, (cx, cy), 5, col,    -1)
        cv2.circle(canvas, (cx, cy), 5, C_WHITE,  1)

        # Centroid coordinates
        cv2.putText(canvas, f"({cx},{cy})", (cx+8, cy-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)


# ─────────────────────────────────────────────────────────────────────────────
#  TERMINAL PRINT
# ─────────────────────────────────────────────────────────────────────────────
def print_terminal(fps, avg, dets_by_priority):
    fps_str = f"{fps:5.1f} {avg:5.1f} |"
    if not dets_by_priority:
        print(f"{fps_str}  —   nothing detected")
        return

    for i, d in enumerate(dets_by_priority):
        pfx   = fps_str if i == 0 else " " * len(fps_str)
        prox  = f"{d['prox']:.3f}"
        print(f"{pfx}  P{d['priority']}  "
              f"{d['colour'].upper():<7}  "
              f"cx:{int(d['cx']):4d}  cy:{int(d['cy']):4d}  "
              f"area:{d['area']:6.0f}  "
              f"prox:{prox}  "
              f"{'← HIGHEST PRIORITY' if d['priority']==1 else ''}")


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("=" * 65)
    print("  WRO 2025  —  OpenCV HSV Detection  (Clean Version)")
    print("=" * 65)
    print(f"  Camera  : index {CAM_INDEX}  {CAP_W}x{CAP_H}")
    print(f"  Smooth N: {SMOOTH_N}")
    print("  Priority: PINK always P1 | RED+GREEN by proximity (area+cy)")
    print("  Q or ESC to quit")
    print("=" * 65)
    print()
    print(f"{'FPS':>5} {'AVG':>5} |  PRIO  COLOUR   CX    CY    AREA    PROX")
    print("─" * 70)

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Camera {CAM_INDEX} not found.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,   CAP_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  CAP_H)
    cap.set(cv2.CAP_PROP_FPS,           120)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE,      -6)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)
    print(f"[Camera] {int(cap.get(3))}x{int(cap.get(4))}")

    WIN = "WRO 2025 Clean Detection  |  Q to quit"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, CAP_W, CAP_H)

    smoother     = FrameSmoother(SMOOTH_N)
    fps_hist     = deque(maxlen=30)
    t_prev       = time.perf_counter()
    t_last_print = time.time()
    frame_count  = 0

    try:
        while True:
            cap.grab()
            ret, frame_bgr = cap.retrieve()
            if not ret:
                continue

            frame_count += 1
            H, W   = frame_bgr.shape[:2]
            canvas  = frame_bgr.copy()

            # Detect
            raw = detect_all(frame_bgr)

            # Smooth independently
            for colour in ("red", "green", "pink"):
                if not smoother.update(colour, raw[colour] is not None):
                    raw[colour] = None

            # Priority
            dets = assign_priority(raw, H)

            # FPS
            t_now  = time.perf_counter()
            fps    = 1.0 / max(t_now - t_prev, 1e-6)
            t_prev = t_now
            fps_hist.append(fps)
            avg = sum(fps_hist) / len(fps_hist)

            # Draw clean window
            draw_clean(canvas, dets, fps, avg)
            cv2.imshow(WIN, canvas)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break

            # Terminal
            now = time.time()
            if now - t_last_print >= PRINT_INTERVAL:
                t_last_print = now
                print_terminal(fps, avg, dets)

    except KeyboardInterrupt:
        print("\n[Stopped]")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"[Done] frames:{frame_count}")
        if fps_hist:
            print(f"[FPS]  avg:{sum(fps_hist)/len(fps_hist):.1f}"
                  f"  min:{min(fps_hist):.1f}"
                  f"  max:{max(fps_hist):.1f}")


if __name__ == "__main__":
    main()