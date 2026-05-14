"""
detection_opencv_fast.py  —  WRO 2025  |  Pure OpenCV HSV  (HIGH FPS)
======================================================================
No model. No Coral. No EdgeTPU needed.
All 3 colours detected via HSV thresholding only.
Expected FPS: 30-60 on Pi.

Priority column printed based on:
  1. Closest to camera  (largest area = closest)
  2. Lowest cy value    (higher in frame = further away, lower = closer)

RUN:
    export DISPLAY=:0
    export QT_QPA_PLATFORM=xcb
    source /home/pi/coral-py39-env/bin/activate
    python3 /home/pi/WRO_2025_PI/detection_opencv_fast.py

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
#  CONFIG — EDIT THESE
# ─────────────────────────────────────────────────────────────────────────────
CAM_INDEX = 1       # ← run camera finder first to confirm this
CAP_W, CAP_H = 640, 360

# ── PASTE YOUR CALIBRATED VALUES HERE (from hsv_calibrate.py) ────────────────
# RED — two ranges because red wraps around H=0 in OpenCV
RED_LOWER_1   = np.array([0,   120,  60], dtype=np.uint8)
RED_UPPER_1   = np.array([10,  255, 255], dtype=np.uint8)
RED_LOWER_2   = np.array([170, 120,  60], dtype=np.uint8)
RED_UPPER_2   = np.array([180, 255, 255], dtype=np.uint8)

# GREEN — single range
GREEN_LOWER   = np.array([40,  80,  40], dtype=np.uint8)
GREEN_UPPER   = np.array([90, 255, 200], dtype=np.uint8)

# PINK / MAGENTA — single range
PINK_LOWER    = np.array([135,  70,  60], dtype=np.uint8)
PINK_UPPER    = np.array([175, 255, 255], dtype=np.uint8)

# ── Detection thresholds ──────────────────────────────────────────────────────
RED_MIN_AREA   = 400    # px² — raise if too many false positives
GREEN_MIN_AREA = 400
PINK_MIN_AREA  = 800    # pink wall is always large

# ── Smoothing ─────────────────────────────────────────────────────────────────
SMOOTH_N = 2   # frames majority vote (1 = instant, 3 = stable)

# ── Terminal print rate ───────────────────────────────────────────────────────
PRINT_INTERVAL = 0.12   # seconds — set to 0 for every frame

# ── BGR colours for drawing ───────────────────────────────────────────────────
C_RED    = (0,   0,   220)
C_GREEN  = (0,   210,   0)
C_PINK   = (220,   0, 220)
C_WHITE  = (255, 255, 255)
C_BLACK  = (0,     0,   0)
C_GREY   = (100, 100, 100)
C_YELLOW = (0,   210, 210)
C_CYAN   = (210, 210,   0)
C_ORANGE = (0,   140, 255)


# ─────────────────────────────────────────────────────────────────────────────
#  TEMPORAL SMOOTHER
# ─────────────────────────────────────────────────────────────────────────────
class FrameSmoother:
    def __init__(self, n=SMOOTH_N):
        self.n = max(1, n)
        self._buf = {}

    def update(self, name: str, detected: bool) -> bool:
        if name not in self._buf:
            self._buf[name] = deque(maxlen=self.n)
        self._buf[name].append(detected)
        return sum(self._buf[name]) > (len(self._buf[name]) // 2)


# ─────────────────────────────────────────────────────────────────────────────
#  COLOUR DETECTOR
# ─────────────────────────────────────────────────────────────────────────────
class ColourDetector:
    """
    Detects red, green, pink via HSV thresholding.
    Returns list of detection dicts per colour, sorted by area descending.
    """
    def __init__(self):
        self._k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    def _mask_to_dets(self, mask, name, min_area, frame_shape):
        clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._k)
        conts, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        dets = []
        for c in conts:
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
                "colour"  : name,
                "cx"      : cx,    "cy"   : cy,
                "area"    : area,
                "x1"      : x,     "y1"   : y,
                "x2"      : x + w, "y2"   : y + h,
                "contour" : c,
            })
        dets.sort(key=lambda d: d["area"], reverse=True)
        return dets

    def detect(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # RED — OR two ranges
        m_r = cv2.bitwise_or(
            cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1),
            cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
        )
        # GREEN — single range
        m_g = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        # PINK — single range
        m_p = cv2.inRange(hsv, PINK_LOWER,  PINK_UPPER)

        sh = frame_bgr.shape
        return {
            "red"  : self._mask_to_dets(m_r, "red",   RED_MIN_AREA,   sh),
            "green": self._mask_to_dets(m_g, "green", GREEN_MIN_AREA, sh),
            "pink" : self._mask_to_dets(m_p, "pink",  PINK_MIN_AREA,  sh),
        }


# ─────────────────────────────────────────────────────────────────────────────
#  PRIORITY LOGIC
#  Priority = who should the robot react to first?
#  Based on:
#    1. Largest area (closest to camera)
#    2. Lowest cy   (bottom of frame = very close)
#    3. Pink always gets a BASE SCORE boost (parking wall = highest priority)
#
#  Lower priority number = act first
# ─────────────────────────────────────────────────────────────────────────────
def compute_priority(all_dets, frame_h):
    """
    all_dets: flat list of detection dicts (any colour).
    Returns same list with 'priority' key added, sorted P1 first.
    """
    if not all_dets:
        return []

    max_area = max(d["area"] for d in all_dets) or 1
    for d in all_dets:
        # Normalised area score: 0-1 (bigger = closer = higher priority)
        area_score = d["area"] / max_area

        # Normalised y score: 0-1 (lower in frame = closer)
        cy_score   = d["cy"] / frame_h

        # Combined proximity score (higher = closer = higher priority)
        proximity  = (0.7 * area_score) + (0.3 * cy_score)

        # Pink wall gets automatic top priority (parking logic)
        if d["colour"] == "pink":
            proximity += 1.0

        d["proximity"] = proximity

    # Sort descending by proximity → assign P1, P2, P3...
    all_dets.sort(key=lambda d: d["proximity"], reverse=True)
    for i, d in enumerate(all_dets):
        d["priority"] = i + 1

    return all_dets


# ─────────────────────────────────────────────────────────────────────────────
#  DRAW HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def colour_bgr(name):
    return {"red": C_RED, "green": C_GREEN, "pink": C_PINK}.get(name, C_WHITE)


def draw_detection(canvas, d):
    """Draw bbox + contour + centroid + priority badge."""
    col = colour_bgr(d["colour"])
    x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
    cx, cy = int(d["cx"]), int(d["cy"])
    prio   = d.get("priority", "?")

    # Semi-transparent fill
    ov = canvas.copy()
    cv2.drawContours(ov, [d["contour"]], -1, col, -1)
    cv2.addWeighted(ov, 0.15, canvas, 0.85, 0, canvas)

    # Bounding box
    cv2.rectangle(canvas, (x1, y1), (x2, y2), col, 2)

    # Label background + text
    label = f"P{prio} {d['colour'].upper()}  A:{d['area']:.0f}"
    (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
    ly = max(y1 - th - bl - 4, 0)
    cv2.rectangle(canvas, (x1, ly), (x1 + tw + 6, y1), col, -1)
    cv2.putText(canvas, label, (x1 + 3, y1 - bl - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, C_BLACK, 1, cv2.LINE_AA)

    # Centroid dot + crosshair
    cv2.circle(canvas, (cx, cy), 6, col, -1)
    cv2.circle(canvas, (cx, cy), 6, C_WHITE, 1)
    cv2.drawMarker(canvas, (cx, cy), C_WHITE,
                   cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)

    # Centroid coordinates
    cv2.putText(canvas, f"({cx},{cy})", (cx + 10, cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, col, 1, cv2.LINE_AA)

    # Priority badge (circle top-right of bbox)
    badge_c = (x2 - 12, y1 + 12)
    cv2.circle(canvas, badge_c, 12, col, -1)
    cv2.circle(canvas, badge_c, 12, C_WHITE, 1)
    cv2.putText(canvas, str(prio), (badge_c[0] - 5, badge_c[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_BLACK, 1, cv2.LINE_AA)


def draw_fps_panel(canvas, fps, avg, hist):
    ov = canvas.copy()
    cv2.rectangle(ov, (0, 0), (220, 92), C_BLACK, -1)
    cv2.addWeighted(ov, 0.55, canvas, 0.45, 0, canvas)
    cv2.putText(canvas, f"FPS {fps:5.1f}   avg {avg:5.1f}",
                (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, C_WHITE, 1, cv2.LINE_AA)
    bw = max(1, 180 // max(len(hist), 1))
    mv = max(max(hist), 1)
    gx, gy = 6, 28
    for i, v in enumerate(hist):
        bh = int((v / mv) * 32)
        c  = C_GREEN if v >= 25 else C_YELLOW if v >= 15 else C_RED
        cv2.rectangle(canvas,
                      (gx + i*bw, gy + 32 - bh),
                      (gx + i*bw + bw - 1, gy + 32), c, -1)
    cv2.rectangle(canvas, (gx-1, gy-1), (gx+181, gy+33), C_WHITE, 1)
    cv2.putText(canvas, f"min {min(hist):.0f}  max {max(hist):.0f}",
                (6, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.40, C_CYAN, 1, cv2.LINE_AA)


def draw_status_panel(canvas, dets_by_priority, W):
    """Top-right panel — one line per detection in priority order."""
    lines = [("── PRIORITY LIST ──", C_YELLOW)]
    if not dets_by_priority:
        lines.append(("  nothing detected", C_GREY))
    else:
        for d in dets_by_priority:
            col  = colour_bgr(d["colour"])
            txt  = (f"  P{d['priority']}  {d['colour'].upper():<6}"
                    f"  A:{d['area']:5.0f}  ({int(d['cx'])},{int(d['cy'])})")
            lines.append((txt, col))
    lines.append(("──────────────────────", C_GREY))
    lines.append(("  Q / ESC = quit",       (80, 80, 80)))

    lh = 18; pw = 280; px = W - pw - 5; py = 5
    ov = canvas.copy()
    cv2.rectangle(ov, (px-4, py),
                  (px+pw, py + len(lines)*lh + 8), C_BLACK, -1)
    cv2.addWeighted(ov, 0.55, canvas, 0.45, 0, canvas)
    for i, (txt, col) in enumerate(lines):
        cv2.putText(canvas, txt, (px, py + 16 + i*lh),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("=" * 65)
    print("  WRO 2025  —  Pure OpenCV HSV Detection  (HIGH FPS)")
    print("=" * 65)
    print(f"  Camera  : index {CAM_INDEX}  {CAP_W}x{CAP_H}")
    print(f"  Smooth N: {SMOOTH_N}")
    print(f"  Min area: RED={RED_MIN_AREA}  GREEN={GREEN_MIN_AREA}  PINK={PINK_MIN_AREA}")
    print("  No Coral needed. No model. Pure OpenCV.")
    print("  Q or ESC to quit.")
    print("=" * 65)
    print()

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Camera index {CAM_INDEX} not found.")
        print("Run: python3 -c \"import cv2; [print(i,'FOUND') "
              "if cv2.VideoCapture(i).isOpened() else None for i in range(6)]\"")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,   CAP_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  CAP_H)
    cap.set(cv2.CAP_PROP_FPS,           120)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE,      -6)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)
    print(f"[Camera] {int(cap.get(3))}x{int(cap.get(4))}")

    WIN = "WRO 2025 OpenCV Detection  |  Q to quit"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, CAP_W, CAP_H)

    detector = ColourDetector()
    smoother = FrameSmoother(n=SMOOTH_N)

    # Terminal header
    print()
    print(f"{'FPS':>5} {'AVG':>5} | "
          f"{'PRIO':>4}  {'COLOUR':<7}  {'CX':>5}  {'CY':>5}  "
          f"{'AREA':>6}  {'DIST_SCORE':>10}")
    print("─" * 65)

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
            H, W = frame_bgr.shape[:2]
            canvas = frame_bgr.copy()

            # Guide lines
            cv2.line(canvas, (W//2, 0), (W//2, H), (50,50,50), 1)
            cv2.line(canvas, (0, 240), (W, 240), (50,50,50), 1)
            cv2.putText(canvas, "y=240", (4, 236),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (70,70,70), 1)

            # ── Detect ────────────────────────────────────────────
            results   = detector.detect(frame_bgr)

            # ── Smoother (independent per colour) ─────────────────
            if not smoother.update("red",   len(results["red"])   > 0):
                results["red"]   = []
            if not smoother.update("green", len(results["green"]) > 0):
                results["green"] = []
            if not smoother.update("pink",  len(results["pink"])  > 0):
                results["pink"]  = []

            # ── Flatten + compute priority ─────────────────────────
            # Take the best (largest) detection per colour
            flat = []
            for colour in ("red", "green", "pink"):
                if results[colour]:
                    flat.append(results[colour][0])   # largest blob per colour

            dets_by_priority = compute_priority(flat, H)

            # ── Draw all detections ───────────────────────────────
            for d in dets_by_priority:
                draw_detection(canvas, d)

            # ── FPS ───────────────────────────────────────────────
            t_now = time.perf_counter()
            fps   = 1.0 / max(t_now - t_prev, 1e-6)
            t_prev = t_now
            fps_hist.append(fps)
            avg = sum(fps_hist) / len(fps_hist)

            draw_fps_panel(canvas, fps, avg, list(fps_hist))
            draw_status_panel(canvas, dets_by_priority, W)

            cv2.imshow(WIN, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break

            # ── Terminal print ────────────────────────────────────
            now = time.time()
            if now - t_last_print >= PRINT_INTERVAL:
                t_last_print = now
                fps_str = f"{fps:5.1f} {avg:5.1f} |"

                if not dets_by_priority:
                    print(f"{fps_str} {'—':>4}  {'nothing':<7}  "
                          f"{'—':>5}  {'—':>5}  {'—':>6}  {'—':>10}")
                else:
                    for i, d in enumerate(dets_by_priority):
                        pfx = fps_str if i == 0 else " " * len(fps_str)
                        prox = f"{d['proximity']:.3f}"
                        print(f"{pfx} {d['priority']:>4}  "
                              f"{d['colour'].upper():<7}  "
                              f"{int(d['cx']):>5}  {int(d['cy']):>5}  "
                              f"{d['area']:>6.0f}  {prox:>10}")

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














# """
# detection_test.py  —  WRO 2025  |  Live Display + Terminal
# ===========================================================
# Shows live annotated camera feed on the Pi monitor.
# ALL colours detected and printed independently.
# Red + Green both True at the same time — no cancellation.
# Pink always from OpenCV HSV (model pink ignored).

# RUN (with monitor connected to Pi):
#     export DISPLAY=:0
#     export QT_QPA_PLATFORM=xcb
#     source /home/pi/coral-py39-env/bin/activate
#     python3 /home/pi/WRO_2025_PI/detection_test.py

# Controls:
#     Q or ESC → quit
# """

# import os
# os.environ["DISPLAY"]         = ":0"
# os.environ["QT_QPA_PLATFORM"] = "xcb"

# import cv2
# import numpy as np
# import time
# from collections import deque

# # ── PyCoral ──────────────────────────────────────────────────────────────────
# try:
#     from pycoral.utils.dataset import read_label_file
#     from pycoral.adapters import common, detect
#     from pycoral.utils.edgetpu import make_interpreter
#     CORAL_AVAILABLE = True
# except ImportError:
#     CORAL_AVAILABLE = False
#     print("[WARN] pycoral not found — OpenCV pink only")

# # ─────────────────────────────────────────────────────────────────────────────
# #  CONFIG
# # ─────────────────────────────────────────────────────────────────────────────
# MODEL_PATH   = "/home/pi/WRO_2025_PI/limelight_neural_detector_8bit_edgetpu.tflite"
# LABELS_PATH  = "/home/pi/WRO_2025_PI/label_map.txt"

# CAM_INDEX        = 1       # ← change to your camera index
# CAP_W, CAP_H     = 640, 360
# MODEL_W, MODEL_H = 320, 180

# CONF_TH       = 0.65   
# MIN_DET_AREA  = 800
# PINK_MIN_AREA = 2000

# # ── Paste your calibrated values from hsv_calibrate.py here ──────────────────
# PINK_LOWER = np.array([135,  70,  60], dtype=np.uint8)
# PINK_UPPER = np.array([170, 255, 255], dtype=np.uint8)

# SMOOTH_N       = 2
# PRINT_INTERVAL = 0   # terminal print rate (seconds)

# # ── BGR draw colours ──────────────────────────────────────────────────────────
# C_RED    = (0,   0,   220)
# C_GREEN  = (0,   210,   0)
# C_PINK   = (220,   0, 220)
# C_WHITE  = (255, 255, 255)
# C_BLACK  = (0,     0,   0)
# C_GREY   = (110, 110, 110)
# C_YELLOW = (0,   210, 210)
# C_CYAN   = (210, 210,   0)


# # ─────────────────────────────────────────────────────────────────────────────
# #  TEMPORAL SMOOTHER — independent per colour
# # ─────────────────────────────────────────────────────────────────────────────
# class FrameSmoother:
#     def __init__(self, n=SMOOTH_N):
#         self.n    = max(1, n)
#         self._buf = {}

#     def update(self, name: str, detected: bool) -> bool:
#         if name not in self._buf:
#             self._buf[name] = deque(maxlen=self.n)
#         self._buf[name].append(detected)
#         return sum(self._buf[name]) > (len(self._buf[name]) // 2)


# # ─────────────────────────────────────────────────────────────────────────────
# #  PINK DETECTION — OpenCV HSV only
# # ─────────────────────────────────────────────────────────────────────────────
# def detect_pink_opencv(frame_bgr):
#     """Returns (detected, cx, cy, contour|None)"""
#     hsv  = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
#     mask = cv2.inRange(hsv, PINK_LOWER, PINK_UPPER)
#     k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=2)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
#                                    cv2.CHAIN_APPROX_SIMPLE)
#     if not contours:
#         return False, 0.0, 0.0, None
#     best = max(contours, key=cv2.contourArea)
#     if cv2.contourArea(best) < PINK_MIN_AREA:
#         return False, 0.0, 0.0, None
#     M = cv2.moments(best)
#     if M["m00"] == 0:
#         return False, 0.0, 0.0, None
#     return True, M["m10"] / M["m00"], M["m01"] / M["m00"], best


# # ─────────────────────────────────────────────────────────────────────────────
# #  DRAW HELPERS
# # ─────────────────────────────────────────────────────────────────────────────
# def draw_bbox(frame, x1, y1, x2, y2, colour, label, conf=None):
#     cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
#     text = label + (f" {conf:.2f}" if conf is not None else "")
#     (tw, th), bl = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
#     ly = max(y1 - th - bl - 4, 0)
#     cv2.rectangle(frame, (x1, ly), (x1 + tw + 6, y1), colour, -1)
#     cv2.putText(frame, text, (x1 + 3, y1 - bl - 2),
#                 cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_BLACK, 1, cv2.LINE_AA)


# def draw_centroid(frame, cx, cy, colour):
#     cx, cy = int(cx), int(cy)
#     cv2.circle(frame, (cx, cy), 6, colour, -1)
#     cv2.circle(frame, (cx, cy), 6, C_WHITE, 1)
#     cv2.drawMarker(frame, (cx, cy), C_WHITE,
#                    cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
#     cv2.putText(frame, f"({cx},{cy})", (cx + 10, cy - 8),
#                 cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 1, cv2.LINE_AA)


# def draw_dashed_rect(frame, x1, y1, x2, y2, colour, gap=10):
#     for (ax, ay, bx, by) in [(x1,y1,x2,y1),(x2,y1,x2,y2),
#                                (x2,y2,x1,y2),(x1,y2,x1,y1)]:
#         dx, dy = bx - ax, by - ay
#         dist   = max(int((dx**2 + dy**2) ** 0.5), 1)
#         steps  = dist // gap
#         for i in range(0, steps, 2):
#             t0 = i / max(steps, 1)
#             t1 = min((i + 1) / max(steps, 1), 1.0)
#             cv2.line(frame,
#                      (int(ax + dx * t0), int(ay + dy * t0)),
#                      (int(ax + dx * t1), int(ay + dy * t1)),
#                      colour, 1)


# def draw_fps_panel(frame, fps, avg_fps, fps_hist):
#     overlay = frame.copy()
#     cv2.rectangle(overlay, (0, 0), (220, 92), C_BLACK, -1)
#     cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
#     cv2.putText(frame, f"FPS {fps:5.1f}   avg {avg_fps:5.1f}",
#                 (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, C_WHITE, 1, cv2.LINE_AA)
#     bw   = max(1, 180 // max(len(fps_hist), 1))
#     maxv = max(max(fps_hist), 1)
#     gx, gy = 6, 28
#     for i, v in enumerate(fps_hist):
#         bh  = int((v / maxv) * 32)
#         col = C_GREEN if v >= 20 else C_YELLOW if v >= 10 else C_RED
#         cv2.rectangle(frame,
#                       (gx + i * bw, gy + 32 - bh),
#                       (gx + i * bw + bw - 1, gy + 32), col, -1)
#     cv2.rectangle(frame, (gx-1, gy-1), (gx+181, gy+33), C_WHITE, 1)
#     mn = min(fps_hist); mx = max(fps_hist)
#     cv2.putText(frame, f"min {mn:.0f}  max {mx:.0f}",
#                 (6, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.40, C_CYAN, 1, cv2.LINE_AA)


# def draw_status_panel(frame, pink, green_dets, red_dets, W):
#     lines = [
#         (f"PINK  : {'DETECTED' if pink  else 'none'}", C_PINK  if pink        else C_GREY),
#         (f"GREEN : {len(green_dets)} det(s)",          C_GREEN if green_dets  else C_GREY),
#         (f"RED   : {len(red_dets)} det(s)",            C_RED   if red_dets    else C_GREY),
#         ("──────────────────────────",                 C_GREY),
#         ("Q / ESC = quit",                             (90, 90, 90)),
#     ]
#     lh = 20; pw = 215; px = W - pw - 5; py = 5
#     overlay = frame.copy()
#     cv2.rectangle(overlay, (px-4, py), (px+pw, py + len(lines)*lh + 8), C_BLACK, -1)
#     cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
#     for i, (txt, col) in enumerate(lines):
#         cv2.putText(frame, txt, (px, py + 16 + i*lh),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.44, col, 1, cv2.LINE_AA)


# # ─────────────────────────────────────────────────────────────────────────────
# #  MAIN
# # ─────────────────────────────────────────────────────────────────────────────
# def main():
#     print("=" * 58)
#     print("  WRO 2025  —  Detection Test (Live Display on Pi Monitor)")
#     print("=" * 58)
#     print(f"  PyCoral : {'YES' if CORAL_AVAILABLE else 'NO'}")
#     print(f"  Camera  : index {CAM_INDEX}  {CAP_W}x{CAP_H}")
#     print(f"  Smooth N: {SMOOTH_N}  |  Min area: {MIN_DET_AREA}")
#     print("  Q or ESC in window to quit")
#     print("=" * 58)
#     print()

#     # ── Load model ────────────────────────────────────────────────
#     interpreter = None
#     labels = {}
#     iw = ih = 0
#     if CORAL_AVAILABLE:
#         try:
#             interpreter = make_interpreter(MODEL_PATH)
#             interpreter.allocate_tensors()
#             ih, iw = common.input_size(interpreter)
#             labels = read_label_file(LABELS_PATH) if LABELS_PATH else {}
#             print(f"[Model] Loaded — input:{iw}x{ih}  labels:{labels}")
#         except Exception as e:
#             print(f"[Model] FAILED: {e}")

#     # ── Camera ───────────────────────────────────────────────────
#     cap = cv2.VideoCapture(CAM_INDEX)
#     if not cap.isOpened():
#         print(f"[Camera] FAILED to open index {CAM_INDEX}")
#         print("Find your camera index:")
#         print("  python3 -c \"import cv2; [print(i,'FOUND') "
#               "if cv2.VideoCapture(i).isOpened() else None for i in range(6)]\"")
#         return

#     cap.set(cv2.CAP_PROP_FRAME_WIDTH,   CAP_W)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  CAP_H)
#     cap.set(cv2.CAP_PROP_FPS,           120)
#     cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
#     cap.set(cv2.CAP_PROP_EXPOSURE,      -6)
#     cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)

#     print(f"[Camera] Opened — actual:{int(cap.get(3))}x{int(cap.get(4))}")
#     print()

#     # ── OpenCV window ─────────────────────────────────────────────
#     WIN = "WRO 2025 — Detection  |  Q to quit"
#     cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
#     cv2.resizeWindow(WIN, CAP_W, CAP_H)

#     # ── Terminal header ───────────────────────────────────────────
#     print(f"{'FPS':>6}  {'AVG':>6}  {'COLOUR':<10}  "
#           f"{'CX':>7}  {'CY':>7}  {'CONF':>6}  {'AREA':>7}  SOURCE")
#     print("─" * 72)

#     smoother     = FrameSmoother(n=SMOOTH_N)
#     fps_history  = deque(maxlen=30)
#     t_prev       = time.perf_counter()
#     t_last_print = time.time()
#     frame_count  = 0

#     try:
#         while True:
#             # Grab discards stale buffer — always latest frame
#             cap.grab()
#             ret, frame_bgr = cap.retrieve()
#             if not ret:
#                 continue

#             frame_count += 1
#             H, W   = frame_bgr.shape[:2]
#             canvas  = frame_bgr.copy()

#             # ── Guide lines ───────────────────────────────────────
#             cv2.line(canvas, (W//2, 0), (W//2, H), (50,50,50), 1)
#             cv2.line(canvas, (0, 240), (W, 240), (50,50,50), 1)
#             cv2.putText(canvas, "y=240 trigger", (4, 236),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80,80,80), 1)

#             # ── PINK — OpenCV HSV ─────────────────────────────────
#             pink_raw, px, py, p_contour = detect_pink_opencv(frame_bgr)
#             pink_det = smoother.update("pink", pink_raw)

#             if pink_det and p_contour is not None:
#                 ov = canvas.copy()
#                 cv2.drawContours(ov, [p_contour], -1, C_PINK, -1)
#                 cv2.addWeighted(ov, 0.18, canvas, 0.82, 0, canvas)
#                 bx, by, bw, bh = cv2.boundingRect(p_contour)
#                 draw_bbox(canvas, bx, by, bx+bw, by+bh,
#                           C_PINK, "PINK [OpenCV]")
#                 draw_centroid(canvas, px, py, C_PINK)
#                 cv2.putText(canvas, f"A:{cv2.contourArea(p_contour):.0f}",
#                             (bx, by+bh+16),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_PINK, 1)

#             # ── RED + GREEN — EdgeTPU ─────────────────────────────
#             red_dets = []
#             green_dets = []
#             model_pink_suppressed = False

#             if interpreter is not None:
#                 small = cv2.resize(frame_bgr, (MODEL_W, MODEL_H))
#                 rgb   = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
#                 inp   = cv2.resize(rgb, (iw, ih))
#                 common.set_input(interpreter, inp)
#                 interpreter.invoke()

#                 sx = W / float(iw)
#                 sy = H / float(ih)

#                 for obj in detect.get_objects(interpreter,
#                                               score_threshold=CONF_TH):
#                     name = labels.get(obj.id, str(obj.id))
#                     b    = obj.bbox
#                     x1 = int(b.xmin*sx); x2 = int(b.xmax*sx)
#                     y1 = int(b.ymin*sy); y2 = int(b.ymax*sy)
#                     cx   = (x1+x2)/2.0
#                     cy   = (y1+y2)/2.0
#                     area = max(0,x2-x1)*max(0,y2-y1)

#                     if name == "pink":
#                         model_pink_suppressed = True
#                         draw_dashed_rect(canvas, x1, y1, x2, y2, C_GREY)
#                         cv2.putText(canvas,
#                                     f"model-pink SUPPRESSED {obj.score:.2f}",
#                                     (x1, y1-4),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_GREY, 1)
#                         continue

#                     if area < MIN_DET_AREA:
#                         continue

#                     entry = {"cx":cx,"cy":cy,"area":area,"conf":obj.score,
#                              "x1":x1,"y1":y1,"x2":x2,"y2":y2}

#                     if name == "red":
#                         red_dets.append(entry)
#                     elif name == "green":
#                         green_dets.append(entry)

#                 red_dets.sort(key=lambda d: d["area"],   reverse=True)
#                 green_dets.sort(key=lambda d: d["area"], reverse=True)

#                 if not smoother.update("red",   len(red_dets)   > 0):
#                     red_dets = []
#                 if not smoother.update("green", len(green_dets) > 0):
#                     green_dets = []

#                 # Draw ALL green
#                 for i, d in enumerate(green_dets):
#                     ov = canvas.copy()
#                     cv2.rectangle(ov,(d["x1"],d["y1"]),(d["x2"],d["y2"]),C_GREEN,-1)
#                     cv2.addWeighted(ov,0.12,canvas,0.88,0,canvas)
#                     lbl = "GREEN" if i==0 else f"GREEN#{i+1}"
#                     draw_bbox(canvas,d["x1"],d["y1"],d["x2"],d["y2"],
#                               C_GREEN,lbl,d["conf"])
#                     draw_centroid(canvas,d["cx"],d["cy"],C_GREEN)
#                     cv2.putText(canvas,f"A:{d['area']:.0f}",
#                                 (d["x1"],d["y2"]+16),
#                                 cv2.FONT_HERSHEY_SIMPLEX,0.42,C_GREEN,1)

#                 # Draw ALL red
#                 for i, d in enumerate(red_dets):
#                     ov = canvas.copy()
#                     cv2.rectangle(ov,(d["x1"],d["y1"]),(d["x2"],d["y2"]),C_RED,-1)
#                     cv2.addWeighted(ov,0.12,canvas,0.88,0,canvas)
#                     lbl = "RED" if i==0 else f"RED#{i+1}"
#                     draw_bbox(canvas,d["x1"],d["y1"],d["x2"],d["y2"],
#                               C_RED,lbl,d["conf"])
#                     draw_centroid(canvas,d["cx"],d["cy"],C_RED)
#                     cv2.putText(canvas,f"A:{d['area']:.0f}",
#                                 (d["x1"],d["y2"]+16),
#                                 cv2.FONT_HERSHEY_SIMPLEX,0.42,C_RED,1)

#             # ── FPS ───────────────────────────────────────────────
#             t_now  = time.perf_counter()
#             fps    = 1.0 / max(t_now - t_prev, 1e-6)
#             t_prev = t_now
#             fps_history.append(fps)
#             avg_fps = sum(fps_history) / len(fps_history)

#             # ── Draw panels ───────────────────────────────────────
#             draw_fps_panel(canvas, fps, avg_fps, list(fps_history))
#             draw_status_panel(canvas, pink_det, green_dets, red_dets, W)

#             # ── Show ──────────────────────────────────────────────
#             cv2.imshow(WIN, canvas)

#             # ── Keys ──────────────────────────────────────────────
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord('q') or key == 27:
#                 print("\n[Quit]")
#                 break

#             # ── Terminal print ────────────────────────────────────
#             now = time.time()
#             if now - t_last_print >= PRINT_INTERVAL:
#                 t_last_print = now
#                 rows = []
#                 if pink_det:
#                     rows.append(("PINK", px, py, None, 0, "OpenCV HSV"))
#                 for i, d in enumerate(green_dets):
#                     lbl = "GREEN" if i==0 else f"GREEN#{i+1}"
#                     rows.append((lbl,d["cx"],d["cy"],d["conf"],d["area"],"model"))
#                 for i, d in enumerate(red_dets):
#                     lbl = "RED" if i==0 else f"RED#{i+1}"
#                     rows.append((lbl,d["cx"],d["cy"],d["conf"],d["area"],"model"))
#                 if model_pink_suppressed:
#                     rows.append(("PINK*",0,0,None,0,"model→SUPPRESSED"))

#                 if not rows:
#                     print(f"{fps:6.1f}  {avg_fps:6.1f}  "
#                           f"{'—':<10}  {'—':>7}  {'—':>7}  "
#                           f"{'—':>6}  {'—':>7}  nothing")
#                 else:
#                     for i,(lbl,cx_,cy_,conf,area,src) in enumerate(rows):
#                         fps_s = f"{fps:6.1f}" if i==0 else " "*6
#                         avg_s = f"{avg_fps:6.1f}" if i==0 else " "*6
#                         c_s   = f"{conf:.2f}" if conf is not None else "  n/a"
#                         print(f"{fps_s}  {avg_s}  {lbl:<10}  "
#                               f"{cx_:7.1f}  {cy_:7.1f}  "
#                               f"{c_s:>6}  {area:7.0f}  {src}")

#     except KeyboardInterrupt:
#         print("\n[Stopped]")
#     finally:
#         cap.release()
#         cv2.destroyAllWindows()
#         print(f"[Done] Frames:{frame_count}")
#         if fps_history:
#             print(f"[FPS]  avg:{sum(fps_history)/len(fps_history):.1f}"
#                   f"  min:{min(fps_history):.1f}"
#                   f"  max:{max(fps_history):.1f}")


# if __name__ == "__main__":
#     main()