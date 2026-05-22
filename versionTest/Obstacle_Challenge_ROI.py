"""
╔══════════════════════════════════════════════════════════════════════════════════╗
║   WRO 2025 Future Engineers — Obstacle Challenge  |  Team Paraducks             ║
║   Obstacle_Challenge_ROI.py                                                     ║
╠══════════════════════════════════════════════════════════════════════════════════╣
║                                                                                 ║
║  WHAT CHANGED FROM YOUR BACKUP (Obstacle_Challenge_Worlds_BACKUP.py)           ║
║  ─────────────────────────────────────────────────────────────────────────────  ║
║                                                                                 ║
║  OLD SYSTEM                          NEW SYSTEM (this file)                    ║
║  ──────────────────────────────────  ─────────────────────────────────────────  ║
║  Coral EdgeTPU ML model detection    Pure OpenCV HSV blob detection             ║
║  RPLidar 4th process                 No LiDAR process at all                   ║
║  lidar_f/l/r for turn trigger        TFmini head + camera ROI turn trigger      ║
║  LIDAR F<950 AND side>1500mm         roi_turn_trigger (camera) + TFmini gate    ║
║  Encoder XY coordinate PID           IMU heading-hold PID (straight line)       ║
║  correctPosition() with x,y,lane     steer_to_target() with ROI pixel target    ║
║  turn_trigger shared value           roi_turn_trigger shared value              ║
║                                                                                 ║
║  WHAT IS KEPT EXACTLY THE SAME                                                  ║
║  • All HSV values from Obstacle_Challenge_Worlds.py (tuned, do not change)     ║
║  • 3-process multiprocessing: P (camera), S (drive), E (encoder)               ║
║  • TFmini class usage for all 4 sensors                                         ║
║  • ESP32 UART protocol: "yaw_float encoder_int\n"                               ║
║  • Servo class, pigpio PWM motor driver, all GPIO pin assignments               ║
║  • Parking state machine (IMU heading turns + TFmini distances)                 ║
║  • Button debounce, LED indicators, Arduino reset_pin logic                     ║
║  • correctAngle() / correctReverseAngle() heading PID                           ║
║  • update_heading(), normalize_angle() helpers                                  ║
║                                                                                 ║
╠══════════════════════════════════════════════════════════════════════════════════╣
║                                                                                 ║
║  ROI SYSTEM — HOW IT REPLACES THE LIDAR TURN TRIGGER                           ║
║  ─────────────────────────────────────────────────────────────────────────────  ║
║                                                                                 ║
║  Camera frame 640 × 360 px (GLOBAL_Y_TOP=100 to GLOBAL_Y_BOTTOM=290):         ║
║                                                                                 ║
║  y=100 ┌────────────────────────────────────────────────────────────┐          ║
║        │  FAR ZONE  (y=100 to ROI_FAR_Y=190)                       │          ║
║        │  Block first appears here → g_flag / r_flag set            │          ║
║        │  Avoidance steering begins                                  │          ║
║  y=190 │────────────── ROI_FAR_Y (blue line in preview) ────────────│          ║
║        │  NEAR ZONE  (y=190 to ROI_NEAR_Y=260)                     │          ║
║        │  Block centroid here = block is getting VERY close         │          ║
║  y=260 │────────────── ROI_NEAR_Y (red line in preview) ─────────── │          ║
║        │  TRIGGER ZONE  (y=260 to 290)                              │          ║
║        │  Block centroid here + TFmini head < TF_TURN_HEAD_CM       │          ║
║        │  → roi_turn_trigger fires → corner turn sequence executes  │          ║
║  y=290 └────────────────────────────────────────────────────────────┘          ║
║                                                                                 ║
║  Orange/Blue LINE ROI (y=200-240, x=280-360) → detects floor direction lines   ║
║  Close-block ROI     (y=230-240, x=250-390) → triggers emergency dodge         ║
║                                                                                 ║
╠══════════════════════════════════════════════════════════════════════════════════╣
║                                                                                 ║
║  ████████████████████  STEP-BY-STEP SETUP  ████████████████████████████████   ║
║                                                                                 ║
║  STEP 1 — Verify TFmini sensors                                                 ║
║    python3 -c "                                                                 ║
║      import pigpio, time                                                        ║
║      from TFmini import TFmini                                                  ║
║      pigpio.pi()                                                                ║
║      t = TFmini(23, 24, 25, 27)                                                 ║
║      time.sleep(1)                                                              ║
║      t.getTFminiData()                                                          ║
║      print('H:', t.distance_head, 'L:', t.distance_left, 'R:', t.distance_right)║
║    "                                                                            ║
║    Expected: all 3 print non-zero cm values.                                   ║
║    If 0: check wiring (TFmini TX → Pi GPIO), check 'sudo pigpiod'.            ║
║                                                                                 ║
║  STEP 2 — Verify ESP32 serial                                                   ║
║    python3 -c "                                                                 ║
║      import serial                                                              ║
║      s = serial.Serial('/dev/UART_USB', 115200, timeout=2)                     ║
║      for _ in range(5): print(s.readline())                                    ║
║    "                                                                            ║
║    Expected: lines like  b'12.34 5678\n'  (yaw_float encoder_int).            ║
║                                                                                 ║
║  STEP 3 — Camera check + HSV                                                    ║
║    Set SHOW_WINDOW = True  (~line 340 in Live_Feed_ROI).                        ║
║    Run the code. Preview window opens.                                          ║
║    Red block in front → red rectangle. Green → green. Pink pillar → pink.     ║
║    If wrong: run hsv_tuner.py and paste new values into HSV CALIBRATION block. ║
║                                                                                 ║
║  STEP 4 — Tune ROI lines (most critical)                                        ║
║    Watch the preview with SHOW_WINDOW=True.                                     ║
║    Blue line = ROI_FAR_Y. Red line = ROI_NEAR_Y.                               ║
║    Drive slowly toward a block and note which pixel-Y row the block centroid   ║
║    is at when you want hard steering to begin. Set ROI_NEAR_Y to that value.  ║
║                                                                                 ║
║  STEP 5 — Tune corner trigger                                                   ║
║    TF_TURN_HEAD_CM (default 55): front TFmini (cm) that confirms a corner.    ║
║    Fires only when BOTH roi_turn_trigger=True AND tf_head < TF_TURN_HEAD_CM.  ║
║    Too early → lower TF_TURN_HEAD_CM. Too late → raise it.                    ║
║                                                                                 ║
║  STEP 6 — Safe first run                                                        ║
║    Set DRIVE_POWER = 60. Press button. Watch terminal logs.                    ║
║    Confirm: direction set, green→steer left, red→steer right,                  ║
║    turn_trigger fires, counter increments. Once clean → DRIVE_POWER = 95.     ║
║                                                                                 ║
╠══════════════════════════════════════════════════════════════════════════════════╣
║                                                                                 ║
║  ████████████████████████  DEBUGGING GUIDE  █████████████████████████████████  ║
║                                                                                 ║
║  ❌ No blocks detected                                                            ║
║     → SHOW_WINDOW=True. Anything visible?                                       ║
║     → Camera auto-scans 0-5. Force CAM_INDEX=1 if needed.                     ║
║     → Raise exposure: -6 → -4 in cap.set(EXPOSURE)                             ║
║     → Lower RED_MIN_AREA / GREEN_MIN_AREA to 200                                ║
║                                                                                 ║
║  ❌ Wall/floor detected as block                                                  ║
║     → Raise RED_MIN_AREA / GREEN_MIN_AREA to 1000                               ║
║     → Narrow HSV: GREEN_UPPER[2] 200 → 150                                     ║
║     → Raise SMOOTH_N from 2 → 4                                                 ║
║                                                                                 ║
║  ❌ Turn trigger fires too early (turns before reaching corner)                   ║
║     → Lower TF_TURN_HEAD_CM (55 → 40)                                          ║
║     → Raise ROI_NEAR_Y (260 → 285)                                             ║
║     → Raise TRIGGER_COOLDOWN (2.0 → 3.0)                                       ║
║                                                                                 ║
║  ❌ Turn trigger fires too late (robot clips wall)                                ║
║     → Raise TF_TURN_HEAD_CM (55 → 70)                                          ║
║     → Lower ROI_NEAR_Y (260 → 230)                                             ║
║                                                                                 ║
║  ❌ Robot steers wrong direction for red/green                                    ║
║     → Check AVOID_OFFSET_PX sign in steer_to_target() calls                   ║
║     → If camera is mirrored: add frame = cv2.flip(frame, 1) in Live_Feed_ROI  ║
║                                                                                 ║
║  ❌ Heading drifts / robot curves even straight                                   ║
║     → Print head.value in runEncoder — check yaw is consistent                ║
║     → ESP32 comma decimal: add .replace(',', '.')  (already done here)        ║
║     → Raise kp: 0.6 → 0.9 for tighter hold                                    ║
║                                                                                 ║
║  ❌ TFmini reads 0 constantly                                                     ║
║     → sudo pigpiod not running → os.system('sudo pigpiod')                    ║
║     → TFmini baud must be 115200. Wrong baud = zeros.                         ║
║     → Pi GPIO RX pin must match TFmini TX wire exactly                         ║
║                                                                                 ║
║  ❌ Multiprocessing crashes / "prevError not defined"                             ║
║     → All PID state vars declared global at top of servoDrive (done here)     ║
║     → "int() on float string": use int(float(x)) — done in runEncoder         ║
║                                                                                 ║
║  ❌ Parking misses the spot                                                       ║
║     → Tune heading_angle offsets in parking STATE machine (+90/-90)            ║
║     → Adjust parking_count encoder ticks for sideways move                     ║
║     → Add tfmini.distance_right/left exit condition instead of pure encoder   ║
║                                                                                 ║
╚══════════════════════════════════════════════════════════════════════════════════╝
"""

# ══════════════════════════════════════════════════════════════════════════════
#  IMPORTS
# ══════════════════════════════════════════════════════════════════════════════
import os
import sys
import time
import math
import serial
import logging
import datetime
import traceback
import multiprocessing
from collections import deque

import cv2
import numpy as np
import pigpio

from Encoder import EncoderCounter
from Servo import Servo
from TFmini import TFmini

# ══════════════════════════════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════════════════════════════
timestamp = datetime.datetime.now().strftime('%d%m%y_%H%M')
LOG_DIR   = '/home/pi/WRO_2025_PI/logs'
os.makedirs(LOG_DIR, exist_ok=True)

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(f"{LOG_DIR}/obstacle_roi_{timestamp}.txt"),
        logging.StreamHandler(sys.stderr),
    ]
)
log = logging.getLogger("WRO_ROI")

# ══════════════════════════════════════════════════════════════════════════════
#  PIGPIO STARTUP
# ══════════════════════════════════════════════════════════════════════════════
def _start_pigpiod(retries: int = 15, interval: float = 1.0) -> pigpio.pi:
    log.info("Killing stale pigpiod…")
    os.system("sudo pkill -9 pigpiod 2>/dev/null || true")
    time.sleep(0.5)
    log.info("Starting pigpiod…")
    os.system("sudo pigpiod -t 0 -p 8888")
    for attempt in range(1, retries + 1):
        time.sleep(interval)
        pi = pigpio.pi()
        if pi.connected:
            log.info(f"pigpiod connected (attempt {attempt}/{retries})")
            return pi
        log.warning(f"pigpiod not ready ({attempt}/{retries})")
    log.critical("pigpiod failed to start — check 'sudo pigpiod' manually")
    sys.exit(1)

_pigpio_main = _start_pigpiod()

# ══════════════════════════════════════════════════════════════════════════════
#  HARDWARE PINS  (unchanged from your backup)
# ══════════════════════════════════════════════════════════════════════════════
RX_Head, RX_Left, RX_Right, RX_Back = 23, 24, 25, 27
button_pin   = 5
exit_pin     = 7
servo_pin    = 8
blue_led     = 26
red_led      = 10
green_led    = 6
reset_pin    = 19
pwm_pin      = 12
direction_pin = 20

UART_PORT = '/dev/UART_USB'
UART_BAUD = 115200

# ══════════════════════════════════════════════════════════════════════════════
#  HARDWARE INIT  (main process only)
# ══════════════════════════════════════════════════════════════════════════════
for _pin in [reset_pin, blue_led, red_led, green_led]:
    _pigpio_main.set_mode(_pin, pigpio.OUTPUT)
    _pigpio_main.write(_pin, 0)
_pigpio_main.set_mode(button_pin, pigpio.INPUT)
_pigpio_main.set_pull_up_down(button_pin, pigpio.PUD_UP)

log.info("Resetting Arduino…")
_pigpio_main.write(reset_pin, 0); _pigpio_main.write(green_led, 1); time.sleep(1)
_pigpio_main.write(reset_pin, 1); _pigpio_main.write(green_led, 0); time.sleep(1)
log.info("Arduino reset complete")

servo  = Servo(servo_pin)
tfmini = TFmini(RX_Head, RX_Left, RX_Right, RX_Back)

# ══════════════════════════════════════════════════════════════════════════════
#  ██  TUNABLE PARAMETERS  ██   ← Start here when field-testing
# ══════════════════════════════════════════════════════════════════════════════

# Drive power (0–100)
DRIVE_POWER      = 95    # ← Use 60 for first safe run. Raise to 95 once clean.
AVOID_POWER      = 55    # speed while actively steering around a block
TURN_POWER       = 100   # speed during corner reverse manoeuvre

# Corner turn trigger — front TFmini threshold
# OLD code: lidar_f < 950 mm  →  NEW: tfmini.distance_head < TF_TURN_HEAD_CM
TF_TURN_HEAD_CM  = 55    # cm. Too high→fires early. Too low→hits wall.
FINISH_HEAD_CM   = 35    # cm front distance used as final stop condition

# Lap counting
TOTAL_LAPS       = 3
TURNS_PER_LAP    = 4
LAST_COUNTER     = TOTAL_LAPS * TURNS_PER_LAP    # = 12

# Avoidance pixel target offset from frame centre
# Green: target_px = 320 + AVOID_OFFSET_PX  (pass to the RIGHT of green)
# Red:   target_px = 320 - AVOID_OFFSET_PX  (pass to the LEFT of red)
AVOID_OFFSET_PX  = 150   # pixels. Larger = robot gives more clearance to block.

# Heading PID gains (straight-line hold + corner turns)
kp   = 0.6
ki   = 0.0
kd   = 0.1
kp_e = 3.0   # kept from backup, used in correctPosition() if ever needed
ki_e = 0.0
kd_e = 40.0

# Camera frame dimensions
CAM_W, CAM_H = 640, 360
FRAME_MID_X  = CAM_W // 2    # = 320

# ROI boundaries (all in absolute frame pixel coordinates)
GLOBAL_Y_TOP    = 60   # rows above this are ignored (sky/ceiling)
GLOBAL_Y_BOTTOM = 350   # rows below this are ignored (robot body)
# Rectangular ROI zones (x, y, w, h) — all in absolute frame coords
ROI_OBJECT_DETECT  = (160, 80,  320, 160)   # far object detection — upper centre
ROI_CLOSE_OBJECT   = (210, 240, 220, 60)    # close object detection — centre
ROI_LEFT_WALL      = (0,   150, 160, 160)   # left wall strip
ROI_RIGHT_WALL     = (480, 150, 160, 160)   # right wall strip
ROI_CLOSE_WALL     = (380, 80,  260, 100)   # close wall — top right
LINE_ROI           = (270, 310, 100, 60)    # orange/blue line — bottom centre
CLOSE_ROI          = (210, 240, 220, 10)    # kept for _detect_all() compatibility

# Turn trigger Y threshold — block centroid inside ROI_CLOSE_OBJECT triggers turn
ROI_NEAR_Y = ROI_CLOSE_OBJECT[1]            # = 240
ROI_FAR_Y  = ROI_OBJECT_DETECT[1]          # = 80

# Frame-to-frame smoothing (majority vote over N frames)
SMOOTH_N  = 2    # raise to 4 if detections flicker in your lighting

# ══════════════════════════════════════════════════════════════════════════════
#  HSV CALIBRATION  — from Obstacle_Challenge_Worlds.py  (known-good values)
#  Only change after re-tuning with hsv_tuner.py on your actual field.
# ══════════════════════════════════════════════════════════════════════════════
RED_LOWER_1  = np.array([  0, 120,  60], dtype=np.uint8)
RED_UPPER_1  = np.array([ 10, 255, 255], dtype=np.uint8)
RED_LOWER_2  = np.array([170, 120,  60], dtype=np.uint8)
RED_UPPER_2  = np.array([180, 255, 255], dtype=np.uint8)

GREEN_LOWER  = np.array([ 40,  80,  40], dtype=np.uint8)
GREEN_UPPER  = np.array([ 90, 255, 200], dtype=np.uint8)

PINK_LOWER   = np.array([135,  70,  60], dtype=np.uint8)
PINK_UPPER   = np.array([175, 255, 255], dtype=np.uint8)

# Floor line colours (from Greenbotics main_v3 HSV_RANGES)
ORANGE_LOWER = np.array([  6,  70,  20], dtype=np.uint8)
ORANGE_UPPER = np.array([ 26, 255, 255], dtype=np.uint8)
BLUE_LOWER   = np.array([ 94,  45,  58], dtype=np.uint8)
BLUE_UPPER   = np.array([140, 226, 185], dtype=np.uint8)

RED_MIN_AREA   = 400
GREEN_MIN_AREA = 400
PINK_MIN_AREA  = 800
LINE_MIN_AREA  = 20

# ══════════════════════════════════════════════════════════════════════════════
#  MULTIPROCESSING SHARED MEMORY
# ══════════════════════════════════════════════════════════════════════════════
counts         = multiprocessing.Value('i', 0)
lane_counter   = multiprocessing.Value('i', 0)

# Block detection flags
red_b          = multiprocessing.Value('b', False)
green_b        = multiprocessing.Value('b', False)
pink_b         = multiprocessing.Value('b', False)

# Block centroids in absolute frame pixel coords
centr_x        = multiprocessing.Value('f', 0.0)   # green cx
centr_y        = multiprocessing.Value('f', 0.0)   # green cy
centr_x_red    = multiprocessing.Value('f', 0.0)
centr_y_red    = multiprocessing.Value('f', 0.0)
centr_x_pink   = multiprocessing.Value('f', 0.0)
centr_y_pink   = multiprocessing.Value('f', 0.0)

# Block detection areas (used to scale steering gain)
area_green     = multiprocessing.Value('f', 0.0)
area_red       = multiprocessing.Value('f', 0.0)

# Close-block flags — block inside CLOSE_ROI strip
close_red_b    = multiprocessing.Value('b', False)
close_green_b  = multiprocessing.Value('b', False)

# Floor line direction indicators
orange_line_b  = multiprocessing.Value('b', False)   # clockwise
blue_line_b    = multiprocessing.Value('b', False)   # counter-clockwise

# ── TURN TRIGGER ─────────────────────────────────────────────────────────────
# Replaces the old RPLidar turn_trigger.
# Set True by camera process when block crosses ROI_NEAR_Y or disappears.
# servoDrive reads it; fires corner turn when also tf_h < TF_TURN_HEAD_CM.
roi_turn_trigger = multiprocessing.Value('b', False)

# IMU heading from ESP32
head           = multiprocessing.Value('f', 0.0)

# Direction flags
left_f         = multiprocessing.Value('b', False)   # CCW
right_f        = multiprocessing.Value('b', False)   # CW

stop_evt       = multiprocessing.Event()

# ══════════════════════════════════════════════════════════════════════════════
#  PID STATE  (globals inside servoDrive process)
# ══════════════════════════════════════════════════════════════════════════════
prevError  = 0.0
totalError = 0.0
corr       = 0.0
corr_pos   = 0.0


# ══════════════════════════════════════════════════════════════════════════════
#  OPENCV DETECTION HELPERS  (from Obstacle_Challenge_Worlds.py)
# ══════════════════════════════════════════════════════════════════════════════

class FrameSmoother:
    """Majority-vote over N frames to suppress single-frame noise."""
    def __init__(self, n: int):
        self.n    = max(1, n)
        self._buf: dict = {}

    def update(self, name: str, detected: bool) -> bool:
        if name not in self._buf:
            self._buf[name] = deque(maxlen=self.n)
        self._buf[name].append(detected)
        return sum(self._buf[name]) > (len(self._buf[name]) // 2)


def _get_best_blob(mask: np.ndarray, min_area: int) -> "dict | None":
    """Return largest blob dict {cx,cy,area,x1,y1,x2,y2} or None."""
    k     = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    conts, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    return {"cx": cx, "cy": cy, "area": area,
            "x1": x,  "y1": y, "x2": x+w, "y2": y+h}


def _detect_all(frame_bgr: np.ndarray) -> dict:
    """
    Runs HSV detection on the ROI slice of the frame.
    Returns dict: red, green, pink, orange_line, blue_line, close_red, close_green.
    Each value: blob dict or None.
    Centroid coords are RELATIVE to the slice (add GLOBAL_Y_TOP for absolute).
    """
    roi_slice = frame_bgr[GLOBAL_Y_TOP:GLOBAL_Y_BOTTOM, :]
    hsv       = cv2.cvtColor(roi_slice, cv2.COLOR_BGR2HSV)

    # ── Full-slice block masks ────────────────────────────────────────────────
    m_r = cv2.bitwise_or(
        cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1),
        cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2),
    )
    m_g  = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
    m_pk = cv2.inRange(hsv, PINK_LOWER,  PINK_UPPER)

    # ── LINE_ROI crop (orange/blue floor lines) ───────────────────────────────
    lx, ly, lw, lh = LINE_ROI
    ly_s = ly - GLOBAL_Y_TOP                   # convert to slice-relative y
    line_crop = hsv[ly_s:ly_s+lh, lx:lx+lw]
    m_or = cv2.inRange(line_crop, ORANGE_LOWER, ORANGE_UPPER)
    m_bl = cv2.inRange(line_crop, BLUE_LOWER,   BLUE_UPPER)

    # ── CLOSE_ROI crop (emergency close block) ────────────────────────────────
    cx2, cy2, cw2, ch2 = CLOSE_ROI
    cy2_s = cy2 - GLOBAL_Y_TOP
    close_crop = hsv[cy2_s:cy2_s+ch2, cx2:cx2+cw2]
    m_cl_r = cv2.bitwise_or(
        cv2.inRange(close_crop, RED_LOWER_1, RED_UPPER_1),
        cv2.inRange(close_crop, RED_LOWER_2, RED_UPPER_2),
    )
    m_cl_g = cv2.inRange(close_crop, GREEN_LOWER, GREEN_UPPER)

    return {
        "red"        : _get_best_blob(m_r,    RED_MIN_AREA),
        "green"      : _get_best_blob(m_g,    GREEN_MIN_AREA),
        "pink"       : _get_best_blob(m_pk,   PINK_MIN_AREA),
        "orange_line": _get_best_blob(m_or,   LINE_MIN_AREA),
        "blue_line"  : _get_best_blob(m_bl,   LINE_MIN_AREA),
        "close_red"  : _get_best_blob(m_cl_r, 5),
        "close_green": _get_best_blob(m_cl_g, 5),
    }


def _draw_overlay(canvas: np.ndarray, dets: dict, fps: float,
                  roi_trigger: bool, ctr: int) -> None:
    CYAN = (255, 220, 0)
    
    # Draw all ROI rectangles
    for (x, y, w, h) in [ROI_OBJECT_DETECT, ROI_CLOSE_OBJECT,
                          ROI_LEFT_WALL, ROI_RIGHT_WALL,
                          ROI_CLOSE_WALL, LINE_ROI]:
        cv2.rectangle(canvas, (x, y), (x+w, y+h), CYAN, 2)

    # Block bounding boxes
    CLR = {"red": (0, 0, 220), "green": (0, 210, 0), "pink": (220, 0, 220)}
    for name in ("red", "green", "pink"):
        d = dets.get(name)
        if d is None:
            continue
        col = CLR[name]
        abs_y1 = d["y1"] + GLOBAL_Y_TOP
        abs_y2 = d["y2"] + GLOBAL_Y_TOP
        abs_cy = int(d["cy"]) + GLOBAL_Y_TOP
        cv2.rectangle(canvas, (d["x1"], abs_y1), (d["x2"], abs_y2), col, 2)
        cv2.circle(canvas, (int(d["cx"]), abs_cy), 5, col, -1)
        cv2.putText(canvas, f"{name.upper()} A:{d['area']:.0f}",
                    (d["x1"]+2, abs_y1-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

    trig_col = (0, 0, 255) if roi_trigger else (0, 255, 0)
    cv2.putText(canvas,
                f"FPS:{fps:.0f}  CTR:{ctr}  TRIG:{'YES' if roi_trigger else 'no'}",
                (6, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, trig_col, 2)

# ══════════════════════════════════════════════════════════════════════════════
#  PROCESS P — LIVE FEED (OpenCV HSV + ROI turn trigger)
#  Replaces: EdgeTPU Live_Feed + RPLidar read_lidar
# ══════════════════════════════════════════════════════════════════════════════

def Live_Feed_ROI(red_b, green_b, pink_b,
                  centr_y, centr_x,
                  centr_y_red, centr_x_red,
                  centr_x_pink, centr_y_pink,
                  area_green, area_red,
                  close_red_b, close_green_b,
                  orange_line_b, blue_line_b,
                  roi_turn_trigger, lane_counter):
    """
    Camera loop: HSV detect → update shared values → set roi_turn_trigger.

    ROI TURN TRIGGER LOGIC (replaces lidar_f < 950 AND lidar_l/r > 1500):
    ─────────────────────────────────────────────────────────────────────────
    CONDITION A — block centroid_y > ROI_NEAR_Y:
        Block is deep in frame = very close = we are at the corner.
        roi_turn_trigger = True.

    CONDITION B — block was visible, now gone for NO_BLOCK_FRAMES frames:
        We just passed the block = we have cleared the straight section.
        roi_turn_trigger = True.

    servoDrive reads roi_turn_trigger AND checks tfmini.distance_head < TF_TURN_HEAD_CM
    before committing to the actual corner turn sequence. This dual-gate prevents
    spurious triggers from momentary detection loss.
    ─────────────────────────────────────────────────────────────────────────
    SSH tip: SHOW_WINDOW = False disables the preview window.
    """
    SHOW_WINDOW     = True       # ← set False when running headless over SSH
    NO_BLOCK_FRAMES = 20         # consecutive frames with no block → trigger

    os.environ.setdefault("DISPLAY", ":0")
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

    smoother       = FrameSmoother(SMOOTH_N)
    fps_history    = deque(maxlen=30)
    t_prev         = time.perf_counter()
    no_block_count = 0
    had_block      = False

    # Auto-scan camera index 0–5
    cap = None
    for idx in range(6):
        c = cv2.VideoCapture(idx)
        if c.isOpened():
            ret, _ = c.read()
            if ret:
                cap = c
                log.info(f"[Camera] using index {idx}")
                break
            c.release()
    if cap is None:
        log.error("[Camera] no working camera on indices 0–5")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,   CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  CAM_H)
    cap.set(cv2.CAP_PROP_FPS,           120)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE,      -6)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)

    if SHOW_WINDOW:
        WIN = "WRO 2025 ROI — press Q to quit"
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, CAM_W, CAM_H)

    try:
        while True:
            cap.grab()
            ok, frame = cap.retrieve()
            if not ok:
                continue

            raw = _detect_all(frame)

            # Per-colour majority-vote smoothing
            for c in ("red", "green", "pink", "orange_line", "blue_line"):
                if not smoother.update(c, raw.get(c) is not None):
                    raw[c] = None

            rd  = raw["red"]
            gd  = raw["green"]
            pk  = raw["pink"]
            orng = raw["orange_line"]
            blln = raw["blue_line"]
            cl_r = raw["close_red"]
            cl_g = raw["close_green"]

            # ── Update shared centroids (exact same logic as Worlds.py) ──────
            # Centroid Y stored as ABSOLUTE frame coord (adds GLOBAL_Y_TOP)
            if pk and not rd and not gd:
                pink_b.value  = True; red_b.value = False; green_b.value = False
                centr_x_pink.value = pk["cx"];  centr_y_pink.value = pk["cy"] + GLOBAL_Y_TOP
                centr_x.value = centr_y.value = centr_x_red.value = centr_y_red.value = 0
                area_green.value = area_red.value = 0

            elif pk and rd and not gd:
                red_b.value   = True; pink_b.value = True; green_b.value = False
                centr_x_red.value  = rd["cx"];  centr_y_red.value  = rd["cy"] + GLOBAL_Y_TOP
                centr_x_pink.value = pk["cx"];  centr_y_pink.value = pk["cy"] + GLOBAL_Y_TOP
                centr_x.value = centr_y.value = 0
                area_red.value = rd["area"]; area_green.value = 0

            elif pk and gd and not rd:
                green_b.value = True; pink_b.value = True; red_b.value = False
                centr_x.value      = gd["cx"];  centr_y.value      = gd["cy"] + GLOBAL_Y_TOP
                centr_x_pink.value = pk["cx"];  centr_y_pink.value = pk["cy"] + GLOBAL_Y_TOP
                centr_x_red.value  = centr_y_red.value = 0
                area_green.value = gd["area"]; area_red.value = 0

            elif gd and not pk:
                green_b.value = True; red_b.value = False; pink_b.value = False
                centr_x.value = gd["cx"];    centr_y.value = gd["cy"] + GLOBAL_Y_TOP
                centr_x_red.value = centr_y_red.value = 0
                centr_x_pink.value = centr_y_pink.value = 0
                area_green.value = gd["area"]; area_red.value = 0

            elif rd and not pk:
                red_b.value   = True; green_b.value = False; pink_b.value = False
                centr_x_red.value = rd["cx"]; centr_y_red.value = rd["cy"] + GLOBAL_Y_TOP
                centr_x.value = centr_y.value = 0
                centr_x_pink.value = centr_y_pink.value = 0
                area_red.value = rd["area"]; area_green.value = 0

            else:
                red_b.value = green_b.value = pink_b.value = False
                centr_x.value = centr_y.value = 0
                centr_x_red.value = centr_y_red.value = 0
                centr_x_pink.value = centr_y_pink.value = 0
                area_red.value = area_green.value = 0

            # ── Close-block & floor line flags ────────────────────────────────
            close_red_b.value   = (cl_r is not None)
            close_green_b.value = (cl_g is not None)
            orange_line_b.value = (orng is not None)
            blue_line_b.value   = (blln is not None)

            # ── ROI TURN TRIGGER ──────────────────────────────────────────────
            block_seen = red_b.value or green_b.value
            if block_seen:
                had_block = True; no_block_count = 0
                active_cx = centr_x.value if green_b.value else centr_x_red.value
                active_cy = centr_y.value if green_b.value else centr_y_red.value
                # CONDITION A: block centroid is inside ROI_CLOSE_OBJECT box
                rx, ry, rw, rh = ROI_CLOSE_OBJECT
                inside_close_roi = (rx < active_cx < rx+rw) and (ry < active_cy < ry+rh)
                roi_turn_trigger.value = inside_close_roi
            else:
                if had_block:
                    no_block_count += 1
                    # CONDITION B: block was seen but now gone → we passed it
                    if no_block_count >= NO_BLOCK_FRAMES:
                        roi_turn_trigger.value = True
                        no_block_count = 0; had_block = False
                else:
                    roi_turn_trigger.value = False

            # ── FPS and preview ───────────────────────────────────────────────
            t_now  = time.perf_counter()
            fps    = 1.0 / max(t_now - t_prev, 1e-6)
            t_prev = t_now
            fps_history.append(fps)

            if SHOW_WINDOW:
                canvas = frame.copy()
                _draw_overlay(canvas, raw, fps,
                              roi_turn_trigger.value, lane_counter.value)
                cv2.imshow(WIN, canvas)
                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        log.info("[Camera] process stopped")


# ══════════════════════════════════════════════════════════════════════════════
#  STEERING AND PID FUNCTIONS  (kept from backup; correctPosition replaced)
# ══════════════════════════════════════════════════════════════════════════════

def correctAngle(setPoint_gyro: float, heading: float, multiplier: float = 1.0):
    """IMU heading PID → servo for forward driving."""
    global corr
    error = heading - setPoint_gyro
    if error > 180: error -= 360
    corr = error
    correction = kp * error * multiplier + kd * error
    correction = max(-60, min(60, correction)) if multiplier == 3 else max(-30, min(30, correction))
    servo.setAngle(90 - correction)


def correctReverseAngle(setPoint_gyro: float, heading: float, multiplier: float = 1.0):
    """IMU heading PID → servo for reverse driving."""
    global corr
    error = heading - setPoint_gyro
    if error > 180: error -= 360
    corr = error
    correction = kp * error * multiplier + kd * error
    correction = max(-55, min(55, correction)) if multiplier == 3 else max(-30, min(30, correction))
    servo.setAngle(90 + correction)


def steer_to_target(block_cx: float, target_px: float,
                    heading: float, heading_sp: float,
                    kp_pix: float = 0.09) -> None:
    """
    NEW — replaces correctPosition() encoder XY logic.

    Steers the robot so that the detected block's pixel X centroid
    aligns with target_px, while also holding the IMU heading.

    block_cx   : detected block centroid X (pixels, absolute frame coords)
    target_px  : desired pixel X for the block (e.g. 320 ± AVOID_OFFSET_PX)
    heading    : current IMU yaw (degrees)
    heading_sp : target IMU heading (degrees)
    kp_pix     : proportional gain mapping pixel error → servo angle

    Equivalent to Greenbotics steer_with_gyro + maneuver target heading approach,
    adapted as a continuous steering function.
    """
    pix_error      = block_cx - target_px          # positive = block right of target
    pixel_angle    = kp_pix * pix_error

    hdg_error      = heading - heading_sp
    if hdg_error > 180: hdg_error -= 360
    hdg_correction = kp * hdg_error                # same gain as correctAngle

    total           = pixel_angle + hdg_correction
    total           = max(-30, min(30, total))
    servo.setAngle(90 - total)


def correctWall(setPoint_distance: float, dist: float,
                sp_h: float, imu_h: float,
                orange: bool, blue: bool,
                pink_l: int, counter: int,
                left: bool, right: bool):
    """TFmini wall-centering PID — unchanged from backup."""
    error_d    = (dist - setPoint_distance) if right else (setPoint_distance - dist)
    correction = max(-40, min(40, 2.5 * error_d))
    if (dist < 30 and setPoint_distance == 35) or \
       (setPoint_distance == 60 and dist <= 25) or \
       (setPoint_distance == 45 and dist < 20):
        correction = 0
    mult = 1.0 if counter % 4 == pink_l else 1.8
    correctAngle(sp_h + correction, imu_h, mult)


def update_heading(counter: int, heading_angle: float,
                   blue: bool, orange: bool) -> float:
    if blue:   return -((90 * counter) % 360)
    if orange: return  (90 * counter) % 360
    return heading_angle


def normalize_angle(angle: float, blue: bool, orange: bool, lane: int) -> float:
    if blue:   return (angle + 360) if (angle < 180 and lane == 0) else angle
    if orange: return (angle - 360) if (angle > 180 and lane == 0) else angle
    return angle


# ══════════════════════════════════════════════════════════════════════════════
#  MOTOR HELPER
# ══════════════════════════════════════════════════════════════════════════════

def runMotor(pwm_h, speed: float, direction: int):
    """direction: 1 = forward, 0 = reverse"""
    pwm_h.set_PWM_dutycycle(pwm_pin, int(speed * 2.55))
    pwm_h.write(direction_pin, direction)


# ══════════════════════════════════════════════════════════════════════════════
#  TELEMETRY
# ══════════════════════════════════════════════════════════════════════════════

def print_telemetry(ctr, hdg_sp, imu, corr_v, tf_h, tf_l, tf_r,
                    g_flag, r_flag, p_flag, orange_flag, blue_flag,
                    power, reset_f, trigger,
                    red_bv, green_bv, pink_bv,
                    cx_g, cy_g, cx_r, cy_r, cx_p, close_r, close_g):
    sep = "─" * 72
    print(sep)
    print(f"  CTR:{ctr:2d}  HDG_SP:{hdg_sp:7.1f}  IMU:{imu:7.1f}  CORR:{corr_v:6.2f}")
    print(f"  DIR:{'CW/ORANGE' if orange_flag else 'CCW/BLUE' if blue_flag else '???'}"
          f"  TRIG:{trigger}  RESET:{reset_f}  PWR:{power:.0f}")
    print(f"  TFmini  H:{tf_h:5.1f}cm  L:{tf_l:5.1f}cm  R:{tf_r:5.1f}cm")
    print(f"  DET  R:{red_bv}  G:{green_bv}  P:{pink_bv}"
          f"  CloseR:{close_r}  CloseG:{close_g}")
    print(f"  CX  G:{cx_g:5.0f}  R:{cx_r:5.0f}  P:{cx_p:5.0f}"
          f"  CY  G:{cy_g:5.0f}  R:{cy_r:5.0f}")
    print(f"  FLAGS  g:{g_flag}  r:{r_flag}  p:{p_flag}")
    print(sep)
    log.debug(f"CTR:{ctr} HDG:{hdg_sp:.0f} IMU:{imu:.1f} CORR:{corr_v:.1f} | "
              f"TF H:{tf_h:.0f} L:{tf_l:.0f} R:{tf_r:.0f} | "
              f"R:{red_bv} G:{green_bv} P:{pink_bv} | PWR:{power:.0f}")


# ══════════════════════════════════════════════════════════════════════════════
#  PROCESS S — SERVO DRIVE
#
#  KEY CHANGES vs backup:
#  1. No lidar_f/l/r → TFmini distances used directly from tfmini object
#  2. roi_turn_trigger replaces turn_trigger (camera-based, not lidar-based)
#  3. steer_to_target() replaces correctPosition() for obstacle avoidance
#  4. correctAngle() for straight-line hold (same PID, simpler interface)
#  5. Close-block emergency dodge (from Greenbotics main_v3)
#  6. Direction detection uses camera orange/blue line + TFmini fallback
# ══════════════════════════════════════════════════════════════════════════════

def servoDrive(red_b, green_b, pink_b,
               counts, centr_y, centr_x,
               centr_y_red, centr_x_red,
               centr_x_pink, centr_y_pink,
               area_green, area_red,
               close_red_b, close_green_b,
               orange_line_b, blue_line_b,
               head, roi_turn_trigger, lane_counter,
               left_f, right_f):

    pwm_h = pigpio.pi()
    global corr, corr_pos, prevError, totalError

    pwm_h.set_mode(pwm_pin,       pigpio.OUTPUT)
    pwm_h.set_mode(direction_pin, pigpio.OUTPUT)
    pwm_h.set_PWM_frequency(pwm_pin, 1000)
    pwm_h.set_PWM_dutycycle(pwm_pin, 0)

    enc = EncoderCounter()

    # ── flags ─────────────────────────────────────────────────────────────────
    button        = trigger = reset_f = False
    blue_flag     = orange_flag = False
    g_flag        = r_flag = p_flag = False
    g_past        = r_past = p_past = False
    lap_finish    = continue_parking = parking_flag = False
    finished      = exit_flag = init = initBot = False
    inParkingatStart = False
    initBot       = True

    # ── scalars ───────────────────────────────────────────────────────────────
    power              = float(DRIVE_POWER)
    prev_power         = 0.0
    counter            = 0
    heading_angle      = 0.0
    target_count       = 0
    button_STATE       = exit_STATE = 0
    debounce_delay     = 0.1
    last_time          = exit_last_time = 0.0
    itr_prev_time      = 0.0
    avoided_time       = time.time()
    reverse_until      = 0.0
    rev_count          = 0
    STATE              = 1
    parking_STATE      = 1
    RESET_STATE        = 1
    pink_wall_lane     = 0
    lane_reset         = 0
    pink_time          = 0.0
    avoid_thres        = 1.7
    green_time         = red_time = 0.0
    parking_right      = True
    parking_left       = False
    STATE_INIT         = 1
    enc_count          = 0
    start_enc_thresh   = corr_thresh = 0
    forward_time       = 0.0
    full_park          = False
    parking_count      = 5000
    parking_count_current = 0
    final_park         = time.time()
    parking_distance   = 0
    p_pass             = 0
    norm_head          = 0.0
    last_trigger_time  = 0.0
    TRIGGER_COOLDOWN   = 2.0   # seconds between corner turns

    servo.setAngle(40)

    try:
        while True:
            imu_head = head.value
            tfmini.getTFminiData()
            tf_h = tfmini.distance_head
            tf_l = tfmini.distance_left
            tf_r = tfmini.distance_right

            # ── Servo init ────────────────────────────────────────────────────
            if not init:
                if tf_h > 0 and head.value != 0:
                    correctAngle(heading_angle, head.value, 1.5)
                    log.debug("[servoDrive] servo initialised")
                    init = True
                else:
                    servo.setAngle(45)

            # ── LEDs ──────────────────────────────────────────────────────────
            if green_b.value:
                pwm_h.write(green_led, 1); pwm_h.write(red_led, 0)
            elif red_b.value:
                pwm_h.write(red_led, 1);   pwm_h.write(green_led, 0)
            elif pink_b.value:
                pwm_h.write(blue_led, 1)
            else:
                pwm_h.write(red_led, 0); pwm_h.write(green_led, 0); pwm_h.write(blue_led, 0)

            # ── DIRECTION DETECTION ───────────────────────────────────────────
            # Primary: camera floor line (orange=CW, blue=CCW)
            # Fallback: TFmini wall proximity (same as backup)
            if not left_f.value and not right_f.value:
                if orange_line_b.value:
                    right_f.value = True
                    log.info("[servoDrive] direction: CLOCKWISE (orange line seen)")
                elif blue_line_b.value:
                    left_f.value = True
                    log.info("[servoDrive] direction: CCW (blue line seen)")
                elif tf_l < 25 and tf_h < 250 and tf_l > 0 and tf_h > 0:
                    right_f.value = True; inParkingatStart = True
                    log.info("[servoDrive] direction: CW (TFmini left wall)")
                elif tf_r < 25 and tf_h < 250 and tf_r > 0 and tf_h > 0:
                    left_f.value = True; inParkingatStart = True
                    log.info("[servoDrive] direction: CCW (TFmini right wall)")

            if right_f.value and not orange_flag:
                orange_flag = True; blue_flag = False
            elif left_f.value and not blue_flag:
                blue_flag = True; orange_flag = False

            # ── LAP FINISH STOP ───────────────────────────────────────────────
            if counter == LAST_COUNTER and not lap_finish:
                reset_f = False
                if not finished:
                    target_count = counts.value + (27000 if parking_right else 22500)
                    finished = True
                if counts.value >= target_count or tf_h < FINISH_HEAD_CM:
                    pink_b.value = False
                    runMotor(pwm_h, 0, 1); time.sleep(1)
                    power = 70; prev_power = 0; lap_finish = True
                    log.info("[servoDrive] stopped — lap_finish set")

            # ── PARKING MANOEUVRE ─────────────────────────────────────────────
            if lap_finish and not continue_parking:
                correctAngle(heading_angle, head.value, 1)
                if orange_flag:
                    while tfmini.distance_head > 110:
                        itr_prev_time = time.time()
                        tfmini.getTFminiData()
                        correctAngle(heading_angle, head.value, 1)
                        runMotor(pwm_h, 50, 1)
                    if parking_STATE == 1:
                        heading_angle += 90
                        correctReverseAngle(heading_angle, head.value, 3)
                        while abs(corr) > 5:
                            itr_prev_time = time.time(); tfmini.getTFminiData()
                            pwm_h.set_PWM_dutycycle(pwm_pin, int(1.2*70*2.55))
                            pwm_h.write(direction_pin, 0)
                            correctReverseAngle(heading_angle, head.value, 3)
                        while tfmini.distance_head < 55:
                            itr_prev_time = time.time(); tfmini.getTFminiData()
                            runMotor(pwm_h, 33, 0)
                            correctReverseAngle(heading_angle, head.value, 3)
                        parking_STATE = 2
                    if parking_STATE == 2:
                        heading_angle += 90 if parking_right else -90
                        correctAngle(heading_angle, head.value, 3)
                        while abs(corr) > 5:
                            itr_prev_time = time.time(); tfmini.getTFminiData()
                            pwm_h.set_PWM_dutycycle(pwm_pin, int(1.2*70*2.55))
                            pwm_h.write(direction_pin, 1)
                            correctAngle(heading_angle, head.value, 3)
                        p_flag = True; continue_parking = True
                        parking_STATE = 3; pink_time = time.time()

                elif blue_flag:
                    while tfmini.distance_head > 160:
                        itr_prev_time = time.time(); tfmini.getTFminiData()
                        correctAngle(heading_angle, head.value, 1)
                        runMotor(pwm_h, 60, 1)
                    heading_angle -= 90
                    if parking_STATE == 1:
                        correctReverseAngle(heading_angle, head.value, 3)
                        while abs(corr) > 5:
                            itr_prev_time = time.time(); tfmini.getTFminiData()
                            runMotor(pwm_h, 36, 0)
                            correctReverseAngle(heading_angle, head.value, 3)
                        while tfmini.distance_head < 50:
                            itr_prev_time = time.time(); tfmini.getTFminiData()
                            runMotor(pwm_h, 36, 0)
                            correctReverseAngle(heading_angle, head.value, 3)
                        parking_STATE = 2
                    if parking_STATE == 2:
                        heading_angle += 90 if parking_right else -90
                        correctAngle(heading_angle, head.value, 3)
                        while abs(corr) > 5:
                            itr_prev_time = time.time(); tfmini.getTFminiData()
                            runMotor(pwm_h, 36, 1)
                            correctAngle(heading_angle, head.value, 3)
                        p_flag = True; continue_parking = True
                        parking_STATE = 3; pink_time = time.time()

            # ── BUTTON DEBOUNCE ───────────────────────────────────────────────
            if time.time() - last_time > debounce_delay:
                prev_bs = button_STATE
                button_STATE = pwm_h.read(button_pin)
                if prev_bs == 1 and button_STATE == 0:
                    button = not button; last_time = time.time()
                    power = float(DRIVE_POWER)
                    log.info(f"[servoDrive] drive {'STARTED' if button else 'STOPPED'}")

            if time.time() - exit_last_time > debounce_delay:
                prev_es = exit_STATE
                exit_STATE = pwm_h.read(exit_pin)
                if prev_es == 1 and exit_STATE == 0:
                    exit_flag = not exit_flag; exit_last_time = time.time()

            if exit_flag and button:
                log.info("[servoDrive] exit triggered — shutting down")
                sys.exit(0)

            # ══════════════════════════════════════════════════════════════════
            #  MAIN DRIVING BLOCK
            # ══════════════════════════════════════════════════════════════════
            if button:
                if initBot: time.sleep(0.2); initBot = False

                x, y = enc.get_position(imu_head, counts.value)

                # ── CLOSE-BLOCK EMERGENCY DODGE ───────────────────────────────
                # (from Greenbotics main_v3: immediate reverse swerve)
                if close_red_b.value and not reset_f:
                    log.info("[servoDrive] CLOSE RED — emergency dodge right")
                    servo.setAngle(70);  runMotor(pwm_h, 60, 0); time.sleep(0.4)
                    servo.setAngle(110); runMotor(pwm_h, 60, 1); time.sleep(0.25)
                    runMotor(pwm_h, DRIVE_POWER, 1)

                elif close_green_b.value and not reset_f:
                    log.info("[servoDrive] CLOSE GREEN — emergency dodge left")
                    servo.setAngle(110); runMotor(pwm_h, 60, 0); time.sleep(0.4)
                    servo.setAngle(70);  runMotor(pwm_h, 60, 1); time.sleep(0.25)
                    runMotor(pwm_h, DRIVE_POWER, 1)

                # ── Brief stop when block first spotted ───────────────────────
                while time.time() < avoided_time:
                    pwm_h.set_PWM_dutycycle(pwm_pin, 0)
                    pwm_h.write(direction_pin, 0)
                    itr_prev_time = time.time()
                    tfmini.getTFminiData()

                while avoided_time > 0 and time.time() < reverse_until:
                    itr_prev_time = time.time(); tfmini.getTFminiData()
                    pwm_h.set_PWM_dutycycle(pwm_pin, int(1.3*70*2.55))
                    pwm_h.write(direction_pin, 0)
                    if r_flag and not reset_f:   servo.setAngle(70)
                    elif g_flag and not reset_f: servo.setAngle(110)

                # ── FINE PARKING STATE MACHINE ────────────────────────────────
                if parking_flag:
                    tfmini.getTFminiData()
                    if STATE == 1:
                        while tfmini.distance_right > 22:
                            itr_prev_time = time.time(); tfmini.getTFminiData()
                            correctAngle(heading_angle, head.value, 1)
                            runMotor(pwm_h, 20, 1)
                        parking_count_current = counts.value
                        while counts.value <= parking_count_current + parking_count:
                            itr_prev_time = time.time()
                            runMotor(pwm_h, 28, 1)
                            correctAngle(heading_angle, head.value, 1)
                            tfmini.getTFminiData()
                        full_park = True; STATE = 2

                    if STATE == 2:
                        tfmini.getTFminiData()
                        heading_angle += -90 if parking_right else 90
                        parking_distance = tfmini.distance_left if parking_right else tfmini.distance_right
                        correctReverseAngle(heading_angle, head.value, 3)
                        while parking_distance > 20 or abs(corr) > 20:
                            itr_prev_time = time.time(); tfmini.getTFminiData()
                            parking_distance = tfmini.distance_left if parking_right else tfmini.distance_right
                            runMotor(pwm_h, 36, 0)
                            correctReverseAngle(heading_angle, head.value, 3)
                        STATE = 3

                    if STATE == 3 and full_park:
                        tfmini.getTFminiData()
                        heading_angle += 95 if parking_right else -95
                        correctReverseAngle(heading_angle, head.value, 3)
                        while abs(corr) > 5 and tfmini.distance_head < 85:
                            itr_prev_time = time.time(); tfmini.getTFminiData()
                            runMotor(pwm_h, 36, 0)
                            correctReverseAngle(heading_angle, head.value, 3)
                        correctAngle(heading_angle+2, head.value, 1)
                        while abs(corr) > 15 or tfmini.distance_head > 8:
                            correctAngle(heading_angle+2, head.value, 1)
                            runMotor(pwm_h, 15, 1); tfmini.getTFminiData()
                        STATE = 4

                    if STATE == 4:
                        runMotor(pwm_h, 0, 0)
                        log.info("[servoDrive] PARKED — exiting"); sys.exit(0)

                else:
                    # ── HEADING RESET AFTER CORNER TURN ──────────────────────
                    if reset_f and counter != LAST_COUNTER:
                        g_past = r_past = g_flag = r_flag = False
                        rev_count = 0

                        if RESET_STATE == 1:
                            # Align heading before reverse turn
                            timer_t = time.time()
                            correctAngle(heading_angle, head.value, 1.5)
                            while (abs(corr) > 8 or tfmini.distance_head > 60) \
                                    and time.time() - timer_t < 1.5:
                                itr_prev_time = time.time()
                                tfmini.getTFminiData()
                                correctAngle(heading_angle, head.value, 1.5)
                                runMotor(pwm_h, 60, 1)
                                x, y = enc.get_position(head.value, counts.value)
                            RESET_STATE = 2

                        if RESET_STATE == 2:
                            # Fine lateral position using TFmini side
                            tfmini.getTFminiData()
                            thresh_dist = tfmini.distance_right if blue_flag else tfmini.distance_left
                            angle_off   = (20 if blue_flag else -20) * (-1 if thresh_dist > 45 else 1)
                            timer_t = time.time()
                            correctAngle(heading_angle + angle_off, head.value, 1.5)
                            while (tfmini.distance_head > 20 or abs(corr) > 5) \
                                    and time.time() - timer_t < 1.8:
                                itr_prev_time = time.time()
                                tfmini.getTFminiData()
                                correctAngle(heading_angle + angle_off, head.value, 1.5)
                                runMotor(pwm_h, 100, 1)
                                x, y = enc.get_position(head.value, counts.value)
                            RESET_STATE = 3

                        if RESET_STATE == 3:
                            # Reverse corner turn
                            counter   += 1
                            lane_reset = counter % 4
                            heading_angle = update_heading(counter, heading_angle,
                                                           blue_flag, orange_flag)
                            lane_counter.value = counter
                            timer_v = time.time()
                            correctReverseAngle(heading_angle, head.value, 1)

                            while (abs(corr) > 5 or time.time() - timer_v < 1.2) \
                                    and time.time() - timer_v < 3:
                                itr_prev_time = time.time()
                                tfmini.getTFminiData()
                                correctReverseAngle(heading_angle, head.value, 2)
                                runMotor(pwm_h, TURN_POWER, 0)
                                x, y = enc.get_position(head.value, counts.value)

                            if counter == LAST_COUNTER:
                                end_t = time.time()
                                while time.time() - end_t < 1.5:
                                    itr_prev_time = time.time(); tfmini.getTFminiData()
                                    correctReverseAngle(heading_angle, head.value, 2)
                                    runMotor(pwm_h, 85, 0)

                            runMotor(pwm_h, 0, 0); time.sleep(0.5)
                            reset_f = False; RESET_STATE = 1
                            log.info(f"[servoDrive] corner done — CTR:{counter} HDG:{heading_angle}")

                    # ── ROI TURN TRIGGER ──────────────────────────────────────
                    # NEW: replaces (lidar_f<950 AND lidar_l/r>1500)
                    # Dual-gate: camera says "trigger" AND TFmini confirms wall close
                    now = time.time()
                    if (roi_turn_trigger.value
                            and not reset_f
                            and counter != LAST_COUNTER
                            and tf_h < TF_TURN_HEAD_CM
                            and now - last_trigger_time > TRIGGER_COOLDOWN):
                        reset_f           = True
                        RESET_STATE       = 1
                        last_trigger_time = now
                        avoided_time      = now + 0.3
                        log.info(f"[servoDrive] ROI TURN TRIGGER fired — "
                                 f"tf_h:{tf_h:.1f}cm  roi_trigger:{roi_turn_trigger.value}")

                    # ── PINK WALL APPROACH ────────────────────────────────────
                    if lap_finish:
                        if p_flag and continue_parking and not parking_flag:
                            power  = AVOID_POWER
                            wall_s = tfmini.distance_left if parking_right else tfmini.distance_right
                            correctWall(60, wall_s, heading_angle, head.value,
                                        orange_flag, blue_flag, pink_wall_lane,
                                        counter, parking_right, parking_left)
                            if time.time() - pink_time > 0.75:
                                tfmini.getTFminiData()
                                if parking_right and tfmini.distance_head < 160: p_pass = 2
                                elif parking_left and tfmini.distance_head > 100: p_pass = 2
                                if p_pass == 2:
                                    p_past = False; parking_flag = True

                    # ── OBSTACLE DETECT + AVOIDANCE STEERING ─────────────────
                    elif not lap_finish:

                        # Green detection
                        if green_b.value and not r_flag and not g_flag \
                                and centr_y.value > ROI_FAR_Y:
                            g_flag = True; g_past = True
                            if rev_count < 1 and 0 < centr_x.value < FRAME_MID_X:
                                avoided_time  = time.time() + 0.3
                                reverse_until = avoided_time + 0.7
                                rev_count += 1
                            log.debug(f"[servoDrive] GREEN flag  cx:{centr_x.value:.0f}")

                        elif g_past and not reset_f:
                            g_flag = True
                            if green_b.value: green_time = time.time()
                            if counter % 4 != pink_wall_lane:
                                if (tf_r <= 35 and tf_r > 0 and not green_b.value) \
                                        or time.time() - green_time > 1.7:
                                    g_flag = False; g_past = False
                            else:
                                av = 0.75 if orange_flag else 0.85
                                if (tf_r <= 20 and tf_r > 0) or time.time() - green_time > av:
                                    g_flag = False; g_past = False

                        # Red detection
                        elif red_b.value and not g_flag and not r_flag \
                                and centr_y_red.value > ROI_FAR_Y:
                            r_flag = True; r_past = True
                            if rev_count < 1 and centr_x_red.value > FRAME_MID_X + 80:
                                avoided_time  = time.time() + 0.3
                                reverse_until = avoided_time + 0.7
                            rev_count += 1
                            log.debug(f"[servoDrive] RED flag  cx:{centr_x_red.value:.0f}")

                        elif r_past and not reset_f:
                            r_flag = True
                            if red_b.value: red_time = time.time()
                            if counter % 4 != pink_wall_lane:
                                if (tf_l <= 35 and tf_l > 0 and not red_b.value) \
                                        or time.time() - red_time > 1.7:
                                    r_flag = False; r_past = False
                            else:
                                av = 0.85 if orange_flag else 0.75
                                if (tf_l <= 25 and tf_l > 0) or time.time() - red_time > av:
                                    r_flag = False; r_past = False

                        else:
                            g_flag = r_flag = p_flag = False
                            r_past = g_past = p_past = False

                        # ── STEERING SELECTION ────────────────────────────────
                        #
                        # GREEN  → steer so block is at 320 + AVOID_OFFSET_PX
                        #          (pass to the RIGHT of the green block)
                        # RED    → steer so block is at 320 - AVOID_OFFSET_PX
                        #          (pass to the LEFT of the red block)
                        # NONE   → hold heading + TFmini wall balance
                        #
                        if g_flag:
                            target_px = FRAME_MID_X + AVOID_OFFSET_PX
                            kp_pix    = min(0.09 + 3e-5 * area_green.value, 0.18)
                            steer_to_target(centr_x.value, target_px,
                                            head.value, heading_angle, kp_pix)

                        elif r_flag:
                            target_px = FRAME_MID_X - AVOID_OFFSET_PX
                            kp_pix    = min(0.09 + 3e-5 * area_red.value, 0.18)
                            steer_to_target(centr_x_red.value, target_px,
                                            head.value, heading_angle, kp_pix)

                        else:
                            # Straight-line: balance TFmini walls if both visible,
                            # else pure heading hold
                            tfmini.getTFminiData()
                            if 5 < tf_l < 120 and 5 < tf_r < 120:
                                wall_err  = tf_l - tf_r
                                wall_corr = max(-20, min(20, 0.5 * wall_err))
                                correctAngle(heading_angle + wall_corr, head.value, 1.5)
                            else:
                                correctAngle(heading_angle, head.value, 1.5)

                # ── MOTOR DRIVE ───────────────────────────────────────────────
                power     = float(AVOID_POWER) if (red_b.value or green_b.value) else float(DRIVE_POWER)
                tot_power = (power * 0.1) + (prev_power * 0.9)
                prev_power = tot_power
                pwm_h.set_PWM_dutycycle(pwm_pin, int(2.55 * tot_power))
                pwm_h.write(direction_pin, 1)

                itr_prev_time      = time.time()
                lane_counter.value = counter

                print_telemetry(counter, heading_angle, imu_head, corr,
                                tf_h, tf_l, tf_r,
                                g_flag, r_flag, p_flag,
                                orange_flag, blue_flag,
                                tot_power, reset_f, roi_turn_trigger.value,
                                red_b.value, green_b.value, pink_b.value,
                                centr_x.value, centr_y.value,
                                centr_x_red.value, centr_y_red.value,
                                centr_x_pink.value,
                                close_red_b.value, close_green_b.value)

            else:
                power = 0.0; prev_power = 0.0
                pwm_h.set_PWM_dutycycle(pwm_pin, 0)
                counter = 0

    except Exception as e:
        log.error(f"[servoDrive] exception: {e}")
        tb = traceback.extract_tb(e.__traceback__)
        fn, ln, fc, tx = tb[-1]
        log.error(f"  ↳ {fn} line {ln} in {fc}: {tx}")
    finally:
        pwm_h.set_PWM_dutycycle(pwm_pin, 0)
        pwm_h.write(direction_pin, 0)
        log.info("[servoDrive] motors stopped safely")
        pwm_h.stop()


# ══════════════════════════════════════════════════════════════════════════════
#  PROCESS E — ENCODER / IMU READER  (unchanged protocol, fixed int(float()))
# ══════════════════════════════════════════════════════════════════════════════

def runEncoder(counts, head, lane_counter, left_f, right_f):
    """
    Reads "yaw_float encoder_int\n" from ESP32 via UART.
    Applies 0.57°/lap IMU drift correction (from backup).
    Bug fixes vs backup:
      • .replace(',', '.')  → handles European decimal comma
      • int(float(x))       → handles ESP32 sending "1.0" instead of "1"
    """
    log.info("[Encoder] process started"); time.sleep(2)
    try:
        _ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        log.info(f"[Encoder] serial open {UART_PORT} @ {UART_BAUD}")
    except Exception as e:
        log.error(f"[Encoder] cannot open serial: {e}"); return

    try:
        while True:
            line     = _ser.readline().decode("utf-8", errors="ignore").strip()
            esp_data = line.split()
            if len(esp_data) >= 2:
                try:
                    yaw_raw = float(esp_data[0].replace(',', '.'))
                    enc_raw = int(float(esp_data[1]))   # handles "1.0" safely
                    lc      = lane_counter.value
                    if right_f.value:
                        head.value = yaw_raw + (0.57 * lc)
                    elif left_f.value:
                        head.value = yaw_raw - (0.57 * lc)
                    else:
                        head.value = yaw_raw
                    counts.value = enc_raw
                except ValueError:
                    log.warning(f"[Encoder] malformed: {esp_data}")
            else:
                log.warning(f"[Encoder] incomplete: {esp_data!r}")
    except Exception as e:
        log.error(f"[Encoder] exception: {e}")
    finally:
        try: _ser.close(); log.info("[Encoder] serial closed")
        except Exception: pass


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT  — 3 processes only (no LiDAR process)
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    log.info("=" * 70)
    log.info("  WRO 2025 — Obstacle Challenge ROI Edition  |  Team Paraducks")
    log.info(f"  DRIVE_POWER:{DRIVE_POWER}  TF_TURN_HEAD_CM:{TF_TURN_HEAD_CM}")
    log.info(f"  ROI_FAR_Y:{ROI_FAR_Y}  ROI_NEAR_Y:{ROI_NEAR_Y}  SMOOTH_N:{SMOOTH_N}")
    log.info(f"  AVOID_OFFSET_PX:{AVOID_OFFSET_PX}  LAST_COUNTER:{LAST_COUNTER}")
    log.info("=" * 70)

    try:
        P = multiprocessing.Process(
            target=Live_Feed_ROI,
            args=(red_b, green_b, pink_b,
                  centr_y, centr_x,
                  centr_y_red, centr_x_red,
                  centr_x_pink, centr_y_pink,
                  area_green, area_red,
                  close_red_b, close_green_b,
                  orange_line_b, blue_line_b,
                  roi_turn_trigger, lane_counter),
            name="P_Camera"
        )
        S = multiprocessing.Process(
            target=servoDrive,
            args=(red_b, green_b, pink_b,
                  counts, centr_y, centr_x,
                  centr_y_red, centr_x_red,
                  centr_x_pink, centr_y_pink,
                  area_green, area_red,
                  close_red_b, close_green_b,
                  orange_line_b, blue_line_b,
                  head, roi_turn_trigger, lane_counter,
                  left_f, right_f),
            name="S_ServoDrive"
        )
        E = multiprocessing.Process(
            target=runEncoder,
            args=(counts, head, lane_counter, left_f, right_f),
            name="E_Encoder"
        )

        P.start(); E.start(); S.start()
        log.info("All 3 processes running — P_Camera | E_Encoder | S_ServoDrive")
        log.info("(No LiDAR process — corner turns from camera ROI + TFmini)")

        P.join(); E.join(); S.join()

    except KeyboardInterrupt:
        log.info("Keyboard interrupt — terminating")
        for proc in (S, P, E):
            proc.terminate()
        for proc in (S, P, E):
            proc.join()
        _pigpio_main.set_PWM_dutycycle(pwm_pin, 0)
        _pigpio_main.stop()
        for _rx in (RX_Head, RX_Left, RX_Right, RX_Back):
            try: _pigpio_main.bb_serial_read_close(_rx)
            except Exception: pass
        log.info("Clean shutdown complete")


# ══════════════════════════════════════════════════════════════════════════════
#  QUICK REFERENCE — Tunable parameters and their locations
# ══════════════════════════════════════════════════════════════════════════════
#
#  Parameter            Default   Effect
#  ─────────────────── ────────  ───────────────────────────────────────────────
#  DRIVE_POWER          95        Overall forward speed (start at 60 for testing)
#  AVOID_POWER          55        Speed while steering around a block
#  TURN_POWER           100       Speed during corner reverse turn
#  TF_TURN_HEAD_CM      55        Front TFmini gate for corner turn trigger (cm)
#  FINISH_HEAD_CM       35        Front TFmini for final lap stop (cm)
#  ROI_FAR_Y            190       Y pixel where block activates avoidance flag
#  ROI_NEAR_Y           260       Y pixel where roi_turn_trigger can fire
#  AVOID_OFFSET_PX      150       How far (px) to pass beside each block
#  SMOOTH_N             2         Frames for majority-vote smoothing
#  NO_BLOCK_FRAMES      20        Frames of "no block" before trigger fires
#  TRIGGER_COOLDOWN     2.0       Min seconds between two corner turns
#  kp / kd              0.6/0.1   Heading-hold PID gains
#  kp_pix (steer)       0.09      Pixel→servo gain during avoidance
#
# ══════════════════════════════════════════════════════════════════════════════
