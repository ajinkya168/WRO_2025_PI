"""
hsv_calibrate_all.py
====================
Live HSV calibration for all 5 colours used in WRO obstacle challenge.
Switch colours with keyboard. Drag trackbars. Press S to print values.

Controls:
    1  →  RED
    2  →  GREEN
    3  →  PINK
    4  →  ORANGE
    5  →  BLUE
    s  →  print current HSV values to terminal
    q  →  quit
"""

import cv2
import numpy as np

CAM_INDEX = 0
W, H = 640, 360

def nothing(x):
    pass

COLOURS = {
    "1_RED":    {"lower": [0,  120,  60], "upper": [10,  255, 255], "display": (0, 0, 220)},
    "2_GREEN":  {"lower": [40,  80,  40], "upper": [90,  255, 200], "display": (0, 220, 0)},
    "3_PINK":   {"lower": [148, 70,  60], "upper": [168, 255, 255], "display": (220, 0, 220)},
    "4_ORANGE": {"lower": [6,   70,  20], "upper": [26,  255, 255], "display": (0, 140, 255)},
    "5_BLUE":   {"lower": [94,  45,  58], "upper": [140, 226, 185], "display": (255, 0, 0)},
}
KEY_MAP = {ord("1"): "1_RED", ord("2"): "2_GREEN", ord("3"): "3_PINK",
           ord("4"): "4_ORANGE", ord("5"): "5_BLUE"}

current = "1_RED"
WIN = "HSV Calibrate"

cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,   W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  H)
cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE,      -6)

cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, W * 2, H + 60)

def set_trackbars(name):
    d = COLOURS[name]
    cv2.setTrackbarPos("H_low",  WIN, d["lower"][0])
    cv2.setTrackbarPos("H_high", WIN, d["upper"][0])
    cv2.setTrackbarPos("S_low",  WIN, d["lower"][1])
    cv2.setTrackbarPos("S_high", WIN, d["upper"][1])
    cv2.setTrackbarPos("V_low",  WIN, d["lower"][2])
    cv2.setTrackbarPos("V_high", WIN, d["upper"][2])

cv2.createTrackbar("H_low",  WIN,   0, 179, nothing)
cv2.createTrackbar("H_high", WIN, 179, 179, nothing)
cv2.createTrackbar("S_low",  WIN,   0, 255, nothing)
cv2.createTrackbar("S_high", WIN, 255, 255, nothing)
cv2.createTrackbar("V_low",  WIN,   0, 255, nothing)
cv2.createTrackbar("V_high", WIN, 255, 255, nothing)
set_trackbars(current)

print("Keys:  1=RED  2=GREEN  3=PINK  4=ORANGE  5=BLUE  s=save  q=quit")

while True:
    cap.grab()
    ok, frame = cap.retrieve()
    if not ok:
        continue

    hl = cv2.getTrackbarPos("H_low",  WIN)
    hh = cv2.getTrackbarPos("H_high", WIN)
    sl = cv2.getTrackbarPos("S_low",  WIN)
    sh = cv2.getTrackbarPos("S_high", WIN)
    vl = cv2.getTrackbarPos("V_low",  WIN)
    vh = cv2.getTrackbarPos("V_high", WIN)

    lower = np.array([hl, sl, vl], dtype=np.uint8)
    upper = np.array([hh, sh, vh], dtype=np.uint8)

    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # RED wraps — combine two masks when in RED mode
    if current == "1_RED":
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower, upper),
            cv2.inRange(hsv, np.array([170, sl, vl]), np.array([180, sh, vh]))
        )
        # Live pink exclusion preview — subtract saved pink range from red mask
        _pk = COLOURS["3_PINK"]
        mask_pink = cv2.inRange(hsv,
                                np.array(_pk["lower"], dtype=np.uint8),
                                np.array(_pk["upper"], dtype=np.uint8))
        _ex_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_pink_dilated = cv2.dilate(mask_pink, _ex_k, iterations=1)
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(mask_pink_dilated))
    else:
        mask = cv2.inRange(hsv, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    overlay = frame.copy()
    col     = COLOURS[current]["display"]
    conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in conts:
        if cv2.contourArea(c) > 300:
            cv2.drawContours(overlay, [c], -1, col, 2)
            M = cv2.moments(c)
            if M["m00"]:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(overlay, (cx, cy), 6, (255, 255, 255), -1)
                cv2.putText(overlay, f"A:{int(cv2.contourArea(c))}",
                            (cx - 20, cy - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # HUD bar at top of overlay
    label = f"  [{current.split('_')[1]}]  H:{hl}-{hh}  S:{sl}-{sh}  V:{vl}-{vh}  " \
            f"| 1=RED 2=GRN 3=PNK 4=ORG 5=BLU | s=save q=quit"
    cv2.rectangle(overlay, (0, 0), (W, 22), (30, 30, 30), -1)
    cv2.putText(overlay, label, (4, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    combined = np.hstack([overlay, mask_bgr])
    cv2.imshow(WIN, combined)

    key = cv2.waitKey(1) & 0xFF

    if key in KEY_MAP:
        # Save current slider state back to dict before switching
        COLOURS[current]["lower"] = [hl, sl, vl]
        COLOURS[current]["upper"] = [hh, sh, vh]
        current = KEY_MAP[key]
        set_trackbars(current)
        print(f"Switched to: {current}")

    elif key == ord("s"):
        # Save current sliders then print ALL colours
        COLOURS[current]["lower"] = [hl, sl, vl]
        COLOURS[current]["upper"] = [hh, sh, vh]
        print("\n# ── Paste into Obstacle_Challenge_ROI.py ────────────────────")
        c = COLOURS["1_RED"]
        print(f"RED_LOWER_1  = np.array([{c['lower'][0]:3d}, {c['lower'][1]:3d}, {c['lower'][2]:3d}], dtype=np.uint8)")
        print(f"RED_UPPER_1  = np.array([{c['upper'][0]:3d}, {c['upper'][1]:3d}, {c['upper'][2]:3d}], dtype=np.uint8)")
        print(f"RED_LOWER_2  = np.array([170, {c['lower'][1]:3d}, {c['lower'][2]:3d}], dtype=np.uint8)")
        print(f"RED_UPPER_2  = np.array([180, {c['upper'][1]:3d}, {c['upper'][2]:3d}], dtype=np.uint8)")
        for key_name, var_prefix in [("2_GREEN","GREEN"), ("3_PINK","PINK"),
                                      ("4_ORANGE","ORANGE"), ("5_BLUE","BLUE")]:
            c = COLOURS[key_name]
            print(f"{var_prefix}_LOWER = np.array([{c['lower'][0]:3d}, {c['lower'][1]:3d}, {c['lower'][2]:3d}], dtype=np.uint8)")
            print(f"{var_prefix}_UPPER = np.array([{c['upper'][0]:3d}, {c['upper'][1]:3d}, {c['upper'][2]:3d}], dtype=np.uint8)")
        print()

    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

