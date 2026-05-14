"""
hsv_calibrate.py  —  WRO 2025  |  HSV Calibration Tool
=======================================================
STEP 1: Run this FIRST to calibrate HSV ranges for your venue lighting.
        Do this before every competition / practice session.

Streams a live split-view to your browser:
  LEFT  = annotated camera feed (contours drawn)
  RIGHT = HSV mask (white = detected, black = not)

Use the trackbars on the right side of the browser to tune each colour.
Press S in terminal to save and print the final values.

INSTALL (one time):
    source /home/pi/coral-py39-env/bin/activate
    pip install flask

RUN:
    source /home/pi/coral-py39-env/bin/activate
    python3 /home/pi/WRO_2025_PI/hsv_calibrate.py

OPEN ON LAPTOP:
    http://192.168.0.132:5000

CONTROLS:
    In terminal — type the colour to calibrate:
        r  → calibrate RED
        g  → calibrate GREEN
        p  → calibrate PINK
        s  → save + print current values (copy into detection_test.py)
    Ctrl+C → quit
"""

import cv2
import numpy as np
import time
import threading
import sys
from flask import Flask, Response, render_template_string

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────────────────
CAM_INDEX = 0       # change to 0/1/2 to match your camera
CAP_W, CAP_H = 640, 360
FLASK_PORT   = 5000
JPEG_QUALITY = 65

# Starting HSV ranges (will be tuned via trackbars)
RANGES = {
    "RED_1" : {"hl": 0,   "hh": 5,  "sl": 155, "sh": 255, "vl": 94,  "vh": 255},
    "RED_2" : {"hl": 172, "hh": 180, "sl": 122, "sh": 255, "vl": 60,  "vh": 255},
    "GREEN" : {"hl": 40,  "hh": 90,  "sl": 80,  "sh": 255, "vl": 40,  "vh": 200},
    "PINK"  : {"hl": 135, "hh": 175, "sl": 70,  "sh": 255, "vl": 60,  "vh": 255},
}

# BGR display colours
C = {"RED": (0,0,220), "GREEN": (0,210,0), "PINK": (220,0,220),
     "WHITE": (255,255,255), "BLACK": (0,0,0)}

# ─────────────────────────────────────────────────────────────────────────────
#  SHARED STATE
# ─────────────────────────────────────────────────────────────────────────────
state = {
    "jpeg"       : None,
    "lock"       : threading.Lock(),
    "active"     : "RED",       # which colour is being calibrated
    "trackbars"  : dict(RANGES),
}

# ─────────────────────────────────────────────────────────────────────────────
#  FLASK
# ─────────────────────────────────────────────────────────────────────────────
app = Flask(__name__)

HTML = """<!DOCTYPE html>
<html>
<head>
  <title>WRO HSV Calibrator</title>
  <style>
    body { background:#111; color:#eee; font-family:monospace;
           display:flex; flex-direction:column; align-items:center;
           padding:10px; gap:8px; }
    h2   { color:#ffdd00; margin:4px 0; }
    img  { width:100%; max-width:1280px; border:2px solid #333; border-radius:4px; }

    .controls { display:flex; flex-wrap:wrap; gap:16px;
                justify-content:center; width:100%; max-width:1280px; }
    .group  { background:#1a1a1a; border:1px solid #333; border-radius:6px;
              padding:12px; min-width:280px; }
    .group h3 { margin:0 0 8px; font-size:0.9rem; }
    .red   { border-color:#cc0000; }  .red   h3 { color:#ff5555; }
    .green { border-color:#00cc00; }  .green h3 { color:#44ff44; }
    .pink  { border-color:#cc00cc; }  .pink  h3 { color:#ff55ff; }

    label  { display:flex; justify-content:space-between;
             font-size:0.75rem; margin:3px 0; }
    input[type=range] { width:180px; accent-color:#ffdd00; }
    .val   { width:30px; text-align:right; color:#ffdd00; }

    .btn   { padding:6px 18px; border-radius:4px; border:none;
             font-family:monospace; font-weight:bold; cursor:pointer;
             font-size:0.85rem; }
    .save  { background:#226622; color:#44ff44; border:1px solid #44ff44; }
    .note  { font-size:0.65rem; color:#555; margin-top:4px; }
  </style>
</head>
<body>
  <h2>WRO 2025 — HSV Calibrator &nbsp;|&nbsp; LEFT: camera &nbsp;|&nbsp; RIGHT: mask</h2>
  <img src="/video_feed" />

  <div class="controls">

    <!-- RED -->
    <div class="group red">
      <h3>🔴 RED (Range 1: H 0-10)</h3>
      <label>H low  <input type="range" id="r1hl" min="0" max="20"  value="0"   oninput="send('R1_HL',this.value)"><span class="val" id="r1hl_v">0</span></label>
      <label>H high <input type="range" id="r1hh" min="0" max="20"  value="10"  oninput="send('R1_HH',this.value)"><span class="val" id="r1hh_v">10</span></label>
      <label>S low  <input type="range" id="r1sl" min="0" max="255" value="120" oninput="send('R1_SL',this.value)"><span class="val" id="r1sl_v">120</span></label>
      <label>V low  <input type="range" id="r1vl" min="0" max="255" value="60"  oninput="send('R1_VL',this.value)"><span class="val" id="r1vl_v">60</span></label>
      <h3 style="margin-top:10px">🔴 RED (Range 2: H 170-180)</h3>
      <label>H low  <input type="range" id="r2hl" min="160" max="180" value="170" oninput="send('R2_HL',this.value)"><span class="val" id="r2hl_v">170</span></label>
      <label>H high <input type="range" id="r2hh" min="160" max="180" value="180" oninput="send('R2_HH',this.value)"><span class="val" id="r2hh_v">180</span></label>
      <label>S low  <input type="range" id="r2sl" min="0" max="255" value="120" oninput="send('R2_SL',this.value)"><span class="val" id="r2sl_v">120</span></label>
      <label>V low  <input type="range" id="r2vl" min="0" max="255" value="60"  oninput="send('R2_VL',this.value)"><span class="val" id="r2vl_v">60</span></label>
    </div>

    <!-- GREEN -->
    <div class="group green">
      <h3>🟢 GREEN</h3>
      <label>H low  <input type="range" id="ghl" min="30" max="90"  value="40"  oninput="send('G_HL',this.value)"><span class="val" id="ghl_v">40</span></label>
      <label>H high <input type="range" id="ghh" min="30" max="100" value="90"  oninput="send('G_HH',this.value)"><span class="val" id="ghh_v">90</span></label>
      <label>S low  <input type="range" id="gsl" min="0" max="255"  value="80"  oninput="send('G_SL',this.value)"><span class="val" id="gsl_v">80</span></label>
      <label>S high <input type="range" id="gsh" min="0" max="255"  value="255" oninput="send('G_SH',this.value)"><span class="val" id="gsh_v">255</span></label>
      <label>V low  <input type="range" id="gvl" min="0" max="255"  value="40"  oninput="send('G_VL',this.value)"><span class="val" id="gvl_v">40</span></label>
      <label>V high <input type="range" id="gvh" min="0" max="255"  value="200" oninput="send('G_VH',this.value)"><span class="val" id="gvh_v">200</span></label>
    </div>

    <!-- PINK -->
    <div class="group pink">
      <h3>🩷 PINK / MAGENTA</h3>
      <label>H low  <input type="range" id="phl" min="100" max="179" value="135" oninput="send('P_HL',this.value)"><span class="val" id="phl_v">135</span></label>
      <label>H high <input type="range" id="phh" min="100" max="179" value="175" oninput="send('P_HH',this.value)"><span class="val" id="phh_v">175</span></label>
      <label>S low  <input type="range" id="psl" min="0" max="255"   value="70"  oninput="send('P_SL',this.value)"><span class="val" id="psl_v">70</span></label>
      <label>S high <input type="range" id="psh" min="0" max="255"   value="255" oninput="send('P_SH',this.value)"><span class="val" id="psh_v">255</span></label>
      <label>V low  <input type="range" id="pvl" min="0" max="255"   value="60"  oninput="send('P_VL',this.value)"><span class="val" id="pvl_v">60</span></label>
      <label>V high <input type="range" id="pvh" min="0" max="255"   value="255" oninput="send('P_VH',this.value)"><span class="val" id="pvh_v">255</span></label>
    </div>

  </div>

  <button class="btn save" onclick="saveCurrent()">💾 SAVE — Print values to terminal</button>
  <div class="note">After saving, copy the printed values into detection_test.py</div>

<script>
function send(key, val) {
  document.getElementById(key.toLowerCase().replace('_','') + '_v') &&
    (document.getElementById(key.toLowerCase().replace('_','') + '_v').innerText = val);
  // update display val
  const vid = key.toLowerCase().replace(/_/g,'') + '_v';
  const el = document.getElementById(vid);
  if(el) el.innerText = val;
  fetch('/update?key=' + key + '&val=' + val);
}
function saveCurrent() {
  fetch('/save').then(r => r.text()).then(t => alert('Saved! Check terminal for values.'));
}
</script>
</body>
</html>"""

@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/update")
def update():
    from flask import request
    key = request.args.get("key", "")
    val = int(request.args.get("val", 0))
    tb  = state["trackbars"]

    mapping = {
        "R1_HL": ("RED_1", "hl"), "R1_HH": ("RED_1", "hh"),
        "R1_SL": ("RED_1", "sl"), "R1_VL": ("RED_1", "vl"),
        "R2_HL": ("RED_2", "hl"), "R2_HH": ("RED_2", "hh"),
        "R2_SL": ("RED_2", "sl"), "R2_VL": ("RED_2", "vl"),
        "G_HL" : ("GREEN", "hl"), "G_HH" : ("GREEN", "hh"),
        "G_SL" : ("GREEN", "sl"), "G_SH" : ("GREEN", "sh"),
        "G_VL" : ("GREEN", "vl"), "G_VH" : ("GREEN", "vh"),
        "P_HL" : ("PINK",  "hl"), "P_HH" : ("PINK",  "hh"),
        "P_SL" : ("PINK",  "sl"), "P_SH" : ("PINK",  "sh"),
        "P_VL" : ("PINK",  "vl"), "P_VH" : ("PINK",  "vh"),
    }
    if key in mapping:
        grp, field = mapping[key]
        tb[grp][field] = val
    return "ok"

@app.route("/save")
def save():
    tb = state["trackbars"]
    print("\n" + "=" * 58)
    print("  COPY THESE INTO detection_test.py")
    print("=" * 58)
    r1, r2, g, p = tb["RED_1"], tb["RED_2"], tb["GREEN"], tb["PINK"]
    print(f"RED_LOWER_1   = np.array([{r1['hl']:3d}, {r1['sl']:3d}, {r1['vl']:3d}], dtype=np.uint8)")
    print(f"RED_UPPER_1   = np.array([{r1['hh']:3d}, 255, 255], dtype=np.uint8)")
    print(f"RED_LOWER_2   = np.array([{r2['hl']:3d}, {r2['sl']:3d}, {r2['vl']:3d}], dtype=np.uint8)")
    print(f"RED_UPPER_2   = np.array([{r2['hh']:3d}, 255, 255], dtype=np.uint8)")
    print(f"GREEN_LOWER   = np.array([{g['hl']:3d}, {g['sl']:3d}, {g['vl']:3d}], dtype=np.uint8)")
    print(f"GREEN_UPPER   = np.array([{g['hh']:3d}, {g['sh']:3d}, {g['vh']:3d}], dtype=np.uint8)")
    print(f"PINK_LOWER    = np.array([{p['hl']:3d}, {p['sl']:3d}, {p['vl']:3d}], dtype=np.uint8)")
    print(f"PINK_UPPER    = np.array([{p['hh']:3d}, {p['sh']:3d}, {p['vh']:3d}], dtype=np.uint8)")
    print("=" * 58 + "\n")
    sys.stdout.flush()
    return "saved"

def generate_frames():
    while True:
        with state["lock"]:
            jpeg = state["jpeg"]
        if jpeg is None:
            time.sleep(0.01)
            continue
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


# ─────────────────────────────────────────────────────────────────────────────
#  MASK GENERATION
# ─────────────────────────────────────────────────────────────────────────────
def build_masks(frame_bgr):
    """Build red, green, pink masks from current trackbar values."""
    tb  = state["trackbars"]
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    r1 = tb["RED_1"]; r2 = tb["RED_2"]
    g  = tb["GREEN"]; p  = tb["PINK"]

    # Red — two ranges OR-ed
    m_r1 = cv2.inRange(hsv,
                       np.array([r1["hl"], r1["sl"], r1["vl"]], np.uint8),
                       np.array([r1["hh"], 255,      255      ], np.uint8))
    m_r2 = cv2.inRange(hsv,
                       np.array([r2["hl"], r2["sl"], r2["vl"]], np.uint8),
                       np.array([r2["hh"], 255,      255      ], np.uint8))
    mask_r = cv2.bitwise_or(m_r1, m_r2)

    mask_g = cv2.inRange(hsv,
                         np.array([g["hl"], g["sl"], g["vl"]], np.uint8),
                         np.array([g["hh"], g["sh"], g["vh"]], np.uint8))

    mask_p = cv2.inRange(hsv,
                         np.array([p["hl"], p["sl"], p["vl"]], np.uint8),
                         np.array([p["hh"], p["sh"], p["vh"]], np.uint8))

    return mask_r, mask_g, mask_p


def draw_contours_on(frame, mask, colour, label):
    """Draw contours + centroid from mask onto frame."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area < 300:
            continue
        cv2.drawContours(frame, [c], -1, colour, 2)
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(frame, (x, y), (x+w, y+h), colour, 1)
        M = cv2.moments(c)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 5, colour, -1)
            cv2.putText(frame, f"{label} A:{area:.0f}",
                        (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.42, colour, 1, cv2.LINE_AA)


# ─────────────────────────────────────────────────────────────────────────────
#  VISION THREAD
# ─────────────────────────────────────────────────────────────────────────────
def vision_loop():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"[Camera] FAILED — try changing CAM_INDEX (currently {CAM_INDEX})")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,   CAP_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  CAP_H)
    cap.set(cv2.CAP_PROP_FPS,           60)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)

    print(f"[Camera] {int(cap.get(3))}x{int(cap.get(4))}")

    while True:
        cap.grab()
        ret, frame = cap.retrieve()
        if not ret:
            continue

        H, W = frame.shape[:2]

        # Build masks
        mask_r, mask_g, mask_p = build_masks(frame)

        # Combined colour mask for display (right panel)
        combined_mask = np.zeros((H, W, 3), dtype=np.uint8)
        combined_mask[mask_r > 0] = C["RED"]
        combined_mask[mask_g > 0] = C["GREEN"]
        combined_mask[mask_p > 0] = C["PINK"]

        # Annotated left panel
        annotated = frame.copy()
        draw_contours_on(annotated, mask_r, C["RED"],   "RED")
        draw_contours_on(annotated, mask_g, C["GREEN"], "GREEN")
        draw_contours_on(annotated, mask_p, C["PINK"],  "PINK")

        # Labels on annotated
        cv2.putText(annotated, "CAMERA + CONTOURS",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, C["WHITE"], 1, cv2.LINE_AA)
        cv2.putText(combined_mask, "COLOUR MASK (R/G/P)",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, C["WHITE"], 1, cv2.LINE_AA)

        # Side by side
        combined = np.hstack([annotated, combined_mask])

        ok, jpeg_buf = cv2.imencode(
            ".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            with state["lock"]:
                state["jpeg"] = jpeg_buf.tobytes()

    cap.release()


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        pi_ip = s.getsockname()[0]
        s.close()
    except Exception:
        pi_ip = "<pi-ip>"

    print("=" * 58)
    print("  WRO 2025 — HSV Calibrator")
    print("=" * 58)
    print(f"  Camera  : index {CAM_INDEX}")
    print()
    print(f"  ► Open browser:  http://{pi_ip}:{FLASK_PORT}")
    print()
    print("  Adjust sliders in browser until ONLY the target colour")
    print("  is white in the mask panel.")
    print("  Click SAVE button → copy values from terminal → paste")
    print("  into detection_test.py CONFIG section.")
    print()
    print("  Ctrl+C to quit")
    print("=" * 58)

    vt = threading.Thread(target=vision_loop, daemon=True)
    vt.start()

    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    app.run(host="0.0.0.0", port=FLASK_PORT,
            debug=False, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()