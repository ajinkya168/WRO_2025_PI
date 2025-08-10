import cv2, time, numpy as np
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import common, detect
from pycoral.utils.dataset import read_label_file
from itertools import combinations
MODEL_PATH = "/home/pi/WRO_2025_PI/limelight_neural_detector_8bit.tflite"
LABELS     = "/home/pi/WRO_2025_PI/label_map.txt"   # put your label file here (id -> name), or set to None
CONF_TH    = 0.5
CAM_INDEX  = 0

# Load model
interpreter = make_interpreter(MODEL_PATH)
interpreter.allocate_tensors()
ih, iw = common.input_size(interpreter)

# Labels (optional)
labels = {}
if LABELS:
    try:
        labels = read_label_file(LABELS)  # {id: "name"}
    except Exception as e:
        print("Label load warn:", e)
FPS = 120
cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
cap.set(cv2.CAP_PROP_FPS,          FPS)

cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # low-latency
cls_name = None
t_prev = time.time()
pairs = []
dets =[]
try:
    while True:
        ok, frame_bgr = cap.read()
        if not ok:
            break
        H, W = frame_bgr.shape[:2]

        # Preprocess
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        inp = cv2.resize(rgb, (iw, ih))
        common.set_input(interpreter, inp)
        interpreter.invoke()

        # Decode detections
        # get_objects returns list of Obj with bbox in input space (iw, ih)
        objs = detect.get_objects(interpreter, score_threshold=CONF_TH)
        scale_x, scale_y = W / float(iw), H / float(ih)
        
        dets = []
        for obj in objs:
            name = labels.get(obj.id, str(obj.id))
            dets.append((name))
        pairs=[]
        if len(dets) >= 2:
            # Normal case: take only the first detected pair
            pairs.append((dets[0], dets[1]))
        elif len(dets) == 1:
            # Only one detection → second is None
            pairs.append((dets[0], None))
        else:
            # No detections at all
            pairs.append((None, None))

        now = time.time()
        fps = 1.0 / max(1e-3, (now - t_prev)); t_prev = now
        print(f"cls_name:{cls_name} fps: {fps} pair:{pairs}")
        #cv2.imshow("Coral SSD Live", frame_bgr)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
