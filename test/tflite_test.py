#!/usr/bin/env python3
import time, os, cv2, numpy as np
from types import SimpleNamespace

# ================== USER CONFIG ==================
MODEL_PATH = "/home/pi/WRO_2025_PI_SUJAL/limelight_neural_detector_8bit.tflite"
LABELS     = "/home/pi/WRO_2025_PI_SUJAL/label_map.txt"   # line-based file: one label per line (0-based)
CONF_TH    = 0.69
CAM_INDEX  = 0
FRAME_W, FRAME_H, FPS = 640, 360, 120
# =================================================

# Prefer tflite-runtime on Pi; fallback to tensorflow.lite
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow as tf
    tflite = tf.lite

# ---------- Minimal "common" helpers (PyCoral-free) ----------
class common:
    @staticmethod
    def input_size(interpreter):
        det = interpreter.get_input_details()[0]["shape"]
        # shape: [1, H, W, C]
        return int(det[1]), int(det[2])

    @staticmethod
    def set_input(interpreter, rgb_image):
        """rgb_image = HxWx3 uint8 or float32 (we'll adapt)."""
        in_det = interpreter.get_input_details()[0]
        ih, iw = int(in_det["shape"][1]), int(in_det["shape"][2])
        img = cv2.resize(rgb_image, (iw, ih))
        if in_det["dtype"] == np.float32:
            tensor = np.expand_dims(img.astype(np.float32) / 255.0, 0)
        else:
            tensor = np.expand_dims(img.astype(np.uint8), 0)
        interpreter.set_tensor(in_det["index"], tensor)

# ---------- Minimal "detect" helpers (PyCoral-free) ----------
class BBox(SimpleNamespace):
    # fields: xmin, ymin, xmax, ymax (in input-size coords)
    pass

class Obj(SimpleNamespace):
    # fields: id (class), score, bbox (BBox)
    pass

def _squeeze(x):
    x = np.array(x)
    if x.ndim >= 1 and x.shape[0] == 1:
        x = np.squeeze(x, axis=0)
    if x.size == 1:
        return np.squeeze(x)
    return x

def _autodetect_output_indices(interpreter):
    """Return indices (boxes_idx, classes_idx, scores_idx, count_idx or None) robustly."""
    out_det = interpreter.get_output_details()
    boxes_idx = classes_idx = scores_idx = count_idx = None

    # Pass 1: by shapes / ranges
    for o in out_det:
        arr = _squeeze(interpreter.get_tensor(o["index"]))
        if arr.ndim == 2 and arr.shape[-1] == 4 and boxes_idx is None:
            boxes_idx = o["index"]
        elif arr.ndim == 1 and arr.size > 1:
            if arr.dtype.kind == 'f' and np.all((arr >= 0.0) & (arr <= 1.0)) and scores_idx is None:
                scores_idx = o["index"]
            elif classes_idx is None:
                classes_idx = o["index"]
        elif arr.size == 1 and count_idx is None:
            count_idx = o["index"]

    # Pass 2: fallback to typical TF2 order if missing
    if boxes_idx is None or classes_idx is None or scores_idx is None:
        if len(out_det) >= 3:
            boxes_idx   = out_det[0]["index"] if boxes_idx   is None else boxes_idx
            classes_idx = out_det[1]["index"] if classes_idx is None else classes_idx
            scores_idx  = out_det[2]["index"] if scores_idx  is None else scores_idx

    return boxes_idx, classes_idx, scores_idx, count_idx

def _read_label_file(path):
    """Return dict {class_id:int -> name:str} for 0-based IDs."""
    if not path or not os.path.isfile(path):
        return {}
    names = []
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                parts = ln.split()
                # Accept "0 red" or "red"
                if parts[0].isdigit():
                    names.append(" ".join(parts[1:]))
                else:
                    names.append(ln)
    return {i: n for i, n in enumerate(names)}

def get_objects(interpreter, score_threshold=0.5):
    """CPU version of detect.get_objects. Returns list[Obj] with bbox in input space."""
    boxes_idx, classes_idx, scores_idx, count_idx = _autodetect_output_indices(interpreter)
    if boxes_idx is None or classes_idx is None or scores_idx is None:
        return []

    boxes   = _squeeze(interpreter.get_tensor(boxes_idx))    # [N,4] (ymin,xmin,ymax,xmax) normalized
    classes = _squeeze(interpreter.get_tensor(classes_idx))  # [N] (float or int)
    scores  = _squeeze(interpreter.get_tensor(scores_idx))   # [N] (float 0..1)

    # Handle shapes
    if boxes.ndim == 1 and boxes.size == 4:
        boxes = boxes.reshape(1, 4)
    if classes.ndim == 0:
        classes = np.array([classes])
    if scores.ndim == 0:
        scores = np.array([scores])

    # num detections if present
    if count_idx is not None:
        num = int(round(float(_squeeze(interpreter.get_tensor(count_idx)))))
    else:
        num = min(len(scores), len(boxes), len(classes))

    N = min(num, len(scores), len(boxes), len(classes))
    # Input size for bbox scaling to input coords
    ih, iw = common.input_size(interpreter)

    # Ensure int classes
    if classes.dtype.kind == 'f':
        classes = np.rint(classes).astype(np.int32)
    else:
        classes = classes.astype(np.int32)

    objs = []
    for i in range(N):
        sc = float(scores[i])
        if sc < score_threshold:
            continue
        ymin, xmin, ymax, xmax = [float(v) for v in boxes[i]]
        # Map normalized → input space (iw, ih)
        x1, y1 = int(xmin * iw), int(ymin * ih)
        x2, y2 = int(xmax * iw), int(ymax * ih)
        bbox = BBox(xmin=x1, ymin=y1, xmax=x2, ymax=y2)
        objs.append(Obj(id=int(classes[i]), score=sc, bbox=bbox))
    return objs

# ---------- Interpreter factory (CPU only) ----------
def make_interpreter(model_path):
    return tflite.Interpreter(model_path=model_path)

# ---------- MAIN LOOP (kept as close to your logic as possible) ----------
def main():
    # Load model
    interpreter = make_interpreter(MODEL_PATH)
    interpreter.allocate_tensors()
    ih, iw = common.input_size(interpreter)
    frames, t0 = 0, time.time()
    # Labels (optional)
    labels = {}
    if LABELS:
        try:
            labels = _read_label_file(LABELS)  # {id: "name"}  0->green,1->pink,2->red
        except Exception as e:
            print("Label load warn:", e)

    # Camera
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS,          FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

    # ----- placeholders for your shared-values/flags -----
    # Replace with your actual multiprocessing.Value(...) or GPIO etc.
    red_b   = SimpleNamespace(value=False)
    green_b = SimpleNamespace(value=False)
    pink_b  = SimpleNamespace(value=False)
    centr_x = SimpleNamespace(value=0)
    centr_y = SimpleNamespace(value=0)
    centr_x_red  = SimpleNamespace(value=0)
    centr_y_red  = SimpleNamespace(value=0)
    centr_x_pink = SimpleNamespace(value=0)
    centr_y_pink = SimpleNamespace(value=0)
    # -----------------------------------------------------

    t_prev = time.time()
    try:
        while True:
            ok, frame_bgr = cap.read()
            if not ok:
                break
            H, W = frame_bgr.shape[:2]
            t_inf0 = time.time()
            
            # Preprocess & run
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            common.set_input(interpreter, rgb)
            interpreter.invoke()
            t_inf1 = time.time()
            scale_x, scale_y = W / float(iw), H / float(ih)

            # Decode detections (CPU version)
            objs = get_objects(interpreter, score_threshold=CONF_TH)

            det = []
            for obj in objs:
                bbox = obj.bbox  # in input space (iw, ih)
                x1 = int(bbox.xmin * scale_x)
                y1 = int(bbox.ymin * scale_y)
                x2 = int(bbox.xmax * scale_x)
                y2 = int(bbox.ymax * scale_y)
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                area = max(0, (x2 - x1)) * max(0, (y2 - y1))
                name = labels.get(obj.id, str(obj.id))
                if area >= 1000:
                    det.append((name, cx, cy, area))
                cv2.rectangle(frame_bgr, (x1,y1), (x2,y2), (0,255,0), 2)

                cv2.putText(frame_bgr, f"{name}", (x1, max(15,y1-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            det.sort(key=lambda d: d[3], reverse=True)

            # Pair logic (unchanged)
            if len(det) >= 2:
                pair = (det[0], det[1])
            elif len(det) == 1:
                pair = (det[0], None)
            else:
                pair = (None, None)

            n1 = pair[0][0] if pair[0] else None
            n2 = pair[1][0] if pair[1] else None

            if n1 == 'pink' and n2 is None:
                pink_b.value = True
                if n1 == 'pink':
                    centr_x_pink.value = pair[0][1]
                    centr_y_pink.value = pair[0][2]
            else:
                pink_b.value = False
                centr_x_pink.value = 0
                centr_y_pink.value = 0

            if (n1, n2) in (('pink', 'red'), ('red', 'pink')):
                red_b.value = True
                green_b.value = False
                pink_b.value = True
                if n1 == 'red':
                    centr_x_red.value = pair[0][1]
                    centr_y_red.value = pair[0][2]
                elif n2 == 'red':
                    centr_x_red.value = pair[1][1]
                    centr_y_red.value = pair[1][2]
                if n1 == 'pink':
                    centr_x_pink.value = pair[0][1]
                    centr_y_pink.value = pair[0][2]
                elif n2 == 'pink':
                    centr_x_pink.value = pair[1][1]
                    centr_y_pink.value = pair[1][2]

            elif (n1, n2) in (('pink', 'green'), ('green', 'pink')):
                green_b.value = True
                red_b.value = False
                pink_b.value = True
                if n1 == 'green':
                    centr_x.value = pair[0][1]
                    centr_y.value = pair[0][2]
                elif n2 == 'green':
                    centr_x.value = pair[1][1]
                    centr_y.value = pair[1][2]
                if n1 == 'pink':
                    centr_x_pink.value = pair[0][1]
                    centr_y_pink.value = pair[0][2]
                elif n2 == 'pink':
                    centr_x_pink.value = pair[1][1]
                    centr_y_pink.value = pair[1][2]

            elif (n1, n2) in (('green', 'red'), ('green', None)):
                green_b.value = True
                red_b.value = False
                pink_b.value = False
                if n1 == 'green':
                    centr_x.value = pair[0][1]
                    centr_y.value = pair[0][2]
                centr_x_pink.value = 0
                centr_y_pink.value = 0

            elif (n1, n2) in (('red', 'green'), ('red', None)):
                red_b.value = True
                green_b.value = False
                pink_b.value = False
                centr_x_pink.value = 0
                centr_y_pink.value = 0
                if n1 == 'red':
                    centr_x_red.value = pair[0][1]
                    centr_y_red.value = pair[0][2]

            elif n1 is None and n2 is None:
                red_b.value = False
                green_b.value = False
                pink_b.value = False
                centr_x_pink.value = 0
                centr_y_pink.value = 0
                centr_x.value = 0
                centr_y.value = 0

            now = time.time()
            fps = 1.0 / max(1e-3, (now - t_prev)); t_prev = now
            print(f"pairs:{pair} red_b:{red_b.value} green_b:{green_b.value} pink_b:{pink_b.value} "
                  f"cx:{locals().get('cx',0)} cy:{locals().get('cy',0)} fps:{fps:.1f}")
            
            frames += 1
            fps = frames / (time.time() - t0)
            inf_fps = 1.0 / (t_inf1 - t_inf0) if (t_inf1 - t_inf0)>0 else 0.0
            cv2.putText(frame_bgr, f"FPS: {fps:.2f} | INF: {inf_fps:.2f}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

            # If you want to see video:
            cv2.imshow("CPU SSD Live", frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # ESC
                break

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
