import ast
import threading
import time

import cv2
import numpy as np
import onnxruntime as ort
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MODEL = "/home/orangepi/yolov8n-oiv7.onnx"
PORT = 8080
SIZE = 320
PHONE_IDS = {339}     # Mobile phone. add 526 (Telephone) / 134 (Corded phone) if wanted
CONF = 0.5            # phone at or above this counts for DETECTED
SHOW_CONF = 0.35      # any class at or above this is drawn
IOU = 0.45
FRAMES = 5            # frames in a row needed for DETECTED
TARGET_FPS = 10
FRAME_PERIOD = 1.0 / TARGET_FPS

sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
name = sess.get_inputs()[0].name
NAMES = ast.literal_eval(sess.get_modelmeta().custom_metadata_map["names"])
NC = len(NAMES)
print("classes:", NC, " output shapes:", [o.shape for o in sess.get_outputs()])

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FPS, 10)

latest = None
lock = threading.Lock()


def preprocess(im):
    h, w = im.shape[:2]
    r = min(SIZE / h, SIZE / w)
    nh, nw = round(h * r), round(w * r)
    resized = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, left = (SIZE - nh) // 2, (SIZE - nw) // 2
    canvas = np.full((SIZE, SIZE, 3), 114, dtype=np.uint8)
    canvas[top:top + nh, left:left + nw] = resized
    blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(blob), r, left, top


def detect(frame):
    """Return list of (class_id, conf, x1, y1, x2, y2) in frame pixels."""
    blob, r, left, top = preprocess(frame)
    o = sess.run(None, {name: blob})[0][0]
    res = []
    if o.shape[0] == 4 + NC:                      # standard (4+NC, N)
        p = o.T
        scores = p[:, 4:]
        cls = scores.argmax(1)
        conf = scores.max(1)
        keep = conf >= SHOW_CONF
        if not keep.any():
            return res
        p, cls, conf = p[keep], cls[keep], conf[keep]
        x = (p[:, 0] - p[:, 2] / 2 - left) / r
        y = (p[:, 1] - p[:, 3] / 2 - top) / r
        bw, bh = p[:, 2] / r, p[:, 3] / r
        boxes = np.stack([x, y, bw, bh], 1)
        idx = cv2.dnn.NMSBoxes(boxes.tolist(), conf.tolist(), SHOW_CONF, IOU)
        for i in np.array(idx).flatten():
            res.append((int(cls[i]), float(conf[i]), int(x[i]), int(y[i]),
                        int(x[i] + bw[i]), int(y[i] + bh[i])))
    elif o.shape[-1] == 6:                        # end2end (N, 6): x1 y1 x2 y2 conf cls
        for row in o:
            if row[4] >= SHOW_CONF:
                res.append((int(row[5]), float(row[4]),
                            int((row[0] - left) / r), int((row[1] - top) / r),
                            int((row[2] - left) / r), int((row[3] - top) / r)))
    return res


def worker():
    global latest
    streak, fps, t_prev = 0, 0.0, time.time()
    while True:
        t0 = time.time()
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        dets = detect(frame)
        phone_best = max([d[1] for d in dets if d[0] in PHONE_IDS], default=0.0)
        streak = streak + 1 if phone_best >= CONF else 0
        for c, s, x1, y1, x2, y2 in dets:
            if c in PHONE_IDS:
                color = (0, 255, 0) if s >= CONF else (0, 255, 255)
            else:
                color = (255, 200, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, "%s %.2f" % (NAMES.get(c, c), s),
                        (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, color, 2)
        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
        t_prev = now
        cv2.putText(frame, "fps %.1f  phone %.2f  streak %d/%d"
                    % (fps, phone_best, streak, FRAMES), (8, 470),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if streak >= FRAMES:
            cv2.putText(frame, "DETECTED", (8, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with lock:
                latest = buf.tobytes()
       	dt = time.time() - t0
        if dt < FRAME_PERIOD:
                time.sleep(FRAME_PERIOD - dt)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path != "/":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with lock:
                    jpg = latest
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpg)).encode()
                                     + b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(0.07)
        except (BrokenPipeError, ConnectionResetError):
            pass


threading.Thread(target=worker, daemon=True).start()
print("streaming on port %d, Ctrl+C to stop" % PORT)
ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
