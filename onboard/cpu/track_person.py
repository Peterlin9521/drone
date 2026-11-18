import threading
import time

import cv2
import numpy as np
import onnxruntime as ort
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MODEL = "/home/orangepi/yolov8n.onnx"
PORT = 8080
SIZE = 320
PERSON = 0
CONF = 0.4
IOU = 0.45
DEADZONE = 0.08
KP_YAW = 60.0
YAW_MAX = 30.0

sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
iname = sess.get_inputs()[0].name

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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


def detect_people(frame):
    blob, r, left, top = preprocess(frame)
    out = sess.run(None, {iname: blob})[0]
    p = out[0].T
    conf = p[:, 4 + PERSON]
    keep = conf >= CONF
    if not keep.any():
        return []
    p, conf = p[keep], conf[keep]
    x = (p[:, 0] - p[:, 2] / 2 - left) / r
    y = (p[:, 1] - p[:, 3] / 2 - top) / r
    bw, bh = p[:, 2] / r, p[:, 3] / r
    boxes = np.stack([x, y, bw, bh], 1)
    idx = cv2.dnn.NMSBoxes(boxes.tolist(), conf.tolist(), CONF, IOU)
    res = []
    for i in np.array(idx).flatten():
        res.append((float(conf[i]), float(x[i]), float(y[i]),
                    float(bw[i]), float(bh[i])))
    return res


def worker():
    global latest
    fps, t_prev = 0.0, time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        h, w = frame.shape[:2]
        people = detect_people(frame)
        target = max(people, key=lambda d: d[3] * d[4]) if people else None

        yaw, err = 0.0, 0.0
        if target:
            s, bx, by, bw, bh = target
            cx = bx + bw / 2
            err = (cx - w / 2) / w
            if abs(err) >= DEADZONE:
                yaw = max(-YAW_MAX, min(YAW_MAX, KP_YAW * err))
            cv2.rectangle(frame, (int(bx), int(by)),
                          (int(bx + bw), int(by + bh)), (0, 255, 0), 2)
            cv2.line(frame, (int(cx), 0), (int(cx), h), (0, 255, 0), 1)

        cv2.line(frame, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
        t_prev = now

        if target:
            if yaw > 0:
                msg = "turn RIGHT %.0f deg/s" % yaw
            elif yaw < 0:
                msg = "turn LEFT %.0f deg/s" % (-yaw)
            else:
                msg = "centered, hold"
            color = (0, 255, 0) if yaw == 0 else (0, 200, 255)
        else:
            msg = "no person"
            color = (0, 0, 255)

        cv2.putText(frame, msg, (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(frame, "fps %.1f  err %+.2f  [print only]" % (fps, err),
                    (8, 466), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with lock:
                latest = buf.tobytes()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
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
print("tracking (print only) on port %d, Ctrl+C to stop" % PORT)
ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
