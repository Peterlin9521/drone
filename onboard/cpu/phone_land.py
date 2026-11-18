import argparse
import ast
import threading
import time

import cv2
import numpy as np
import onnxruntime as ort
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--live", action="store_true",
                help="actually send LAND. default: dry-run, print only")
ap.add_argument("--conf", type=float, default=0.5)
ap.add_argument("--frames", type=int, default=8)
a = ap.parse_args()

MODEL = "/home/orangepi/yolov8n-oiv7.onnx"
CONN = "tcp:127.0.0.1:5678"
PORT = 8080
SIZE = 320
PHONE_IDS = {339}        # Mobile phone (oiv7)
SHOW_CONF = 0.35
IOU = 0.45

sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
iname = sess.get_inputs()[0].name
NAMES = ast.literal_eval(sess.get_modelmeta().custom_metadata_map["names"])

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

latest = None
lock = threading.Lock()
state = {"triggered": False, "last_send": 0.0}

# MAVLink connection (shared). None until connected.
mav = None


def mav_connect():
    global mav
    try:
        m = mavutil.mavlink_connection(CONN)
        if m.wait_heartbeat(timeout=5) is not None:
            mav = m
            print("MAVLink: heartbeat ok (sys=%d)" % m.target_system)
        else:
            print("MAVLink: no heartbeat; LAND will not be sent")
    except Exception as e:
        print("MAVLink connect failed:", e)


def heartbeat_fresh():
    """True if we got a heartbeat in the last ~3 s."""
    if mav is None:
        return False
    hb = mav.recv_match(type="HEARTBEAT", blocking=False)
    return hb is not None


def send_land():
    if mav is None:
        print(">>> would LAND but MAVLink not connected")
        return
    modes = mav.mode_mapping() or {}
    if "LAND" not in modes:
        print(">>> LAND not in mode map")
        return
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        modes["LAND"], 0, 0, 0, 0, 0)
    print(">>> LAND command SENT")


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
    blob, r, left, top = preprocess(frame)
    o = sess.run(None, {iname: blob})[0][0]
    p = o.T
    scores = p[:, 4:]
    cls = scores.argmax(1)
    conf = scores.max(1)
    keep = conf >= SHOW_CONF
    if not keep.any():
        return []
    p, cls, conf = p[keep], cls[keep], conf[keep]
    x = (p[:, 0] - p[:, 2] / 2 - left) / r
    y = (p[:, 1] - p[:, 3] / 2 - top) / r
    bw, bh = p[:, 2] / r, p[:, 3] / r
    boxes = np.stack([x, y, bw, bh], 1)
    idx = cv2.dnn.NMSBoxes(boxes.tolist(), conf.tolist(), SHOW_CONF, IOU)
    res = []
    for i in np.array(idx).flatten():
        res.append((int(cls[i]), float(conf[i]), int(x[i]), int(y[i]),
                    int(x[i] + bw[i]), int(y[i] + bh[i])))
    return res


def worker():
    global latest
    streak, fps, t_prev = 0, 0.0, time.time()
    fail = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            fail += 1
            if fail % 20 == 1:
                print("camera read failed (not triggering)")
            streak = 0
            time.sleep(0.1)
            continue
        fail = 0
        dets = detect(frame)
        phone_best = max([d[1] for d in dets if d[0] in PHONE_IDS], default=0.0)
        streak = streak + 1 if phone_best >= a.conf else 0

        if streak >= a.frames and not state["triggered"]:
            if time.time() - state["last_send"] > 10:
                if not heartbeat_fresh():
                    print(">>> phone detected but no fresh heartbeat; not sending")
                elif a.live:
                    send_land()
                    state["triggered"] = True
                    state["last_send"] = time.time()
                else:
                    print(">>> WOULD LAND (dry-run). conf=%.2f streak=%d"
                          % (phone_best, streak))
                    state["triggered"] = True
                    state["last_send"] = time.time()
        if streak == 0:
            state["triggered"] = False

        for c, s, x1, y1, x2, y2 in dets:
            if c in PHONE_IDS:
                color = (0, 255, 0) if s >= a.conf else (0, 255, 255)
            else:
                color = (255, 200, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, "%s %.2f" % (NAMES.get(c, c), s),
                        (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2)
        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
        t_prev = now
        mode = "LIVE" if a.live else "DRY-RUN"
        cv2.putText(frame, "fps %.1f  phone %.2f  streak %d/%d  %s"
                    % (fps, phone_best, streak, a.frames, mode), (8, 470),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if state["triggered"]:
            cv2.putText(frame, "LAND", (8, 36), cv2.FONT_HERSHEY_SIMPLEX,
                        1.1, (0, 0, 255), 3)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with lock:
                latest = buf.tobytes()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *x):
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


mav_connect()
threading.Thread(target=worker, daemon=True).start()
print("phone->LAND  mode=%s  conf>=%.2f  frames=%d  port=%d"
      % ("LIVE" if a.live else "DRY-RUN", a.conf, a.frames, PORT))
ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
