import argparse
import threading
import time

import cv2
import numpy as np
import onnxruntime as ort
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--live", action="store_true")
ap.add_argument("--target-bh", type=int, default=260)
a = ap.parse_args()

MODEL = "/home/orangepi/yolov8n.onnx"
CONN = "tcp:127.0.0.1:5678"
PORT = 8080
SIZE = 320
PERSON = 0
CONF = 0.4
IOU = 0.45

DEADZONE_YAW = 0.08
KP_YAW = 0.8
YAW_MAX = 0.5
BH_DEADZONE = 30
KP_FWD = 0.004
FWD_MAX = 0.35
SEND_HZ = 5
TARGET_FPS = 10
FRAME_PERIOD = 1.0 / TARGET_FPS

# tracker
MATCH_DIST = 120        # px; centroid must be within this to be same person
MAX_MISS = 15           # frames a track survives without a detection
LOST_GIVEUP = 15        # frames before the locked target is released

MAV = mavutil.mavlink
mav = None


def mav_connect():
    global mav
    try:
        m = mavutil.mavlink_connection(CONN)
        if m.wait_heartbeat(timeout=5) is not None:
            mav = m
            print("MAVLink heartbeat ok (sys=%d)" % m.target_system)
        else:
            print("No heartbeat; will not send")
    except Exception as e:
        print("MAVLink connect failed:", e)


def cur_mode():
    if mav is None:
        return None
    mav.recv_match(type="HEARTBEAT", blocking=False)
    return mav.flightmode


def send_vel(vx, yaw_rate):
    if mav is None:
        return
    mav.mav.set_position_target_local_ned_send(
        0, mav.target_system, mav.target_component,
        MAV.MAV_FRAME_BODY_NED, 0b0000011111000111,
        0, 0, 0, vx, 0, 0, 0, 0, 0, 0, yaw_rate)


sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
iname = sess.get_inputs()[0].name
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FPS, 10)

latest = None
lock = threading.Lock()
cmd = {"vx": 0.0, "yaw": 0.0, "ok": False, "mode": None}

# tracks: id -> dict(cx, cy, bh, box, miss)
tracks = {}
next_id = [1]
locked = [None]
lost = [0]


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
        res.append((float(x[i]), float(y[i]), float(bw[i]), float(bh[i])))
    return res


def update_tracks(dets):
    """Greedy nearest-centroid association. dets: list of (x,y,bw,bh)."""
    for t in tracks.values():
        t["matched"] = False
    cents = [(dx + dbw / 2, dy + dbh / 2, dbw, dbh, dx, dy)
             for (dx, dy, dbw, dbh) in dets]
    used = [False] * len(cents)
    # match existing tracks to nearest detection
    for tid, t in tracks.items():
        best, bj = MATCH_DIST, -1
        for j, c in enumerate(cents):
            if used[j]:
                continue
            d = ((t["cx"] - c[0]) ** 2 + (t["cy"] - c[1]) ** 2) ** 0.5
            if d < best:
                best, bj = d, j
        if bj >= 0:
            c = cents[bj]
            t.update(cx=c[0], cy=c[1], bh=c[3], box=(c[4], c[5], c[2], c[3]),
                     miss=0, matched=True)
            used[bj] = True
    # unmatched detections -> new tracks
    for j, c in enumerate(cents):
        if not used[j]:
            tracks[next_id[0]] = dict(cx=c[0], cy=c[1], bh=c[3],
                                      box=(c[4], c[5], c[2], c[3]),
                                      miss=0, matched=True)
            next_id[0] += 1
    # age unmatched tracks, drop stale
    for tid in list(tracks.keys()):
        if not tracks[tid]["matched"]:
            tracks[tid]["miss"] += 1
            if tracks[tid]["miss"] > MAX_MISS:
                del tracks[tid]


def pick_lock(w, h):
    """Lock the person nearest image center, if none locked / lost too long."""
    if locked[0] in tracks and tracks[locked[0]]["miss"] == 0:
        lost[0] = 0
        return
    if locked[0] is not None:
        lost[0] += 1
        if lost[0] < LOST_GIVEUP and locked[0] in tracks:
            return
    best, bid = 1e9, None
    for tid, t in tracks.items():
        if t["miss"] == 0:
            d = abs(t["cx"] - w / 2)
            if d < best:
                best, bid = d, tid
    if bid is not None:
        locked[0] = bid
        lost[0] = 0


def sender():
    while True:
        mode = cur_mode()
        cmd["mode"] = mode
        if a.live and mode == "GUIDED" and cmd["ok"]:
            send_vel(cmd["vx"], cmd["yaw"])
        elif a.live and mode == "GUIDED":
            send_vel(0.0, 0.0)
        time.sleep(1.0 / SEND_HZ)


def worker():
    global latest
    fps, t_prev = 0.0, time.time()
    while True:
        t0 = time.time()
        ok, frame = cap.read()
        if not ok:
            cmd["ok"] = False
            time.sleep(0.1)
            continue
        h, w = frame.shape[:2]
        dets = detect_people(frame)
        update_tracks(dets)
        pick_lock(w, h)

        vx, yaw = 0.0, 0.0
        tgt = tracks.get(locked[0]) if locked[0] is not None else None
        have = tgt is not None and tgt["miss"] == 0
        if have:
            cx = tgt["cx"]
            bh = tgt["bh"]
            ex = (cx - w / 2) / w
            if abs(ex) >= DEADZONE_YAW:
                yaw = max(-YAW_MAX, min(YAW_MAX, KP_YAW * ex))
            ebh = a.target_bh - bh
            if abs(ebh) >= BH_DEADZONE:
                vx = max(-FWD_MAX, min(FWD_MAX, KP_FWD * ebh))
            cmd["vx"], cmd["yaw"], cmd["ok"] = vx, yaw, True
        else:
            cmd["ok"] = False

        # draw all tracks; locked = green, others = blue
        for tid, t in tracks.items():
            if t["miss"] != 0:
                continue
            bx, by, bw, bh = t["box"]
            c = (0, 255, 0) if tid == locked[0] else (255, 150, 0)
            cv2.rectangle(frame, (int(bx), int(by)),
                          (int(bx + bw), int(by + bh)), c, 2)
            cv2.putText(frame, "ID %d%s" % (tid, " LOCK" if tid == locked[0]
                        else ""), (int(bx), max(15, int(by) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)

        cv2.line(frame, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
        t_prev = now
        md = "LIVE" if a.live else "DRY"
        head = ("ID %s  yaw %+.2f vx %+.2f  mode=%s %s"
                % (locked[0], yaw, vx, cmd["mode"], md)) if have \
            else "target lost -> stop  %s" % md
        cv2.putText(frame, head, (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 200, 255) if have else (0, 0, 255), 2)
        cv2.putText(frame, "fps %.1f  bh %.0f/%d  tracks %d"
                    % (fps, (tgt["bh"] if have else 0), a.target_bh, len(tracks)),
                    (8, 466), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with lock:
                latest = buf.tobytes()

        dt = time.time() - t0
        if dt < FRAME_PERIOD:
            time.sleep(FRAME_PERIOD - dt)

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
threading.Thread(target=sender, daemon=True).start()
print("follow+track  mode=%s  target_bh=%d  port=%d"
      % ("LIVE" if a.live else "DRY-RUN", a.target_bh, PORT))
try:
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
except KeyboardInterrupt:
    if a.live and mav is not None:
        send_vel(0.0, 0.0)
    print("stopped")
