import sys, threading, time
import cv2
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
sys.path.insert(0, "/home/orangepi/npu")
from npu_yolo import NPUYolo

PORT = 8080
PHONE_IDS = {339}     # Mobile phone
CONF = 0.5            # phone at or above this counts for DETECTED
SHOW_CONF = 0.35      # any class at or above this is drawn
IOU = 0.45
FRAMES = 5            # frames in a row needed for DETECTED
TARGET_FPS = 30
FRAME_PERIOD = 1.0 / TARGET_FPS

det = NPUYolo(conf=SHOW_CONF, iou=IOU)
PHONE_NAMES = {det.names[i] for i in PHONE_IDS}

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
cap.set(cv2.CAP_PROP_FPS, 30)

latest = None
lock = threading.Lock()

def worker():
    global latest
    streak, fps, t_prev = 0, 0.0, time.time()
    while True:
        t0 = time.time()
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        t1 = time.time()
        dets = det.detect(frame)
        npu_ms = (time.time() - t1) * 1000
        phone_best = max([d[1] for d in dets if d[0] in PHONE_NAMES], default=0.0)
        streak = streak + 1 if phone_best >= CONF else 0
        for name, s, x1, y1, x2, y2 in dets:
            if name in PHONE_NAMES:
                color = (0, 255, 0) if s >= CONF else (0, 255, 255)
            else:
                color = (255, 200, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, "%s %.2f" % (name, s), (x1, max(15, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
        t_prev = now
        cv2.putText(frame, "NPU fps %.1f  %.1fms  phone %.2f  streak %d/%d"
                    % (fps, npu_ms, phone_best, streak, FRAMES), (8, 470),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        if streak >= FRAMES:
            cv2.putText(frame, "DETECTED", (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
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
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with lock:
                    jpg = latest
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpg)).encode()
                                     + b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(FRAME_PERIOD)
        except (BrokenPipeError, ConnectionResetError):
            pass

threading.Thread(target=worker, daemon=True).start()
print("streaming on port %d, Ctrl+C to stop" % PORT)
ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
