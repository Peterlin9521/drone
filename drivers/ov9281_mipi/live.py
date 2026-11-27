#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Live view in a web browser: RAW10 frames on stdin -> MJPEG over HTTP.

Open http://<Pi IP>:8080 on the PC. The picture is shown at half size
(640x400), contrast-stretched, with the focus score of the centre window
(the green box) drawn on top. Use with live.sh.
"""
import io
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image, ImageDraw

W, H = 1280, 800
FRAME = W * H * 2
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
FPS = int(sys.argv[2]) if len(sys.argv) > 2 else 5   # browser update rate
latest = b""
cond = threading.Condition()


def reader():
    global latest
    buf, n, lo, hi, best = sys.stdin.buffer, 0, 0.0, 1023.0, 1.0
    step = max(1, 60 // FPS)
    while True:
        raw = buf.read(FRAME)
        if len(raw) < FRAME:
            break
        n += 1
        if n % step:
            continue
        f = np.frombuffer(raw, dtype="<u2").reshape(H, W)[::2, ::2].astype(np.float32)
        if f.max() > 1023:
            f /= 64.0
        if n % (step * 15) == step:          # re-stretch about once a second
            lo, hi = np.percentile(f, 1), np.percentile(f, 99.5)
        c = f[100:300, 220:420]               # centre window (half-size coords)
        lap = c[1:-1, :-2] + c[1:-1, 2:] + c[:-2, 1:-1] + c[2:, 1:-1] - 4 * c[1:-1, 1:-1]
        score = float(lap.var())
        best = max(best, score)
        img = np.clip((f - lo) / max(hi - lo, 1.0) * 255.0, 0, 255).astype(np.uint8)
        im = Image.fromarray(img).convert("RGB")
        d = ImageDraw.Draw(im)
        d.rectangle([220, 100, 420, 300], outline=(0, 255, 0))
        d.rectangle([0, 0, 330, 24], fill=(0, 0, 0))
        d.text((8, 8), f"sharp {score:.0f}  best {best:.0f}  mean {f.mean():.0f}/1023",
               fill=(0, 255, 0))
        out = io.BytesIO()
        im.save(out, "JPEG", quality=80)
        with cond:
            latest = out.getvalue()
            cond.notify_all()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path != "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(b"<html><body style='margin:0;background:#111'>"
                             b"<img src='/stream' style='width:100%'></body></html>")
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
        self.end_headers()
        try:
            while True:
                with cond:
                    cond.wait(timeout=2)
                    jpg = latest
                if not jpg:
                    continue
                self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass


threading.Thread(target=reader, daemon=True).start()
print(f"live view: http://<Pi IP>:{PORT}   (Ctrl-C to stop)", flush=True)
srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
srv.daemon_threads = True
try:
    srv.serve_forever()
except KeyboardInterrupt:
    pass
