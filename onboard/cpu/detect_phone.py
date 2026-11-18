#!/usr/bin/env python3
"""
detect_phone.py - run yolov8n.onnx (320x320) on ONE image file and print what it
finds. Step 1 only: reads a file, NO camera, NO flight controller, sends nothing.
 
  python detect_phone.py phone.jpg
  python detect_phone.py phone.jpg --all          # list every class, not just phone
  python detect_phone.py phone.jpg --conf 0.15 --runs 20 --save out.jpg
"""
import argparse
import time
 
import cv2
import numpy as np
import onnxruntime as ort
 
SIZE = 320
CELL_PHONE = 67  # COCO class index
 
ap = argparse.ArgumentParser()
ap.add_argument("image")
ap.add_argument("--model", default="/home/orangepi/yolov8n.onnx")
ap.add_argument("--conf", type=float, default=0.25)
ap.add_argument("--iou", type=float, default=0.45)
ap.add_argument("--runs", type=int, default=10, help="timing repeats")
ap.add_argument("--all", action="store_true", help="show all classes")
ap.add_argument("--save", help="write an annotated copy of the image here")
a = ap.parse_args()
 
img = cv2.imread(a.image)
if img is None:
    raise SystemExit(f"Could not read image: {a.image}")
h0, w0 = img.shape[:2]
 
 
def preprocess(im):
    """Letterbox to SIZE x SIZE (grey padding), BGR->RGB, 0-1, NCHW."""
    h, w = im.shape[:2]
    r = min(SIZE / h, SIZE / w)
    nh, nw = round(h * r), round(w * r)
    resized = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, left = (SIZE - nh) // 2, (SIZE - nw) // 2
    canvas = np.full((SIZE, SIZE, 3), 114, dtype=np.uint8)
    canvas[top:top + nh, left:left + nw] = resized
    blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(blob), r, left, top
 
 
def postprocess(out, r, left, top, only_phone=True):
    """out: (1, 84, 2100) -> list of (class_id, score, x1, y1, x2, y2)."""
    p = out[0].T                      # (2100, 84): cx, cy, w, h, 80 class scores
    scores = p[:, 4:]
    if only_phone:
        cls = np.full(len(p), CELL_PHONE)
        conf = scores[:, CELL_PHONE]
    else:
        cls = scores.argmax(1)
        conf = scores.max(1)
    keep = conf >= a.conf
    p, cls, conf = p[keep], cls[keep], conf[keep]
    if len(p) == 0:
        return []
    cx, cy, w, h = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
    x1 = (cx - w / 2 - left) / r
    y1 = (cy - h / 2 - top) / r
    boxes = np.stack([x1, y1, w / r, h / r], 1)       # x, y, w, h (original px)
    idx = cv2.dnn.NMSBoxes(boxes.tolist(), conf.tolist(), a.conf, a.iou)
    res = []
    for i in np.array(idx).flatten():
        bx, by, bw, bh = boxes[i]
        res.append((int(cls[i]), float(conf[i]),
                    max(0, bx), max(0, by), min(w0, bx + bw), min(h0, by + bh)))
    return res
 
 
sess = ort.InferenceSession(a.model, providers=["CPUExecutionProvider"])
name = sess.get_inputs()[0].name
 
blob, r, left, top = preprocess(img)
out = sess.run(None, {name: blob})[0]
 
dets = postprocess(out, r, left, top, only_phone=not a.all)
print(f"image {a.image}: {w0}x{h0}, model input {SIZE}x{SIZE}, conf>={a.conf}")
if not dets:
    print("no detections")
for c, s, x1, y1, x2, y2 in dets:
    label = "cell phone" if c == CELL_PHONE else f"class {c}"
    print(f"  {label}: {s:.2f} box=({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f})")
 
# timing (preprocess + inference + postprocess), after one warm-up run above
t0 = time.time()
for _ in range(a.runs):
    b, r, left, top = preprocess(img)
    o = sess.run(None, {name: b})[0]
    postprocess(o, r, left, top, only_phone=not a.all)
dt = (time.time() - t0) / a.runs
print(f"timing: {dt * 1000:.0f} ms per frame  ->  about {1 / dt:.1f} fps "
      f"(average of {a.runs} runs)")
 
if a.save:
    for c, s, x1, y1, x2, y2 in dets:
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(img, f"{c}:{s:.2f}", (int(x1), max(15, int(y1) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imwrite(a.save, img)
    print("saved", a.save)
