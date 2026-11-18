import ctypes, json, time, sys, numpy as np, cv2
D = "/home/orangepi/npu/"
L = ctypes.CDLL(D + "libnpu.so")
L.npu_init.argtypes = [ctypes.c_char_p]
L.npu_out_scale.restype = ctypes.c_float
L.npu_run.argtypes = [ctypes.c_void_p] * 3

class NPUYolo:
    def __init__(self, nb="/home/orangepi/yolov8-oiv7.nb", names=D + "oiv7_names.json",
                 size=320, conf=0.25, iou=0.45, classes=None):
        n = L.npu_init(nb.encode())
        if n != 2: raise RuntimeError("npu_init 失敗,回傳 %d" % n)
        self.size, self.conf, self.iou = size, conf, iou
        sz = [L.npu_out_size(i) for i in range(2)]
        self.out = [np.empty(s, np.uint8) for s in sz]
        self.bi = 0 if sz[0] < sz[1] else 1; self.ci = 1 - self.bi
        self.A = sz[self.bi] // 4; self.nc = sz[self.ci] // self.A
        self.sc = [L.npu_out_scale(i) for i in range(2)]
        self.zp = [L.npu_out_zp(i) for i in range(2)]
        self.inp = np.empty(L.npu_in_size(), np.uint8)
        self.names = json.load(open(names))
        self.classes = None if classes is None else np.asarray(classes, np.int64)

    def ids(self, names):
        return [self.names.index(n) for n in names if n in self.names]

    def detect(self, frame):
        S = self.size; h0, w0 = frame.shape[:2]
        r = min(S / h0, S / w0); nh, nw = round(h0 * r), round(w0 * r)
        c = np.full((S, S, 3), 114, np.uint8); t, l = (S - nh) // 2, (S - nw) // 2
        c[t:t + nh, l:l + nw] = cv2.resize(frame, (nw, nh))
        self.inp[:] = c[:, :, ::-1].transpose(2, 0, 1).ravel()   # uint8 RGB CHW
        if L.npu_run(self.inp.ctypes.data, self.out[0].ctypes.data, self.out[1].ctypes.data):
            raise RuntimeError("npu_run 失敗")
        cq = self.out[self.ci].reshape(self.nc, self.A)
        if self.classes is not None:
            cq = cq[self.classes]                 # 只在指定類別裡取最高分
        qmax = cq.max(0)
        thr = self.conf / self.sc[self.ci] + self.zp[self.ci]
        keep = np.nonzero(qmax > thr)[0]
        if keep.size == 0: return []
        cls = cq[:, keep].argmax(0)
        if self.classes is not None: cls = self.classes[cls]
        score = (qmax[keep].astype(np.float32) - self.zp[self.ci]) * self.sc[self.ci]
        b = (self.out[self.bi].reshape(4, self.A)[:, keep].astype(np.float32) - self.zp[self.bi]) * self.sc[self.bi]
        x1, y1, w, h = b[0] - b[2] / 2, b[1] - b[3] / 2, b[2], b[3]
        idx = cv2.dnn.NMSBoxes(np.stack([x1, y1, w, h], 1).tolist(), score.tolist(), self.conf, self.iou)
        res = []
        for k in np.array(idx).flatten():
            X1, Y1 = (x1[k] - l) / r, (y1[k] - t) / r
            res.append((self.names[cls[k]], float(score[k]), int(X1), int(Y1), int(X1 + w[k] / r), int(Y1 + h[k] / r)))
        return res

    def close(self): L.npu_close()

if __name__ == "__main__":
    det = NPUYolo()
    img = cv2.imread(sys.argv[1] if len(sys.argv) > 1 else "/opt/yolov5/input_data/dog.jpg")
    for d in det.detect(img): print("%-20s %.2f  [%d,%d,%d,%d]" % d)
    t0 = time.time()
    for _ in range(50): det.detect(img)
    print("平均每幀 %.1f ms" % ((time.time() - t0) / 50 * 1000))
    det.close()
