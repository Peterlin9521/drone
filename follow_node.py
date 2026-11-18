#!/usr/bin/env python3
import time, math, os
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from ultralytics import YOLO

CONF, IOU = 0.4, 0.45
DEADZONE_YAW, BH_DEADZONE, REF_H = 0.08, 30, 480
MATCH_DIST, TRACK_TTL, LOST_GIVEUP = 120, 1.5, 1.5
COAST, HOLD = 1.5, 1.5            # 秒:遺失後先照預測轉 / 再原地等,之後才搜尋
clip = lambda v, m: max(-m, min(m, v))
wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi

class BearingKF:
    """狀態 = [目標方位角, 方位角速度](世界座標,逆時針為正)"""
    def __init__(self, psi):
        self.x = np.array([psi, 0.0]); self.P = np.diag([0.05, 1.0])
    def predict(self, dt, q=4.0):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = q * np.array([[dt**4 / 4, dt**3 / 2], [dt**3 / 2, dt**2]])
        self.x = F @ self.x; self.P = F @ self.P @ F.T + Q
    def update(self, psi, r=0.0009):
        y = wrap(psi - self.x[0]); K = self.P[:, 0] / (self.P[0, 0] + r)
        self.x = self.x + K * y; self.P = self.P - np.outer(K, self.P[0, :])

class Follow(Node):
    def __init__(self):
        super().__init__('follow')
        P = lambda n, d: self.declare_parameter(n, d).value
        self.live, self.ff = P('live', False), P('ff', False)
        self.target_bh, self.imgsz = P('target_bh', 0.35), P('imgsz', 640)
        self.kp_yaw, self.yaw_max, self.search_yaw = P('kp_yaw', 2.0), P('yaw_max', 1.0), P('search_yaw', 1.0)
        self.kp_fwd, self.fwd_max = P('kp_fwd', 0.01), P('fwd_max', 1.5)
        self.conf = P('conf', CONF)
        self.coast_decay = P('coast_decay', 0.85)
        self.model = YOLO('yolov8n.pt')
        self.state, self.yaw_d, self.fx = State(), None, None
        self.pos, self.walker = None, None
        self.tracks, self.next_id, self.locked, self.lost_since = {}, 1, None, None
        self.cmd, self.cmd_t = (0.0, 0.0, False), 0.0
        self.fps, self.t_prev = 0.0, time.time()
        self.last_ex, self.last_seen, self.kf, self.kf_id = None, 0.0, None, None
        self.rec = []
        self.search_dir = 1.0
        os.makedirs(os.path.expanduser('~/sim/logs'), exist_ok=True)
        self.log_path = os.path.expanduser('~/sim/logs/follow_%s_%s.csv' % ('FF' if self.ff else 'P', time.strftime('%H%M%S')))
        self.log = open(self.log_path, 'w'); self.log.write('t,have,ex,bh,yaw,vx,ndet,nrej,rtrue,extrue,px,py,yawd,wx,wy\n')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image', self.on_image, qos)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/walker/pose', self.on_walker, 10)
        self.pub_cmd = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.pub_img = self.create_publisher(Image, '/follow/image', 1)
        self.create_timer(0.1, self.send)
        self.get_logger().info('%s  控制=%s  log=%s' % ('LIVE' if self.live else 'DRY-RUN', 'KF+前饋' if self.ff else 'P', self.log_path))

    def on_state(self, m): self.state = m
    def on_info(self, m): self.fx = m.k[0]
    def on_walker(self, m): self.walker = (m.pose.position.x, m.pose.position.y)
    def truth(self, w):
        if None in (self.walker, self.pos, self.yaw_d) or not self.fx: return (float('nan'),) * 7
        dx, dy = self.walker[0] - self.pos[0], self.walker[1] - self.pos[1]
        a = wrap(self.yaw_d - math.atan2(dy, dx))
        return (math.hypot(dx, dy), self.fx * math.tan(a) / w if abs(a) < 1.4 else float('nan'), self.pos[0], self.pos[1], self.yaw_d, self.walker[0], self.walker[1])
    def on_pose(self, m):
        self.pos = (m.pose.position.x, m.pose.position.y)
        q = m.pose.orientation
        self.yaw_d = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def send(self):
        if not (self.live and self.state.mode == 'GUIDED' and self.state.armed):
            return
        vx, yaw, ok = self.cmd
        if time.time() - self.cmd_t > 0.5: ok = False
        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_BODY_NED
        m.type_mask = 0b0000011111000111
        m.velocity.x = vx if ok else 0.0
        m.yaw_rate = -yaw if ok else 0.0          # 內部 yaw 正值 = 右轉;ROS 正值 = 左轉
        self.pub_cmd.publish(m)

    def ff_yaw(self, w):
        e = wrap(self.kf.x[0] - self.yaw_d)                 # 目標方位 - 機頭方向
        ex_f = -math.tan(clip(e, 1.4)) * self.fx / w        # 換回「畫面偏移比例」,沿用同一個 kp
        return clip(self.kp_yaw * ex_f - self.kf.x[1], self.yaw_max)

    def on_image(self, msg):
        h, w = msg.height, msg.width
        a = np.frombuffer(msg.data, np.uint8).reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        frame = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a.copy()
        now = time.time(); dt = min(max(now - self.t_prev, 1e-3), 0.2)

        r = self.model.predict(frame, classes=[0], conf=self.conf, iou=IOU, imgsz=self.imgsz, verbose=False)[0]
        cents, rejected = [], []
        for x1, y1, x2, y2 in r.boxes.xyxy.cpu().numpy().tolist():
            ok = y2 > 0.5 * h and (y2 - y1) > 1.2 * (x2 - x1)     # 腳在地平線以下、形狀直立
            (cents if ok else rejected).append(((x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1, x1, y1))
        for c in rejected:                                         # 被濾掉的:紅色細框
            cv2.rectangle(frame, (int(c[4]), int(c[5])), (int(c[4] + c[2]), int(c[5] + c[3])), (0, 0, 255), 1)

        used = [False] * len(cents)
        for t in self.tracks.values():
            t['seen'] = False
            best, bj = MATCH_DIST * w / 640, -1
            for j, c in enumerate(cents):
                if used[j]: continue
                d = ((t['cx'] - c[0]) ** 2 + (t['cy'] - c[1]) ** 2) ** 0.5
                if d < best: best, bj = d, j
            if bj >= 0:
                c = cents[bj]; used[bj] = True
                t.update(cx=c[0], cy=c[1], bh=c[3], box=(c[4], c[5], c[2], c[3]), last=now, seen=True)
        for j, c in enumerate(cents):
            if not used[j]:
                self.tracks[self.next_id] = dict(cx=c[0], cy=c[1], bh=c[3], box=(c[4], c[5], c[2], c[3]), last=now, seen=True)
                self.next_id += 1
        self.tracks = {k: v for k, v in self.tracks.items() if now - v['last'] <= TRACK_TTL}

        lk = self.tracks.get(self.locked)
        seen = [(tid, t) for tid, t in self.tracks.items() if t['seen']]
        if lk is not None and not lk['seen'] and seen:      # 同一個人換了 ID:立刻接手
            px = lk['cx']
            if self.kf is not None and self.yaw_d is not None and self.fx:
                px = w / 2 - self.fx * math.tan(clip(wrap(self.kf.x[0] - self.yaw_d), 1.4))
            dmin, tid = min((abs(t['cx'] - px), tid) for tid, t in seen)
            if dmin < 0.5 * w:
                if self.kf_id == self.locked: self.kf_id = tid
                self.locked = tid; lk = self.tracks[tid]
        if lk is not None and lk['seen']:
            self.lost_since = None
        else:
            if self.locked is not None and self.lost_since is None: self.lost_since = now
            keep = lk is not None and self.lost_since is not None and now - self.lost_since < LOST_GIVEUP
            if not keep:
                cand = [(abs(t['cx'] - w / 2), tid) for tid, t in self.tracks.items() if t['seen']]
                if cand: self.locked, self.lost_since = min(cand)[1], None

        vx = yaw = 0.0; status = 'LOCK'
        tgt = self.tracks.get(self.locked)
        have = tgt is not None and tgt['seen']
        use_ff = self.ff and self.yaw_d is not None and self.fx is not None
        ex = float('nan')
        if have:
            ex = (tgt['cx'] - w / 2) / w
            self.last_ex, self.last_seen = ex, now
            if use_ff:
                psi = wrap(self.yaw_d - math.atan((tgt['cx'] - w / 2) / self.fx))
                if self.kf is None or self.kf_id != self.locked:
                    self.kf, self.kf_id = BearingKF(psi), self.locked
                else:
                    self.kf.predict(dt); self.kf.update(psi)
                yaw = self.ff_yaw(w)
            elif abs(ex) >= DEADZONE_YAW:
                yaw = clip(self.kp_yaw * ex, self.yaw_max)
            ebh = (self.target_bh - tgt['bh'] / h) * REF_H
            if abs(ebh) >= BH_DEADZONE: vx = clip(self.kp_fwd * ebh, self.fwd_max)
        elif self.last_ex is not None:
            lost = now - self.last_seen
            if use_ff and self.kf is not None and lost < COAST and abs(self.last_ex) < 0.2:
                self.kf.predict(dt); self.kf.x[1] *= self.coast_decay        # 看不到時:假設人可能停下或掉頭
                yaw = self.ff_yaw(w); status = 'coast'
                if abs(self.last_ex) > 0.05: self.search_dir = 1.0 if self.last_ex > 0 else -1.0
            else:
                if abs(self.last_ex) >= 0.2 or not use_ff:       # 從邊緣出去:往那一側找
                    self.search_dir = 1.0 if self.last_ex > 0 else -1.0
                yaw = self.search_yaw * self.search_dir; status = 'search'
            if lost >= COAST: self.kf = None
        else:
            status = 'no target'
        self.cmd, self.cmd_t = (vx, yaw, self.last_ex is not None), now
        self.rec.append((have, abs(ex)))
        self.log.write('%.3f,%d,%.4f,%.4f,%.3f,%.3f,%d,%d,%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f\n' % ((now, have, ex, (tgt['bh'] / h if have else float('nan')), yaw, vx, len(cents), len(rejected)) + self.truth(w)))

        for tid, t in self.tracks.items():
            if not t['seen']: continue
            bx, by, bw, bh = t['box']
            c = (0, 255, 0) if tid == self.locked else (255, 150, 0)
            cv2.rectangle(frame, (int(bx), int(by)), (int(bx + bw), int(by + bh)), c, 2)
            cv2.putText(frame, 'ID %d%s' % (tid, ' LOCK' if tid == self.locked else ''),
                        (int(bx), max(15, int(by) - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
        cv2.line(frame, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
        self.fps = 0.9 * self.fps + 0.1 / dt; self.t_prev = now
        cv2.putText(frame, '%s [%s] yaw %+.2f vx %+.2f  %s %s' % (status, 'KF+FF' if use_ff else 'P', yaw, vx, self.state.mode, 'LIVE' if self.live else 'DRY'),
                    (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255) if have else (0, 0, 255), 2)
        cv2.putText(frame, 'fps %.1f  bh %.2f/%.2f  tracks %d' % (self.fps, (tgt['bh'] / h if have else 0), self.target_bh, len(self.tracks)),
                    (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        out = Image(); out.header = msg.header
        out.height, out.width, out.encoding, out.step = h, w, 'bgr8', w * 3
        out.data = frame.tobytes()
        self.pub_img.publish(out)

    def summary(self):
        self.log.close()
        if not self.rec: return
        have = np.array([r[0] for r in self.rec]); ex = np.array([r[1] for r in self.rec])[have]
        print('\n===== %s 統計(%d 幀)=====' % ('KF+前饋' if self.ff else 'P 控制', len(self.rec)))
        print('看得到目標的比例: %.1f%%' % (100 * have.mean()))
        if ex.size:
            print('平均偏離中央: %.1f%% 畫面寬   最大: %.1f%%' % (100 * ex.mean(), 100 * ex.max()))
        print('記錄檔:', self.log_path)

rclpy.init()
node = Follow()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.summary()
