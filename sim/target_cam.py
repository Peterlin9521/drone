#!/usr/bin/env python3
"""相機估計目標位置:YOLO 找人 -> 框的底部(腳) -> 光線打到地面的點 -> Kalman filter -> /target/pose
同時記錄 /walker/pose(真實位置)供校正與驗證。"""
import os, sys, time, math
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from collections import deque
from ultralytics import YOLO
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cam_geom import quat_to_R, cam_R, pixel_to_ground

class KF:
    """世界座標等速模型 [x, y, vx, vy]"""
    def __init__(self, z):
        self.x = np.r_[z, 0.0, 0.0]; self.P = np.diag([0.5, 0.5, 2.0, 2.0])
    def predict(self, dt, q=1.5):
        F = np.eye(4); F[0, 2] = F[1, 3] = dt
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x; self.P = F @ self.P @ F.T + q * G @ G.T
    def update(self, z, sigma):
        H = np.zeros((2, 4)); H[0, 0] = H[1, 1] = 1
        S = H @ self.P @ H.T + np.eye(2) * sigma ** 2
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x); self.P = (np.eye(4) - K @ H) @ self.P

class TargetCam(Node):
    def __init__(self):
        super().__init__('target_cam')
        P = lambda n, d: self.declare_parameter(n, d).value
        self.tilt = math.radians(P('tilt_deg', 12.0)); self.stab = P('stabilized', True)
        self.cam_dz, self.conf, self.gate = P('cam_dz', 0.0), P('conf', 0.4), P('gate', 2.0)
        self.imgsz = P('imgsz', 640)
        self.fx_ovr = P('fx', 0.0)    # >0 時覆蓋 camera_info 的焦距
        self.latency, self.kf_q, self.sigma0 = P('latency', 0.1), P('kf_q', 0.5), P('sigma0', 0.25)
        self.pose_hist = deque(maxlen=200)
        self.model = YOLO('yolov8n.pt')
        self.fx = self.cx = self.cy = None
        self.pose = None; self.truth = None
        self.kf, self.last_upd, self.t_prev = None, 0.0, time.time()
        os.makedirs(os.path.expanduser('~/sim/logs'), exist_ok=True)
        self.log_path = os.path.expanduser('~/sim/logs/cam_%s.csv' % time.strftime('%H%M%S'))
        self.log = open(self.log_path, 'w')
        self.log.write('t,upd,u,v2,gx,gy,kx,ky,px,py,pz,roll,pitch,yaw,wx,wy,ncand,nrej\n')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image', self.on_image, qos)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/walker/pose', lambda m: setattr(self, 'truth', (m.pose.position.x, m.pose.position.y)), 10)
        self.pub = self.create_publisher(PoseStamped, '/target/pose', 10)
        self.pub_odom = self.create_publisher(Odometry, '/target/odom', 10)
        self.pub_img = self.create_publisher(Image, '/perception/image', 1)
        self.n_frames = self.n_upd = 0; self.errs = []
        self.get_logger().info('tilt=%.1f°  stabilized=%s  log=%s' % (math.degrees(self.tilt), self.stab, self.log_path))

    def on_info(self, m): self.fx, self.cx, self.cy = (self.fx_ovr if self.fx_ovr > 0 else m.k[0]), m.k[2], m.k[5]
    def on_pose(self, m):
        q = m.pose.orientation
        self.pose = (np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]), quat_to_R(q.x, q.y, q.z, q.w))
        self.pose_hist.append((time.time(), self.pose))

    def on_image(self, msg):
        if self.pose is None or self.fx is None: return
        h, w = msg.height, msg.width
        a = np.frombuffer(msg.data, np.uint8).reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        frame = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a.copy()
        now = time.time(); dt = min(max(now - self.t_prev, 1e-3), 0.3); self.t_prev = now
        p, Rb = self.pose                                   # 用『拍攝當下』的姿態(補償影像延遲)
        for tp_, ps_ in reversed(self.pose_hist):
            if tp_ <= now - self.latency: p, Rb = ps_; break
        yaw = math.atan2(Rb[1, 0], Rb[0, 0]); pitch = -math.asin(max(-1, min(1, Rb[2, 0]))); roll = math.atan2(Rb[2, 1], Rb[2, 2])
        Rc = cam_R(Rb, yaw, self.tilt, self.stab)
        pc = p + np.array([0, 0, self.cam_dz])

        r = self.model.predict(frame, classes=[0], conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        cand, nrej = [], 0
        for x1, y1, x2, y2 in r.boxes.xyxy.cpu().numpy().tolist():
            ok = (y2 - y1) > 1.2 * (x2 - x1) and y2 < h - 3          # 直立、腳沒有被畫面切掉
            g = pixel_to_ground((x1 + x2) / 2, y2, self.fx, self.cx, self.cy, pc, Rc) if ok else None
            if g is not None and np.linalg.norm(g - p[:2]) < 20:
                cand.append((g, (x1, y1, x2, y2)))
            else:
                nrej += 1
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 1)

        if self.kf is not None: self.kf.predict(dt, self.kf_q)
        sel = None
        if self.kf is not None and now - self.last_upd < 2.0:            # 追蹤中:選最接近預測位置的
            if cand:
                dmin, sel = min(((np.linalg.norm(c[0] - self.kf.x[:2]), c) for c in cand), key=lambda t: t[0])
                if dmin > self.gate + 0.5 * (now - self.last_upd) * 2: sel = None
        elif cand:                                                       # 沒在追蹤:選最靠近畫面中央的
            sel = min(cand, key=lambda c: abs((c[1][0] + c[1][2]) / 2 - w / 2))
            self.kf = KF(sel[0])
        upd = sel is not None
        if upd:
            rng = np.linalg.norm(sel[0] - p[:2])
            self.kf.update(sel[0], self.sigma0 + 0.008 * rng * rng); self.last_upd = now; self.n_upd += 1
        self.n_frames += 1

        valid = self.kf is not None and now - self.last_upd < 1.0
        if valid:
            m = PoseStamped(); m.header.stamp = msg.header.stamp; m.header.frame_id = 'map'
            m.pose.position.x, m.pose.position.y = float(self.kf.x[0]), float(self.kf.x[1]); m.pose.orientation.w = 1.0
            self.pub.publish(m)
            o = Odometry(); o.header = m.header; o.pose.pose = m.pose
            o.twist.twist.linear.x, o.twist.twist.linear.y = float(self.kf.x[2]), float(self.kf.x[3])
            self.pub_odom.publish(o)
            if self.truth is not None: self.errs.append(np.hypot(self.kf.x[0] - self.truth[0], self.kf.x[1] - self.truth[1]))

        u = v2 = gx = gy = float('nan')
        if upd:
            x1, y1, x2, y2 = sel[1]; u, v2 = (x1 + x2) / 2, y2; gx, gy = sel[0]
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, '%.1f m' % np.linalg.norm(sel[0] - p[:2]), (int(x1), max(15, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        for c in cand:
            if sel is None or c is not sel:
                x1, y1, x2, y2 = c[1]; cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 150, 0), 1)
        kx, ky = (self.kf.x[0], self.kf.x[1]) if self.kf is not None else (float('nan'),) * 2
        wx, wy = self.truth if self.truth is not None else (float('nan'),) * 2
        txt = 'TRACK' if valid else 'LOST'
        if valid and self.truth is not None: txt += '  err %.2f m' % self.errs[-1]
        cv2.putText(frame, txt, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255) if valid else (0, 0, 255), 2)
        out = Image(); out.header = msg.header; out.height, out.width, out.encoding, out.step = h, w, 'bgr8', w * 3
        out.data = frame.tobytes(); self.pub_img.publish(out)
        self.log.write('%.3f,%d,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%d,%d\n' % (
            now, upd, u, v2, gx, gy, kx, ky, p[0], p[1], p[2], roll, pitch, yaw, wx, wy, len(cand), nrej))

    def summary(self):
        self.log.close()
        if not self.n_frames: return
        print('\n===== 相機估位置 =====')
        print('有更新的幀: %.1f%%' % (100 * self.n_upd / self.n_frames))
        if self.errs:
            e = np.array(self.errs)
            print('與真實位置的誤差: 平均 %.2f m  中位數 %.2f m  90%% %.2f m' % (e.mean(), np.median(e), np.percentile(e, 90)))
        print('記錄檔:', self.log_path)

rclpy.init()
node = TargetCam()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.summary()
