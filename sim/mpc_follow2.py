#!/usr/bin/env python3
"""MPC 跟隨 v2:為「相機估計的目標」設計(雜訊、丟幀、跟飛機動作耦合)
和 v1 的差別
  1. 目標濾波自己做:只吃位置,速度用低頻寬 KF 估(人最快 2 m/s),不用差分、不用上游的速度
  2. 機身傾斜 > tilt_gate 或量測跳太遠(gate)時,這一筆量測不用
  2c. 安全層(控制屏障函數):距離逼近 d_min 時強制加上往外的速度;距離用『估計』和『最新原始量測』中比較近的那個
  2d. 機動偵測:創新量連續偏大(人轉彎/折返)時,暫時把 KF 的 q 放大 6 倍,減少估計落後
  2b. 命令整形:合速度上限 vmax、加速度上限 amax(機身傾斜約 atan(amax/g),1.5 m/s^2 約 9°)
  3. 人的前進方向:速度穩定夠快才更新,而且低通
  4. 轉向:只對準濾波後的目標,增益較低,不用雜訊大的方位角速度前饋
  5. 丟失 > lost_t 秒:原地停住,朝最後看到的方向慢慢轉著找;不再沿舊速度往外推
  6. 有 /walker/pose 時,另外用真實位置算統計(只評估,不參與控制)
"""
import os, sys, time, math
import numpy as np

wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
clip = lambda v, m: max(-m, min(m, v))


class TargetKF:
    """2D 等速 KF。q = 加速度雜訊 (m/s^2),r = 量測雜訊 (m)"""
    def __init__(self, z, q=0.6, r=0.4, vmax=2.0):
        self.x = np.array([z[0], z[1], 0.0, 0.0]); self.P = np.diag([r * r, r * r, 1.0, 1.0])
        self.q, self.r, self.vmax = q, r, vmax
    def predict(self, dt):
        F = np.eye(4); F[0, 2] = F[1, 3] = dt
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x; self.P = F @ self.P @ F.T + self.q ** 2 * G @ G.T
        sp = math.hypot(self.x[2], self.x[3])
        if sp > self.vmax: self.x[2:] *= self.vmax / sp
    def innov(self, z): return np.asarray(z) - self.x[:2]
    def update(self, z, r=None):
        r = self.r if r is None else r
        H = np.zeros((2, 4)); H[0, 0] = H[1, 1] = 1
        S = H @ self.P @ H.T + r ** 2 * np.eye(2); K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ self.innov(z); self.P = (np.eye(4) - K @ H) @ self.P


class FollowRef:
    """目標方向 = 人的前進方向轉 angle 度;前進方向只在速度穩定 > v_on 時低通更新;方向角速度上限 w_max"""
    def __init__(self, angle_deg=90.0, w_max=0.4, v_on=0.5, h_tau=1.0, flip_margin=0.5):
        self.a, self.w_max, self.v_on, self.h_tau, self.flip_margin = math.radians(angle_deg), w_max, v_on, h_tau, flip_margin
        self.h = None; self.side = None; self.phi = None
    def __call__(self, p, t, vt, dt):
        rel = p - t; phi0 = math.atan2(rel[1], rel[0])
        if self.phi is None: self.phi = phi0
        sp = float(np.linalg.norm(vt))
        if sp > self.v_on:
            hn = math.atan2(vt[1], vt[0])
            if self.h is None: self.h = hn
            elif abs(wrap(hn - self.h)) > math.radians(120): self.h = hn          # 折返:直接換方向
            else: self.h = wrap(self.h + min(1.0, dt / self.h_tau) * wrap(hn - self.h))
        if self.h is None:
            self.phi = phi0; return np.array([math.cos(phi0), math.sin(phi0)])
        c = math.sin(wrap(phi0 - self.h))
        if self.side is None or c * self.side < -self.flip_margin:
            self.side = 1.0 if c >= 0 else -1.0
        target = self.h + self.side * self.a
        d = wrap(target - self.phi)
        if abs(d) > math.radians(150): d = self.side * abs(d)
        self.phi = wrap(self.phi + clip(d, self.w_max * dt))
        return np.array([math.cos(self.phi), math.sin(self.phi)])


class Follower:
    """不依賴 ROS 的控制核心,方便離線測試"""
    def __init__(self, mpc, d=3.0, d_min=2.0, angle=90.0, kf_q=0.6, kf_r=0.4, gate=2.5, tilt_gate=12.0,
                 lost_t=0.7, search_rate=0.5, k_yaw=1.5, yaw_max=1.0, w_ref=0.4, amax=1.5, vmax_h=1.6, cbf_a=1.0):
        self.mpc, self.d, self.d_min = mpc, d, d_min
        self.amax, self.vmax_h, self.cbf_a = amax, vmax_h, cbf_a
        self.n_safe = 0                                          # 安全層介入的步數                    # 水平加速度上限 (m/s^2) -> 機身傾斜約 atan(amax/g);水平速度上限(合速度)
        self.ref = FollowRef(angle, w_max=w_ref)
        self.kf_q, self.kf_r, self.gate, self.tilt_gate = kf_q, kf_r, gate, math.radians(tilt_gate)
        self.lost_t, self.search_rate, self.k_yaw, self.yaw_max = lost_t, search_rate, k_yaw, yaw_max
        self.kf = None; self.t_kf = None; self.t_meas = None; self.n_rej = 0; self.n_tilt = 0; self.n_ok = 0
        self.z_raw = None; self.t_raw = -1e9                           # 最新一筆原始量測(安全層用,沒有濾波延遲)
        self.nis = []; self.t_boost = -1e9                             # 機動偵測:創新量持續偏大 -> 暫時放大 q
        self.u = np.zeros(2); self.last_brg = None; self.status = 'wait'

    def _advance(self, t):
        if self.kf is not None and t > self.t_kf:
            self.kf.predict(t - self.t_kf); self.t_kf = t

    def measure(self, z, t, tilt, rng=None):
        """z: 目標位置量測;tilt: 當下機身傾斜 (rad);rng: 飛機到目標距離(量測雜訊隨距離放大,以 3 m 為基準)"""
        r_eff = self.kf_r * max(1.0, (rng or 3.0) / 3.0)
        if tilt > self.tilt_gate: self.n_tilt += 1; return
        if self.kf is None:
            self.kf = TargetKF(z, self.kf_q, self.kf_r); self.t_kf = t; self.t_meas = t; self.n_ok += 1; return
        lost = t - self.t_meas > self.lost_t
        self._advance(t)
        if not lost and np.linalg.norm(self.kf.innov(z)) > self.gate:
            self.n_rej += 1
            if self.n_rej % 15 == 0: self.kf = TargetKF(z, self.kf_q, self.kf_r); self.t_meas = t   # 連續被拒:重新鎖定
            return
        if lost: self.kf = TargetKF(z, self.kf_q, self.kf_r)                                       # 找回來:重新開始
        else:
            inn = self.kf.innov(z); S = self.kf.P[:2, :2] + r_eff ** 2 * np.eye(2)
            self.nis = (self.nis + [float(inn @ np.linalg.solve(S, inn))])[-6:]
            if len(self.nis) == 6 and np.mean(self.nis) > 4.0: self.t_boost = t           # 人在轉彎/折返
            self.kf.q = self.kf_q * (6.0 if t - self.t_boost < 1.0 else 1.0)
            self.kf.update(z, r_eff)
        self.z_raw, self.t_raw = np.asarray(z, float), t
        self.t_meas = t; self.n_ok += 1

    def _safety(self, u, p, tgt, vt):
        """安全層(控制屏障函數):要求 d/dt(距離) >= -cbf_a * (距離 - d_min)。
        人朝飛機走、或估計落後時,直接加上往外的速度,不管 MPC 想去哪裡。回傳 (u, 是否介入)"""
        r = p - tgt; dist = np.linalg.norm(r)
        if dist < 1e-3: return u, False
        n = r / dist
        need = float(n @ vt) - self.cbf_a * (dist - self.d_min)      # 往外速度的下限
        cur = float(n @ u)
        if cur >= need: return u, False
        u = u + (need - cur) * n
        rad = float(n @ u); tan = u - rad * n
        if np.linalg.norm(u) > self.vmax_h:                           # 超速時先砍橫向,保留往外的分量
            rad = min(rad, self.vmax_h)
            tmax = math.sqrt(max(self.vmax_h ** 2 - rad ** 2, 0.0)); tn = np.linalg.norm(tan)
            if tn > tmax: tan = tan * tmax / tn
            u = rad * n + tan
        return u, True

    def _shape(self, u, dt, urgent=False):
        """命令整形:合速度上限 + 加速度上限(讓機身傾斜小,相機量測才準);安全層介入時加速度上限加倍"""
        u = np.asarray(u, float); sp = np.linalg.norm(u)
        if sp > self.vmax_h: u = u * self.vmax_h / sp
        du = u - self.u; n = np.linalg.norm(du); lim = self.amax * (2.0 if urgent else 1.0) * max(dt, 1e-3)
        if n > lim: u = self.u + du * lim / n
        self.u = u; return u.copy()

    def step(self, p, v, yaw, t, dt):
        """回傳 (水平速度命令, 偏航角速度命令)"""
        if self.kf is None: self.status = 'wait'; return self._shape(np.zeros(2), dt), self.search_rate   # 還沒看過人:原地慢慢轉著找
        if t - self.t_meas > self.lost_t:                     # 丟失:平順停下、朝最後方向轉著找
            self.status = 'lost'
            tgt = self.kf.x[:2]; brg = math.atan2(tgt[1] - p[1], tgt[0] - p[0])
            e = wrap(brg - yaw)
            yr = clip(self.k_yaw * e, self.yaw_max) if abs(e) > 0.3 else math.copysign(self.search_rate, e if e != 0 else 1.0)
            return self._shape(np.zeros(2), dt), yr
        self.status = 'track'
        self._advance(t)
        tgt, vt = self.kf.x[:2].copy(), self.kf.x[2:].copy()
        nr = self.ref(p, tgt, vt, dt)
        u, _, _ = self.mpc.solve(p, v, self.u, tgt, vt, self.d, self.d_min, nr)
        ts = tgt
        if self.z_raw is not None and t - self.t_raw < 0.3 and np.linalg.norm(self.z_raw - p) < np.linalg.norm(tgt - p):
            ts = self.z_raw                                            # 安全層取比較近的那個(估計 or 原始量測),保守
        u, hit = self._safety(np.asarray(u, float), p, ts, vt)
        self.n_safe += hit
        u = self._shape(u, dt, hit)
        rel = tgt - p; brg = math.atan2(rel[1], rel[0]); r2 = max(rel @ rel, 1e-6)
        ff = (rel[0] * (vt - v)[1] - rel[1] * (vt - v)[0]) / r2
        yr = clip(self.k_yaw * wrap(brg - yaw) + 0.5 * ff, self.yaw_max)
        return u, yr


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import State, PositionTarget
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from mpc_core import FollowMPC

    class Node2(Node):
        def __init__(self):
            super().__init__('mpc_follow2')
            P = lambda n, d: self.declare_parameter(n, d).value
            self.live, self.alt = P('live', False), P('alt', 2.0)
            d, dmin, vmax = P('dist', 4.0), P('dist_min', 2.5), P('vmax', 2.0)
            mpc = FollowMPC(vmax=vmax, r_du=P('r_du', 6.0))
            self.f = Follower(mpc, d, dmin, P('angle', 90.0), P('kf_q', 1.5), P('kf_r', 0.55), P('gate', 2.5),
                              P('tilt_gate', 15.0), P('lost_t', 0.7), P('search_rate', 0.5), P('k_yaw', 1.5),
                              P('yaw_max', 1.0), P('w_ref', 0.8), P('amax', 1.5), vmax)
            self.d, self.dmin = d, dmin
            topic = P('target_topic', '/target/meas')               # 預設吃 target_cam 的原始量測,只在這裡濾波一次
            self.state, self.p, self.z, self.yaw, self.tilt, self.v = State(), None, None, None, 0.0, np.zeros(2)
            self.truth = None; self.rows = []; self.t_last = None
            os.makedirs(os.path.expanduser('~/sim/logs'), exist_ok=True)
            self.log_path = os.path.expanduser('~/sim/logs/mpc2_%s.csv' % time.strftime('%H%M%S'))
            self.log = open(self.log_path, 'w')
            self.log.write('t,status,px,py,yaw,tilt,kx,ky,kvx,kvy,wx,wy,ux,uy,yr,ms\n')
            q = qos_profile_sensor_data
            if topic.endswith('odom'): self.create_subscription(Odometry, topic, lambda m: self.on_tgt(m.pose.pose.position), 10)
            else: self.create_subscription(PoseStamped, topic, lambda m: self.on_tgt(m.pose.position), 10)
            self.create_subscription(PoseStamped, '/walker/pose', lambda m: setattr(self, 'truth', np.array([m.pose.position.x, m.pose.position.y])), 10)
            self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, q)
            self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', lambda m: setattr(self, 'v', np.array([m.twist.linear.x, m.twist.linear.y])), q)
            self.create_subscription(State, '/mavros/state', lambda m: setattr(self, 'state', m), 10)
            self.pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
            self.create_timer(0.05, self.step)
            self.get_logger().info('%s  距離 %.1f m (最小 %.1f)  目標來源 %s  log=%s' % ('LIVE' if self.live else 'DRY-RUN', d, dmin, topic, self.log_path))

        def on_pose(self, m):
            self.p = np.array([m.pose.position.x, m.pose.position.y]); self.z = m.pose.position.z
            q = m.pose.orientation
            self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            zz = 1 - 2 * (q.x * q.x + q.y * q.y)                       # 機身 z 軸與垂直的夾角
            self.tilt = math.acos(max(-1.0, min(1.0, zz)))
        def on_tgt(self, pos):
            z = np.array([pos.x, pos.y])
            rng = float(np.linalg.norm(z - self.p)) if self.p is not None else None
            self.f.measure(z, time.time(), self.tilt, rng)

        def step(self):
            if self.p is None or self.yaw is None: return
            now = time.time(); dt = 0.05 if self.t_last is None else now - self.t_last; self.t_last = now
            t0 = time.perf_counter()
            u, yr = self.f.step(self.p, self.v, self.yaw, now, dt)
            ms = (time.perf_counter() - t0) * 1000
            vz = clip(1.0 * (self.alt - self.z), 1.0)
            if self.live and self.state.mode == 'GUIDED' and self.state.armed:
                m = PositionTarget(); m.header.stamp = self.get_clock().now().to_msg()
                m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                m.type_mask = 0b0000011111000111
                m.velocity.x, m.velocity.y, m.velocity.z = float(u[0]), float(u[1]), float(vz)
                m.yaw_rate = float(yr); self.pub.publish(m)
            k = self.f.kf.x if self.f.kf is not None else np.full(4, np.nan)
            w = self.truth if self.truth is not None else np.full(2, np.nan)
            self.log.write('%.3f,%s,%.2f,%.2f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f\n' % (
                now, self.f.status, self.p[0], self.p[1], self.yaw, self.tilt, k[0], k[1], k[2], k[3], w[0], w[1], u[0], u[1], yr, ms))
            self.rows.append((now, self.f.status, self.p.copy(), w.copy(), k[:2].copy(), self.tilt, ms))

        def summary(self):
            self.log.close(); R = self.rows[int(len(self.rows) * 0.1):]
            if len(R) < 20: return
            t = np.array([r[0] for r in R]); st = np.array([r[1] for r in R])
            P_ = np.array([r[2] for r in R]); W = np.array([r[3] for r in R]); K = np.array([r[4] for r in R])
            tilt = np.degrees([r[5] for r in R]); ms = np.array([r[6] for r in R])
            print('\n===== MPC v2 (%d 步, %.0f s) =====' % (len(R), t[-1] - t[0]))
            if np.isfinite(W).all():
                dt_ = np.hypot(*(W - P_).T)
                print('真實距離   平均 %.2f  最近 %.2f  最遠 %.2f m   < %.1f m 的時間 %.0f%%   (目標 %.1f)' % (
                    dt_.mean(), dt_.min(), dt_.max(), self.dmin, 100 * np.mean(dt_ < self.dmin), self.d))
                ek = np.hypot(*(K - W).T); ek = ek[np.isfinite(ek)]
                if len(ek): print('目標估計誤差  中位 %.2f  90%% %.2f m' % (np.median(ek), np.percentile(ek, 90)))
            lost = st == 'lost'; runs, cur = [], 0
            for i in range(1, len(st)):
                if lost[i]: cur += t[i] - t[i - 1]
                elif cur: runs.append(cur); cur = 0
            if cur: runs.append(cur)
            print('丟失時間 %.0f%%  次數 %d  最長 %.1f s' % (100 * lost.mean(), len(runs), max(runs) if runs else 0))
            print('機身傾斜  中位 %.1f°  90%% %.1f°   量測:採用 %d  傾斜過大略過 %d  跳太遠拒絕 %d' % (
                np.median(tilt), np.percentile(tilt, 90), self.f.n_ok, self.f.n_tilt, self.f.n_rej))
            print('安全層介入 %d 步 (%.0f%%)' % (self.f.n_safe, 100 * self.f.n_safe / max(len(self.rows), 1)))
            print('MPC 求解  平均 %.1f ms  最慢 %.1f ms' % (ms.mean(), ms.max()))
            print('記錄檔:', self.log_path)

    rclpy.init(); n = Node2()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.summary()


if __name__ == '__main__':
    main()
