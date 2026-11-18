#!/usr/bin/env python3
"""MPC 側邊跟隨(第一版:目標位置用 /walker/pose)"""
import os, sys, time, math
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mpc_core import FollowMPC

wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
clip = lambda v, m: max(-m, min(m, v))

class FollowRef:
    """目標方向 = 人的行進方向轉 angle 度(90 側邊、180 正後方)。人走得夠快才更新行進方向;
    側邊依飛機目前位置選(有遲滯);目標方向最多每秒轉 w_max,正後方折返時從飛機所在的那一側繞過去。"""
    def __init__(self, angle_deg=90.0, w_max=0.7, v_on=0.4, flip_margin=0.5):
        self.a, self.w_max, self.v_on, self.flip_margin = math.radians(angle_deg), w_max, v_on, flip_margin
        self.h = self.side = self.phi = None
    def __call__(self, p, t, vt, dt):
        rel = p - t; phi0 = math.atan2(rel[1], rel[0])
        if self.phi is None: self.phi = phi0
        if np.linalg.norm(vt) > self.v_on: self.h = math.atan2(vt[1], vt[0])
        if self.h is None:
            self.phi = phi0; return np.array([math.cos(phi0), math.sin(phi0)])
        c = math.sin(wrap(phi0 - self.h))
        if self.side is None or c * self.side < -self.flip_margin:
            self.side = 1.0 if c >= 0 else -1.0
        d = wrap(self.h + self.side * self.a - self.phi)
        if abs(d) > math.radians(150): d = self.side * abs(d)
        self.phi = wrap(self.phi + max(-self.w_max * dt, min(self.w_max * dt, d)))
        return np.array([math.cos(self.phi), math.sin(self.phi)])

class MPCFollow(Node):
    def __init__(self):
        super().__init__('mpc_follow')
        P = lambda n, d: self.declare_parameter(n, d).value
        self.live, self.side = P('live', False), P('side', True)
        self.d, self.d_min, self.alt = P('dist', 3.0), P('dist_min', 2.0), P('alt', 2.0)
        self.vmax, self.yaw_max, self.noise = P('vmax', 2.0), P('yaw_max', 2.0), P('noise', 0.0)
        self.angle, self.r_du, self.w_ref = P('angle', 90.0), P('r_du', 6.0), P('w_ref', 0.7)
        self.mpc, self.ref = FollowMPC(vmax=self.vmax, r_du=self.r_du), FollowRef(self.angle, self.w_ref)
        self.state, self.p, self.z, self.yaw, self.v = State(), None, None, None, np.zeros(2)
        self.tgt, self.tgt_t, self.tv, self.u = None, 0.0, np.zeros(2), np.zeros(2)
        self.rng = np.random.default_rng(0); self.rows = []
        os.makedirs(os.path.expanduser('~/sim/logs'), exist_ok=True)
        self.log_path = os.path.expanduser('~/sim/logs/mpc_%s_%s.csv' % ('side' if self.side else 'dist', time.strftime('%H%M%S')))
        self.log = open(self.log_path, 'w'); self.log.write('t,px,py,yaw,tx,ty,tvx,tvy,ux,uy,dist,ms\n')
        q = qos_profile_sensor_data
        self.create_subscription(PoseStamped, '/walker/pose', self.on_walker, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, q)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.on_vel, q)
        self.create_subscription(State, '/mavros/state', lambda m: setattr(self, 'state', m), 10)
        self.pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.create_timer(0.05, self.step)
        self.get_logger().info('%s  %s  距離 %.1f m(最小 %.1f m)  log=%s' % (
            'LIVE' if self.live else 'DRY-RUN', ('跟隨角度 %.0f°' % self.angle) if self.side else '只保持距離MPC', self.d, self.d_min, self.log_path))

    def on_pose(self, m):
        self.p = np.array([m.pose.position.x, m.pose.position.y]); self.z = m.pose.position.z
        q = m.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    def on_vel(self, m):
        self.v = np.array([m.twist.linear.x, m.twist.linear.y])
    def on_walker(self, m):
        now = time.time()
        z = np.array([m.pose.position.x, m.pose.position.y]) + self.noise * self.rng.standard_normal(2)
        if self.tgt is not None and now - self.tgt_t > 1e-3:
            self.tv = 0.85 * self.tv + 0.15 * (z - self.tgt) / (now - self.tgt_t)
        self.tgt, self.tgt_t = z, now

    def step(self):
        if self.p is None or self.yaw is None or self.tgt is None: return
        t0 = time.perf_counter()
        nr = self.ref(self.p, self.tgt, self.tv, 0.05) if self.side else None
        u, _, _ = self.mpc.solve(self.p, self.v, self.u, self.tgt, self.tv, self.d, self.d_min, nr)
        self.u = np.asarray(u, float); ms = (time.perf_counter() - t0) * 1000
        rel = self.tgt - self.p; r2 = max(rel @ rel, 1e-6)
        brg_rate = (rel[0] * (self.tv - self.v)[1] - rel[1] * (self.tv - self.v)[0]) / r2
        yr = clip(3.0 * wrap(math.atan2(rel[1], rel[0]) - self.yaw) + brg_rate, self.yaw_max)
        vz = clip(1.0 * (self.alt - self.z), 1.0)
        if self.live and self.state.mode == 'GUIDED' and self.state.armed:
            m = PositionTarget()
            m.header.stamp = self.get_clock().now().to_msg()
            m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            m.type_mask = 0b0000011111000111
            m.velocity.x, m.velocity.y, m.velocity.z = float(self.u[0]), float(self.u[1]), float(vz)
            m.yaw_rate = float(yr)
            self.pub.publish(m)
        dist = math.sqrt(r2)
        self.rows.append((dist, ms, self.tgt.copy(), self.tv.copy(), self.p.copy()))
        self.log.write('%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f\n' % (
            time.time(), self.p[0], self.p[1], self.yaw, self.tgt[0], self.tgt[1], self.tv[0], self.tv[1], self.u[0], self.u[1], dist, ms))

    def summary(self):
        self.log.close()
        if len(self.rows) < 20: return
        R = self.rows[int(len(self.rows) * 0.1):]
        d = np.array([r[0] for r in R]); ms = np.array([r[1] for r in R])
        on = [abs(float((r[2] - r[4]) @ r[3])) / (r[0] * np.linalg.norm(r[3])) > math.cos(math.radians(30))
              for r in R if np.linalg.norm(r[3]) > 0.3]
        print('\n===== %s(%d 步)=====' % ('側邊MPC' if self.side else '只保持距離MPC', len(R)))
        print('距離  平均 %.2f m  最近 %.2f m  最遠 %.2f m   (目標 %.1f,最小 %.1f)' % (d.mean(), d.min(), d.max(), self.d, self.d_min))
        print('在人的行進線上(前後 30° 內)的比例: %.0f%%' % (100 * np.mean(on) if on else 0))
        print('MPC 求解  平均 %.1f ms  最慢 %.1f ms' % (ms.mean(), ms.max()))
        print('記錄檔:', self.log_path)

rclpy.init()
node = MPCFollow()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.summary()
