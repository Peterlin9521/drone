#!/usr/bin/env python3
"""直走誤差比轉身大 -> 懷疑時間差。用正確內參重算地面點,分別把『飛機姿態/位置』和『真實位置』往前後挪,看誤差在哪個偏移最小。
用法: diag_lag.py [cam.csv] [fx] [tilt_deg] [cam_dz]"""
import sys, os, glob, math
import numpy as np

a = sys.argv[1:]
f = a[0] if a else sorted(glob.glob(os.path.expanduser('~/sim/logs/cam_*.csv')), key=os.path.getmtime)[-1]
FX = float(a[1]) if len(a) > 1 else 265.0; TILT = math.radians(float(a[2]) if len(a) > 2 else 15.5); DZ = float(a[3]) if len(a) > 3 else -0.5
CX, CY = 320.0, 240.0
d = np.genfromtxt(f, delimiter=',', names=True)
d = d[np.isfinite(d['wx'])]; t = d['t'] - d['t'][0]; N = len(d)
ok = (d['upd'] > 0) & np.isfinite(d['v2']); j = np.where(ok)[0]
print(os.path.basename(f), '  fx %.0f  傾角 %.1f°  高度修正 %+.1f   可用 %d 幀' % (FX, math.degrees(TILT), DZ, len(j)))

def Rzyx(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty((len(r), 3, 3))
    R[:, 0, 0] = cy * cp; R[:, 0, 1] = cy * sp * sr - sy * cr; R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp; R[:, 1, 1] = sy * sp * sr + cy * cr; R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp;     R[:, 2, 1] = cp * sr;                R[:, 2, 2] = cp * cr
    return R
ct, st = math.cos(TILT), math.sin(TILT); Ry = np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
ray = np.stack([np.full(len(j), FX), -(d['u'][j] - CX), -(d['v2'][j] - CY)], 1)
def interp(col, tt): return np.interp(tt, t, d[col])
def interp_ang(col, tt): return np.angle(np.interp(tt, t, np.cos(d[col])) + 1j * np.interp(tt, t, np.sin(d[col])))

def ground(pose_shift):
    tt = t[j] + pose_shift
    Rc = Rzyx(interp_ang('roll', tt), interp_ang('pitch', tt), interp_ang('yaw', tt)) @ Ry
    dv = np.einsum('nij,nj->ni', Rc, ray)
    s = np.where(dv[:, 2] < -1e-6, -(interp('pz', tt) + DZ) / np.minimum(dv[:, 2], -1e-6), np.nan)
    return np.c_[interp('px', tt), interp('py', tt)] + s[:, None] * dv[:, :2]
def truth(shift):
    tt = t[j] + shift; return np.c_[interp('wx', tt), interp('wy', tt)]
def med(g, W):
    e = np.hypot(*(g - W).T); e = e[np.isfinite(e)]; return np.median(e), np.percentile(e, 90)

shifts = np.round(np.arange(-1.0, 1.01, 0.1), 2)
print('\n偏移(s)   挪飛機姿態/位置        挪真實位置')
G0, W0 = ground(0.0), truth(0.0)
for s in shifts:
    a1 = med(ground(s), W0); a2 = med(G0, truth(s))
    print('  %+4.1f    中位 %.2f  90%% %.2f     中位 %.2f  90%% %.2f' % (s, a1[0], a1[1], a2[0], a2[1]))

def dirs(cols, tt):
    x2, y2 = interp(cols[0], tt + 0.5), interp(cols[1], tt + 0.5); x1, y1 = interp(cols[0], tt - 0.5), interp(cols[1], tt - 0.5)
    v = np.c_[x2 - x1, y2 - y1]; sp = np.hypot(*v.T); return v / np.maximum(sp, 1e-6)[:, None], sp
hw, sw = dirs(('wx', 'wy'), t[j]); hp, sp_ = dirs(('px', 'py'), t[j])
e = G0 - W0
m = (sw > 0.3) & np.isfinite(e).all(1)
print('\n量測點相對真實位置: 沿「人前進方向」平均 %+.2f m   (人速 中位 %.2f m/s)' % (np.mean((e * hw).sum(1)[m]), np.median(sw[m])))
m2 = (sp_ > 0.3) & np.isfinite(e).all(1)
print('                    沿「飛機前進方向」平均 %+.2f m   (機速 中位 %.2f m/s)' % (np.mean((e * hp).sum(1)[m2]), np.median(sp_[m2])))
