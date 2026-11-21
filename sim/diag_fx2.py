#!/usr/bin/env python3
"""從水平方位直接估實際焦距(跟高度、傾角無關),再用這個焦距重做地面投影校正"""
import sys, os, glob, math
import numpy as np

f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.expanduser('~/sim/logs/cam_*.csv')), key=os.path.getmtime)[-1]
CX, CY = 320.0, 240.0
d = np.genfromtxt(f, delimiter=',', names=True)
d = d[(d['upd'] > 0) & np.isfinite(d['wx']) & np.isfinite(d['v2']) & np.isfinite(d['u'])]
rel = np.c_[d['wx'] - d['px'], d['wy'] - d['py']]; rg = np.hypot(*rel.T)
rb = (np.arctan2(rel[:, 1], rel[:, 0]) - d['yaw'] + np.pi) % (2 * np.pi) - np.pi
print(os.path.basename(f), ' 可用的幀 %d' % len(d))

print('\n[1] 水平方位:  fx   中位方位誤差')
best = None
for fx in np.arange(120, 451, 10.0):
    r = rb - np.arctan2(CX - d['u'], fx); r -= np.median(r)
    m = np.degrees(np.median(np.abs(r)))
    if best is None or m < best[0]: best = (m, fx)
    if fx % 40 == 0 or abs(fx - 380) < 5: print('   %4.0f   %.2f°' % (fx, m))
fxb = float(sys.argv[2]) if len(sys.argv) > 2 else best[1]
print('   -> 最佳 fx = %.0f   (水平視角 %.0f°;camera_info 說 381.5 = 80°)' % (fxb, 2 * math.degrees(math.atan(CX / fxb))))

def Rzyx(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty((len(r), 3, 3))
    R[:, 0, 0] = cy * cp; R[:, 0, 1] = cy * sp * sr - sy * cr; R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp; R[:, 1, 1] = sy * sp * sr + cy * cr; R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp;     R[:, 2, 1] = cp * sr;                R[:, 2, 2] = cp * cr
    return R
W = np.c_[d['wx'], d['wy']]; P = np.c_[d['px'], d['py']]
def ground_err(fx, tilt, dz, stab):
    Rb = Rzyx(0 * rg, 0 * rg, d['yaw']) if stab else Rzyx(d['roll'], d['pitch'], d['yaw'])
    ct, st = math.cos(tilt), math.sin(tilt)
    Rc = Rb @ np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
    ray = np.stack([np.full(len(rg), fx), -(d['u'] - CX), -(d['v2'] - CY)], 1)
    dv = np.einsum('nij,nj->ni', Rc, ray)
    s = np.where(dv[:, 2] < -1e-6, -(d['pz'] + dz) / np.minimum(dv[:, 2], -1e-6), np.nan)
    e = np.hypot(*(P + s[:, None] * dv[:, :2] - W).T); e[~np.isfinite(e) | (e > 20)] = 20.0
    return e
print('\n[2] 用 fx=%.0f 重新校正地面投影' % fxb)
for fx in (381.5, fxb):
    for stab in (True, False):
        b = min((np.median(ground_err(fx, t, z, stab)), t, z) for t in np.radians(np.arange(0, 45.01, 1.0)) for z in np.arange(-1.0, 1.01, 0.1))
        e = ground_err(fx, b[1], b[2], stab)
        print('   fx %5.1f  %-8s  傾角 %4.1f°  高度修正 %+4.1f  中位 %.2f m  90%% %.2f m' % (fx, '雲台穩定' if stab else '機身姿態', math.degrees(b[1]), b[2], np.median(e), np.percentile(e, 90)))
