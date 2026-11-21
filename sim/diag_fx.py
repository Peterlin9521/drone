#!/usr/bin/env python3
"""焦距 fx 對不對?把 fx 一起放進搜尋:若最佳 fx 明顯不是 381.5,而且誤差大降,就是內參錯了"""
import sys, os, glob, math
import numpy as np

f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.expanduser('~/sim/logs/cam_*.csv')), key=os.path.getmtime)[-1]
d = np.genfromtxt(f, delimiter=',', names=True)
CX, CY = 320.0, 240.0
ok = (d['upd'] > 0) & np.isfinite(d['wx']) & np.isfinite(d['v2'])
j = np.where(ok)[0]
print(os.path.basename(f), ' 可用的幀 %d / %d   v2 範圍 %.0f~%.0f   u 範圍 %.0f~%.0f' % (
    len(j), len(d), d['v2'][j].min(), d['v2'][j].max(), d['u'][j].min(), d['u'][j].max()))

def Rzyx(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty((len(r), 3, 3))
    R[:, 0, 0] = cy * cp; R[:, 0, 1] = cy * sp * sr - sy * cr; R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp; R[:, 1, 1] = sy * sp * sr + cy * cr; R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp;     R[:, 2, 1] = cp * sr;                R[:, 2, 2] = cp * cr
    return R

W = np.c_[d['wx'][j], d['wy'][j]]; P = np.c_[d['px'][j], d['py'][j]]
Rb = {True: Rzyx(0 * j, 0 * j, d['yaw'][j]), False: Rzyx(d['roll'][j], d['pitch'][j], d['yaw'][j])}
def ground(fx, tilt, dz, stab):
    ct, st = math.cos(tilt), math.sin(tilt)
    Rc = Rb[stab] @ np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
    ray = np.stack([np.full(len(j), fx), -(d['u'][j] - CX), -(d['v2'][j] - CY)], 1)
    dv = np.einsum('nij,nj->ni', Rc, ray)
    s = np.where(dv[:, 2] < -1e-6, -(d['pz'][j] + dz) / np.minimum(dv[:, 2], -1e-6), np.nan)
    return P + s[:, None] * dv[:, :2]
def err(g):
    e = np.hypot(*(g - W).T); e[~np.isfinite(e) | (e > 20)] = 20.0; return e

fxs = np.arange(200, 801, 25.0); tilts = np.radians(np.arange(0, 50.01, 1.0)); dzs = np.arange(-1.0, 1.01, 0.2)
print('\n  fx     雲台穩定: 傾角 高度  中位      機身姿態: 傾角 高度  中位')
best = None
for fx in fxs:
    line = '  %3.0f ' % fx
    for stab in (True, False):
        b = min((np.median(err(ground(fx, t, z, stab))), t, z) for t in tilts for z in dzs)
        line += '        %4.0f° %+4.1f  %.2f m' % (math.degrees(b[1]), b[2], b[0])
        if best is None or b[0] < best[0]: best = (b[0], fx, b[1], b[2], stab)
    print(line + ('   <- 目前用的' if abs(fx - 381.5) < 13 else ''))

m, fx, t, z, stab = best
e = err(ground(fx, t, z, stab)); g = ground(fx, t, z, stab)
print('\n最佳: fx %.0f  %s  傾角 %.0f°  高度修正 %+.1f  中位 %.2f m  90%% %.2f m' % (fx, '雲台穩定' if stab else '機身姿態', math.degrees(t), z, np.median(e), np.percentile(e, 90)))
rel = W - P; rg = np.linalg.norm(rel, axis=1); u = rel / np.maximum(rg, 1e-6)[:, None]; rad = ((g - W) * u).sum(1)
print('最佳組合的沿視線偏差 vs 距離:')
for a, b in ((0, 2.5), (2.5, 3.5), (3.5, 4.5), (4.5, 99)):
    mm = (rg >= a) & (rg < b) & np.isfinite(rad)
    if mm.any(): print('  %3.1f-%-4.1f m  n=%4d  偏差 %+.2f  中位誤差 %.2f' % (a, min(b, 9.9), mm.sum(), np.median(rad[mm]), np.median(e[mm])))
