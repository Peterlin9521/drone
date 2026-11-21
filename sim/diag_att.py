#!/usr/bin/env python3
"""姿態補償診斷:測試 pitch/roll 正負號、姿態時間偏移、傾角、高度,看哪個組合最能解釋閉環資料"""
import sys, os, glob, math
import numpy as np

f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.expanduser('~/sim/logs/cam_*.csv')), key=os.path.getmtime)[-1]
d = np.genfromtxt(f, delimiter=',', names=True)
FX, CX, CY = 381.5, 320.0, 240.0
N = len(d)
ok = (d['upd'] > 0) & np.isfinite(d['wx']) & np.isfinite(d['v2'])
print(os.path.basename(f), ' 可用的幀 %d / %d' % (ok.sum(), N))

def Rzyx(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty((len(r), 3, 3))
    R[:, 0, 0] = cy * cp; R[:, 0, 1] = cy * sp * sr - sy * cr; R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp; R[:, 1, 1] = sy * sp * sr + cy * cr; R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp;     R[:, 2, 1] = cp * sr;                R[:, 2, 2] = cp * cr
    return R

def ground(idx_pose, tilt, dz, sp, sr, stab):
    i = idx_pose
    r = np.zeros(len(i)) if stab else sr * d['roll'][i]
    p = np.zeros(len(i)) if stab else sp * d['pitch'][i]
    Rb = Rzyx(r, p, d['yaw'][i])
    ct, st = math.cos(tilt), math.sin(tilt)
    Ry = np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
    Rc = Rb @ Ry
    j = np.where(ok)[0]
    ray = np.stack([np.full(len(j), FX), -(d['u'][j] - CX), -(d['v2'][j] - CY)], 1)
    dv = np.einsum('nij,nj->ni', Rc, ray)
    pc = np.stack([d['px'][i], d['py'][i], d['pz'][i] + dz], 1)
    s = np.where(dv[:, 2] < -1e-6, -pc[:, 2] / np.minimum(dv[:, 2], -1e-6), np.nan)
    g = pc[:, :2] + s[:, None] * dv[:, :2]
    e = np.hypot(g[:, 0] - d['wx'][j], g[:, 1] - d['wy'][j])
    e[~np.isfinite(e) | (e > 20)] = 20.0
    return e, g

j = np.where(ok)[0]
tilts = np.radians(np.arange(5, 45.01, 1.0)); dzs = np.arange(-0.6, 0.81, 0.1)

def best_over(sp, sr, shift, stab):
    idx = np.clip(j + shift, 0, N - 1)
    b = None
    for t in tilts:
        for z in dzs:
            m = np.median(ground(idx, t, z, sp, sr, stab)[0])
            if b is None or m < b[0]: b = (m, t, z)
    return b

print('\n[A] 正負號 x 模型 (不偏移)')
rows = [('雲台穩定(只用機頭)', 1, 1, True)] + [('機身 pitch%+d roll%+d' % (a, c), a, c, False) for a in (1, -1) for c in (1, -1)]
res = []
for name, a, c, stab in rows:
    m, t, z = best_over(a, c, 0, stab); res.append((m, name, a, c, stab))
    print('  %-20s 傾角 %4.1f°  高度修正 %+4.1f  中位誤差 %.2f m' % (name, math.degrees(t), z, m))
res.sort(); _, bname, ba, bc, bstab = res[0]

print('\n[B] 姿態/位置時間偏移 (用最好的 %s;正=用較晚的姿態)' % bname)
dt = np.median(np.diff(d['t']))
sh_res = []
for sh in range(-8, 9):
    m, t, z = best_over(ba, bc, sh, bstab); sh_res.append((m, sh, t, z))
    print('  偏移 %+3d 幀 (%+5.2f s)  傾角 %4.1f°  高度修正 %+4.1f  中位誤差 %.2f m' % (sh, sh * dt, math.degrees(t), z, m))
m, sh, t, z = min(sh_res)
e, g = ground(np.clip(j + sh, 0, N - 1), t, z, ba, bc, bstab)
print('\n最佳組合: %s  偏移 %+d 幀  傾角 %.1f°  高度修正 %+.1f  中位 %.2f m  90%% %.2f m' % (bname, sh, math.degrees(t), z, np.median(e), np.percentile(e, 90)))

P = np.c_[d['px'][j], d['py'][j]]; W = np.c_[d['wx'][j], d['wy'][j]]
rel = W - P; rg = np.linalg.norm(rel, axis=1); u = rel / np.maximum(rg, 1e-6)[:, None]
rad = ((g - W) * u).sum(1)
print('\n[C] 最佳組合的沿視線偏差 vs 距離')
for a, b in ((0, 2.5), (2.5, 3.5), (3.5, 4.5), (4.5, 99)):
    mm = (rg >= a) & (rg < b) & np.isfinite(rad)
    if mm.any(): print('  %3.1f-%-4.1f m  n=%4d  偏差 %+.2f  中位誤差 %.2f' % (a, min(b, 9.9), mm.sum(), np.median(rad[mm]), np.median(e[mm])))
pd = np.degrees(d['pitch'][j]); rd = np.degrees(d['roll'][j])
print('\n[D] 誤差 vs 機身 pitch / roll (最佳組合)')
for lab, x in (('pitch', pd), ('roll', rd)):
    for a, b in ((-90, -10), (-10, -3), (-3, 3), (3, 10), (10, 90)):
        mm = (x >= a) & (x < b)
        if mm.sum() > 10: print('  %-5s %+4d~%+-4d°  n=%4d  沿視線偏差 %+.2f  中位誤差 %.2f' % (lab, max(a, -99), min(b, 99), mm.sum(), np.nanmedian(rad[mm]), np.median(e[mm])))
