#!/usr/bin/env python3
"""真實位置(/walker/pose)是否跟畫面裡的人對不上時間?測試把真實位置往前/往後挪,看誤差是否大幅下降"""
import sys, os, glob, math
import numpy as np

f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.expanduser('~/sim/logs/cam_*.csv')), key=os.path.getmtime)[-1]
d = np.genfromtxt(f, delimiter=',', names=True)
FX, CX, CY = 381.5, 320.0, 240.0
t = d['t'] - d['t'][0]
okw = np.isfinite(d['wx'])
ok = (d['upd'] > 0) & okw & np.isfinite(d['v2'])
j = np.where(ok)[0]
print(os.path.basename(f), ' 可用的幀 %d / %d' % (len(j), len(d)))

def Rzyx(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty((len(r), 3, 3))
    R[:, 0, 0] = cy * cp; R[:, 0, 1] = cy * sp * sr - sy * cr; R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp; R[:, 1, 1] = sy * sp * sr + cy * cr; R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp;     R[:, 2, 1] = cp * sr;                R[:, 2, 2] = cp * cr
    return R

def truth_at(shift):
    tt = t[j] + shift
    return np.c_[np.interp(tt, t[okw], d['wx'][okw]), np.interp(tt, t[okw], d['wy'][okw])]

ray = np.stack([np.full(len(j), FX), -(d['u'][j] - CX), -(d['v2'][j] - CY)], 1)
def ground(tilt, dz, stab):
    r = np.zeros(len(j)) if stab else d['roll'][j]
    p = np.zeros(len(j)) if stab else d['pitch'][j]
    ct, st = math.cos(tilt), math.sin(tilt)
    Rc = Rzyx(r, p, d['yaw'][j]) @ np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
    dv = np.einsum('nij,nj->ni', Rc, ray)
    pz = d['pz'][j] + dz
    s = np.where(dv[:, 2] < -1e-6, -pz / np.minimum(dv[:, 2], -1e-6), np.nan)
    return np.c_[d['px'][j], d['py'][j]] + s[:, None] * dv[:, :2]

tilts = np.radians(np.arange(10, 40.01, 1.0)); dzs = np.arange(-0.6, 0.81, 0.2)
G = {(stab, ti, zi): ground(t_, z_, stab) for stab in (True, False) for ti, t_ in enumerate(tilts) for zi, z_ in enumerate(dzs)}

def err(g, W):
    e = np.hypot(*(g - W).T); e[~np.isfinite(e) | (e > 20)] = 20.0; return e

print('\n真實位置時間偏移   雲台穩定模型          機身姿態模型')
best = None
for sh in np.round(np.arange(-1.5, 1.51, 0.1), 2):
    W = truth_at(sh); line = '  %+4.1f s ' % sh
    for stab in (True, False):
        b = min((np.median(err(G[(stab, ti, zi)], W)), ti, zi) for ti in range(len(tilts)) for zi in range(len(dzs)))
        line += '   %4.1f° %+4.1f  中位 %.2f m' % (math.degrees(tilts[b[1]]), dzs[b[2]], b[0])
        if best is None or b[0] < best[0]: best = (b[0], sh, stab, b[1], b[2])
    print(line)

m, sh, stab, ti, zi = best
W0 = truth_at(0.0); W = truth_at(sh); g = G[(stab, ti, zi)]; e = err(g, W)
print('\n最佳: 真實位置偏移 %+.1f s  %s  傾角 %.0f°  高度修正 %+.1f  中位 %.2f m  90%% %.2f m' % (
    sh, '雲台穩定' if stab else '機身姿態', math.degrees(tilts[ti]), dzs[zi], np.median(e), np.percentile(e, 90)))

Wall = np.c_[d['wx'], d['wy']]
ib = np.clip(np.searchsorted(t, t[j] - 0.5), 0, len(t) - 1); ia = np.clip(np.searchsorted(t, t[j] + 0.5), 0, len(t) - 1)
v = (Wall[ia] - Wall[ib]) / np.maximum((t[ia] - t[ib])[:, None], 1e-3); sp = np.hypot(*v.T)
mv = sp > 0.3; h = v / np.maximum(sp, 1e-6)[:, None]
g0 = G[(stab, ti, zi)]
along = ((g0 - W0) * h).sum(1)
print('\n人的速度 中位 %.2f m/s' % np.median(sp[mv]))
print('不偏移時,量測點相對真實位置「沿人前進方向」的偏差: 平均 %+.2f m  (正=量測點在人前面)' % np.nanmean(along[mv]))
