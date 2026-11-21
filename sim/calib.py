#!/usr/bin/env python3
"""用真實位置校正相機俯仰角:找出讓『腳的像素 -> 地面點』最接近真實位置的傾角與模型"""
import sys, os, glob, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cam_geom import cam_R, pixel_to_ground, Rz

f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.expanduser('~/sim/logs/cam_*.csv')), key=os.path.getmtime)[-1]
d = np.genfromtxt(f, delimiter=',', names=True)
m = (d['upd'] > 0) & np.isfinite(d['wx']) & np.isfinite(d['v2'])
d = d[m]
print(os.path.basename(f), ' 可用的幀: %d' % len(d))
if len(d) < 30: sys.exit('資料太少,至少要 30 幀有偵測到人')
FX, CX, CY = 381.5, 320.0, 240.0

def Rbody(r, p, y):
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]); Ry_ = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Rz(y) @ Ry_ @ Rx

def errors(tilt, stab, dz):
    e = []
    for row in d:
        Rc = cam_R(Rbody(row['roll'], row['pitch'], row['yaw']), row['yaw'], tilt, stab)
        g = pixel_to_ground(row['u'], row['v2'], FX, CX, CY, np.array([row['px'], row['py'], row['pz'] + dz]), Rc)
        e.append(np.hypot(g[0] - row['wx'], g[1] - row['wy']) if g is not None else 99.0)
    return np.array(e)

print('\n模型               最佳傾角   相機高度修正   中位誤差   90% 誤差')
best = None
for stab in (True, False):
    for dz in (-1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.3):
        for tdeg in np.arange(0, 45.01, 0.5):
            e = errors(math.radians(tdeg), stab, dz); med = np.median(e)
            if best is None or med < best[0]: best = (med, stab, dz, tdeg, np.percentile(e, 90))
    r = [b for b in [best] if b[1] == stab]
    sb = min(((np.median(errors(math.radians(t), stab, z)), t, z) for z in (-1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.3) for t in np.arange(0, 45.01, 0.5)))
    e = errors(math.radians(sb[1]), stab, sb[2])
    print('%-16s  %6.1f°     %+5.1f m      %5.2f m    %5.2f m' % ('雲台穩定(只用機頭)' if stab else '跟著機身姿態', sb[1], sb[2], np.median(e), np.percentile(e, 90)))
print('\n建議參數: -p tilt_deg:=%.1f -p stabilized:=%s -p cam_dz:=%.1f' % (best[3], 'true' if best[1] else 'false', best[2]))
