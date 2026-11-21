#!/usr/bin/env python3
"""直接比角度:用真實位置算出『腳應該在的角度』,跟像素算出的角度做線性回歸。
斜率 = 1 代表焦距/視角正確;水平和垂直斜率都偏離 1 -> 影像的實際視角跟 camera_info 不一樣"""
import sys, os, glob, math
import numpy as np

f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.expanduser('~/sim/logs/cam_*.csv')), key=os.path.getmtime)[-1]
FX, CX, CY = 381.5, 320.0, 240.0
d = np.genfromtxt(f, delimiter=',', names=True)
ok = (d['upd'] > 0) & np.isfinite(d['wx']) & np.isfinite(d['v2']) & np.isfinite(d['u'])
d = d[ok]
print(os.path.basename(f), ' 可用的幀 %d' % len(d))
rel = np.c_[d['wx'] - d['px'], d['wy'] - d['py']]; rg = np.hypot(*rel.T)
brg = np.arctan2(rel[:, 1], rel[:, 0])
rb = (brg - d['yaw'] + np.pi) % (2 * np.pi) - np.pi
pix_h = np.arctan2(CX - d['u'], FX)
pix_v = np.arctan2(d['v2'] - CY, FX)

def fit(y, X, names):
    A = np.c_[X, np.ones(len(y))]; c, *_ = np.linalg.lstsq(A, y, rcond=None); r = y - A @ c
    print('   ' + '  '.join('%s %+.3f' % (n, v) for n, v in zip(names + ['常數'], c)) + '   殘差 %.2f°' % np.degrees(np.std(r)))
    return c

print('\n[水平] 真實方位 = a x 像素水平角 + b x roll + 常數')
fit(rb, np.c_[pix_h, d['roll']], ['a', 'b'])
for dz in (-0.5, 0.0, 0.5):
    print('\n[垂直] 相機高度修正 %+.1f m:  真實俯角 = a x 像素垂直角 + b x pitch + 常數(=傾角)' % dz)
    dep = np.arctan2(d['pz'] + dz, rg)
    c = fit(dep, np.c_[pix_v, d['pitch']], ['a', 'b'])
    print('   -> 常數 = %.1f°' % np.degrees(c[2]))
print('\n距離範圍 %.1f ~ %.1f m   像素垂直角範圍 %.1f ~ %.1f°' % (rg.min(), rg.max(), np.degrees(pix_v.min()), np.degrees(pix_v.max())))
