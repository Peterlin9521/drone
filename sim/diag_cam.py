#!/usr/bin/env python3
"""閉環相機誤差拆解:量測(gx,gy) vs 濾波(kx,ky),依距離/機身傾斜/轉身/畫面邊緣分組"""
import sys, os, glob, math
import numpy as np

f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.expanduser('~/sim/logs/cam_*.csv')), key=os.path.getmtime)[-1]
d = np.genfromtxt(f, delimiter=',', names=True)
d = d[np.isfinite(d['wx'])]
t = d['t'] - d['t'][0]
print(os.path.basename(f), ' 幀數 %d  時長 %.0f s' % (len(d), t[-1]))

W = np.c_[d['wx'], d['wy']]; P = np.c_[d['px'], d['py']]
K = np.c_[d['kx'], d['ky']]; G = np.c_[d['gx'], d['gy']]
upd = d['upd'] > 0
ok_g = upd & np.isfinite(G).all(1)
ek = np.hypot(*(K - W).T)
eg = np.full(len(d), np.nan); eg[ok_g] = np.hypot(*(G[ok_g] - W[ok_g]).T)
rng = np.hypot(*(W - P).T)
tilt = np.degrees(np.hypot(d['roll'], d['pitch']))

def s(e):
    e = e[np.isfinite(e)]
    return '  n=%4d  中位 %.2f  90%% %.2f' % (len(e), np.median(e), np.percentile(e, 90)) if len(e) else '  n=   0'

print('\n[1] 量測 vs 濾波')
print('原始量測 (有偵測的幀)      ' + s(eg))
print('KF 輸出 (有偵測的幀)       ' + s(ek[upd]))
print('KF 輸出 (沒偵測、靠預測)   ' + s(ek[~upd]))

rel = W - P; u = rel / np.maximum(np.linalg.norm(rel, axis=1, keepdims=True), 1e-6)
dg = G - W
rad = np.where(ok_g, (dg * u).sum(1), np.nan)
tan = np.where(ok_g, dg[:, 0] * -u[:, 1] + dg[:, 1] * u[:, 0], np.nan)
print('\n[2] 量測偏差  沿視線 平均 %+.2f m (正=估太遠)   橫向 平均 %+.2f m' % (np.nanmean(rad), np.nanmean(tan)))

print('\n[3] 依真實距離分組 (原始量測)')
for a, b in ((0, 2.0), (2.0, 2.5), (2.5, 3.5), (3.5, 4.5), (4.5, 99)):
    m = (rng >= a) & (rng < b)
    print('  %3.1f-%-4.1f m  時間佔 %3.0f%%' % (a, min(b, 9.9), 100 * m.mean()) + s(eg[m]) + '   沿視線偏差 %+.2f' % (np.nanmean(rad[m]) if np.isfinite(rad[m]).any() else np.nan))

print('\n[4] 依機身傾斜分組 (原始量測)')
for a, b in ((0, 5), (5, 10), (10, 15), (15, 90)):
    m = (tilt >= a) & (tilt < b)
    print('  %2d-%-2d°   時間佔 %3.0f%%' % (a, b, 100 * m.mean()) + s(eg[m]))

print('\n[5] 腳的像素位置')
edge_v = ok_g & (d['v2'] >= 470); edge_u = ok_g & ((d['u'] <= 15) | (d['u'] >= 625))
print('  腳貼畫面下緣 (v2>=470)   佔偵測 %3.0f%%' % (100 * edge_v.sum() / max(ok_g.sum(), 1)) + s(eg[edge_v]))
print('  人貼畫面左右緣            佔偵測 %3.0f%%' % (100 * edge_u.sum() / max(ok_g.sum(), 1)) + s(eg[edge_u]))
print('  其他                                    ' + s(eg[ok_g & ~edge_v & ~edge_u]))

ib = np.clip(np.searchsorted(t, t - 1.5), 0, len(t) - 1); ia = np.clip(np.searchsorted(t, t + 1.5), 0, len(t) - 1)
vb, va = W - W[ib], W[ia] - W
cosang = (vb * va).sum(1) / np.maximum(np.linalg.norm(vb, axis=1) * np.linalg.norm(va, axis=1), 1e-6)
turn = cosang < 0.5
print('\n[6] 人轉身前後 1.5 s vs 直走  (KF 輸出)')
print('  轉身  ' + s(ek[turn])); print('  直走  ' + s(ek[~turn]))

gaps, run, prev = [], 0.0, t[0]
for ti, ui in zip(t, upd):
    if not ui: run += ti - prev
    elif run > 0: gaps.append(run); run = 0.0
    prev = ti
if run > 0: gaps.append(run)
print('\n[7] 沒偵測的連續時間  最長 %.1f s   >1 s 的次數 %d' % (max(gaps) if gaps else 0, sum(g > 1 for g in gaps)))
print('    沒有候選框的幀 %.0f%%   被過濾掉的候選 共 %d' % (100 * np.mean(d['ncand'] == 0), int(d['nrej'].sum())))
