#!/usr/bin/env python3
"""mpc2 記錄檔的時間軸:每 2 秒一行,看太近/太遠/丟失是什麼時候、怎麼發生的"""
import sys, os, glob
import numpy as np

f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.expanduser('~/sim/logs/mpc2_*.csv')), key=os.path.getmtime)[-1]
d = np.genfromtxt(f, delimiter=',', names=True, dtype=None, encoding='utf-8')
t = d['t'] - d['t'][0]
P = np.c_[d['px'], d['py']]; W = np.c_[d['wx'], d['wy']]; K = np.c_[d['kx'], d['ky']]
dist = np.hypot(*(W - P).T); est = np.hypot(*(K - P).T); err = np.hypot(*(K - W).T)
print(os.path.basename(f))
print('   t   狀態   真實距離 估計距離 估計誤差   飛機(x,y)      人(x,y)      估計(x,y)    命令速度  傾斜')
nxt = 0.0
for i in range(len(t)):
    flag = dist[i] < 2.0 or d['status'][i] != 'track'
    if t[i] >= nxt or (flag and (i == 0 or t[i] - t[i - 1] > 0 and (dist[i - 1] >= 2.0) != (dist[i] >= 2.0))):
        print('%5.1f  %-5s  %6.2f  %6.2f  %6.2f   (%5.1f,%5.1f)  (%5.1f,%5.1f)  (%5.1f,%5.1f)  %4.2f  %4.1f°%s' % (
            t[i], d['status'][i], dist[i], est[i], err[i], P[i, 0], P[i, 1], W[i, 0], W[i, 1], K[i, 0], K[i, 1],
            np.hypot(d['ux'][i], d['uy'][i]), np.degrees(d['tilt'][i]), '   <-- 太近' if dist[i] < 2.0 else ''))
        nxt = t[i] + 2.0
close = dist < 2.0
print('\n太近 (<2 m) 的時段:')
i = 0
while i < len(t):
    if close[i]:
        j = i
        while j + 1 < len(t) and close[j + 1]: j += 1
        s = d['status'][i:j + 1]
        print('  %5.1f ~ %5.1f s (%.1f s)  最近 %.2f m  當時狀態 %s  估計距離 中位 %.2f m' % (
            t[i], t[j], t[j] - t[i], dist[i:j + 1].min(), ','.join(sorted(set(s))), np.nanmedian(est[i:j + 1])))
        i = j + 1
    else: i += 1
