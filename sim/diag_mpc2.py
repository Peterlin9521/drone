#!/usr/bin/env python3
"""機身為什麼一直傾斜?用 mpc2 記錄檔看:傾斜跟我們的命令有沒有關係"""
import sys, os, glob
import numpy as np

f = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.expanduser('~/sim/logs/mpc2_*.csv')), key=os.path.getmtime)[-1]
d = np.genfromtxt(f, delimiter=',', names=True, dtype=None, encoding='utf-8')
t = d['t'] - d['t'][0]; st = d['status']; tilt = np.degrees(d['tilt'])
u = np.c_[d['ux'], d['uy']]; sp = np.hypot(*u.T)
dtt = np.diff(t); du = np.hypot(*np.diff(u, axis=0).T) / np.maximum(dtt, 1e-3)
print(os.path.basename(f), ' %d 筆  %.0f s' % (len(t), t[-1]))
print('\n[1] 機身傾斜 依狀態  (lost = 我們送的水平速度是 0,只慢慢轉頭)')
for s in ('track', 'lost', 'wait'):
    m = st == s
    if m.sum() > 5: print('  %-6s %4.0f%% 的時間  傾斜 中位 %5.1f°  90%% %5.1f°   命令速度 中位 %.2f m/s' % (s, 100 * m.mean(), np.median(tilt[m]), np.percentile(tilt[m], 90), np.median(sp[m])))
m = st == 'lost'
if m.sum() > 20:
    idx = np.where(m)[0]; start = np.r_[True, np.diff(idx) > 1]; t_start = np.maximum.accumulate(np.where(start, t[idx], -1e9))
    late = idx[t[idx] - t_start > 1.0]
    if len(late) > 5: print('  lost 超過 1 s(應該已經懸停)  傾斜 中位 %.1f°  90%% %.1f°   n=%d' % (np.median(tilt[late]), np.percentile(tilt[late], 90), len(late)))
print('\n[2] 命令變化  水平速度命令變化率 中位 %.2f  90%% %.2f m/s^2   偏航命令 中位 |%.2f| rad/s' % (
    np.median(du), np.percentile(du, 90), np.median(np.abs(d['yr']))))
ang = np.arctan2(u[:, 1], u[:, 0]); flips = np.sum((sp[1:] > 0.3) & (sp[:-1] > 0.3) & (np.cos(np.diff(ang)) < 0))
print('    命令方向在一步內反轉 %d 次 (%.2f 次/秒)' % (flips, flips / max(t[-1], 1)))
tt = np.arange(0, t[-1], 0.05); ti = np.interp(tt, t, tilt) - np.mean(tilt)
F = np.abs(np.fft.rfft(ti)); fr = np.fft.rfftfreq(len(ti), 0.05); k = np.argmax(F[1:]) + 1
print('\n[3] 傾斜最強的晃動頻率 %.2f Hz (週期 %.1f s)' % (fr[k], 1 / fr[k]))
