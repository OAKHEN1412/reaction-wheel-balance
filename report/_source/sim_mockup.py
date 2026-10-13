"""Figure for the chapter 3 simulation section: a "simulator window" built from a real run.

    py -X utf8 sim_mockup.py <path to the repo's sim/ folder>

Every number on the figure comes from sim/ (params.py, design_gains.py, model.py) with the
design-stage assumptions used before the motor was measured: tau_m = 0.06 s, speed-command
mode. This matches chapter 4 section 4.6 ("the model assumed 0.06 s"). Writes figs/f_sim_mockup.png.
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import FancyBboxPatch, Circle, Rectangle

SIM = sys.argv[1]
sys.path.insert(0, SIM)
import params        # noqa: E402
import model         # noqa: E402
from design_gains import lqr_gains, evaluate_run, DEFAULT_Q   # noqa: E402

FD = os.path.join(os.environ['LOCALAPPDATA'], 'Microsoft', 'Windows', 'Fonts')
for f in ['THSarabunNew.ttf', 'THSarabunNew Bold.ttf']:
    fm.fontManager.addfont(os.path.join(FD, f))
plt.rcParams.update({'font.family': 'TH Sarabun New', 'font.size': 14, 'axes.unicode_minus': False,
                     'mathtext.fontset': 'dejavusans', 'mathtext.default': 'it', 'savefig.dpi': 220})

# ---------------------------------------------------------------- run the design + simulation
OVR = dict(tau_m=0.06, tau_m_est=0.06, control_mode='speed')
p = dict(params.NOMINAL); p.update(OVR)
d = params.derive(p)
g = lqr_gains(d, **DEFAULT_Q)
gains = dict(Kp=g['Kp'], Kd=g['Kd'], Kw=g['Kw'], Ki=g['Ki'])
eig = sorted(g['eigvals'], key=lambda z: z.real)

N, T = 20, 3.5
runs = [evaluate_run(model.simulate(d, gains, theta0=np.deg2rad(5.0), T=T, noisy=True, seed=s)) for s in range(N)]
n_ok = sum(r['success'] for r in runs)
n_sat = sum(r['saturated'] for r in runs)
settle = [r['settling_time'] for r in runs if r['success'] and not np.isnan(r['settling_time'])]
peak = [r['peak_wheel_rpm'] for r in runs]
hist = model.simulate(d, gains, theta0=np.deg2rad(5.0), T=2.0, noisy=True, seed=0)

w_max_rpm = d['omega_max'] * 60 / (2 * np.pi)
pole = np.sqrt(d['B'] / d['C'])
tau_g5 = d['B'] * np.sin(np.deg2rad(5.0))
print(f"gains {gains}, success {n_ok}/{N}, sat {n_sat}, settle {np.mean(settle):.2f}s, "
      f"peak mean {np.mean(peak):.0f} max {np.max(peak):.0f} rpm, pole {pole:.2f}, tau_g5 {tau_g5:.3f}")

# ---------------------------------------------------------------- draw the window
W, H = 6.3, 8.6
fig = plt.figure(figsize=(W, H))
fig.patch.set_facecolor('white')
DARK, BODY, PANEL, EDGE, ACC, MUTED = '#2b2d31', '#eef0f3', 'white', '#c9ced6', '#1f6feb', '#5f6670'


def box(x, y, w, h, fc, ec=EDGE, r=0.006, lw=0.8, z=1):
    fig.add_artist(FancyBboxPatch((x, y), w, h, boxstyle=f'round,pad=0,rounding_size={r}',
                                  transform=fig.transFigure, fc=fc, ec=ec, lw=lw, zorder=z))


def txt(x, y, s, size=14, color='black', weight='normal', ha='left', va='center', z=5):
    fig.text(x, y, s, fontsize=size, color=color, fontweight=weight, ha=ha, va=va, zorder=z)


box(0.005, 0.005, 0.99, 0.99, BODY, ec='#9aa1ab', r=0.012, lw=1.0)
box(0.005, 0.945, 0.99, 0.05, DARK, ec=DARK, r=0.012)
fig.add_artist(Rectangle((0.005, 0.945), 0.99, 0.02, transform=fig.transFigure, fc=DARK, ec=DARK, zorder=1))
for i, c in enumerate(['#ff5f57', '#febc2e', '#28c840']):
    fig.add_artist(Circle((0.03 + i * 0.025, 0.970), 0.0075, transform=fig.transFigure, fc=c, ec='none', zorder=3))
txt(0.5, 0.970, 'Reaction Wheel Simulator  —  ออกแบบและจำลองระบบควบคุมการทรงตัว', 15, 'white', 'bold', ha='center')

# toolbar
box(0.02, 0.900, 0.96, 0.036, 'white')
for i, (lab, active) in enumerate([('พารามิเตอร์', False), ('ออกแบบเกน LQR', False), ('จำลอง', True), ('Monte Carlo', False)]):
    x0 = 0.03 + i * 0.16
    if active:
        box(x0 - 0.005, 0.905, 0.15, 0.026, '#dbe8ff', ec=ACC)
    txt(x0 + 0.07, 0.918, lab, 13.5, ACC if active else MUTED, 'bold' if active else 'normal', ha='center')
box(0.80, 0.905, 0.17, 0.026, ACC, ec=ACC)
txt(0.885, 0.918, 'เริ่มจำลอง', 13.5, 'white', 'bold', ha='center')

# panel 1: inputs
px, py, pw, ph = 0.02, 0.585, 0.46, 0.305
box(px, py, pw, ph, PANEL)
txt(px + 0.015, py + ph - 0.022, '1) พารามิเตอร์ของระบบ', 15, 'black', 'bold')
rows = [
    ('รัศมีล้อ', r'$R_w$', f"{p['R_w']:.2f}", 'm'),
    ('มวลล้อ', r'$m_w$', f"{p['m_w']:.2f}", 'kg'),
    ('มวลโครงสร้าง', r'$m_b$', f"{p['m_b']:.2f}", 'kg'),
    ('ระยะจุดหมุนถึงจุดศูนย์กลางมวล', r'$l_b$', f"{p['l_b']:.2f}", 'm'),
    ('ระยะจุดหมุนถึงแกนล้อ', r'$l_w$', f"{p['l_w']:.2f}", 'm'),
    ('อัตราทดสายพาน', r'$N$', f"{p['gear_ratio']:.0f} : 1", ''),
    ('ความเร็วมอเตอร์ไม่มีโหลด', '', f"{p['motor_no_load_rpm']:.0f}", 'rpm'),
    ('แรงบิดหยุดนิ่งของมอเตอร์', '', f"{p['tau_stall_motor']:.2f}", r'N$\cdot$m'),
    ('ค่าคงที่เวลามอเตอร์ (สมมติ)', r'$\tau_m$', f"{p['tau_m']:.2f}", 's'),
    ('อัตราการควบคุม', '', f"{1 / d['dt_ctrl']:.0f}", 'Hz'),
    ('ตัวกรองคอมพลีเมนทารี', r'$\alpha$', f"{p['comp_alpha']:.2f}", ''),
]
for i, (lab, sym, val, unit) in enumerate(rows):
    y = py + ph - 0.052 - i * 0.0228
    if i % 2 == 0:
        fig.add_artist(Rectangle((px + 0.008, y - 0.0114), pw - 0.016, 0.0228, transform=fig.transFigure,
                                 fc='#f6f8fa', ec='none', zorder=2))
    txt(px + 0.015, y, lab, 13)
    txt(px + 0.29, y, sym, 8.5, MUTED, ha='center')
    box(px + 0.315, y - 0.009, 0.085, 0.018, 'white', ec='#b9c0ca', r=0.003, lw=0.6, z=3)
    txt(px + 0.395, y, val, 12.5, 'black', ha='right')
    txt(px + 0.405, y, unit, 11, MUTED)

# panel 2: computed values
qx, qy, qw, qh = 0.50, 0.585, 0.48, 0.305
box(qx, qy, qw, qh, PANEL)
txt(qx + 0.015, qy + qh - 0.022, '2) ค่าที่คำนวณได้', 15, 'black', 'bold')
calc = [
    ('โมเมนต์ความเฉื่อยของล้อ', r'$I_w = %.5f\ \mathrm{kg\,m^2}$' % d['I_w']),
    ('ความเฉื่อยรวมรอบจุดหมุน', r'$J = %.4f\ \mathrm{kg\,m^2}$' % d['C']),
    ('ค่าคงที่แรงโน้มถ่วง', r'$m g l = %.3f\ \mathrm{N\,m/rad}$' % d['B']),
    ('โพลไม่เสถียรของระบบเปิด', r'$\sqrt{mgl/J} = %.2f\ \mathrm{rad/s}$' % pole),
    ('แรงบิดแรงโน้มถ่วงที่ 5°', r'$%.3f\ \mathrm{N\,m}$' % tau_g5),
    ('แรงบิดสูงสุดที่ล้อ', r'$%.3f\ \mathrm{N\,m}$  (%.1f เท่า)' % (d['tau_stall_wheel'], d['tau_stall_wheel'] / tau_g5)),
    ('ความเร็วล้อสูงสุด', r'$%.0f\ \mathrm{rpm}$' % w_max_rpm),
]
for i, (lab, val) in enumerate(calc):
    y = qy + qh - 0.052 - i * 0.021
    txt(qx + 0.015, y, lab, 13)
    txt(qx + qw - 0.012, y, val, 8.5, ha='right')
gy = qy + qh - 0.052 - len(calc) * 0.021 - 0.010
box(qx + 0.012, qy + 0.012, qw - 0.024, gy - qy - 0.002, '#f3f7ff', ec='#b7cdf5')
txt(qx + 0.025, gy - 0.016, 'อัตราขยายจาก LQR', 13.5, ACC, 'bold')
for i, (k, v) in enumerate([('K_p', gains['Kp']), ('K_d', gains['Kd']), ('K_w', gains['Kw']), ('K_i', gains['Ki'])]):
    x = qx + 0.03 + (i % 2) * 0.22
    y = gy - 0.036 - (i // 2) * 0.021
    txt(x, y, r'$%s = %s$' % (k, f'{v:.4g}' if abs(v) < 10 else f'{v:.1f}'), 9.5)
ev = ', '.join(f'{z.real:.2f}' + (f'{z.imag:+.2f}j' if abs(z.imag) > 1e-3 else '') for z in eig)
txt(qx + 0.03, qy + 0.024, 'โพลวงปิด: ' + ev, 11.5, MUTED)

# panel 3: time response of one run
rx, ry, rw, rh = 0.02, 0.215, 0.96, 0.355
box(rx, ry, rw, rh, PANEL)
txt(rx + 0.015, ry + rh - 0.022, '3) ผลจำลองการกลับสู่สมดุลจากมุมเอียงเริ่มต้น 5° (มีสัญญาณรบกวนของเซนเซอร์และเวลาหน่วง)', 15, 'black', 'bold')
t = hist['t']
ax1 = fig.add_axes([0.10, 0.290, 0.37, 0.215]); ax1.set_zorder(10)
ax1.plot(t, np.rad2deg(hist['theta']), color=ACC, lw=1.4)
ax1.axhspan(-1, 1, color='tab:green', alpha=0.12)
ax1.axhline(0, color='0.5', lw=0.6)
ax1.set_xlabel('เวลา (s)', fontsize=13); ax1.set_ylabel('มุมเอียง (°)', fontsize=13)
ax1.set_xlim(0, t[-1]); ax1.grid(alpha=0.3); ax1.tick_params(labelsize=11)
ax2 = fig.add_axes([0.59, 0.290, 0.37, 0.215]); ax2.set_zorder(10)
ax2.plot(t, hist['omega_w'] * 60 / (2 * np.pi), color='#d9480f', lw=1.4)
ax2.axhline(0, color='0.5', lw=0.6)
ax2.set_xlabel('เวลา (s)', fontsize=13); ax2.set_ylabel('ความเร็วล้อ (rpm)', fontsize=13)
ax2.set_xlim(0, t[-1]); ax2.grid(alpha=0.3); ax2.tick_params(labelsize=11)

# panel 4: Monte Carlo summary tiles
sx, sy, sw, sh = 0.02, 0.055, 0.96, 0.145
box(sx, sy, sw, sh, PANEL)
txt(sx + 0.015, sy + sh - 0.020, f'4) สรุปการจำลอง {N} ครั้ง (สุ่มสัญญาณรบกวนต่างกัน)', 15, 'black', 'bold')
tiles = [
    (f'{n_ok}/{N}', f'กลับสู่สมดุลได้ ({100 * n_ok / N:.0f}%)'),
    (f'{np.mean(settle):.2f} s', 'เวลาเข้าสู่ ±1° เฉลี่ย'),
    (f'{np.max(peak):.0f} rpm', f'ความเร็วล้อสูงสุด (จาก {w_max_rpm:.0f})'),
    (f'{n_sat}/{N}', 'ครั้งที่ duty อิ่มตัว'),
]
for i, (big, small) in enumerate(tiles):
    x = sx + 0.015 + i * 0.235
    box(x, sy + 0.012, 0.22, 0.085, '#f6f8fa')
    txt(x + 0.11, sy + 0.068, big, 20, ACC, 'bold', ha='center')
    txt(x + 0.11, sy + 0.030, small, 12.5, MUTED, ha='center')

# status bar
txt(0.03, 0.030, f'สถานะ: จำลองเสร็จ {N} ครั้ง  ·  RK4 ขั้นละ 0.5 ms  ·  ควบคุม {1 / d["dt_ctrl"]:.0f} Hz  ·  '
    'ค่าคงที่เวลามอเตอร์เป็นค่าสมมติก่อนวัดจริง (หัวข้อ 4.6)', 11.5, MUTED)

os.makedirs('figs', exist_ok=True)
fig.savefig('figs/f_sim_mockup.png')
print('saved figs/f_sim_mockup.png')
