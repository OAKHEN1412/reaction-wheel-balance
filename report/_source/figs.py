import numpy as np, re, os
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from ana import load, episodes

FD = os.path.join(os.environ['LOCALAPPDATA'], 'Microsoft', 'Windows', 'Fonts')
for f in ['THSarabunNew.ttf', 'THSarabunNew Bold.ttf']:
    fm.fontManager.addfont(os.path.join(FD, f))
plt.rcParams.update({'font.family': 'TH Sarabun New', 'font.size': 17, 'axes.titlesize': 18, 'axes.labelsize': 17,
                     'legend.fontsize': 15, 'xtick.labelsize': 15, 'ytick.labelsize': 15, 'axes.unicode_minus': False,
                     'savefig.dpi': 220, 'mathtext.fontset': 'dejavusans', 'mathtext.default': 'it', 'axes.grid': True, 'grid.alpha': 0.35, 'lines.linewidth': 1.0})
W = 6.3  # inches ~16 cm
os.makedirs('figs', exist_ok=True)


def ep_arr(e):
    a = np.array([[r[0], r[2], r[3], r[4], r[5], r[6]] for r in e], dtype=float)
    a[:, 0] = (a[:, 0] - a[0, 0]) / 1000
    return a


def save(fig, name):
    fig.tight_layout()
    fig.savefig(f'figs/{name}.png')
    plt.close(fig)
    print('saved', name)


def band(ax):
    ax.axhspan(-5, 5, color='tab:green', alpha=0.08)
    ax.axhline(5, ls='--', color='tab:green', lw=0.8)
    ax.axhline(-5, ls='--', color='tab:green', lw=0.8)


# ---- step response
raw = open('logs/step_test.log', encoding='utf-8', errors='replace').read()
series = {}
cur = None
for l in raw.split('\n'):
    m = re.match(r'step: duty=(\d+)', l)
    if m:
        cur = int(m.group(1)); series[cur] = []; continue
    m = re.match(r'^(\d+),(-?[\d.]+)$', l.strip())
    if m and cur is not None:
        series[cur].append((int(m.group(1)), float(m.group(2))))
res = {int(m.group(1)): (float(m.group(2)), int(m.group(3))) for m in
       re.finditer(r'RESULT duty=(\d+) frac=[\d.]+ steady_wheel_rpm=([\d.]+) t63_ms=(\d+)', raw)}
print('step series', {k: len(v) for k, v in series.items()}, res)
fig, ax = plt.subplots(figsize=(W, 3.4))
for (d, v), ls in zip(sorted(series.items()), ['-', '--', '-.']):
    v = np.array(v)
    ax.plot(v[:, 0] / 1000, v[:, 1], ls, color='k', label=f'duty {d/1023:.2f}')
    if d in res:
        ss, t63 = res[d]
        ax.plot([t63 / 1000], [0.632 * ss], 'o', color='tab:red', ms=5)
ax.plot([], [], 'o', color='tab:red', ms=5, label='จุด 63.2% (t63)')
ax.set_xlabel('เวลา (s)'); ax.set_ylabel('ความเร็วล้อ (rpm)'); ax.legend(loc='center right'); ax.set_xlim(0, 2.9)
save(fig, 'f_step')

# ---- FG glitch (balance_attempt2 around 73.9 s)
rows = load('logs/balance_attempt2.log')
a = np.array([[r[0], r[2], r[3], r[4], r[5], r[6]] for r in rows if 72500 <= r[0] <= 75200], dtype=float)
a[:, 0] = (a[:, 0] - a[0, 0]) / 1000
fig, axs = plt.subplots(3, 1, figsize=(W, 5.6), sharex=True)
axs[0].plot(a[:, 0], a[:, 1], 'k'); axs[0].set_ylabel('มุมเอียง (°)')
axs[1].semilogy(a[:, 0], np.abs(a[:, 3]) + 1, 'k'); axs[1].set_ylabel('|ความเร็วล้อ|\nที่อ่านได้ (rpm)')
axs[1].axhline(585, ls='--', color='tab:red')
axs[1].text(0.02, 800, 'ความเร็วสูงสุดจริง ≈ 585 rpm', color='tab:red', fontsize=14)
axs[2].plot(a[:, 0], a[:, 5], 'k'); axs[2].set_ylabel('duty'); axs[2].set_xlabel('เวลา (s)'); axs[2].set_ylim(-0.05, 1.1)
save(fig, 'f_fg_glitch')


# ---- Mega Kd comparison
def longest(fn):
    eps = episodes([r for r in load(fn) if r[1] == 'BALANCING'])
    return max(eps, key=lambda e: e[-1][0] - e[0][0])


panels = [('Kd = 220', ep_arr(longest('logs/kd220.log')), 0)]
for k, lab in [('kd300', 'Kd = 300'), ('kd400', 'Kd = 400'), ('kd500', 'Kd = 500')]:
    panels.append((lab, ep_arr(longest(f'logs/{k}.log')), 20))
fig, axs = plt.subplots(4, 1, figsize=(W, 6.6), sharex=True)
for ax, (lab, a, skip) in zip(axs, panels):
    m = a[:, 0] >= skip
    t = a[m, 0] - a[m, 0][0]; th = a[m, 1]; sel = t <= 10
    ax.plot(t[sel], th[sel], 'k'); band(ax)
    ax.set_ylim(-20, 20); ax.set_ylabel('มุมเอียง (°)'); ax.set_title(lab, loc='left', fontsize=16, pad=2)
axs[-1].set_xlabel('เวลา (s)'); axs[-1].set_xlim(0, 10)
save(fig, 'f_kd_mega')

# ---- Nano Kd comparison
rows = [r for r in load('logs/nano_kd_sweep.log') if r[1] == 'BALANCING']


def seg(a0, b0):
    return ep_arr([r for r in rows if a0 <= r[0] < b0])


ep600 = [e for e in episodes(rows) if e[0][0] == 513212][0]
a600 = ep_arr(ep600)
pn = [('Kd = 400', seg(640000, 650000)), ('Kd = 450', seg(780000, 790000)), ('Kd = 500', seg(900000, 910000)),
      ('Kd = 600 (10 วินาทีสุดท้ายก่อนล้ม)', a600[a600[:, 0] >= a600[-1, 0] - 10])]
fig, axs = plt.subplots(4, 1, figsize=(W, 6.6), sharex=True)
for ax, (lab, a) in zip(axs, pn):
    t = a[:, 0] - a[0, 0]
    ax.plot(t, a[:, 1], 'k'); band(ax)
    ax.set_ylim(-20, 20); ax.set_ylabel('มุมเอียง (°)'); ax.set_title(lab, loc='left', fontsize=16, pad=2)
axs[-1].set_xlabel('เวลา (s)'); axs[-1].set_xlim(0, 10)
save(fig, 'f_kd_nano')

# ---- long run
T0, T1 = 552994, 1068716
a = seg(T0, T1); t = a[:, 0]
bounds = [(T0, '600'), (560852, '500'), (568866, '450'), (618802, '400'), (674767, '450'), (861815, '500'), (942030, '450')]
fig, axs = plt.subplots(3, 1, figsize=(W, 6.6), sharex=True)
axs[0].plot(t, a[:, 1], 'k', lw=0.4); axs[0].axhline(5, ls='--', color='tab:green', lw=0.9); axs[0].axhline(-5, ls='--', color='tab:green', lw=0.9)
axs[0].set_ylim(-6, 6); axs[0].set_ylabel('มุมเอียง (°)')
axs[0].text(4, 3.4, 'ขอบเขต ±5°', color='tab:green', fontsize=14)
axs[1].plot(t, a[:, 3], 'k', lw=0.4); axs[1].set_ylabel('ความเร็วล้อ (rpm)')
axs[2].plot(t, a[:, 5], 'k', lw=0.4); axs[2].set_ylabel('duty'); axs[2].set_ylim(0, 1); axs[2].set_xlabel('เวลา (s)')
for i, (b, k) in enumerate(bounds):
    xs = (b - T0) / 1000
    if i > 0:
        for ax in axs:
            ax.axvline(xs, color='0.5', lw=0.7, ls=':')
    xe = ((bounds[i + 1][0] if i + 1 < len(bounds) else T1) - T0) / 1000
    if xe - xs > 25:
        axs[2].text((xs + xe) / 2, 0.82, f'Kd {k}', ha='center', fontsize=14)
axs[2].set_xlim(0, t[-1])
save(fig, 'f_longrun')

# ---- zoom 10 s at Kd 450
a = seg(1000000, 1010000); t = a[:, 0]
fig, axs = plt.subplots(3, 1, figsize=(W, 5.8), sharex=True)
axs[0].plot(t, a[:, 1], 'k'); axs[0].set_ylabel('มุมเอียง (°)'); axs[0].set_ylim(-2.5, 2.5)
axs[1].plot(t, a[:, 3], 'k', label='ความเร็วล้อ (ประมาณค่า)')
axs[1].plot(t, a[:, 4], '--', color='tab:blue', label='คำสั่ง (ความเร็วเทียบเท่าแรงดัน)')
axs[1].set_ylabel('rpm'); axs[1].legend(loc='lower right', fontsize=13, ncol=2)
axs[2].plot(t, a[:, 5], 'k'); axs[2].set_ylabel('duty'); axs[2].set_xlabel('เวลา (s)'); axs[2].set_xlim(0, 10)
save(fig, 'f_zoom')

# ---- histogram
a = seg(T0, T1)
p95 = np.percentile(np.abs(a[:, 1]), 95)
fig, ax = plt.subplots(figsize=(W, 3.2))
ax.hist(a[:, 1], bins=np.arange(-2.5, 2.51, 0.1), color='0.6', edgecolor='k', lw=0.4)
for x in [-p95, p95]:
    ax.axvline(x, ls='--', color='tab:red', lw=0.9)
ax.text(0.97, 0.88, f'95% ของข้อมูลอยู่ในช่วง ±{p95:.2f}°', transform=ax.transAxes, ha='right', color='tab:red', fontsize=15)
ax.set_xlabel('มุมเอียง (°)'); ax.set_ylabel('จำนวนตัวอย่าง')
save(fig, 'f_hist')

# ---- hand-push test (section 4.5.2), 2026-10-13 session 2: pushes 3-6 of table 4.12
# Events come from tuning/tools/pushtest.py; angles are relative to the steady
# median angle of the session (-1.12 deg), matching the table.
PUSH_REF = -1.12
rows = [r for r in load('logs/nano_2026-10-13_push_test.log') if 762400 <= r[0] <= 778800]
a = np.array([[r[0], r[2], r[3], r[4], r[5], r[6]] for r in rows], dtype=float)
a[:, 0] = (a[:, 0] - a[0, 0]) / 1000
t0 = rows[0][0]
marks = [(763279, '3'), (772652, '4'), (775010, '5'), (776724, '6')]
fig, axs = plt.subplots(3, 1, figsize=(W, 6.0), sharex=True)
axs[0].plot(a[:, 0], a[:, 1] - PUSH_REF, 'k')
axs[0].axhspan(-1, 1, color='tab:green', alpha=0.12)
axs[0].set_ylabel('มุมเอียงเทียบ\nมุมสมดุล (°)'); axs[0].set_ylim(-20, 20)
axs[1].plot(a[:, 0], a[:, 3], 'k'); axs[1].set_ylabel('ความเร็วล้อ (rpm)')
axs[2].plot(a[:, 0], a[:, 5], 'k'); axs[2].set_ylabel('duty'); axs[2].set_ylim(-0.05, 1.1)
for ts, lab in marks:
    x = (ts - t0) / 1000
    for ax in axs:
        ax.axvline(x, color='tab:red', lw=0.8, ls=':')
    axs[0].text(x + 0.12, 14, f'ครั้งที่ {lab}', color='tab:red', fontsize=13)
axs[2].set_xlabel('เวลา (s)'); axs[2].set_xlim(0, a[-1, 0])
save(fig, 'f_push')
