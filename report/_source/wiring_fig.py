import os
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from PIL import Image, ImageOps

FD = os.path.join(os.environ['LOCALAPPDATA'], 'Microsoft', 'Windows', 'Fonts')
for f in ['THSarabunNew.ttf', 'THSarabunNew Bold.ttf']:
    fm.fontManager.addfont(os.path.join(FD, f))
plt.rcParams.update({'font.family': 'TH Sarabun New', 'font.size': 15})

W, H = 1600, 860
fig = plt.figure(figsize=(W / 150, H / 150), dpi=100)
ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis('off')

RED, BLK, BLUE, WHITE, GRN, YEL = '#e53935', '#111111', '#1e63d6', '#f4f4f4', '#2e9e44', '#e8b400'
SDA_C, SCL_C = '#16a085', '#ef6c00'


def img(im, x, y, w):
    h = w * im.height / im.width
    ax.imshow(im, extent=(x, x + w, y + h, y), zorder=2)
    return x, y, w, h


def wire(pts, c, lw=3.0, z=3):
    xs, ys = zip(*pts)
    if c == WHITE:
        ax.plot(xs, ys, color='#555555', lw=lw + 1.8, solid_capstyle='round', solid_joinstyle='round', zorder=z)
    ax.plot(xs, ys, color=c, lw=lw, solid_capstyle='round', solid_joinstyle='round', zorder=z + 0.1)


def dot(x, y, c, r=7, z=6):
    ax.add_patch(plt.Circle((x, y), r, color=c, ec='#222222', lw=1.2, zorder=z))


def text(x, y, s, c='#222222', size=18, ha='left', va='center', bold=False, box=False, z=8):
    kw = dict(color=c, fontsize=size, ha=ha, va=va, zorder=z, fontweight='bold' if bold else 'normal')
    if box:
        kw['bbox'] = dict(boxstyle='round,pad=0.2', fc='white', ec='none', alpha=0.95)
    ax.text(x, y, s, **kw)


# ---------------- Arduino Nano (center)
nano = Image.open('ch3img/nano_crop.png').convert('RGB')
nx, ny, nw, nh = img(nano, 690, 120, 230)
sc = nw / nano.width


def npin(col, i):
    return (nx + (22 if col == 'L' else 243) * sc, ny + (60 + 36.6 * i) * sc)


LP = {'D2': 4, 'D6': 8, 'D7': 9, 'D9': 11}
RP = {'GND': 1, '5V': 3, 'A5': 6, 'A4': 7}
text(nx + nw / 2, ny + nh + 22, 'Arduino Nano (ATmega328P)', size=22, ha='center', bold=True)

# ---------------- BLDC motor (left) -- mirrored so the cable exits toward the Nano
motor = ImageOps.mirror(Image.open('ch3img/p06_x51.png').convert('RGB'))
mx, my, mw, mh = img(motor, 40, 300, 250)
text(15, my - 30, 'BLDC MOTOR (BLDC-3640)', size=21, ha='left', bold=True)
ex, ey = mx + mw * 0.95, my + mh * 0.20          # cable exit
fan_x = 360                                       # where the six wires separate
sig = [(YEL, 'เหลือง (FG) : ขา D2', 'D2'), (GRN, 'เขียว (BRAKE) : ขา D6', 'D6'),
       (WHITE, 'ขาว (DIR) : ขา D7', 'D7'), (BLUE, 'น้ำเงิน (PWM) : ขา D9', 'D9')]
for k, (c, name, pin) in enumerate(sig):
    px, py = npin('L', LP[pin])
    wire([(ex, ey), (fan_x - 30 + 8 * k, ey), (fan_x + 8 * k, py), (px, py)], c)
    dot(px, py, c)
    text(fan_x + 60, py + (17 if pin in ('D7', 'D9') else -17), name, size=16, box=True)
# motor supply wires down to the buses
BUS_R, BUS_B = 790, 818
wire([(ex, ey), (fan_x - 60, ey + 4), (fan_x - 60, BUS_R), (1238, BUS_R)], RED)
wire([(ex, ey), (fan_x - 80, ey + 8), (fan_x - 80, BUS_B), (1222, BUS_B)], BLK)
text(fan_x - 50, 640, 'แดง : +12 V', RED, size=16, box=True)
text(fan_x - 50, 690, 'ดำ : GND', BLK, size=16, box=True)
text(760, BUS_R - 14, 'บัส +12 V', RED, size=17, bold=True)
text(760, BUS_B + 20, 'กราวด์ร่วม (Common GND)', BLK, size=17, bold=True)

# ---------------- Step-down module (top right)
sd = Image.open('ch3img/p09_x65.png').convert('RGB')
sx, sy, sw, sh = img(sd, 1190, 60, 320)
text(sx + sw / 2, sy - 24, 'STEP DOWN (XL4015)', size=21, ha='center', bold=True)
in_p, in_n = (sx + sw * 0.955, sy + sh * 0.40), (sx + sw * 0.955, sy + sh * 0.50)
out_p, out_n = (sx + sw * 0.11, sy + sh * 0.63), (sx + sw * 0.11, sy + sh * 0.75)
for p, c in [(in_p, RED), (in_n, BLK), (out_p, RED), (out_n, BLK)]:
    dot(*p, c, r=6)
text(out_p[0] - 20, out_p[1] - 30, 'OUT +5 V', RED, size=17, ha='right', bold=True)

# ---------------- MPU-6050 (right middle)
mpu = Image.open('ch3img/p05_x48.png').convert('RGB')
qx, qy, qw, qh = img(mpu, 1250, 360, 210)
text(qx + qw / 2, qy + qh + 18, 'MPU-6050', size=21, ha='center', bold=True)
hx = 1150
MPU = {n: (hx, qy + 45 + k * 38) for k, n in enumerate(['VCC', 'GND', 'SCL', 'SDA'])}
for n, (x, y) in MPU.items():
    wire([(x, y), (qx + 30, y)], '#888888', lw=2)
    text(x + 14, y - 13, n, size=16, bold=True)

# ---------------- Battery (bottom right)
bat = Image.open('ch3img/p08_x56.png').convert('RGB')
bx, by, bw, bh = img(bat, 1290, 685, 270)
text(bx + bw / 2 + 20, by - 12, 'แบตเตอรี่ 12 V', size=20, ha='center', bold=True)
jack = (bx + bw * 0.055, by + bh * 0.88)
dot(*jack, '#777777', r=6)

# battery -> buses and step-down input
wire([(1238, BUS_R), (1238, jack[1] - 6), (jack[0] - 4, jack[1] - 6)], RED)
wire([(1222, BUS_B), (jack[0] + 4, BUS_B), (jack[0] + 4, jack[1] + 6)], BLK)
wire([(1238, BUS_R), (1238, 628), (1575, 628), (1575, in_p[1]), in_p], RED)
wire([(1222, BUS_B), (1222, 612), (1555, 612), (1555, in_n[1]), in_n], BLK)
text(1545, 345, 'IN +12 V', RED, size=18, ha='right', bold=True, box=True)

# ---------------- Step-down OUT -> Nano 5V / GND, Nano -> MPU
p5, pg = npin('R', RP['5V']), npin('R', RP['GND'])
pa4, pa5 = npin('R', RP['A4']), npin('R', RP['A5'])
wire([out_p, (1060, out_p[1]), (1060, p5[1]), p5], RED)
wire([out_n, (1045, out_n[1]), (1045, pg[1]), pg], BLK)
wire([p5, (1000, p5[1]), (1000, MPU['VCC'][1]), MPU['VCC']], RED, lw=2.5)
wire([pg, (985, pg[1]), (985, MPU['GND'][1]), MPU['GND']], BLK, lw=2.5)
wire([pa5, (970, pa5[1]), (970, MPU['SCL'][1]), MPU['SCL']], SCL_C, lw=2.5)
wire([pa4, (955, pa4[1]), (955, MPU['SDA'][1]), MPU['SDA']], SDA_C, lw=2.5)
for p, c in [(p5, RED), (pg, BLK), (pa4, SDA_C), (pa5, SCL_C)]:
    dot(*p, c)
for n, (x, y) in MPU.items():
    dot(x, y, {'VCC': RED, 'GND': BLK, 'SCL': SCL_C, 'SDA': SDA_C}[n], r=6)
text(pg[0] + 16, pg[1] - 16, 'GND', BLK, size=16, bold=True, box=True)
text(p5[0] + 16, p5[1] - 16, '5V', RED, size=16, bold=True, box=True)
text(955, 590, 'SCL : ขา A5', SCL_C, size=17, bold=True)
text(955, 616, 'SDA : ขา A4', SDA_C, size=17, bold=True)

fig.savefig('figs/f_wiring_nano.png', dpi=260)
print('ok')
