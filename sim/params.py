"""
sim/params.py
=============
พารามิเตอร์ทางกายภาพของระบบ Reaction Wheel Balance (ยังไม่ทราบค่าจริง เพราะยังไม่ได้ชั่ง/วัด
ชิ้นงานจริง) — ไฟล์นี้เก็บ "ค่าประมาณตั้งต้น" (nominal) และ "ช่วงความเป็นไปได้" (range) ของแต่ละ
พารามิเตอร์ ใช้สำหรับจำลอง (sim/model.py), ออกแบบเกน (sim/design_gains.py) และทำ Monte Carlo
sweep เพื่อดูความทนทานของเกนต่อความไม่แน่นอนของพารามิเตอร์จริง

สัญลักษณ์ตรงกับ project-brief.md และ sim/README.md:
    theta   : มุมเอียงเฟรม (rad), 0 = ตั้งตรง (upright)
    theta_d : อัตราการเอียง (rad/s) จากไจโร
    omega_w : ความเร็วล้อ "สัมพัทธ์กับเฟรม" (rad/s) วัดจาก FG tach ของมอเตอร์/เกียร์
    I_b     : โมเมนต์ความเฉื่อยของ "ตัวเฟรม" รอบจุด CM ของมันเอง (ไม่รวม parallel-axis)
    I_w     : โมเมนต์ความเฉื่อยของล้อ (flywheel) รอบแกนหมุนของมันเอง
    m_b     : มวลเฟรม+มอเตอร์+แบต+อิเล็กทรอนิกส์ (ไม่รวมล้อ)
    m_w     : มวลล้อ (flywheel)
    l_b     : ระยะจากจุดหมุน (pivot) ถึง CM ของเฟรม (m)
    l_w     : ระยะจากจุดหมุนถึงแกนล้อ (m)
    gear_ratio : อัตราทดจากมอเตอร์ไปล้อ (motor_speed / wheel_speed)
    tau_m   : time constant ของลูปความเร็วมอเตอร์+ไดรเวอร์ (s)
    tau_stall_motor : ทอร์กสตอลล์โดยประมาณที่แกนมอเตอร์ (N·m)
    gear_eff: ประสิทธิภาพการส่งกำลังของเกียร์/สายพาน (0-1)
"""

import sys
import numpy as np

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

G = 9.81  # m/s^2

# ---------------------------------------------------------------------------
# ค่านิยาม (nominal) — ใช้เป็นจุดออกแบบเกนหลัก
# ---------------------------------------------------------------------------
NOMINAL = dict(
    # --- ล้อ (flywheel), เส้นผ่านศูนย์กลาง 20 cm -> รัศมี 0.10 m ---
    R_w=0.10,           # m, รัศมีล้อ (คงที่ตามสเปคโครงงาน 20 cm dia)
    m_w=0.35,           # kg, มวลล้อ (PLA, ประมาณ กลางช่วง 0.25-0.5 kg)
    k_w=0.7,            # shape factor: I_w = k_w * m_w * R_w^2
                        #   0.5 = จานกลมทึบสมบูรณ์, 1.0 = ห่วงบาง (rim-heavy)
                        #   0.7 = ล้อพิมพ์ 3D ที่มีเนื้อวัสดุกระจุกออกไปทางขอบ (มี spoke เว้นกลาง)

    # --- เฟรม + มอเตอร์ + แบต + บอร์ด ---
    m_b=0.7,            # kg, มวลรวมของทุกอย่างยกเว้นล้อ
    l_b=0.11,           # m, ความสูง pivot -> CM ของเฟรม
    k_b=0.25,           # shape factor: I_b(own) = k_b * m_b * l_b^2

    # --- ตำแหน่งแกนล้อ ---
    l_w=0.14,           # m, ความสูง pivot -> แกนล้อ

    # --- ระบบขับ (มอเตอร์ + เกียร์/สายพาน) ---
    # หมายเหตุ margin แรงบิด: แรงบิดโน้มถ่วงที่ทำให้ล้ม ~ B*sin(theta) = 1.236*sin(5deg)=0.108 N*m
    # ที่ 5 องศา และ 0.215 N*m ที่ 10 องศา (คำนวณจาก B nominal) ในขณะที่แรงบิดล้อสูงสุดที่ทำได้ =
    # tau_stall_motor*gear_ratio*gear_eff จึงต้อง "เผื่อ margin" ไม่ให้ tau_stall_wheel ชนกับ
    # แรงบิดโน้มถ่วง (ไม่งั้นล้อ "แพ้" แรงโน้มถ่วง ไล่ตามไม่ทันแม้เกนจะจูนดีแค่ไหนก็ตาม)
    # -> เลือก nominal gear_ratio และ tau_stall_motor ไปทาง "แรงบิดสูง" ของช่วงที่เป็นไปได้
    #    (แลกกับความเร็วสูงสุดที่ลดลง) เพื่อให้มี margin ใช้งานได้จริงที่จุดออกแบบหลัก
    gear_ratio=2.5,     # อัตราทด มอเตอร์:ล้อ (ช่วง 1-3 แต่เลือกสูงหน่อยเพื่อแรงบิด)
    motor_no_load_rpm=4000.0,   # rpm ที่แกนมอเตอร์ (ไม่มีโหลด, 12V)
    tau_stall_motor=0.09,       # N*m ที่แกนมอเตอร์ (BLDC-3640 เล็ก, ประมาณ 0.05-0.1, เลือกค่อนสูง)
    gear_eff=0.85,      # ประสิทธิภาพเกียร์/สายพาน

    tau_m=0.06,         # s, time constant ลูปความเร็วมอเตอร์+ไดรเวอร์ภายใน (ประมาณ, สวีป 0.02-0.15)

    # --- เซนเซอร์ ---
    comp_alpha=0.98,    # complementary filter alpha (ให้น้ำหนักไจโร)
    gyro_noise_std=0.02,     # rad/s, สัญญาณรบกวนไจโร (1-sigma)
    gyro_bias_std=0.01,      # rad/s, bias ไจโรที่สุ่มคงที่ต่อการทดลองหนึ่งครั้ง (drift ช้าๆ, ประมาณเป็นค่าคงที่)
    accel_noise_std=0.015,   # rad, สัญญาณรบกวนมุมที่ประมาณจาก accelerometer (แปลงเป็นมุมแล้ว)
    omega_w_noise_std=0.5,   # rad/s, สัญญาณรบกวนการวัดความเร็วล้อจาก FG tach
    omega_w_quant=1.0,       # rad/s, ขั้นควอนไทซ์ของ FG tach (ความละเอียดจำกัด)

    dt_ctrl=0.002,      # s, รอบลูปควบคุม (500 Hz ตามที่ระบุ ESP32-C3)
    ctrl_delay_samples=1,  # 1 sample delay (คำนวณ a จากค่าที่วัดได้ในรอบก่อนหน้า)
)

# ---------------------------------------------------------------------------
# ช่วงความเป็นไปได้ (สำหรับ Monte Carlo sweep) — (min, max)
# ---------------------------------------------------------------------------
RANGES = dict(
    m_w=(0.25, 0.5),
    k_w=(0.5, 0.9),
    m_b=(0.5, 0.9),
    l_b=(0.08, 0.15),
    k_b=(0.15, 0.4),
    l_w=(0.10, 0.18),
    gear_ratio=(1.0, 3.0),
    tau_stall_motor=(0.05, 0.10),
    gear_eff=(0.75, 0.95),
    tau_m=(0.02, 0.15),
    gyro_noise_std=(0.005, 0.03),
    gyro_bias_std=(0.0, 0.02),
    accel_noise_std=(0.005, 0.03),
    omega_w_noise_std=(0.1, 1.0),
)


def derive(p: dict) -> dict:
    """
    รับ dict พารามิเตอร์ดิบ (เช่น จาก NOMINAL หรือจากการสุ่มใน Monte Carlo) แล้วคำนวณ
    ปริมาณที่ต้องใช้ในสมการพลวัต (I_w, I_b, C, B, omega_max, alpha_max, ...)

    สมการ Lagrangian (อนุพันธ์ไว้ใน sim/README.md):
        (I_b + m_b*l_b^2 + m_w*l_w^2) * theta_dd = (m_b*l_b + m_w*l_w)*g*sin(theta) - tau_w
        I_w * (omega_w_dot + theta_dd) = tau_w

    รวมสองสมการ (เพื่อกำจัด tau_w แบบ implicit) จะได้:
        (C + I_w) * theta_dd + I_w * omega_w_dot = B * sin(theta)
    โดย
        C = I_b + m_b*l_b^2 + m_w*l_w^2   (ความเฉื่อยรวมของระบบรอบจุดหมุน "ไม่รวม" การหมุนของ
                                            ล้อรอบแกนตัวเอง)
        B = (m_b*l_b + m_w*l_w) * g       (สัมประสิทธิ์แรงบิดโน้มถ่วงที่ทำให้ล้ม, ไม่เสถียร)
    """
    out = dict(p)
    R_w = p["R_w"]
    I_w = p["k_w"] * p["m_w"] * R_w ** 2
    I_b = p["k_b"] * p["m_b"] * p["l_b"] ** 2
    C = I_b + p["m_b"] * p["l_b"] ** 2 + p["m_w"] * p["l_w"] ** 2
    B = (p["m_b"] * p["l_b"] + p["m_w"] * p["l_w"]) * G

    motor_no_load_rad_s = p["motor_no_load_rpm"] * 2 * np.pi / 60.0
    omega_max = motor_no_load_rad_s / p["gear_ratio"]  # rad/s, ที่ล้อ (สัมพัทธ์)

    tau_stall_wheel = p["tau_stall_motor"] * p["gear_ratio"] * p["gear_eff"]
    alpha_max = tau_stall_wheel / I_w  # rad/s^2, ความเร่งสัมพัทธ์สูงสุดของล้อที่มอเตอร์ทำได้

    out.update(
        I_w=I_w,
        I_b=I_b,
        C=C,
        B=B,
        omega_max=omega_max,
        tau_stall_wheel=tau_stall_wheel,
        alpha_max=alpha_max,
    )
    return out


def nominal_derived() -> dict:
    return derive(NOMINAL)


def sample_random(rng: np.random.Generator) -> dict:
    """สุ่มพารามิเตอร์ 1 ชุดแบบ uniform ภายในช่วง RANGES เต็มช่วง (min-max ทั้งหมด, สำหรับวัด
    'feasibility เมื่อรู้พารามิเตอร์จริงแล้วค่อยออกแบบเกนใหม่') ค่าที่ไม่อยู่ใน RANGES ใช้ NOMINAL"""
    p = dict(NOMINAL)
    for key, (lo, hi) in RANGES.items():
        p[key] = rng.uniform(lo, hi)
    return derive(p)


def perturb_nominal(rng: np.random.Generator, pct: float = 0.20, keys=None) -> dict:
    """สุ่มพารามิเตอร์ 1 ชุดแบบ uniform รอบค่า NOMINAL แค่ +-pct (ค่าเริ่มต้น +-20%) แทนที่จะ
    สุ่มเต็มช่วง RANGES -- ใช้จำลองกรณี 'รู้พารามิเตอร์คร่าวๆ แล้ว (ชั่ง/วัดมาบ้าง) แต่ยังมี
    ความคลาดเคลื่อนเล็กน้อยจากการวัด' เพื่อทดสอบความทนทาน (robustness) ของเกนชุดเดียวที่ตรึงไว้
    ต่อความคลาดเคลื่อนระดับนี้ (ต่างจาก sample_random ที่สุ่มเต็มช่วงความไม่รู้ทั้งหมด)"""
    p = dict(NOMINAL)
    for key in (keys or RANGES.keys()):
        base = NOMINAL[key]
        p[key] = base * (1.0 + rng.uniform(-pct, pct))
    return derive(p)


if __name__ == "__main__":
    d = nominal_derived()
    print("=== Nominal derived parameters ===")
    for k in ["I_w", "I_b", "C", "B", "omega_max", "tau_stall_wheel", "alpha_max"]:
        print(f"  {k:16s} = {d[k]:.6g}")
    print(f"  omega_max (rpm equiv @wheel) = {d['omega_max']*60/(2*np.pi):.1f} rpm")
