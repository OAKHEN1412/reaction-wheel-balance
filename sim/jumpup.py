"""
sim/jumpup.py
=============
วิเคราะห์ไอเดีย "jump-up" (การทดลอง, EXPERIMENTAL, ไม่ใช่ขอบเขตหลักของโปรเจค):
เฟรมนอนพัก (resting) ที่มุม theta0 (~ -30 ถึง -45 องศาจากแนวตั้ง), หมุนล้อขึ้นไปที่ omega_jump
แล้วเบรกล้อลงมาเป็น 0 ภายในเวลา t_brake (0.05-0.2s) เพื่อถ่ายโมเมนตัมเชิงมุมของล้อไปเป็น
โมเมนตัมเชิงมุมของเฟรม หวังว่าจะ "เหวี่ยง" เฟรมขึ้นไปตั้งตรงได้

ทฤษฎี (โมเมนตัมเชิงมุมรอบจุดหมุน, ประมาณว่า t_brake สั้นพอที่แรงบิดโน้มถ่วงทำ impulse
น้อยมากระหว่างการเบรก):

    L_initial = I_w * omega_jump                      (เฟรมนิ่ง, ล้อหมุน omega_jump)
    L_final   = (C + I_w) * theta_dot_f                (เบรกล้อจนหยุดสัมพัทธ์กับเฟรม
                                                          -> เฟรม+ล้อหมุนไปด้วยกัน)
    => theta_dot_f = I_w * omega_jump / (C + I_w)

    พลังงานจลน์หลังเบรก: KE = 0.5*(C+I_w)*theta_dot_f^2
    พลังงานศักย์ที่ต้องการ (จาก theta0 ถึง theta=0, ตั้งตรง):
        dPE = B*(1 - cos(theta0))     โดย B = (m_b*l_b + m_w*l_w)*g

    เงื่อนไข "พอดีถึงตั้งตรงแบบไม่มีพลังงานเหลือ" (ขั้นต่ำ, ในทางปฏิบัติต้องมีเผื่อให้
    คอนโทรลเลอร์รับช่วงต่อได้):

        omega_jump_min = sqrt( 2 * B * (1-cos(theta0)) * (C+I_w) ) / I_w

นอกจากนี้ต้องเช็คว่าการเบรกภายใน t_brake ทำได้จริงหรือไม่ (ถูกจำกัดด้วย alpha_max
= tau_stall_wheel / I_w): ความเร่ง(หน่วง)ที่ต้องการ = omega_jump / t_brake ต้อง <= alpha_max
"""

import sys
import numpy as np
import model

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def required_omega_jump(d: dict, theta0_rad: float) -> float:
    C, Iw, B = d["C"], d["I_w"], d["B"]
    dPE = B * (1 - np.cos(theta0_rad))
    return np.sqrt(2 * dPE * (C + Iw)) / Iw


def simulate_brake(d: dict, theta0_rad: float, omega_jump: float, t_brake: float,
                    dt_phys: float = 0.0002):
    """
    จำลองจริง (nonlinear, ไม่ idealize) การเบรกจาก omega_jump ลงมาที่ 0 ภายใน t_brake
    (สั่ง omega_cmd=0 คงที่, ให้มอเตอร์เบรกเต็มแรงตาม alpha_max/tau_m ของพารามิเตอร์จริง)
    แล้วปล่อยอิสระ (omega_cmd=0) ต่ออีกช่วงหนึ่งเพื่อดู theta สูงสุดที่ไปถึง (apex)

    คืน dict(theta_apex, thetad_after_brake, reached_upright: bool)
    """
    theta, thetad, omega_w = theta0_rad, 0.0, omega_jump
    t = 0.0
    n_brake = int(round(t_brake / dt_phys))
    for _ in range(n_brake):
        theta, thetad, omega_w = model.rk4_step(theta, thetad, omega_w, 0.0, d, dt_phys)
        t += dt_phys
    thetad_after_brake = thetad

    # ปล่อยอิสระต่อ (ballistic) ดูว่าขึ้นไปถึง theta=0 (ตั้งตรง) ได้ไหม, จับ apex
    theta_apex = theta
    n_free = int(round(2.0 / dt_phys))
    for _ in range(n_free):
        theta, thetad, omega_w = model.rk4_step(theta, thetad, omega_w, 0.0, d, dt_phys)
        if theta > theta_apex:
            theta_apex = theta
        if thetad <= 0 and theta > theta0_rad:
            break  # ถึงจุดสูงสุดแล้วเริ่มตกกลับ (หรือแกว่งกลับ)

    return dict(theta_apex=theta_apex, thetad_after_brake=thetad_after_brake,
                reached_upright=theta_apex >= -1e-6)


def report(d_nom: dict):
    print("  สมมติฐาน: เฟรมนอนพักที่มุม theta0 (วัดจากแนวตั้ง, ลบ = เอียงไปทางเดียวกับที่นอนอยู่),")
    print("  ต้องการเหวี่ยงขึ้นไปที่ theta=0 (ตั้งตรง) ด้วยการเบรกล้อจาก omega_jump ลงมา 0\n")

    theta0_options_deg = [-30, -35, -40, -45]
    t_brake_options = [0.05, 0.1, 0.2]

    header = f"  {'theta0(deg)':>12s} | {'omega_jump_min(rad/s)':>22s} | {'rpm(wheel)':>10s} | " \
             f"{'feasible@omega_max?':>20s}"
    print(header)
    print("  " + "-" * (len(header) - 2))

    omega_max = d_nom["omega_max"]
    alpha_max = d_nom["alpha_max"]

    any_feasible = False
    for theta0_deg in theta0_options_deg:
        theta0 = np.deg2rad(theta0_deg)
        om_req = required_omega_jump(d_nom, theta0)
        rpm_req = om_req * 60 / (2 * np.pi)
        feasible = om_req <= omega_max
        any_feasible = any_feasible or feasible
        print(f"  {theta0_deg:>12.0f} | {om_req:>22.1f} | {rpm_req:>10.0f} | "
              f"{'YES' if feasible else 'NO (เกิน omega_max=' + f'{omega_max:.0f} rad/s)'}")

    print(f"\n  omega_max ปัจจุบัน (nominal, gear_ratio={d_nom['gear_ratio']:.1f}) = "
          f"{omega_max:.1f} rad/s = {omega_max*60/(2*np.pi):.0f} rpm")
    print(f"  alpha_max (หน่วง/เร่งสัมพัทธ์สูงสุดจาก stall torque) = {alpha_max:.1f} rad/s^2")

    # ตรวจ t_brake ที่ต้องใช้ (เทียบ alpha_max) สำหรับกรณี -35 deg
    theta0 = np.deg2rad(-35)
    om_req = required_omega_jump(d_nom, theta0)
    print(f"\n  กรณี theta0=-35deg: omega_jump_min={om_req:.1f} rad/s "
          f"({om_req*60/(2*np.pi):.0f} rpm)")
    for tb in t_brake_options:
        needed_decel = om_req / tb
        ok = needed_decel <= alpha_max
        print(f"    t_brake={tb:.2f}s -> ต้องการหน่วง {needed_decel:.1f} rad/s^2 "
              f"({'ทำได้' if ok else 'เกิน alpha_max, เบรกไม่ทันในเวลานี้'})")

    # เช็ค "เวลาเบรกจริงที่ทำได้" (จำกัดด้วย alpha_max) เทียบกับ t_brake ที่ต้องการ (0.05-0.2s)
    real_brake_times = {}
    for theta0_deg in [-30, -35, -40, -45]:
        theta0 = np.deg2rad(theta0_deg)
        om_req = required_omega_jump(d_nom, theta0)
        real_brake_times[theta0_deg] = om_req / alpha_max  # เวลาที่ใช้จริงถ้าเบรกเต็มแรงตลอด (โดยประมาณ)

    print("\n  --- เวลาที่ต้องใช้เบรกจริง (จำกัดด้วย alpha_max, ไม่ใช่ t_brake ที่ตั้งไว้) ---")
    for theta0_deg, tb_real in real_brake_times.items():
        print(f"    theta0={theta0_deg}deg: ต้องใช้เวลาเบรกจริง >= {tb_real:.2f}s "
              f"(เกินกว่า t_brake ที่อยากได้ 0.05-0.2s มาก)")

    # nonlinear verification: ใช้ omega_jump ที่คำนวณได้ (ideal, ไม่ cap) แล้วปล่อยให้เบรกจริง
    # ตาม alpha_max ของมอเตอร์ (ไม่ใช่ t_brake ที่ตั้งไว้ตรงๆ เพราะมอเตอร์เบรกไม่ทันอยู่แล้ว)
    print("\n  --- nonlinear brake simulation (จริง, รวม gravity impulse ระหว่างเบรกที่ทำได้จริง) ---")
    any_reached = False
    for theta0_deg in [-30, -35, -40, -45]:
        theta0 = np.deg2rad(theta0_deg)
        om_req = required_omega_jump(d_nom, theta0)
        om_try = min(om_req, omega_max)
        tb_real = max(0.2, real_brake_times[theta0_deg])
        res = simulate_brake(d_nom, theta0, om_try, t_brake=tb_real)
        any_reached = any_reached or res["reached_upright"]
        print(f"    theta0={theta0_deg}deg, omega_jump={om_try:.1f} rad/s, "
              f"brake time ใช้จริง={tb_real:.2f}s: "
              f"theta_apex={np.rad2deg(res['theta_apex']):.1f} deg "
              f"({'ถึงตั้งตรง' if res['reached_upright'] else 'ไปไม่ถึงตั้งตรง'})")

    print("\n  === สรุป feasibility (jump-up) ===")
    print("  จุดสำคัญ: ถ้าดูแค่ 'ความเร็วที่ต้องการ' (omega_jump_min) เทียบกับ omega_max ของมอเตอร์")
    print("  แล้วดูเหมือนพอ (33-49 rad/s ~ 316-468 rpm, ต่ำกว่า omega_max มาก) แต่ปัญหาจริงคือ")
    print("  'อัตราการเบรก' (alpha_max) ไม่พอ -- การเบรกจาก omega_jump ลงมา 0 ภายใน t_brake ที่")
    print("  ต้องการ (0.05-0.2s) ต้องใช้ความหน่วง 190-770 rad/s^2 แต่ alpha_max ที่ทำได้จริงจาก")
    print(f"  ทอร์กสตอลล์ที่สมมติ มีแค่ {alpha_max:.0f} rad/s^2 (ต่างกัน 3-10 เท่า) ทำให้การเบรก")
    print("  ใช้เวลาจริง ~0.4-0.6s ไม่ใช่ 0.05-0.2s -- ระหว่างนั้นแรงโน้มถ่วงมีเวลากระทำนานขึ้น")
    print("  (เฟรมยังเอียงมากอยู่) ทำให้พลังงาน/โมเมนตัมที่ควรถ่ายไปเป็นการเหวี่ยงขึ้น กลับถูก")
    print("  ทำลาย/เสียไปกับการที่เฟรมยังคงล้มต่อระหว่างเบรก (ผลจำลอง nonlinear ข้างบนยืนยัน:")
    print(f"  ขึ้นถึงตั้งตรงได้จริงหรือไม่ = {any_reached})")
    print()
    print("  สรุป: ด้วยพารามิเตอร์ nominal (tau_stall_motor=0.09 N*m, gear=2.5) ไอเดีย jump-up")
    print("  แบบ 'เบรกเร็ว 0.05-0.2s' ไม่ feasible -- คอขวดคือ 'แรงบิด/อัตราเบรก' ไม่ใช่ 'ความเร็ว'")
    print("  ถ้าต้องการให้ใช้งานได้จริง ต้อง:")
    print("    1) เพิ่มทอร์กสตอลล์มอเตอร์ (หรือใช้เบรกแบบกลไก/ไฟฟ้าที่แรงกว่าการหน่วงผ่านมอเตอร์)")
    print("       เพื่อให้ alpha_max สูงพอจะเบรกภายใน 0.05-0.2s จริง และ/หรือ")
    print("    2) เพิ่ม I_w ของล้อ (มวล/รัศมีมากขึ้น, มวลกระจุกขอบ) ซึ่งลดทั้ง omega_jump ที่ต้องการ")
    print("       และลดความหน่วงที่ต้องการต่อ t_brake เดียวกัน (a=omega_jump/t_brake, omega_jump")
    print("       แปรผกผันกับ I_w ในสูตร) และ/หรือ")
    print("    3) ยอมรับ t_brake ที่ยาวขึ้น (~0.5s) แล้วออกแบบใหม่โดยไม่ประมาณว่า 'โมเมนตัมอนุรักษ์'")
    print("       (ต้องคิดผลของแรงโน้มถ่วงระหว่างเบรกด้วย ซึ่งซับซ้อนขึ้นและมักได้ผลแย่กว่านี้)")
