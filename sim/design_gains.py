"""
sim/design_gains.py
====================
1) Linearize plant รอบจุดตั้งตรง (theta=0) แล้วเสริม state ตัวรวมอินทิกรัลของ theta (integral
   state) เพื่อออกแบบ LQR ที่ให้ผลลัพธ์ตรงกับฟอร์มไฟร์มแวร์ทันที: a = Kp*theta + Kd*theta_dot
   + Kw*omega_w + Ki*integral(theta)

2) แปลงเกน LQR (u = -K x) เป็น Kp,Kd,Kw,Ki ตาม sign convention ของไฟร์มแวร์

3) ยืนยันด้วย nonlinear sim (มี delay, noise, saturation, motor lag) สองแบบแยกกัน (อ่านหัวข้อ
   README "feasibility vs robustness" ก่อน — สำคัญมากที่จะไม่สับสนสองอย่างนี้):
     (a) FEASIBILITY: สุ่มพารามิเตอร์เต็มช่วง RANGES, ออกแบบเกนใหม่ "เฉพาะชุดนั้น" (เหมือนได้วัด
         ของจริงแล้วมา tune) แล้วเทส -- วัดว่า "ระบบพอจะทำงานได้ไหมถ้ารู้พารามิเตอร์จริง"
     (b) ROBUSTNESS: ใช้เกนชุดเดียว (nominal) คงที่ ทดสอบกับพารามิเตอร์ที่คลาดเคลื่อนจาก nominal
         แค่ +-20% (ไม่ใช่เต็มช่วง) -- วัดว่า "เกน nominal ชุดเดียวทนต่อความคลาดเคลื่อนการวัด
         ระดับปกติได้แค่ไหน"

4) `--set key=value ...` : ออกแบบ/ตรวจเกนสำหรับพารามิเตอร์ที่วัดจริงแล้ว พิมพ์ #define พร้อมใช้
   สำหรับ firmware (ดู `py -X utf8 design_gains.py --help`)

รัน:  py -X utf8 sim/design_gains.py                       (รายงานเต็ม)
      py -X utf8 sim/design_gains.py --set m_b=0.8 ... --quick   (CLI สำหรับพารามิเตอร์วัดจริง)
"""

import sys
import argparse
import numpy as np
from scipy.linalg import solve_continuous_are

import params
import model
import jumpup

# กัน UnicodeEncodeError บน Windows console (cp1252) เวลาพิมพ์ข้อความไทย โดยไม่ต้องพึ่ง
# `py -X utf8` เสมอไป (ถึงจะแนะนำให้ใช้ -X utf8 ใน README ก็ตาม แต่กันพลาดไว้ก่อน)
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


# Q,R เริ่มต้นสำหรับ "robust" gain set -- q_omega=1.0 (ไม่ใช่ 0.02 เหมือนรุ่นก่อนหน้า) ดูเหตุผล
# เต็มในหัวข้อ bias_drift_check() ข้างล่าง: q_omega เล็กทำให้ Kw เล็ก (~0.14) ซึ่งปล่อยให้ wheel
# speed ลอย (drift) ช้าๆ แบบไม่มีสิ้นสุดเมื่อมี sensor bias คงที่ (constant IMU offset) -- ค่า
# q_omega=1.0 (Kw~1.0) ลดอัตรา drift ลงกว่า 40 เท่า โดยแทบไม่กระทบการตอบสนองที่ 5 องศา
DEFAULT_Q = dict(q_theta=400.0, q_thetad=4.0, q_omega=1.0, q_int=40.0, r=1.0)


# ---------------------------------------------------------------------------
# 1) LQR design (augmented with integral-of-theta state)
# ---------------------------------------------------------------------------
def lqr_gains(d: dict, q_theta=400.0, q_thetad=4.0, q_omega=1.0, q_int=40.0, r=1.0,
              leak_eps=0.02):
    """
    State x = [theta, thetad, omega_w, integral(theta)], input u = a (idealized wheel accel,
    motor lag/limit ignored ที่ขั้นตอนออกแบบเชิงเส้นนี้ -- นำไปเทสกับ nonlinear sim ทีหลัง)

    หมายเหตุสำคัญ: ระบบ [theta, thetad, omega_w] ดิบมี "โหมดที่ควบคุมไม่ได้" (uncontrollable
    mode) ที่ eigenvalue = 0 พอดี ซึ่งตรงกับปริมาณอนุรักษ์ทางฟิสิกส์ L = (C+I_w)*thetad +
    I_w*omega_w (โมเมนตัมเชิงมุมรวมรอบจุดหมุน, dL/dt = B*sin(theta) ไม่ขึ้นกับ a โดยตรง
    เพราะแรงบิดจากมอเตอร์เป็นแรงภายในคู่ที่หักล้างกันในระบบรวม) ถ้า augment integral state
    z=integral(theta) แบบ "บริสุทธิ์" (zdot=theta) ตรงๆ จะไปชนโหมดนี้ ทำให้ solve_continuous_are
    หาคำตอบไม่ได้ (marginal mode ที่สังเกตได้ใน Q แต่ควบคุมไม่ได้ -> cost ไม่จำกัด)

    แก้ด้วยการทำ "leaky integrator": zdot = theta - leak_eps*z (leak_eps เล็กมาก) ทำให้โหมดนั้น
    เสถียรแบบอ่อนๆ (eigenvalue = -leak_eps แทนที่จะเป็น 0 พอดี) พอให้ ARE แก้ได้ และให้ Ki ที่
    เป็นผลจาก LQR pole placement จริงๆ (ไม่ใช่เดามือ) โดย leak_eps เล็กพอที่จะไม่กระทบ steady
    state accuracy อย่างมีนัยสำคัญในช่วงเวลาที่สนใจ (~สิบวินาที) -- แต่โปรดทราบว่าโหมดนี้ที่
    ควบคุมไม่ได้จริง (physical, ไม่ใช่แค่ leak_eps) หมายความว่า sensor bias คงที่จะทำให้ wheel
    speed drift ช้าๆ ไม่มีวันหยุดสนิท (ดู bias_drift_check()) ไม่ว่าจะจูน Kw เท่าไหร่ก็ตาม
    ทำได้แค่ "ลดอัตรา drift ให้ช้าจนไม่มีนัยสำคัญในทางปฏิบัติ" เท่านั้น

    คืน dict(Kp,Kd,Kw,Ki, K_raw, A, B, Q, R)
    """
    Aeff = d["C"] + d["I_w"]
    Blin = d["B"]
    Iw = d["I_w"]

    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [Blin / Aeff, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, -leak_eps],
    ])
    Bmat = np.array([[0.0], [-Iw / Aeff], [1.0], [0.0]])

    Q = np.diag([q_theta, q_thetad, q_omega, q_int])
    R = np.array([[r]])

    P = solve_continuous_are(A, Bmat, Q, R)
    K = np.linalg.solve(R, Bmat.T @ P)  # 1x4, u = -K x

    K = K.flatten()
    Kp, Kd, Kw, Ki = -K[0], -K[1], -K[2], -K[3]

    # เช็คเสถียรภาพลูปปิดเชิงเส้น (idealized, ไม่มี delay/lag)
    Acl = A - Bmat @ K.reshape(1, 4)
    eigvals = np.linalg.eigvals(Acl)
    stable = np.all(eigvals.real < 0)

    return dict(Kp=Kp, Kd=Kd, Kw=Kw, Ki=Ki, K_raw=K, A=A, B=Bmat, Q=Q, R=R,
                eigvals=eigvals, stable=stable)


def lqr_gains_dict(d: dict, **kwargs) -> dict:
    """เหมือน lqr_gains() แต่คืนแค่ {'Kp','Kd','Kw','Ki'} เอาไปใช้กับ model.simulate() ตรงๆ"""
    g = lqr_gains(d, **kwargs)
    return dict(Kp=g["Kp"], Kd=g["Kd"], Kw=g["Kw"], Ki=g["Ki"])


# ---------------------------------------------------------------------------
# 1b) ทางเลือก "soft" gain set ด้วย pole-placement ตรงๆ
# ---------------------------------------------------------------------------
# หมายเหตุสำคัญที่ค้นพบจากการลอง sweep q_theta/q_thetad/r ของ lqr_gains(): Kp,Kd ที่ได้จาก LQR
# แทบไม่ขยับเลย (ค้างอยู่ที่ ~1000-1030 / ~128-130) ไม่ว่าจะปรับ Q,R อย่างไร (ลอง r ตั้งแต่ 1
# ถึง 1e7!) เพราะช่องทางที่ input "a" มีผลต่อ theta_dd อ่อนมาก (สัมประสิทธิ์ -I_w/(C+I_w) ~ -0.14
# เท่านั้น) ทำให้ LQR ต้อง "ยัน" ค่า Kp,Kd ไว้ที่ระดับหนึ่งเสมอเพื่อยังคงเสถียรภาพ (เป็นข้อจำกัด
# ทางฟิสิกส์ ไม่ใช่พารามิเตอร์ design ที่ปรับได้อย่างอิสระ) -- ถ้าต้องการชุดเกนที่ "นิ่มกว่าจริง"
# (ความเร่ง/ความเร็วล้อ peak ต่ำกว่า, less aggressive) ต้องทำ pole-placement ตรงๆ แทน โดยลด
# target natural frequency wn ลงจาก sqrt(B/Aeff) (~7.88 rad/s, จุดที่ LQR ลงเอง)
#
# ในทางกลับกัน Kw ไวต่อ q_omega มาก (ดู DEFAULT_Q และ bias_drift_check()) -- ดังนั้น "Kp,Kd ถูก
# บังคับโดยฟิสิกส์" แต่ "Kw ถูกกำหนดโดย design choice (q_omega) อย่างแท้จริง" สองอย่างนี้ต่างกัน
def pole_placement_gains(d: dict, wn: float, zeta: float = 1.0, Kw: float = 1.0, Ki: float = 0.02):
    """
    วาง pole ของ subsystem (theta, thetad) ตรงๆ ผ่านสมการ:
        theta_dd = (B/Aeff)*theta - (I_w/Aeff)*a   (ประมาณ a = domega_w โดยตรง, ideal actuator)
    ให้ a = Kp*theta + Kd*thetad ทำให้ระบบปิดเป็น:
        theta_dd + (I_w/Aeff)*Kd*thetad + [(I_w/Aeff)*Kp - B/Aeff]*theta = 0
    เทียบกับฟอร์มมาตรฐาน s^2 + 2*zeta*wn*s + wn^2 = 0:
        Kd = 2*zeta*wn*Aeff / I_w
        Kp = (B + wn^2*Aeff) / I_w

    wn ต้อง > sqrt(B/Aeff) พอสมควรเพื่อให้เสถียรจริง (สำหรับ wn ต่ำเกินไป ระบบจะเสถียรจากทฤษฎี
    linearized แต่ margin แคบมากจนไม่รอด nonlinear/noise/delay จริง -- ทดสอบแล้วว่า wn~3.5 คือ
    จุดที่เริ่มไม่ผ่านแม้ noiseless -- ใช้ wn>=4 เป็นขั้นต่ำที่ทดสอบแล้วผ่าน)
    """
    Aeff = d["C"] + d["I_w"]
    Blin = d["B"]
    Iw = d["I_w"]
    Kd = 2 * zeta * wn * Aeff / Iw
    Kp = (Blin + wn ** 2 * Aeff) / Iw
    return dict(Kp=Kp, Kd=Kd, Kw=Kw, Ki=Ki)


# ---------------------------------------------------------------------------
# 2) Nonlinear verification metrics
# ---------------------------------------------------------------------------
def evaluate_run(hist, settle_theta=np.deg2rad(1.0), settle_thetad=0.05, settle_frac=0.2):
    n = len(hist["t"])
    tail = int(n * (1 - settle_frac))
    theta_tail = hist["theta"][tail:]
    thetad_tail = hist["thetad"][tail:]
    settled = np.all(np.abs(theta_tail) < settle_theta) and \
              np.all(np.abs(thetad_tail) < settle_thetad)
    diverged = np.max(np.abs(hist["theta"])) > np.deg2rad(60)

    settle_idx = None
    below = np.abs(hist["theta"]) < settle_theta
    for i in range(len(below)):
        if np.all(below[i:]):
            settle_idx = i
            break
    settling_time = hist["t"][settle_idx] if settle_idx is not None else np.nan

    peak_wheel_rpm = np.max(np.abs(hist["omega_w"])) * 60 / (2 * np.pi)
    any_saturated = np.any(hist["saturated"])

    return dict(success=(settled and not diverged), diverged=diverged,
                settling_time=settling_time, peak_wheel_rpm=peak_wheel_rpm,
                saturated=bool(any_saturated))


def summarize(results, label):
    n = len(results)
    n_success = sum(r["success"] for r in results)
    n_sat = sum(r["saturated"] for r in results)
    settle_times = [r["settling_time"] for r in results if r["success"] and not np.isnan(r["settling_time"])]
    peak_rpms = [r["peak_wheel_rpm"] for r in results]
    print(f"--- {label} (n={n}) ---")
    print(f"  success rate        : {n_success}/{n} = {100*n_success/n:.1f}%")
    print(f"  runs that saturated  : {n_sat}/{n} = {100*n_sat/n:.1f}%")
    if settle_times:
        print(f"  settling time (successful runs): mean={np.mean(settle_times):.2f}s "
              f"p90={np.percentile(settle_times,90):.2f}s max={np.max(settle_times):.2f}s")
    print(f"  peak wheel speed: mean={np.mean(peak_rpms):.0f} rpm  max={np.max(peak_rpms):.0f} rpm")
    return dict(success_rate=n_success / n, sat_rate=n_sat / n,
                settle_mean=np.mean(settle_times) if settle_times else np.nan,
                peak_rpm_max=np.max(peak_rpms))


def print_gain_table(name, g):
    print(f"\n=== {name} ===")
    print(f"  Kp = {g['Kp']:+.4f}   [ (rad/s^2) / rad ]")
    print(f"  Kd = {g['Kd']:+.4f}   [ (rad/s^2) / (rad/s) ]")
    print(f"  Kw = {g['Kw']:+.4f}   [ (rad/s^2) / (rad/s) ]")
    print(f"  Ki = {g['Ki']:+.4f}   [ (rad/s^2) / (rad*s) ]")
    if "eigvals" in g:
        print(f"  linear closed-loop eigenvalues (idealized, no delay): {g['eigvals']}")
        print(f"  linearly stable: {g['stable']}")


# ---------------------------------------------------------------------------
# 3a) FEASIBILITY Monte Carlo -- เกนถูก "ออกแบบใหม่เฉพาะแต่ละชุดพารามิเตอร์" (เหมือนรู้ค่าจริงแล้ว)
# ---------------------------------------------------------------------------
def feasibility_monte_carlo(n_samples=100, angles=(5.0, 10.0), T=3.5, seed=0, noisy=True,
                             q_kwargs=None):
    """
    สุ่มพารามิเตอร์เต็มช่วง params.RANGES แล้ว "ออกแบบเกนใหม่เฉพาะชุดนั้น" (dg.lqr_gains ของ
    พารามิเตอร์ที่สุ่มได้ ไม่ใช่เกน nominal คงที่) แล้วเทส -- นี่คือคำตอบของคำถาม "ถ้าเราวัด
    พารามิเตอร์จริงได้แล้ว retune เกนตามนั้น ระบบพอจะทรงตัวได้ไหม" (ต่างจาก robustness_monte_carlo
    ที่ใช้เกนคงที่ชุดเดียว)
    """
    q_kwargs = q_kwargs or DEFAULT_Q
    rng = np.random.default_rng(seed)
    results = {a: [] for a in angles}
    for i in range(n_samples):
        dp = params.sample_random(rng)
        try:
            gains = lqr_gains_dict(dp, **q_kwargs)
        except Exception:
            # ARE บางชุดพารามิเตอร์สุดขั้วอาจแก้ไม่ได้ (ไม่ stabilizable เชิงตัวเลข) -- นับเป็น fail
            for a in angles:
                results[a].append(dict(success=False, diverged=True, settling_time=np.nan,
                                        peak_wheel_rpm=np.nan, saturated=True))
            continue
        for a in angles:
            hist = model.simulate(dp, gains, theta0=np.deg2rad(a), T=T, noisy=noisy,
                                   seed=int(rng.integers(0, 1_000_000)))
            results[a].append(evaluate_run(hist))
    return results


# ---------------------------------------------------------------------------
# 3b) ROBUSTNESS Monte Carlo -- เกนชุดเดียวคงที่ (nominal), พารามิเตอร์คลาดเคลื่อน +-pct
# ---------------------------------------------------------------------------
def robustness_monte_carlo(gains, n_samples=100, angles=(5.0, 10.0), T=3.5, seed=1,
                            pct=0.20, noisy=True):
    """
    ใช้เกนชุดเดียว (คำนวณจาก nominal params) คงที่ตลอด ทดสอบกับพารามิเตอร์ที่สุ่มรอบ nominal
    แค่ +-pct (ค่าเริ่มต้น 20%, ไม่ใช่เต็มช่วง RANGES) -- จำลองกรณี "ชั่ง/วัดของจริงมาระดับหนึ่ง
    แล้ว (คลาดเคลื่อนได้บ้าง) แต่ยังไม่ retune เกนใหม่" คำตอบคือ "เกนชุดนี้ tolerant แค่ไหน"
    """
    rng = np.random.default_rng(seed)
    results = {a: [] for a in angles}
    for i in range(n_samples):
        dp = params.perturb_nominal(rng, pct=pct)
        for a in angles:
            hist = model.simulate(dp, gains, theta0=np.deg2rad(a), T=T, noisy=noisy,
                                   seed=int(rng.integers(0, 1_000_000)))
            results[a].append(evaluate_run(hist))
    return results


# ---------------------------------------------------------------------------
# 4) Bias-drift check: sensor offset คงที่ (ไม่ใช่ noise สุ่ม) ทำให้ wheel speed "ลอย" ไหม
# ---------------------------------------------------------------------------
def bias_drift_check(d, gains, bias_deg=0.5, T=120.0, verbose=True):
    """
    รันแบบ deterministic (noisy=False, ดู pure effect ของ bias อย่างเดียวไม่ปนกับ noise สุ่ม)
    theta0=0 (ตั้งตรงอยู่แล้ว), accel_bias คงที่ bias_deg องศา (คลาดเคลื่อนแบบ IMU/mounting ปกติ)
    วัดอัตรา drift ของ wheel speed ในช่วงท้าย (40% สุดท้าย, ให้ transient เริ่มต้นหายไปก่อน) แล้ว
    ประเมินเวลาที่จะถึง omega_max ถ้า drift ต่อไปด้วยอัตราคงที่นี้

    คืน dict(drift_rate_rpm_s, time_to_sat_s, theta_ss_deg, omega_w_end_rpm)
    """
    h = model.simulate(d, gains, theta0=0.0, T=T, noisy=False, seed=0,
                        accel_bias=np.deg2rad(bias_deg))
    n = len(h["t"])
    i1 = int(n * 0.6)
    i2 = n - 1
    dt = h["t"][i2] - h["t"][i1]
    domega = h["omega_w"][i2] - h["omega_w"][i1]
    drift_rate = domega / dt  # rad/s^2 -- ใช้เป็น "rad/s ต่อ s"
    drift_rate_rpm_s = drift_rate * 60 / (2 * np.pi)

    omega_max = d["omega_max"]
    omega_now = abs(h["omega_w"][i2])
    if abs(drift_rate) > 1e-9:
        time_to_sat_s = max(0.0, (omega_max - omega_now) / abs(drift_rate))
    else:
        time_to_sat_s = np.inf

    result = dict(drift_rate_rpm_s=drift_rate_rpm_s, time_to_sat_s=time_to_sat_s,
                  theta_ss_deg=np.rad2deg(h["theta"][-1]),
                  omega_w_end_rpm=h["omega_w"][-1] * 60 / (2 * np.pi))
    if verbose:
        print(f"  bias={bias_deg}deg, Kw={gains['Kw']:.4f}: theta_ss={result['theta_ss_deg']:+.5f}deg, "
              f"omega_w@{T:.0f}s={result['omega_w_end_rpm']:+.1f}rpm, "
              f"drift_rate={drift_rate_rpm_s:+.4f} rpm/s, "
              f"est. time-to-saturation={time_to_sat_s/3600:.1f} hr" if np.isfinite(time_to_sat_s)
              else f"  bias={bias_deg}deg: ไม่ drift เลย (rate=0)")
    return result


# ---------------------------------------------------------------------------
# 5) CLI: --set key=value ... สำหรับ retune ด้วยพารามิเตอร์วัดจริง
# ---------------------------------------------------------------------------
def parse_set_args(pairs):
    """['m_b=0.82','l_b=0.105',...] -> {'m_b':0.82,'l_b':0.105,...}"""
    out = {}
    for tok in pairs:
        if "=" not in tok:
            raise ValueError(f"--set รับ key=value เท่านั้น ได้ '{tok}'")
        k, v = tok.split("=", 1)
        k = k.strip()
        if k not in params.NOMINAL:
            raise ValueError(f"ไม่รู้จักพารามิเตอร์ '{k}' (ดูชื่อที่ใช้ได้ใน params.NOMINAL)")
        out[k] = v if k == "control_mode" else float(v)
    return out


def params_from_overrides(overrides: dict) -> dict:
    p = dict(params.NOMINAL)
    p.update(overrides)
    return params.derive(p)


def quick_report(overrides: dict, n_verify=20, T=3.5):
    """โหมด --quick: ออกแบบเกนสำหรับพารามิเตอร์ที่ระบุ, ตรวจสอบเร็วๆ (n_verify seeds ที่ 5deg),
    พิมพ์ #define พร้อมวางในเฟิร์มแวร์"""
    dp = params_from_overrides(overrides)
    print("=== พารามิเตอร์ที่ใช้ (nominal + overrides จาก --set) ===")
    for k, v in overrides.items():
        print(f"  {k} = {v}  (override)")
    print("--- ค่าที่คำนวณได้ ---")
    for k in ["I_w", "I_b", "C", "B", "omega_max", "alpha_max", "tau_stall_wheel"]:
        print(f"  {k:16s} = {dp[k]:.6g}")

    g = lqr_gains(dp, **DEFAULT_Q)
    print_gain_table("เกนที่ออกแบบสำหรับพารามิเตอร์นี้ (LQR)", g)
    gains = dict(Kp=g["Kp"], Kd=g["Kd"], Kw=g["Kw"], Ki=g["Ki"])

    print(f"\n--- ตรวจสอบเร็ว (N={n_verify}, 5 deg, nonlinear+noise+delay) ---")
    results = []
    for seed in range(n_verify):
        hist = model.simulate(dp, gains, theta0=np.deg2rad(5.0), T=T, noisy=True, seed=seed)
        results.append(evaluate_run(hist))
    summarize(results, f"quick verification @5deg N={n_verify}")

    bias_res = bias_drift_check(dp, gains, bias_deg=0.5, T=60.0, verbose=True)

    print("\n=== วางในเฟิร์มแวร์ (#define, หน่วย: theta[rad], theta_dot[rad/s], "
          "omega_w[rad/s ที่ล้อ สัมพัทธ์กับเฟรม], a output[rad/s^2 ของล้อ]) ===")
    print(f"  #define DEFAULT_KP   {g['Kp']:.4f}f")
    print(f"  #define DEFAULT_KD   {g['Kd']:.4f}f")
    print(f"  #define DEFAULT_KW   {g['Kw']:.4f}f")
    print(f"  #define DEFAULT_KI   {g['Ki']:.4f}f")
    print(f"  #define MOTOR_TAU_S  {dp['tau_m_est']:.4f}f   // time constant มอเตอร์ที่ใช้ตอนออกแบบ"
          f" (measure จริงจาก motor_test FG log ถ้าเป็นไปได้)")
    print(f"  #define CONTROL_MODE_VOLTAGE {str(dp['control_mode'] == 'voltage').lower()}")
    print(f"  #define GEAR_RATIO   {dp['gear_ratio']:.4f}f")
    print(f"  // deadband แนะนำ ~0.3 rad/s (จาก sim, กันคำสั่งเล็กเกินไปที่ ESC/มอเตอร์ไม่ขยับจริง)")
    print(f"  // omega_max (clamp) = {dp['omega_max']:.2f} rad/s = "
          f"{dp['omega_max']*60/(2*np.pi):.0f} rpm ที่ล้อ")


# ---------------------------------------------------------------------------
# 6) รายงานเต็ม (default, ไม่ใส่ --set)
# ---------------------------------------------------------------------------
def full_report():
    d_nom = params.nominal_derived()
    print("=== Nominal derived plant parameters ===")
    for k in ["I_w", "I_b", "C", "B", "omega_max", "alpha_max"]:
        print(f"  {k:10s} = {d_nom[k]:.6g}")

    robust = lqr_gains(d_nom, **DEFAULT_Q)
    print_gain_table("Robust gain set (LQR)", robust)
    gains_robust = dict(Kp=robust["Kp"], Kd=robust["Kd"], Kw=robust["Kw"], Ki=robust["Ki"])

    soft = pole_placement_gains(d_nom, wn=4.0, zeta=1.0, Kw=robust["Kw"], Ki=robust["Ki"] * 0.2)
    print_gain_table("Conservative (soft) gain set (pole-placement, wn=4 rad/s)", soft)

    # --- (a) FEASIBILITY: เกนออกแบบใหม่เฉพาะแต่ละชุดพารามิเตอร์ ---
    print("\n############ (a) FEASIBILITY Monte Carlo ############")
    print("# เกน 'ออกแบบใหม่เฉพาะแต่ละชุดพารามิเตอร์ที่สุ่มได้' (เหมือนวัดค่าจริงแล้ว retune)")
    print("# ตอบคำถาม: 'ถ้ารู้พารามิเตอร์จริงแล้ว จะทรงตัวได้ไหม' (ไม่ใช่ผลของเกนคงที่ชุดเดียว)")
    for angle in (5.0, 10.0):
        res = feasibility_monte_carlo(n_samples=100, angles=(angle,), T=3.5, seed=42, noisy=True)
        summarize(res[angle], f"FEASIBILITY, theta0={angle} deg, N=100 (tailored gains per sample)")

    # --- (b) ROBUSTNESS: เกน nominal คงที่, พารามิเตอร์คลาดเคลื่อน +-20% ---
    print("\n############ (b) ROBUSTNESS Monte Carlo (nominal gains, +-20% params) ############")
    print("# เกน ROBUST (nominal) ชุดเดียวคงที่, พารามิเตอร์คลาดเคลื่อนจาก nominal +-20% เท่านั้น")
    print("# ตอบคำถาม: 'เกนชุดนี้ทนความคลาดเคลื่อนจากการวัด/ประมาณค่าระดับปกติได้ไหม'")
    for angle in (5.0, 10.0):
        res = robustness_monte_carlo(gains_robust, n_samples=100, angles=(angle,), T=3.5, seed=7,
                                      pct=0.20, noisy=True)
        summarize(res[angle], f"ROBUSTNESS (+-20%), theta0={angle} deg, N=100")

    print("\n############ Sign convention sanity ############")
    a_pos_theta = robust["Kp"] * np.deg2rad(5.0)
    print(f"  theta=+5deg -> Kp*theta term = {a_pos_theta:+.4f} rad/s^2 "
          f"(ควรเป็นค่าเดียวกันเครื่องหมายกับ Kp เพราะ theta>0)")
    print(f"  Kp sign = {'+' if robust['Kp']>0 else '-'} "
          f"(บวก: เมื่อเอียง +, สั่งเร่งล้อไปทาง + สัมพัทธ์กับเฟรม -> ตามสมการ "
          f"theta_dd = (B*sin(theta) - I_w*omega_w_dot)/(C+I_w), omega_w_dot บวกมากขึ้นจะ "
          f"หัก theta_dd ให้ลดลง จึงต้านการล้มได้ -- ดูรายละเอียดเต็มใน README.md)")

    print("\n############ Kw verdict: LQR output จริง หรือค้างคงที่? ############")
    print("  ทดสอบ: Kw เปลี่ยนตาม q_omega อย่างมีนัยสำคัญ (ไม่ใช่ค่าคงที่ที่ LQR 'ดันคงที่' แบบ Kp,Kd)")
    for qo in (0.02, 0.2, 1.0, 5.0):
        g = lqr_gains(d_nom, q_theta=400.0, q_thetad=4.0, q_omega=qo, q_int=40.0, r=1.0)
        print(f"    q_omega={qo:6.2f} -> Kw={g['Kw']:.4f}  Ki={g['Ki']:.4f}  (Kp,Kd แทบไม่ขยับ: "
              f"Kp={g['Kp']:.1f}, Kd={g['Kd']:.2f})")
    print("  ดังนั้น Kw *เป็น* LQR output จริงและ design-able (ต่างจาก Kp,Kd ที่ถูกบังคับโดยฟิสิกส์)")
    print(f"  -- ใช้ q_omega={DEFAULT_Q['q_omega']} เป็นค่า default ใหม่ (จากเดิม 0.02) ดูเหตุผลด้านล่าง:")

    print("\n############ Bias-drift check: constant 0.5deg IMU offset, 120s run ############")
    print("  q_omega เดิม (0.02, Kw~0.14):")
    g_old = lqr_gains(d_nom, q_theta=400.0, q_thetad=4.0, q_omega=0.02, q_int=40.0, r=1.0)
    bias_drift_check(d_nom, dict(Kp=g_old["Kp"], Kd=g_old["Kd"], Kw=g_old["Kw"], Ki=g_old["Ki"]),
                      bias_deg=0.5, T=120.0)
    print("  q_omega ใหม่ (1.0, Kw~1.0, ค่า default ปัจจุบัน):")
    bias_drift_check(d_nom, gains_robust, bias_deg=0.5, T=120.0)
    print("  สรุป: โหมด 'โมเมนตัมเชิงมุมรวม' เป็น structurally uncontrollable (พิสูจน์ทาง")
    print("  คณิตศาสตร์ใน lqr_gains() docstring) -- ไม่มีค่า Kw ใดทำให้ wheel speed ไม่ drift เลย")
    print("  ถ้ามี sensor bias คงที่ (เป็นฟิสิกส์ ไม่ใช่บั๊ก) แต่ q_omega สูงขึ้นลดอัตรา drift ได้")
    print("  มาก (>40 เท่า) จนเวลาที่จะอิ่มตัว (saturate) ยาวเป็นหลักวัน ไม่ใช่ปัญหาจริงในการใช้งาน")

    print("\n############ Jump-up feasibility (experimental) ############")
    jumpup.report(d_nom)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def build_argparser():
    p = argparse.ArgumentParser(description="ออกแบบ/ตรวจสอบเกนควบคุมสำหรับ Reaction Wheel Balance")
    p.add_argument("--set", nargs="*", default=None, metavar="key=value",
                    help="override พารามิเตอร์จาก params.NOMINAL เช่น --set m_b=0.82 gear_ratio=3")
    p.add_argument("--quick", action="store_true",
                    help="โหมดเร็ว: ออกแบบเกน+ตรวจสอบสั้นๆ (N=20) แทนรายงานเต็ม (ใช้คู่กับ --set)")
    return p


if __name__ == "__main__":
    args = build_argparser().parse_args()
    if args.set is not None or args.quick:
        overrides = parse_set_args(args.set or [])
        quick_report(overrides, n_verify=20 if args.quick else 60)
    else:
        full_report()
