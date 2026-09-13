"""
sim/hardware_requirements.py
=============================
สวีปพารามิเตอร์ฮาร์ดแวร์ทีละตัว (ตัวอื่นคงที่ที่ nominal) รอบค่า gear_ratio, tau_stall_motor,
tau_m -- ออกแบบเกนใหม่ (LQR) เฉพาะกรณีนั้นๆ ทุกครั้ง (ไม่ใช้เกนคงที่ชุดเดียว) แล้วเทส nonlinear
sim ที่ 5° (ขอบเขตโครงงาน) N=20 seeds/กรณี เพื่อหา:

  - success rate ต่อกรณี
  - กฎง่ายๆ (rule of thumb): แรงบิดล้อขั้นต่ำที่ต้องการ (เทียบ B*sin(5deg)) และ tau_m สูงสุด
    ที่ยังยอมรับได้

บันทึกตารางเป็น sim/out/hardware_requirements.csv

รัน:  py -X utf8 sim/hardware_requirements.py
"""

import os
import sys
import csv
import numpy as np

import params
import model
import design_gains as dg

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

OUT_DIR = os.path.join(os.path.dirname(__file__), "out")
os.makedirs(OUT_DIR, exist_ok=True)

N_SEEDS = 20
T = 3.5
THETA0_DEG = 5.0


def run_case(param_name, value, n_seeds=N_SEEDS):
    """override param_name=value รอบ NOMINAL, ออกแบบเกนใหม่เฉพาะกรณีนี้, เทส N seeds ที่ 5 deg
    คืน dict สรุปผล 1 แถวตาราง"""
    p = dict(params.NOMINAL)
    p[param_name] = value
    dp = params.derive(p)

    torque_margin = dp["tau_stall_wheel"] / (dp["B"] * np.sin(np.deg2rad(THETA0_DEG)))

    try:
        gains = dg.lqr_gains_dict(dp, **dg.DEFAULT_Q)
        design_ok = True
    except Exception:
        gains = None
        design_ok = False

    if design_ok:
        results = []
        for seed in range(n_seeds):
            hist = model.simulate(dp, gains, theta0=np.deg2rad(THETA0_DEG), T=T, noisy=True,
                                   seed=seed)
            results.append(dg.evaluate_run(hist))
        n_success = sum(r["success"] for r in results)
        success_rate = n_success / n_seeds
        settle_times = [r["settling_time"] for r in results
                         if r["success"] and not np.isnan(r["settling_time"])]
        settle_mean = float(np.mean(settle_times)) if settle_times else float("nan")
        peak_rpm = float(np.max([r["peak_wheel_rpm"] for r in results]))
        sat_rate = sum(r["saturated"] for r in results) / n_seeds
    else:
        success_rate = 0.0
        settle_mean = float("nan")
        peak_rpm = float("nan")
        sat_rate = 1.0

    return dict(param=param_name, value=value, torque_margin=torque_margin,
                success_rate=success_rate, settle_mean=settle_mean, peak_rpm=peak_rpm,
                sat_rate=sat_rate, gear_ratio=dp["gear_ratio"], tau_stall_motor=dp["tau_stall_motor"],
                tau_m=dp["tau_m"], omega_max_rpm=dp["omega_max"] * 60 / (2 * np.pi))


def main():
    sweeps = {
        "gear_ratio": [1.0, 1.5, 2.0, 3.0, 4.0, 6.0],
        "tau_stall_motor": [0.04, 0.06, 0.08, 0.10, 0.12],
        "tau_m": [0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15],
    }

    all_rows = []
    print(f"=== Hardware requirements sweep (N={N_SEEDS} seeds/case, theta0={THETA0_DEG}deg, T={T}s) ===")
    for param_name, values in sweeps.items():
        print(f"\n--- sweep: {param_name} (others held at NOMINAL) ---")
        header = f"  {'value':>10s} | {'torque_margin':>13s} | {'success%':>9s} | " \
                 f"{'sat%':>6s} | {'settle(s)':>9s} | {'peak_rpm':>8s}"
        print(header)
        print("  " + "-" * (len(header) - 2))
        for v in values:
            row = run_case(param_name, v)
            all_rows.append(row)
            settle_str = f"{row['settle_mean']:.2f}" if not np.isnan(row["settle_mean"]) else "  -"
            print(f"  {v:>10.3f} | {row['torque_margin']:>13.2f} | "
                  f"{100*row['success_rate']:>8.1f}% | {100*row['sat_rate']:>5.1f}% | "
                  f"{settle_str:>9s} | {row['peak_rpm']:>8.0f}")

    # เขียน CSV
    csv_path = os.path.join(OUT_DIR, "hardware_requirements.csv")
    fieldnames = ["param", "value", "torque_margin", "success_rate", "sat_rate", "settle_mean",
                  "peak_rpm", "gear_ratio", "tau_stall_motor", "tau_m", "omega_max_rpm"]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in all_rows:
            writer.writerow(row)
    print(f"\nบันทึกตารางที่ {csv_path}")

    # --- กฎง่ายๆ (rule of thumb) ---
    print("\n=== กฎง่ายๆ (rules of thumb) จากผลสวีป ===")
    SUCCESS_THRESHOLD = 0.80  # ถือว่า "ใช้งานได้จริง" ถ้า success rate >= 80%

    gear_rows = [r for r in all_rows if r["param"] == "gear_ratio"]
    ok_gear = [r for r in gear_rows if r["success_rate"] >= SUCCESS_THRESHOLD]
    if ok_gear:
        min_gear = min(r["value"] for r in ok_gear)
        min_margin_gear = min(r["torque_margin"] for r in ok_gear)
        print(f"  gear_ratio: ต้องการ >= {min_gear:g} (torque margin >= {min_margin_gear:.2f}x) "
              f"เพื่อ success rate >= {SUCCESS_THRESHOLD*100:.0f}% ที่ 5 deg")
    else:
        print("  gear_ratio: ไม่มีค่าที่ทดสอบให้ success rate >= 80% -- ต้องการแรงบิดมากกว่าที่สวีปไว้")

    stall_rows = [r for r in all_rows if r["param"] == "tau_stall_motor"]
    ok_stall = [r for r in stall_rows if r["success_rate"] >= SUCCESS_THRESHOLD]
    if ok_stall:
        min_stall = min(r["value"] for r in ok_stall)
        print(f"  tau_stall_motor: ต้องการ >= {min_stall:g} N*m (ที่ gear_ratio nominal="
              f"{params.NOMINAL['gear_ratio']}) เพื่อ success rate >= {SUCCESS_THRESHOLD*100:.0f}%")
    else:
        print("  tau_stall_motor: ไม่มีค่าที่ทดสอบให้ success rate >= 80%")

    # torque margin โดยรวม (รวมทุก sweep ที่มี torque margin เกี่ยวข้อง)
    torque_related = gear_rows + stall_rows
    ok_torque = [r for r in torque_related if r["success_rate"] >= SUCCESS_THRESHOLD]
    fail_torque = [r for r in torque_related if r["success_rate"] < SUCCESS_THRESHOLD]
    if ok_torque and fail_torque:
        min_margin_ok = min(r["torque_margin"] for r in ok_torque)
        max_margin_fail = max(r["torque_margin"] for r in fail_torque)
        print(f"  โดยรวม: torque margin (tau_stall_wheel / (B*sin(5deg))) ที่ >= ~{min_margin_ok:.2f}x "
              f"มักจะสำเร็จ, ที่ <= ~{max_margin_fail:.2f}x มักจะล้มเหลว")
        print(f"  --> แนะนำออกแบบให้ torque margin >= 1.5x เป็นอย่างน้อยที่มุม 5 deg")

    tau_m_rows = [r for r in all_rows if r["param"] == "tau_m"]
    ok_tau_m = [r for r in tau_m_rows if r["success_rate"] >= SUCCESS_THRESHOLD]
    if ok_tau_m:
        max_tau_m = max(r["value"] for r in ok_tau_m)
        print(f"  tau_m (motor lag): ยอมรับได้สูงสุด ~{max_tau_m:g} s เพื่อ success rate >= "
              f"{SUCCESS_THRESHOLD*100:.0f}% (ที่ torque margin ระดับ nominal)")
    else:
        print("  tau_m: แม้แต่ค่าต่ำสุดที่สวีปก็ยังไม่ถึง 80% -- เช็ค torque margin ประกอบด้วย")


if __name__ == "__main__":
    main()
