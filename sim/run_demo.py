"""
sim/run_demo.py
================
สร้างกราฟตัวอย่าง (PNG) เก็บไว้ที่ sim/out/ สำหรับใช้ในรายงานโปรเจค:

    1. recovery_5deg.png     : กู้คืนจากมุมเอียงเริ่มต้น 5 องศา (ขอบเขตโครงงาน)
    2. recovery_10deg.png    : กู้คืนจากมุมเอียงเริ่มต้น 10 องศา (ทดสอบเกินขอบเขต)
    3. disturbance_impulse.png : ตั้งตรงอยู่แล้ว โดนรบกวนด้วยแรงบิดกระแทก (impulse)
    4. gain_comparison.png   : เทียบ robust vs soft gain set ที่ 5 องศา

รัน:  py sim/run_demo.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import params
import model
import design_gains as dg

OUT_DIR = os.path.join(os.path.dirname(__file__), "out")
os.makedirs(OUT_DIR, exist_ok=True)


def plot_run(hist, title, fname, omega_max=None):
    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)

    axes[0].plot(hist["t"], np.rad2deg(hist["theta"]), label="theta (true)")
    axes[0].plot(hist["t"], np.rad2deg(hist["theta_hat"]), label="theta_hat (estimated)",
                 alpha=0.6, linewidth=0.8)
    axes[0].axhline(0, color="k", linewidth=0.5)
    axes[0].set_ylabel("tilt angle (deg)")
    axes[0].legend(loc="upper right")
    axes[0].set_title(title)
    axes[0].grid(alpha=0.3)

    rpm = hist["omega_w"] * 60 / (2 * np.pi)
    rpm_cmd = hist["omega_cmd"] * 60 / (2 * np.pi)
    axes[1].plot(hist["t"], rpm, label="wheel speed (actual)")
    axes[1].plot(hist["t"], rpm_cmd, label="wheel speed (commanded)", alpha=0.6, linewidth=0.8)
    if omega_max is not None:
        rpm_max = omega_max * 60 / (2 * np.pi)
        axes[1].axhline(rpm_max, color="r", linestyle="--", linewidth=0.7, label="omega_max")
        axes[1].axhline(-rpm_max, color="r", linestyle="--", linewidth=0.7)
    axes[1].set_ylabel("wheel speed (rpm)")
    axes[1].legend(loc="upper right")
    axes[1].grid(alpha=0.3)

    axes[2].plot(hist["t"], hist["a"], label="a (commanded wheel accel, rad/s^2)")
    sat_t = hist["t"][hist["saturated"]]
    if len(sat_t) > 0:
        axes[2].scatter(sat_t, np.zeros_like(sat_t), color="r", s=4, label="saturated", zorder=5)
    axes[2].set_ylabel("a (rad/s^2)")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right")
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    path = os.path.join(OUT_DIR, fname)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"  saved {path}")


def main():
    d = params.nominal_derived()
    robust = dg.lqr_gains(d, **dg.DEFAULT_Q)
    gains_robust = dict(Kp=robust["Kp"], Kd=robust["Kd"], Kw=robust["Kw"], Ki=robust["Ki"])
    gains_soft = dg.pole_placement_gains(d, wn=4.0, zeta=1.0, Kw=robust["Kw"], Ki=robust["Ki"] * 0.2)

    print("Generating demo plots into sim/out/ ...")

    # 1) 5 deg recovery (project scope)
    h5 = model.simulate(d, gains_robust, theta0=np.deg2rad(5.0), T=4.0, noisy=True, seed=1)
    plot_run(h5, "Recovery from 5 deg initial tilt (robust gains, noisy)",
              "recovery_5deg.png", omega_max=d["omega_max"])

    # 2) 10 deg recovery (beyond scope -- shows torque-margin limits honestly)
    h10 = model.simulate(d, gains_robust, theta0=np.deg2rad(10.0), T=4.0, noisy=True, seed=1)
    plot_run(h10, "Recovery attempt from 10 deg initial tilt (robust gains, noisy)",
              "recovery_10deg.png", omega_max=d["omega_max"])

    # 3) disturbance impulse while upright
    def disturbance(t):
        # แรงบิดกระแทกสั้นๆ (impulse) ที่ t=1.0s นาน 20ms (เช่น โดนสะกิด/ชนเบาๆ)
        # ขนาด 0.3 N*m x 20ms เทียบเท่า angular impulse ที่ทำให้เอียงพุ่งขึ้นไป ~1.5-2 deg
        # (ถ้าแรงมากกว่านี้มาก จะเกินงบแรงบิดของล้อที่มี ดู sim/README.md เรื่อง torque margin)
        if 1.0 <= t < 1.02:
            return 0.3  # N*m
        return 0.0

    h_dist = model.simulate(d, gains_robust, theta0=0.0, T=4.0, noisy=True, seed=2,
                             disturbance=disturbance)
    plot_run(h_dist, "Disturbance impulse rejection (upright, robust gains, noisy)",
              "disturbance_impulse.png", omega_max=d["omega_max"])

    # 4) robust vs soft comparison at 5 deg
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for label, g, seed in [("robust", gains_robust, 3), ("soft", gains_soft, 3)]:
        h = model.simulate(d, g, theta0=np.deg2rad(5.0), T=4.0, noisy=True, seed=seed)
        ax.plot(h["t"], np.rad2deg(h["theta"]), label=label)
    ax.axhline(0, color="k", linewidth=0.5)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("tilt angle (deg)")
    ax.set_title("Robust vs conservative(soft) gains -- 5 deg recovery")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, "gain_comparison.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"  saved {path}")

    print("Done.")


if __name__ == "__main__":
    main()
