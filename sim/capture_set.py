"""Map handover states that the measured balance actuator can capture.

This is deliberately separate from the older design model.  The motor model
uses the measured sustained wheel acceleration (40 rad/s^2), measured
duty-dependent lag (0.16..0.22 s), the firmware voltage law, speed guard and
350 ms capture Kp fade.

Run from the repository root::

    py -X utf8 sim/capture_set.py

The CSV records the nominal decision and an uncertainty ensemble.  Wheel RPM
is signed *relative to the acceleration initially requested by the balance
controller*: positive means that the wheel is already moving in the direction
the controller first needs to push; negative means extra headroom in that
direction.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path


DEG = math.pi / 180.0
RPM = 2.0 * math.pi / 60.0


@dataclass(frozen=True)
class Plant:
    # Only the ratios B/(C+I_w) and I_w/(C+I_w) are identifiable from the
    # campaign telemetry.  Normalise C+I_w to one.  These central values come
    # from integrated regression over the first 1 s, including viscous pivot
    # damping: d(theta_dot)=g_eff*integral(sin(theta))-reaction*d(omega)
    #                    - damping*d(theta).
    # The fits are noisy and window-dependent; uncertainty_plants() retains
    # that spread instead of pretending the old mass estimates are measured.
    I_w: float = 0.022958
    C: float = 0.977042
    B: float = 23.627
    pivot_damping: float = 2.980  # theta acceleration term, 1/s
    alpha_limit: float = 40.0
    tau_low: float = 0.16
    tau_high: float = 0.22
    omega_fs: float = 583.3333333333 * RPM
    omega_guard: float = 0.85 * 583.3333333333 * RPM
    kp: float = 1137.4
    kd: float = 450.0
    kw: float = 3.0
    ki: float = 0.0387
    tau_est: float = 0.18
    blend_s: float = 0.350
    blend_kp0: float = 0.25


NOMINAL = Plant()


def motor_tau(duty: float, p: Plant) -> float:
    """Measured lag rises approximately linearly with absolute duty."""
    return p.tau_low + (p.tau_high - p.tau_low) * min(1.0, abs(duty))


def simulate(angle_deg: float, rate_dps: float, wheel_rel_rpm: float, side: int,
             p: Plant = NOMINAL, duration: float = 3.0, dt: float = 0.002) -> dict:
    """Simulate one firmware handover, with perfect initial state measurement.

    ``rate_dps`` is positive toward upright. ``wheel_rel_rpm`` uses the
    controller-push-relative convention documented in the module docstring.
    """
    theta = side * angle_deg * DEG
    theta_d = -side * rate_dps * DEG

    # At all campaign handovers Kd dominates the faded Kp, so the first needed
    # acceleration has sign -side. Convert the user-facing relative sign to
    # the firmware/global wheel sign.
    omega = -side * wheel_rel_rpm * RPM
    integral = 0.0
    max_abs_angle = abs(angle_deg)
    max_opposite = 0.0
    sat_steps = 0
    first_second_steps = 0
    guard_steps = 0
    above_12_s = 0.0
    failed = False
    fail_reason = ""

    steps = int(round(duration / dt))
    for k in range(steps):
        t = k * dt
        kp_scale = (p.blend_kp0 + (1.0 - p.blend_kp0) * t / p.blend_s
                    if t < p.blend_s else 1.0)
        ki = 0.0 if t < p.blend_s else p.ki
        a = p.kp * kp_scale * theta + p.kd * theta_d + p.kw * omega + ki * integral
        a = max(-4000.0, min(4000.0, a))
        if omega * a > 0.0 and abs(omega) >= p.omega_guard:
            a = 0.0
            guard_steps += 1

        target = omega + p.tau_est * a
        target = max(-p.omega_fs, min(p.omega_fs, target))
        duty = target / p.omega_fs
        if t < 1.0:
            first_second_steps += 1
            sat_steps += abs(duty) >= 0.999999

        alpha = (target - omega) / motor_tau(duty, p)
        alpha = max(-p.alpha_limit, min(p.alpha_limit, alpha))
        theta_dd = ((p.B * math.sin(theta) - p.I_w * alpha) / (p.C + p.I_w)
                    - p.pivot_damping * theta_d)

        # Semi-implicit Euler is stable here and matches the 500 Hz firmware
        # update rate.  Halving dt changes sampled map boundaries by <=1 cell.
        omega += alpha * dt
        theta_d += theta_dd * dt
        theta += theta_d * dt
        if not (t < p.blend_s):
            integral = max(-1.0, min(1.0, integral + theta * dt))

        angle_now = theta / DEG
        max_abs_angle = max(max_abs_angle, abs(angle_now))
        max_opposite = max(max_opposite, max(0.0, -side * angle_now))
        above_12_s = above_12_s + dt if abs(angle_now) > 12.0 else 0.0
        if abs(angle_now) > 20.0 or above_12_s >= 0.300:
            failed = True
            fail_reason = "20deg" if abs(angle_now) > 20.0 else "12deg_300ms"
            break

    # Match the campaign's operational definition: BALANCING held >=3 s.
    recoverable = not failed and steps * dt >= 3.0
    return {
        "recoverable": recoverable,
        "fail_reason": fail_reason,
        "sat_pct_1s": 100.0 * sat_steps / max(1, first_second_steps),
        "guard_pct": 100.0 * guard_steps / max(1, k + 1),
        "max_abs_angle_deg": max_abs_angle,
        "max_opposite_deg": max_opposite,
        "final_angle_deg": theta / DEG,
        "final_rate_dps": theta_d / DEG,
        "final_wheel_rpm": omega / RPM,
    }


# Exact eight rows printed by tuning/tools/jumpruns.py after gzip decompression.
# Kept here so validation is reproducible without importing or modifying tuning/.
CAMPAIGN = [
    # n, side, cap angle, global rate, global wheel rpm, observed held
    (1, -1, -6.322, 40.323, -47.66, False),
    (2, +1, +6.246, -47.753, +28.86, False),
    (3, -1, -6.493, 57.010, -93.64, True),
    (4, +1, +6.106, -36.853, +72.43, True),
    (5, +1, +7.621, -52.395, +39.13, False),
    (6, +1, +6.776, -40.685, +19.36, True),
    (7, +1, +5.708, -47.021, -31.35, False),
    (8, +1, +7.333, -44.547, -23.13, False),
]


def campaign_validation(p: Plant = NOMINAL) -> list[dict]:
    out = []
    for n, side, theta, rate_global, wheel_global, observed in CAMPAIGN:
        rate_toward = -side * rate_global
        wheel_rel = wheel_global / (-side)
        result = simulate(abs(theta), rate_toward, wheel_rel, side, p)
        out.append({
            "n": n, "side": "+" if side > 0 else "-", "angle_deg": abs(theta),
            "rate_toward_dps": rate_toward, "wheel_rel_rpm": wheel_rel,
            "observed": observed, "predicted": result["recoverable"], **result,
        })
    return out


def uncertainty_plants() -> list[Plant]:
    """Fits from different telemetry windows plus measured motor uncertainty.

    This is intentionally broad.  The fitted gravity/reaction ratios move a
    lot as failed runs hit the stop and the FG estimator diverges from true
    speed.  The spread is evidence that the boundary is poorly identified.
    """
    fits = [
        # g_eff, reaction ratio, damping.  Central fit is listed first.
        (23.627, 0.022958, 2.980),  # 1 s integrated fit with damping
        (72.60, 0.04520, 0.0),     # first 350 ms, no damping term
        (43.93, 0.03180, 0.0),     # first 600 ms
        (36.50, 0.03221, 0.0),     # first 800 ms
        (24.79, 0.02123, 0.0),     # first 1 s, no damping term
    ]
    plants = []
    for i, (g_eff, reaction, damping) in enumerate(fits):
        # Central plant uses measured nominal motor values.  The other fits
        # alternate across the stated acceleration/lag uncertainty envelope.
        alpha = (40.0, 35.0, 45.0, 35.0, 45.0)[i]
        tau_lo, tau_hi = ((0.16, 0.22), (0.22, 0.22), (0.16, 0.16),
                          (0.16, 0.22), (0.16, 0.22))[i]
        plants.append(replace(NOMINAL, I_w=reaction, C=1.0-reaction, B=g_eff,
                              pivot_damping=damping, alpha_limit=alpha,
                              tau_low=tau_lo, tau_high=tau_hi))
    return plants


def frange(start: float, stop: float, step: float):
    n = round((stop - start) / step)
    for i in range(n + 1):
        yield start + i * step


def write_grid(out_dir: Path) -> tuple[int, int]:
    out_dir.mkdir(parents=True, exist_ok=True)
    ensemble = uncertainty_plants()
    fields = [
        "side", "angle_deg", "rate_toward_dps", "wheel_relative_rpm",
        "recoverable_nominal", "ensemble_recovered", "ensemble_total", "classification",
        "sat_pct_1s", "speed_guard_pct", "max_abs_angle_deg", "max_opposite_deg",
        "final_angle_deg", "final_rate_dps", "final_wheel_rpm",
    ]
    total = recovered = 0
    with (out_dir / "capture_grid.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for side in (-1, +1):
            for angle in frange(2.0, 14.0, 0.5):
                for rate in frange(0.0, 130.0, 5.0):
                    for wheel in frange(-400.0, 400.0, 50.0):
                        nominal = simulate(angle, rate, wheel, side)
                        votes = sum(simulate(angle, rate, wheel, side, q)["recoverable"]
                                    for q in ensemble)
                        cls = "robust" if votes == len(ensemble) else (
                            "not_recoverable" if votes == 0 else "boundary")
                        total += 1
                        recovered += nominal["recoverable"]
                        w.writerow({
                            "side": "+" if side > 0 else "-", "angle_deg": angle,
                            "rate_toward_dps": rate, "wheel_relative_rpm": wheel,
                            "recoverable_nominal": int(nominal["recoverable"]),
                            "ensemble_recovered": votes, "ensemble_total": len(ensemble),
                            "classification": cls,
                            "sat_pct_1s": f'{nominal["sat_pct_1s"]:.1f}',
                            "speed_guard_pct": f'{nominal["guard_pct"]:.1f}',
                            "max_abs_angle_deg": f'{nominal["max_abs_angle_deg"]:.3f}',
                            "max_opposite_deg": f'{nominal["max_opposite_deg"]:.3f}',
                            "final_angle_deg": f'{nominal["final_angle_deg"]:.3f}',
                            "final_rate_dps": f'{nominal["final_rate_dps"]:.3f}',
                            "final_wheel_rpm": f'{nominal["final_wheel_rpm"]:.2f}',
                        })
    return recovered, total


def write_validation(out_dir: Path, rows: list[dict]) -> int:
    fields = ["n", "side", "angle_deg", "rate_toward_dps", "wheel_rel_rpm",
              "observed", "predicted", "sat_pct_1s", "max_abs_angle_deg",
              "max_opposite_deg", "fail_reason"]
    with (out_dir / "campaign_validation.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    return sum(r["observed"] == r["predicted"] for r in rows)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path(__file__).parent / "out" / "capture_set")
    ap.add_argument("--validate-only", action="store_true")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rows = campaign_validation()
    score = write_validation(args.out, rows)
    print(f"campaign validation: {score}/8")
    for r in rows:
        print(f"  {r['n']}: observed={'held' if r['observed'] else 'fell':4s} "
              f"predicted={'held' if r['predicted'] else 'fell':4s} "
              f"sat={r['sat_pct_1s']:.1f}% max={r['max_abs_angle_deg']:.1f}deg")
    if not args.validate_only:
        recovered, total = write_grid(args.out)
        print(f"grid: {recovered}/{total} nominally recoverable -> {args.out / 'capture_grid.csv'}")


if __name__ == "__main__":
    main()
