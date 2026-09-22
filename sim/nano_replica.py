"""Firmware-faithful replica of the Nano build, identified from telemetry.

STATUS (2026-09-22): passes Gate 1 on amplitude/duty/wheel-bias/static-limit
but not on the 2.1 Hz mode; FAILS Gate 2 -- it cannot classify the known
launches (see out/nano_replica/README.md).  Do not use the grid outputs as a
handover map.  What is solid:

* Motor (tuning/logs/nano_jump_campaign.log.gz, 8 full-duty spin-ups):
  voltage-mode first order, tau 0.24 s, 593 rpm full scale, same-direction
  drive capped at ~200 rad/s^2 (rms 2-3 rpm per launch).  The "40 rad/s^2
  sustained" figure in the docs is what this model gives at 350-450 rpm:
  back-EMF, not a torque cap.  Plugging (opposite voltage) in the kick
  telemetry is uncapped first order, 460 rad/s^2 from 511 rpm -- but the
  catch data contradict such strong plugging (README).
* Plant: theta_dd = g_eff*sin(theta) - r*omega_dot, r = I_w/(C+I_w),
  g_eff = B/(C+I_w).  r ~0.036 from the kick response; g_eff <= 30 from
  steady-balance stability, >= 29 needed for the wind-up failures.
* Friction offset 0.035 duty (explains the -57 rpm wheel bias and 0.14 mean
  duty); accelerometer lever arm 0.08 m (explains the +4 deg angle jumps at
  handover); MPU-6050 DLPF delay.
* Controller, wheel-speed estimator, motor direction logic, capture blend,
  FALLEN rules and the jump controller are transcribed from
  firmware/reaction_wheel_balance/*.h.

Run from the repository root:

    py -X utf8 sim/nano_replica.py            # gates + margin grid + launch scan
    py -X utf8 sim/nano_replica.py --quick    # gates only
"""
from __future__ import annotations

import argparse
import csv
import math
import random
from dataclasses import dataclass, replace
from pathlib import Path

DEG = math.pi / 180.0
RPM = 2.0 * math.pi / 60.0
WFS = 583.3333 * RPM          # MOTOR_FULL_SCALE_WHEEL_RADPS (3500 rpm / 6)
GUARD = 0.85 * WFS            # MAX_WHEEL_SPEED_RADPS
DEADBAND = 0.3 / WFS          # MOTOR_DEADBAND_FRACTION
DT = 0.002                    # 500 Hz loop


@dataclass(frozen=True)
class Params:
    g_eff: float = 28.0        # B/(C+I_w), 1/s^2 (steady balance stable to 30, unstable at 31)
    r: float = 0.036           # I_w/(C+I_w)
    tau_m: float = 0.22        # plant motor time constant, s (spin-up 0.24, joint spin+kick 0.20)
    tau_in: float = 0.0        # driver input lag (PWM -> effective voltage), s
    alpha_max: float = 200.0   # same-direction drive cap, rad/s^2
    alpha_max_plug: float = 1e9  # opposite-voltage (plugging) cap, rad/s^2
    torque_scale: float = 1.0  # multiplies alpha (weak machine / supply droop)
    stop_deg: float = 17.0     # rest stop angle
    u_fric: float = 0.035      # duty-equivalent friction: omega_ss = (u - u_fric)*WFS
    h_imu: float = 0.08        # IMU lever arm, m: accel angle picks up -h*theta_dd
    delay_samples: int = 2     # MPU-6050 DLPF_CFG=3 group delay (~4.9 ms) in 2 ms samples
    gyro_noise_dps: float = 0.4
    accel_noise_deg: float = 0.6
    # controller
    kp: float = 1137.4
    kd: float = 450.0
    kw: float = 3.0
    ki: float = 0.0387
    tau_est: float = 0.18
    blend_ms: float = 350.0
    blend_kp0: float = 0.25
    # jump
    target_rate_dps: float = 75.0
    rate_band_dps: float = 15.0


NOMINAL = Params()


class WheelEstimator:
    """wheel_speed_estimator.h, tau = MOTOR_TAU_S (0.18)."""
    kCorrTau, kGateMin, kGateFrac, kResync = 0.03, 15.0, 0.35, 0.10

    def __init__(self, tau=0.18, omega=0.0):
        self.tau, self.est, self.dis = tau, omega, 0.0

    def update(self, omega_cmd, fg_mag, fg_fresh, dt):
        self.est += (omega_cmd - self.est) * (dt / self.tau)
        if fg_fresh:
            sign = 1.0 if self.est >= 0 else -1.0
            m = abs(self.est)
            err = fg_mag - m
            gate = max(self.kGateMin, self.kGateFrac * m)
            in_gate = abs(err) <= gate
            if in_gate or self.dis >= self.kResync:
                self.est += sign * err * min(1.0, dt / self.kCorrTau)
            self.dis = 0.0 if in_gate else self.dis + dt
        return self.est


class Controller:
    """balance_controller.h in voltage mode."""

    def __init__(self, p: Params):
        self.p, self.integral, self.omega_cmd = p, 0.0, 0.0

    def reset(self):
        self.integral, self.omega_cmd = 0.0, 0.0

    def update(self, theta, theta_dot, omega, dt, kp_scale=1.0, ki_on=True):
        p = self.p
        old = self.integral
        ki = p.ki if ki_on else 0.0
        if ki != 0.0:
            self.integral = max(-1.0, min(1.0, self.integral + theta * dt))
        a = p.kp * kp_scale * theta + p.kd * theta_dot + p.kw * omega + ki * self.integral
        a = max(-4000.0, min(4000.0, a))
        if omega * a > 0.0 and abs(omega) >= GUARD:
            a = 0.0
        target = omega + p.tau_est * a
        self.omega_cmd = max(-WFS, min(WFS, target))
        if target != self.omega_cmd and target * ki * theta > 0.0:
            self.integral = old
        return self.omega_cmd

    def duty(self):
        return max(-1.0, min(1.0, self.omega_cmd / WFS))


class MotorLogic:
    """motor_logic.h: DIR never changes while duty was nonzero last step."""

    def __init__(self):
        self.dir, self.applied = 0, 0.0

    def step(self, desired):
        desired = max(-1.0, min(1.0, desired))
        mag = abs(desired)
        if mag < DEADBAND:
            self.applied = 0.0
            return 0.0
        want = 1 if desired > 0 else -1
        if self.dir == 0:
            self.dir = want
        elif want != self.dir:
            if self.applied > 0.0:
                self.applied = 0.0
                return 0.0
            self.dir = want
        self.applied = mag
        return self.dir * mag

    def coast(self):
        self.applied = 0.0


def motor_alpha(u, omega, p: Params):
    """Wheel angular acceleration for signed duty u and wheel speed omega."""
    if abs(omega) < 0.5 and abs(u) <= p.u_fric:
        return 0.0
    s = (1.0 if omega > 0 else -1.0) if abs(omega) >= 0.5 else (1.0 if u > 0 else -1.0)
    a = ((u - p.u_fric * s) * WFS - omega) / p.tau_m
    cap = p.alpha_max if u * omega >= 0.0 else p.alpha_max_plug
    a = max(-cap, min(cap, a))
    return a * p.torque_scale


class Plant:
    def __init__(self, p: Params, theta, theta_dot, omega, u0=0.0):
        self.p, self.theta, self.theta_dot, self.omega = p, theta, theta_dot, omega
        self.theta_dd = 0.0
        self.u_eff = u0  # driver-side effective voltage after input lag

    def step(self, u, dt, sub=4):
        p = self.p
        h = dt / sub
        stop = p.stop_deg * DEG
        for _ in range(sub):
            if p.tau_in > 0.0:
                self.u_eff += (u - self.u_eff) * (h / p.tau_in)
            else:
                self.u_eff = u
            alpha = motor_alpha(self.u_eff, self.omega, p)
            self.theta_dd = p.g_eff * math.sin(self.theta) - p.r * alpha
            self.omega += alpha * h
            self.theta_dot += self.theta_dd * h
            self.theta += self.theta_dot * h
            if self.theta > stop:
                self.theta = stop
                if self.theta_dot > 0: self.theta_dot = 0.0
                if self.theta_dd > 0: self.theta_dd = 0.0   # pinned: no real acceleration
            elif self.theta < -stop:
                self.theta = -stop
                if self.theta_dot < 0: self.theta_dot = 0.0
                if self.theta_dd < 0: self.theta_dd = 0.0


class Imu:
    """MPU-6050 path: gyro (42 Hz DLPF) and accel tilt through the 0.98 CF."""

    def __init__(self, p: Params, rng, theta0):
        self.p, self.rng = p, rng
        self.cf = theta0
        self.gyro_f = 0.0
        self.acc_f = (math.sin(theta0) * 9.81, math.cos(theta0) * 9.81)
        self.fifo = []

    def read(self, plant: Plant, dt):
        p = self.p
        gyro = plant.theta_dot + self.rng.gauss(0, p.gyro_noise_dps * DEG)
        # accelerometer sees gravity plus the tangential acceleration of the IMU
        # (sign measured from the handover jumps in the campaign log)
        ay = 9.81 * math.sin(plant.theta) - p.h_imu * plant.theta_dd
        az = 9.81 * math.cos(plant.theta) - p.h_imu * plant.theta_dot ** 2
        self.fifo.append((gyro, ay, az))
        if len(self.fifo) > p.delay_samples + 1:
            self.fifo.pop(0)
        gyro, ay, az = self.fifo[0]
        self.gyro_f += (gyro - self.gyro_f) * (dt / 0.0038)  # ~42 Hz DLPF pole
        fy, fz = self.acc_f
        fy += (ay - fy) * (dt / 0.0036); fz += (az - fz) * (dt / 0.0036)
        self.acc_f = (fy, fz)
        mag = math.hypot(fy, fz) / 9.81
        acc_angle = math.atan2(fy, fz) + self.rng.gauss(0, p.accel_noise_deg * DEG)
        if abs(mag - 1.0) <= 0.25:
            self.cf = 0.98 * (self.cf + self.gyro_f * dt) + 0.02 * acc_angle
        else:
            self.cf += self.gyro_f * dt
        return self.cf, self.gyro_f


def dominant_period(x, lo=1.0, hi=15.0):
    """Period of the strongest spectral peak of x between lo and hi Hz."""
    if len(x) < 256:
        return 0.0
    import numpy as np
    xa = np.asarray(x) - sum(x) / len(x)
    X = np.abs(np.fft.rfft(xa)) ** 2
    f = np.fft.rfftfreq(len(xa), DT)
    Xs = np.convolve(X, np.ones(9) / 9, mode="same")
    m = (f > lo) & (f < hi)
    return 1.0 / f[m][int(np.argmax(Xs[m]))]


@dataclass
class RunResult:
    held: bool
    held_s: float
    fail_reason: str
    sat_pct_1s: float
    max_abs_deg: float
    overshoot_deg: float
    theta_sd_deg: float
    gyro_sd_dps: float
    duty_mean: float
    duty_max: float
    wheel_mean_rpm: float
    wheel_min_rpm: float
    wheel_max_rpm: float
    period_s: float
    trace: list


def run_balance(p: Params, theta0_deg, rate0_dps, wheel0_rpm, duration=3.0,
                capture=True, seed=0, trace=False, sat_hold_ms=300):
    """Run BALANCING from a handover state.  Rate/angle in firmware sign
    (positive = +theta side), wheel signed as the firmware's estimate."""
    rng = random.Random(seed)
    plant = Plant(p, theta0_deg * DEG, rate0_dps * DEG, wheel0_rpm * RPM, u0=0.0)
    imu = Imu(p, rng, plant.theta)
    est = WheelEstimator(0.18, plant.omega)
    ctl = Controller(p)
    ml = MotorLogic()
    if wheel0_rpm != 0.0:
        ml.dir = 1 if wheel0_rpm > 0 else -1
    u = 0.0
    steps = int(round(duration / DT))
    thetas, gyros, duties, wheels, tr = [], [], [], [], []
    sat = 0
    beyond = 0.0
    fail = ""
    side = 1 if theta0_deg > 0 else -1
    max_abs = abs(theta0_deg)
    over = 0.0
    k_end = steps
    for k in range(steps):
        t = k * DT
        theta_m, gyro_m = imu.read(plant, DT)
        omega_est = est.update(u * WFS, abs(plant.omega), True, DT)
        if capture and t < p.blend_ms / 1000.0:
            frac = t / (p.blend_ms / 1000.0)
            kp_scale = p.blend_kp0 + (1.0 - p.blend_kp0) * frac
            ki_on = False
        else:
            kp_scale, ki_on = 1.0, True
        ctl.update(theta_m, gyro_m, omega_est, DT, kp_scale, ki_on)
        u = ml.step(ctl.duty())
        plant.step(u, DT)
        deg = plant.theta / DEG
        max_abs = max(max_abs, abs(deg))
        over = max(over, -side * deg)
        if t < 1.0 and abs(u) >= 0.99:
            sat += 1
        thetas.append(theta_m); gyros.append(gyro_m); duties.append(abs(u)); wheels.append(plant.omega)
        if trace:
            tr.append((t, deg, plant.theta_dot / DEG, plant.omega / RPM, omega_est / RPM, u, theta_m / DEG))
        if abs(deg) > 20.0:
            fail = "20deg"; k_end = k; break
        beyond = beyond + DT if abs(deg) > 12.0 else 0.0
        if beyond >= sat_hold_ms / 1000.0:
            fail = "12deg_300ms"; k_end = k; break
    held_s = k_end * DT
    n1 = min(steps, int(1.0 / DT))
    # steady-state stats over the last 60% of a surviving run
    i0 = int(0.4 * len(thetas)) if not fail else 0
    th = thetas[i0:]; gy = gyros[i0:]
    def sd(x):
        if len(x) < 2: return 0.0
        m = sum(x) / len(x)
        return math.sqrt(sum((v - m) ** 2 for v in x) / len(x))
    period = dominant_period(gy)
    w = wheels[i0:] or [0.0]
    d = duties[i0:] or [0.0]
    return RunResult(
        held=(not fail) and held_s >= 3.0 - 1e-9, held_s=held_s, fail_reason=fail,
        sat_pct_1s=100.0 * sat / n1, max_abs_deg=max_abs, overshoot_deg=over,
        theta_sd_deg=sd(th) / DEG, gyro_sd_dps=sd(gy) / DEG,
        duty_mean=sum(d) / len(d), duty_max=max(d),
        wheel_mean_rpm=sum(w) / len(w) / RPM, wheel_min_rpm=min(w) / RPM, wheel_max_rpm=max(w) / RPM,
        period_s=period, trace=tr)


def spin_duty_for(rpm):
    d = rpm / 450.0
    return 0.6 if d < 0.6 else (1.0 if d > 1.0 else d)


def run_jump(p: Params, spin_rpm=550.0, kick_ms=450, capture_deg=8.0, rate_dps=75.0,
             rest_deg=-17.0, seed=0, duration_after=3.0, trace=False):
    """Full launch with jump_controller.h logic, then BALANCING with capture blend."""
    rng = random.Random(seed)
    p = replace(p, target_rate_dps=rate_dps)
    plant = Plant(p, rest_deg * DEG, 0.0, 0.0)
    imu = Imu(p, rng, plant.theta)
    est = WheelEstimator(0.18, 0.0)
    ctl = Controller(p)
    ml = MotorLogic()
    side = 1 if rest_deg > 0 else -1
    phase = "SPINUP"
    t = 0.0
    t_phase = 0.0
    u = 0.0
    spin_duty = spin_duty_for(spin_rpm)
    tr = []
    handover = None
    abort = ""
    max_steps = int(8.0 / DT)
    for k in range(max_steps):
        t = k * DT
        theta_m, gyro_m = imu.read(plant, DT)
        omega_est = est.update(u * WFS, abs(plant.omega), True, DT)
        th_deg, rate_dps_m, fg_rpm = theta_m / DEG, gyro_m / DEG, abs(plant.omega) / RPM
        if phase in ("SPINUP", "KICK", "COAST"):
            if abs(th_deg) > 25: abort = "25deg"; break
            if t >= 2.0: abort = "timeout2s"; break
            if phase == "SPINUP":
                if t - t_phase >= 1.5: abort = "spin_timeout"; break
                if abs(th_deg - rest_deg) > 4: abort = "moved"; break
                if abs(rate_dps_m) > 40: abort = "rate40"; break
                model_rpm = 583.0 * spin_duty * (1.0 - math.exp(-(t - t_phase) / 0.20))
                if fg_rpm >= spin_rpm or model_rpm >= spin_rpm:
                    phase, t_phase = "KICK", t
                    desired = side * 1.0
                else:
                    desired = -side * spin_duty
            if phase == "KICK":
                rising = -side * rate_dps_m
                if rising > p.target_rate_dps + p.rate_band_dps:
                    phase, desired = "COAST", 0.0
                elif abs(th_deg) < capture_deg and rising >= p.target_rate_dps - p.rate_band_dps:
                    phase = "CAPTURED"
                elif t - t_phase >= kick_ms / 1000.0:
                    abort = "kick_expired"; break
                else:
                    desired = side * 1.0
            if phase == "COAST":
                rising = -side * rate_dps_m
                if rising < p.target_rate_dps - p.rate_band_dps:
                    phase, t_phase, desired = "KICK", t, side * 1.0
                elif abs(th_deg) < capture_deg:
                    phase = "CAPTURED"
                else:
                    desired = 0.0
            if phase == "CAPTURED":
                ml.coast(); u = 0.0
                ctl.reset()
                handover = dict(t=t, theta_deg=plant.theta / DEG, theta_meas_deg=th_deg,
                                rate_dps=plant.theta_dot / DEG, wheel_rpm=plant.omega / RPM,
                                wheel_est_rpm=omega_est / RPM)
                t_cap = t
                phase = "BALANCING"
            else:
                u = ml.step(desired)
        else:  # BALANCING
            since = t - t_cap
            if since < p.blend_ms / 1000.0:
                kp_scale = p.blend_kp0 + (1.0 - p.blend_kp0) * since / (p.blend_ms / 1000.0)
                ki_on = False
            else:
                kp_scale, ki_on = 1.0, True
            ctl.update(theta_m, gyro_m, omega_est, DT, kp_scale, ki_on)
            u = ml.step(ctl.duty())
        plant.step(u, DT)
        if trace:
            tr.append((t, phase, plant.theta / DEG, plant.theta_dot / DEG, plant.omega / RPM, omega_est / RPM, u, th_deg))
        if phase == "BALANCING":
            deg = plant.theta / DEG
            if abs(deg) > 20.0: abort = "fell"; break
            if t - t_cap >= duration_after: break
    held_s = (t - t_cap) if handover else 0.0
    return dict(handover=handover, abort=abort, held=(handover is not None and abort == ""),
                held_s=held_s, trace=tr)


# ----------------------------------------------------------------------------
# datasets
# ----------------------------------------------------------------------------
# tuning/logs/nano_jump_campaign.log.gz via tuning/tools/jumpruns.py --csv
CAMPAIGN = [  # n, side, cap_deg, cap_dps, cap_rpm, sat_pct, held
    (1, -1, -6.322, 40.323, -47.66, 31, False),
    (2, +1, 6.246, -47.753, 28.86, 16, False),
    (3, -1, -6.493, 57.010, -93.64, 0, True),
    (4, +1, 6.106, -36.853, 72.43, 0, True),
    (5, +1, 7.621, -52.395, 39.13, 22, False),
    (6, +1, 6.776, -40.685, 19.36, 0, True),
    (7, +1, 5.708, -47.021, -31.35, 26, False),
    (8, +1, 7.333, -44.547, -23.13, 27, False),
]


def load_today(path: Path):
    rows = []
    with path.open(encoding="utf-8") as f:
        for rec in csv.DictReader(f):
            if not rec.get("cap_deg"):
                continue
            rows.append(dict(n=int(rec["n"]), rest=float(rec["rest_deg"]), cap_deg=float(rec["cap_deg"]),
                             cap_dps=float(rec["cap_dps"]), cap_rpm=float(rec["cap_rpm"]),
                             sat=float(rec["sat_pct"]), peak=float(rec["peak_dps"]),
                             held=(rec["result"] == "held")))
    return rows


def confusion(rows_pred_obs):
    tp = sum(1 for pr, ob in rows_pred_obs if pr and ob)
    fn = sum(1 for pr, ob in rows_pred_obs if (not pr) and ob)
    fp = sum(1 for pr, ob in rows_pred_obs if pr and (not ob))
    tn = sum(1 for pr, ob in rows_pred_obs if (not pr) and (not ob))
    return tp, fn, fp, tn


def gate1(p: Params, out: Path):
    lines = []
    r = run_balance(p, 2.0, 0.0, 0.0, duration=20.0, capture=False, seed=1)
    lines.append(f"Gate 1: from 2 deg, wheel stopped, 20 s: held={r.held} fail={r.fail_reason!r} "
                 f"theta_meas SD={r.theta_sd_deg:.2f} deg gyro SD={r.gyro_sd_dps:.2f} dps period={r.period_s:.2f} s "
                 f"duty mean={r.duty_mean:.3f} max={r.duty_max:.2f} sat1s={r.sat_pct_1s:.1f}% "
                 f"wheel mean={r.wheel_mean_rpm:.0f} [{r.wheel_min_rpm:.0f},{r.wheel_max_rpm:.0f}] rpm max|theta|={r.max_abs_deg:.1f}")
    lines.append("Kd sweep (real: 400 -> 1.73 Hz SD 0.57, 450 -> 2.1 Hz SD 0.43, 500 -> 2.22 Hz SD 0.35, 600 -> unstable/saturated):")
    for kd in (300, 400, 450, 500, 600, 700):
        rr = run_balance(replace(p, kd=kd), 1.0, 0.0, -50.0, duration=30.0, capture=False, seed=2)
        f = (1.0 / rr.period_s) if rr.period_s > 0 else 0.0
        lines.append(f"  Kd {kd}: held={rr.held} SD={rr.theta_sd_deg:.2f} deg gyroSD={rr.gyro_sd_dps:.2f} dps "
                     f"fast mode {f:.2f} Hz duty mean={rr.duty_mean:.3f} max={rr.duty_max:.2f} wheel [{rr.wheel_min_rpm:.0f},{rr.wheel_max_rpm:.0f}]")
    lines.append("Recovery limit, wheel stopped, zero rate (real: ~8 deg):")
    lim = None
    for a in range(2, 16):
        rr = run_balance(p, float(a), 0.0, 0.0, duration=3.0, capture=False, seed=3)
        if not rr.held:
            lim = a; break
    lines.append(f"  largest recovered static angle: {(lim - 1) if lim else '>=15'} deg")
    txt = "\n".join(lines)
    print(txt)
    (out / "gate1.txt").write_text(txt + "\n", encoding="utf-8")
    return r


def gate2(p: Params, out: Path, today_path: Path, label="nominal"):
    lines = [f"=== Gate 2 ({label}) ==="]
    preds = []
    rows_out = []
    for n, side, cd, cr, cw, sat, held in CAMPAIGN:
        rr = run_balance(p, cd, cr, cw, duration=3.0, capture=True, seed=n)
        preds.append((rr.held, held))
        rows_out.append(dict(dataset="campaign", n=n, side="+" if side > 0 else "-", cap_deg=cd, cap_dps=cr, cap_rpm=cw,
                             obs_sat=sat, obs_held=held, pred_held=rr.held, pred_sat=round(rr.sat_pct_1s, 1),
                             pred_fail=rr.fail_reason, pred_held_s=round(rr.held_s, 2), pred_max_abs=round(rr.max_abs_deg, 1)))
        lines.append(f"  campaign {n} {'+' if side>0 else '-'} {cd:6.2f} deg {cr:6.1f} dps {cw:7.1f} rpm | obs held={held!s:5} sat={sat:3d}% | pred held={rr.held!s:5} sat={rr.sat_pct_1s:5.1f}% {rr.fail_reason}")
    tp, fn, fp, tn = confusion(preds)
    lines.append(f"  campaign confusion: successes caught {tp}/{tp+fn}, failures caught {tn}/{tn+fp}  (FP {fp}, FN {fn})  score {tp+tn}/8")
    today = load_today(today_path)
    preds2 = []
    for row in today:
        rr = run_balance(p, row["cap_deg"], row["cap_dps"], row["cap_rpm"], duration=3.0, capture=True, seed=100 + row["n"])
        preds2.append((rr.held, row["held"]))
        rows_out.append(dict(dataset="today_left", n=row["n"], side="-", cap_deg=row["cap_deg"], cap_dps=row["cap_dps"], cap_rpm=row["cap_rpm"],
                             obs_sat=row["sat"], obs_held=row["held"], pred_held=rr.held, pred_sat=round(rr.sat_pct_1s, 1),
                             pred_fail=rr.fail_reason, pred_held_s=round(rr.held_s, 2), pred_max_abs=round(rr.max_abs_deg, 1)))
        lines.append(f"  today {row['n']:2d} - {row['cap_deg']:6.2f} deg {row['cap_dps']:6.1f} dps {row['cap_rpm']:7.1f} rpm | obs held={row['held']!s:5} sat={row['sat']:3.0f}% | pred held={rr.held!s:5} sat={rr.sat_pct_1s:5.1f}% {rr.fail_reason}")
    tp2, fn2, fp2, tn2 = confusion(preds2)
    lines.append(f"  today confusion: successes caught {tp2}/{tp2+fn2}, failures caught {tn2}/{tn2+fp2}  (FP {fp2}, FN {fn2})  score {tp2+tn2}/{len(today)}")
    txt = "\n".join(lines)
    print(txt)
    with (out / f"gate2_{label}.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()))
        w.writeheader(); w.writerows(rows_out)
    (out / f"gate2_{label}.txt").write_text(txt + "\n", encoding="utf-8")
    return (tp, fn, fp, tn), (tp2, fn2, fp2, tn2)


def g_crit(p: Params, theta_deg, rate_dps, wheel_rpm, lo=20.0, hi=40.0, seed=0, iters=6):
    """Largest g_eff (torque-to-gravity margin proxy) at which this handover
    still holds 3 s.  lo = definitely holds, hi = definitely falls."""
    if not run_balance(replace(p, g_eff=lo), theta_deg, rate_dps, wheel_rpm, 3.0, True, seed).held:
        return lo - 1.0
    if run_balance(replace(p, g_eff=hi), theta_deg, rate_dps, wheel_rpm, 3.0, True, seed).held:
        return hi + 1.0
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if run_balance(replace(p, g_eff=mid), theta_deg, rate_dps, wheel_rpm, 3.0, True, seed).held:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def ranking_validation(p: Params, out: Path, today_path: Path):
    """Do the real successes get a larger simulated margin than the failures?"""
    rows = []
    for n, side, cd, cr, cw, sat, held in CAMPAIGN:
        rows.append(dict(dataset="campaign", n=n, held=held, obs_sat=sat, g_crit=g_crit(p, cd, cr, cw, seed=n)))
    for row in load_today(today_path):
        rows.append(dict(dataset="today_left", n=row["n"], held=row["held"], obs_sat=row["sat"],
                         g_crit=g_crit(p, row["cap_deg"], row["cap_dps"], row["cap_rpm"], seed=100 + row["n"])))
    lines = ["=== margin ranking: g_crit = largest g_eff at which the handover still holds ==="]
    for ds in ("campaign", "today_left"):
        sub = [r for r in rows if r["dataset"] == ds]
        succ = [r["g_crit"] for r in sub if r["held"]]
        fail = [r["g_crit"] for r in sub if not r["held"]]
        pairs = sum(1 for a in succ for b in fail if a > b) + 0.5 * sum(1 for a in succ for b in fail if a == b)
        auc = pairs / (len(succ) * len(fail)) if succ and fail else float("nan")
        lines.append(f"  {ds}: held g_crit {sorted(round(x, 1) for x in succ)}  fell g_crit {sorted(round(x, 1) for x in fail)}  AUC={auc:.2f}")
        lines.append("   " + "  ".join(f"{r['n']}:{'H' if r['held'] else 'f'}{r['g_crit']:.1f}" for r in sub))
    txt = "\n".join(lines)
    print(txt)
    (out / "margin_ranking.txt").write_text(txt + "\n", encoding="utf-8")
    with (out / "margin_ranking.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    return rows


def margin_grid(p: Params, out: Path):
    """Left-side handover map.  wheel_rpm < 0 = still spinning the way the
    kick drove it (kick momentum not yet spent)."""
    import collections
    fields = ["angle_deg", "rate_dps", "wheel_rpm", "g_crit", "holds_g28", "sat_pct_g28"]
    best = []
    with (out / "margin_grid.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader()
        for angle in (4.0, 5.0, 6.0, 7.0, 8.0, 9.0):
            for rate in range(20, 100, 10):
                for wheel in range(-350, 200, 50):
                    gc = g_crit(p, -angle, float(rate), float(wheel))
                    rr = run_balance(replace(p, g_eff=28.0), -angle, float(rate), float(wheel), 3.0, True, 0)
                    w.writerow(dict(angle_deg=angle, rate_dps=rate, wheel_rpm=wheel, g_crit=round(gc, 2),
                                    holds_g28=int(rr.held), sat_pct_g28=round(rr.sat_pct_1s, 1)))
                    best.append((gc, angle, rate, wheel))
    best.sort(reverse=True)
    lines = ["=== handover margin grid (left side; g_crit high = forgiving) top 15 ==="]
    lines += [f"  g_crit {gc:5.1f}: angle {a:.0f} deg, rate {r} dps, wheel {wh} rpm" for gc, a, r, wh in best[:15]]
    for key, idx in (("angle", 1), ("rate", 2), ("wheel", 3)):
        acc = collections.defaultdict(list)
        for row in best:
            acc[row[idx]].append(row[0])
        lines.append(f"  mean g_crit by {key}: " + ", ".join(f"{k}:{sum(v)/len(v):.1f}" for k, v in sorted(acc.items())))
    txt = "\n".join(lines)
    print(txt)
    (out / "margin_grid.txt").write_text(txt + "\n", encoding="utf-8")


def launch_scan(p: Params, out: Path):
    """Which jump parameters give the most forgiving handover?  Score = largest
    g_eff at which the whole launch (spin, kick, coast, capture, 3 s balance)
    survives, from rest at -17 deg."""
    def survives(g, spin, kick, cap, rate):
        return run_jump(replace(p, g_eff=g), spin, kick, cap, rate, rest_deg=-17.0)["held"]

    def crit(spin, kick, cap, rate, lo=20.0, hi=40.0):
        if not survives(lo, spin, kick, cap, rate):
            return lo - 1.0
        if survives(hi, spin, kick, cap, rate):
            return hi + 1.0
        for _ in range(5):
            mid = 0.5 * (lo + hi)
            if survives(mid, spin, kick, cap, rate):
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    rows = []
    for spin in (350, 450, 550):
        for cap in (5.0, 8.0, 10.0):
            for rate in (40, 50, 55, 60, 65, 75, 90):
                gc = crit(spin, 450, cap, rate)
                j = run_jump(replace(p, g_eff=28.0), spin, 450, cap, rate, rest_deg=-17.0)
                h = j["handover"] or {}
                nan = float("nan")
                rows.append(dict(spin_rpm=spin, kick_ms=450, capture_deg=cap, rate_dps=rate, g_crit=round(gc, 2),
                                 held_g28=int(j["held"]), abort=j["abort"],
                                 cap_theta=round(h.get("theta_deg", nan), 1), cap_rate=round(h.get("rate_dps", nan), 0),
                                 cap_wheel=round(h.get("wheel_rpm", nan), 0),
                                 t_cap_ms=(round(1000 * h["t"]) if h else -1)))
    with (out / "launch_scan.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    rows.sort(key=lambda r: -r["g_crit"])
    lines = ["=== launch parameter scan from -17 deg (kick 450 ms), ranked by survivable g_eff ==="]
    lines += [f"  g_crit {r['g_crit']:5.1f}: jump {r['spin_rpm']} 450 {r['capture_deg']:.0f} {r['rate_dps']} -> handover {r['cap_theta']} deg {r['cap_rate']} dps wheel {r['cap_wheel']} rpm at {r['t_cap_ms']} ms {r['abort']}" for r in rows[:20]]
    cur = next(r for r in rows if r["spin_rpm"] == 550 and r["capture_deg"] == 8.0 and r["rate_dps"] == 75)
    lines.append(f"  current default (550 450 8 75): g_crit {cur['g_crit']:.1f}, handover {cur['cap_theta']} deg {cur['cap_rate']} dps {cur['cap_wheel']} rpm")
    txt = "\n".join(lines)
    print(txt)
    (out / "launch_scan.txt").write_text(txt + "\n", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path(__file__).parent / "out" / "nano_replica")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--g", type=float, default=NOMINAL.g_eff)
    ap.add_argument("--r", type=float, default=NOMINAL.r)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    p = replace(NOMINAL, g_eff=args.g, r=args.r)
    today = Path(__file__).parent.parent / "tuning" / "nano_2026-09-22_left_side.csv"
    gate1(p, args.out)
    gate2(p, args.out, today, label=f"g{p.g_eff:g}")
    gate2(replace(p, g_eff=30.0), args.out, today, label="g30_marginal")
    gate2(replace(p, g_eff=30.0, torque_scale=0.9), args.out, today, label="g30_torque0.9")
    if args.quick:
        return
    ranking_validation(p, args.out, today)
    margin_grid(p, args.out)
    launch_scan(p, args.out)


if __name__ == "__main__":
    main()
