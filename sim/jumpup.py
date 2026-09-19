"""16-degree stop self-righting with measured voltage dynamics (2026-09-19).

Independent of the legacy torque-capped model: plugging must be able to exceed
stall torque. Inertia is an uncertain estimate at fixed measured tau, NOT an
added-mass experiment. See out/jumpup/README.md for assumptions and results.
"""
import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import params

RAD = math.pi / 180
RPM = 2 * math.pi / 60
GAINS = dict(Kp=1137.4, Kd=500.0, Kw=1.0, Ki=0.0387)
CASES = {"nominal": (1.0, 0.0), "half_20ms": (0.5, 0.020),
         "half_50ms": (0.5, 0.050)}


def required_omega_jump(d, theta0_rad):
    """Ideal instantaneous stop-only impulse; NOT a voltage-kick threshold."""
    return math.sqrt(2*d["B"]*(1-math.cos(theta0_rad))*(d["C"]+d["I_w"])) / d["I_w"]


def acceleration(omega, u, tau=0.18, plug_factor=1.0):
    a = (61.0*u - omega) / tau
    if u*omega < 0:
        a *= plug_factor
    elif u == 0:
        a *= 0.3  # weak coast/brake; measured value still unknown
    return a


def physics_step(s, u, iw, C, B, dt, plug_factor, tau=0.18):
    def f(x):
        th, rate, w = x
        a = acceleration(w, u, tau, plug_factor)
        return rate, (B*math.sin(th)-iw*a)/(C+iw), a
    k1 = f(s)
    k2 = f(tuple(x+dt*k/2 for x, k in zip(s, k1)))
    k3 = f(tuple(x+dt*k/2 for x, k in zip(s, k2)))
    k4 = f(tuple(x+dt*k for x, k in zip(s, k3)))
    return tuple(x+dt*(a+2*b+2*c+d)/6 for x,a,b,c,d in zip(s,k1,k2,k3,k4))


def simulate(spin_rpm=100, kick_ms=20, capture_deg=10, inertia_scale=1.0,
             case="nominal", side=1, dt=0.002, trace=False, tau=0.18, physics_substeps=1, feedback="ideal"):
    if not (0 < spin_rpm < 61/RPM and 0 < kick_ms <= 500 and
            0 < capture_deg < 16 and inertia_scale > 0 and side in (-1, 1) and dt > 0 and physics_substeps >= 1
            and feedback in ("ideal", "fg_dropout")):
        raise ValueError("invalid jump parameters")
    d = params.nominal_derived()
    iw, C, B = 0.00245*inertia_scale, d["C"], d["B"]
    plug, dead = CASES[case]
    stop = 16*RAD
    s = (side*stop, 0.0, 0.0)
    phase, phase_start = "SPINUP", 0.0
    capture_time = upright_time = kick_time = None
    integral = 0.0
    estimate = applied_u = 0.0
    flip_time, flip_magnitude = -10.0, 0.0
    direction, previous_u, blank_until = 0, 0.0, 0.0
    lifted = impacted = False
    min_signed = stop
    tail_angle = tail_rate = 0.0
    history = []
    # 2 s jump deadline + 3 s observation after capture, plus settling tail.
    end = 5.5
    for n in range(int(end/dt)+1):
        t = n*dt
        th, rate, w = s
        estimate += (applied_u*61-estimate)*dt/0.18
        measured_w = w
        if feedback == "fg_dropout":
            age = t-flip_time
            magnitude = abs(w)
            if age < 0.3:
                magnitude = min(flip_magnitude, 2*math.pi/(72*max(age, 1e-9)))
            elif flip_time > 0:
                # FG unavailable after reversal: model fallback after tach timeout.
                magnitude = abs(estimate)
            measured_w = math.copysign(magnitude, estimate)
        just_captured = False
        elapsed = t-phase_start
        if phase in ("SPINUP", "KICK") and (t >= 2 or abs(th) > 25*RAD):
            phase = "FALLEN"
        if phase == "SPINUP":
            if -side*w >= spin_rpm*RPM:
                phase, phase_start, kick_time = "KICK", t, t
            elif elapsed >= 1.5:
                phase = "FALLEN"
        if phase == "KICK":
            if abs(th) < capture_deg*RAD:
                phase, capture_time = "BALANCING", t
                just_captured = True
                end = t+3.0
            elif t-phase_start >= kick_ms/1000:
                phase = "FALLEN"
        if phase == "BALANCING" and (abs(th) > 20*RAD or impacted):
            phase = "FALLEN"
        if phase == "SPINUP":
            requested = -side
        elif phase == "KICK":
            requested = side
        elif phase == "BALANCING" and not just_captured:
            old = integral
            integral = max(-1, min(1, integral+th*dt))
            a = max(-4000, min(4000, 1137.4*th+500*rate+measured_w+0.0387*integral))
            if measured_w*a > 0 and abs(measured_w) >= 0.85*61:
                a = 0
            target = measured_w+0.18*a
            requested = max(-1, min(1, target/61))
            if abs(target) > 61 and target*th > 0:
                integral = old
        else:
            requested = 0.0
        if abs(requested)*61 < 0.3:
            requested = 0.0
        desired_dir = 1 if requested > 0 else -1 if requested < 0 else 0
        u = requested
        if desired_dir and desired_dir != direction:
            if previous_u != 0:
                u = 0.0  # firmware zero-duty interval BEFORE changing DIR
            else:
                if direction:
                    flip_time, flip_magnitude = t, abs(w)
                    blank_until = t+dead  # additional driver dead time AFTER DIR flip
                direction = desired_dir
        previous_u = u
        if t < blank_until:
            u = 0.0
        applied_u = u
        if trace:
            history.append(dict(t=t, theta_deg=th/RAD, rate_dps=rate/RAD,
                                wheel_rpm=w/RPM, duty=u, phase=phase))
        min_signed = min(min_signed, side*th)
        if upright_time is None and side*th <= 0:
            upright_time = t
        if capture_time is not None and t >= end-0.5:
            tail_angle = max(tail_angle, abs(th)/RAD)
            tail_rate = max(tail_rate, abs(rate)/RAD)
        if t >= end:
            break
        for _ in range(physics_substeps):
            s = physics_step(s, u, iw, C, B, dt/physics_substeps, plug, tau)
            th, rate, w = s
            if abs(th) < stop-1e-6:
                lifted = True
            if abs(th) >= stop:
                if lifted:
                    impacted = True
                s = (math.copysign(stop, th), 0.0, w) # inelastic stop contact
    result = dict(spin_rpm=spin_rpm, kick_ms=kick_ms, capture_deg=capture_deg,
                  inertia_scale=inertia_scale, I_w=iw, case=case, tau=tau, feedback=feedback, physics_substeps=physics_substeps,
                  success=bool(capture_time is not None and phase == "BALANCING" and
                               not impacted and tail_angle < 2 and tail_rate < 5),
                  peak_opposite_deg=max(0, -min_signed/RAD),
                  rise_deg=16-min_signed/RAD, upright_s=upright_time,
                  capture_s=capture_time, kick_start_s=kick_time,
                  tail_angle_deg=tail_angle if capture_time is not None else None,
                  tail_rate_dps=tail_rate if capture_time is not None else None,
                  impacted=impacted, final_phase=phase)
    return (result, history) if trace else result


def report(d_nom=None):
    print("16-degree voltage jump model supersedes the old 30-45 degree brake study.")
    print("Run: py -X utf8 jumpup.py --out out/jumpup (see saved report).")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path(__file__).parent/"out"/"jumpup")
    parser.add_argument("--extras-only", action="store_true", help="reuse sweep.json; regenerate plots and sensitivity/refinement runs")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if args.extras_only:
        rows = json.loads((args.out/"sweep.json").read_text(encoding="utf8"))["rows"]
    else:
        rows = []
        for scale, case, spin, capture, kick in itertools.product(
                (0.5, 0.75, 1.0, 1.25, 1.5), CASES,
                (100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 580),
                (5, 10, 15), (20, 50, 100, 200, 350, 500)):
            rows.append(simulate(spin, kick, capture, scale, case))
    with (args.out/"sweep.csv").open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    summary = []
    for scale, case in itertools.product((0.5, 0.75, 1, 1.25, 1.5), CASES):
        subset = [r for r in rows if r["inertia_scale"] == scale and r["case"] == case]
        wins = [r for r in subset if r["success"] and r["spin_rpm"] <= 550]
        best = min(wins, key=lambda r:(r["spin_rpm"], r["peak_opposite_deg"], r["kick_ms"])) if wins else None
        summary.append(dict(inertia_scale=scale, case=case, successes=sum(r["success"] for r in subset),
                            trials=len(subset), minimum_legal_success=best))
    metadata = dict(date="2026-09-19", command="py -X utf8 jumpup.py --out out/jumpup",
                    gains=GAINS, omega_fs=61, tau=0.18, stop_deg=16, dt=0.002,
                    coast_factor=0.3, cases=CASES, summary=summary, rows=rows)
    (args.out/"sweep.json").write_text(json.dumps(metadata, indent=2), encoding="utf8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    examples = [("safe_nominal", dict()), ("safe_high_inertia", dict(inertia_scale=1.5)),
                ("safe_pessimistic", dict(case="half_50ms"))]
    for case in CASES:
        examples.append(("recommended_"+case, dict(spin_rpm=100, kick_ms=200, capture_deg=10, case=case)))
        examples.append(("dropout_"+case, dict(spin_rpm=100, kick_ms=200, capture_deg=10, case=case, feedback="fg_dropout")))
    for case, feedback in itertools.product(CASES, ("ideal", "fg_dropout")):
        examples.append(("expected_"+case+"_"+feedback,
                         dict(spin_rpm=200, kick_ms=350, capture_deg=10, case=case, feedback=feedback)))
    for item in summary:
        if item["inertia_scale"] == 1 and item["minimum_legal_success"]:
            r = item["minimum_legal_success"]
            examples.append(("working_"+item["case"], {k:r[k] for k in
                             ("spin_rpm", "kick_ms", "capture_deg", "case")}))
    example_results = {}
    for name, kw in examples:
        result, h = simulate(**kw, trace=True)
        example_results[name] = result
        with (args.out/(name+".csv")).open("w", newline="", encoding="utf8") as f:
            writer = csv.DictWriter(f, fieldnames=h[0]); writer.writeheader(); writer.writerows(h)
        fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        for ax, key in zip(axes, ("theta_deg", "wheel_rpm", "duty")):
            ax.plot([v["t"] for v in h], [v[key] for v in h]); ax.set_ylabel(key); ax.grid()
            for mark in (result["kick_start_s"], result["capture_s"]):
                if mark is not None: ax.axvline(mark, color="gray", ls="--")
        axes[-1].set_xlabel("seconds from manual command")
        fig.suptitle(f"{name}\n{result['spin_rpm']} rpm / {result['kick_ms']} ms limit / "
                     f"{result['capture_deg']} deg capture / inertia x{result['inertia_scale']}")
        fig.tight_layout()
        fig.savefig(args.out/(name+".png")); plt.close(fig)
    (args.out/"examples.json").write_text(json.dumps(example_results, indent=2), encoding="utf8")
    fig, ax = plt.subplots(figsize=(9, 5))
    for case in CASES:
        items = [v for v in summary if v["case"] == case]
        ax.plot([v["inertia_scale"] for v in items],
                [v["minimum_legal_success"]["spin_rpm"] if v["minimum_legal_success"] else float("nan") for v in items],
                marker="o", label=case)
    ax.set(xlabel="I_w / 0.00245 (fixed measured tau)", ylabel="Minimum successful sampled wheel rpm (<=550)")
    ax.legend(); ax.grid(); fig.tight_layout(); fig.savefig(args.out/"thresholds.png"); plt.close(fig)
    sensitivity = []
    for case, scale, spin, kick, capture in itertools.product(CASES, (0.5, 1.0, 1.5), (100,), (20, 200), (10,)):
        for extra in (dict(physics_substeps=4), dict(feedback="fg_dropout"), dict(tau=0.18*scale)):
            sensitivity.append(simulate(spin, kick, capture, scale, case, **extra))
    (args.out/"sensitivity.json").write_text(json.dumps(sensitivity, indent=2), encoding="utf8")
    refinement = [simulate(spin, 350, capture, 1, case, feedback=feedback)
                  for spin, capture, case, feedback in itertools.product(
                      (100,150,200,250,300,350,400,450,500,550), (5,10,15), CASES, ("ideal", "fg_dropout"))]
    (args.out/"working_refinement.json").write_text(json.dumps(refinement, indent=2), encoding="utf8")
    convergence = [dict(case=case, feedback=feedback,
                        runs=[simulate(200, 350, 10, case=case, feedback=feedback,
                                       physics_substeps=n) for n in (1, 2, 4, 8)])
                   for case, feedback in itertools.product(CASES, ("ideal", "fg_dropout"))]
    (args.out/"expected_convergence.json").write_text(json.dumps(convergence, indent=2), encoding="utf8")
    print(json.dumps(summary, indent=2))
    print(json.dumps(example_results, indent=2))

if __name__ == "__main__":
    main()
