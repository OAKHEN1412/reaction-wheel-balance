# Jump-up feasibility, 2026-09-19

Voltage-mode study from the measured **+/-16 deg stop**, superseding the old 30-45 deg brake-only conclusions. These are conditional simulation results, not a hardware demonstration.

## Reproduce

From `sim/` (Python via `py -X utf8`; requires NumPy and Matplotlib):

```powershell
py -X utf8 jumpup.py --out out/jumpup
py -X utf8 jumpup.py --out out/jumpup --extras-only
py -X utf8 -m unittest test_model
```

`--extras-only` reuses the saved main sweep and reruns the refinement/sensitivity cases and plots. The main run also generates these extras. `README.md` is the reviewed interpretation, not generated text.

## Model and protocol

- Main sweep: **2,970 deterministic runs**: wheel RPM {100,150,200,250,300,350,400,450,500,550,580}; inertia multiplier {0.5,0.75,1,1.25,1.5}; capture {5,10,15} deg; kick deadline {20,50,100,200,350,500} ms; three plugging cases. 580 rpm is simulation-only; firmware rejects >550.
- Voltage law: `omega_dot=(61*u-omega)/0.18`, `u` clipped to  deg1. Opposite-voltage acceleration is multiplied by 1 (nominal) or 0.5 (pessimistic). No legacy stall acceleration cap: nominal plugging can reach twice stall. Zero voltage uses 0.3 times ordinary deceleration, an unmeasured weak-coast/brake assumption.
- Each DIR reversal has the existing 2 ms zero-duty interval, plus 0/20/50 ms driver dead time after the flip. Reduced plugging and dead time also apply during balancing.
- Frame equation: `(C+I_w)*theta_dd = B*sin(theta)-I_w*omega_dot`; `C=0.0174475 kg m^2`, `B=1.23606 N m`, nominal `I_w=0.00245 kg m^2`. Frame mass/COM/inertia remain estimates.
- **Inertia sweep holds measured tau fixed**: it represents uncertainty in what inertia was present during the measurement, not adding rim mass to a fixed motor. Added mass usually raises tau; the supplemental `tau=0.18*inertia_scale` runs test that alternative. Neither interpretation substitutes for measuring inertia.
- Inelastic stops at +/-16 deg hold the frame during spin-up and permit release when torque lifts it. Any later stop impact disqualifies success. No bounce, slip, pivot friction, flexible frame, bus-voltage sag, current/thermal limit, or IMU noise/delay is modeled.
- Positive rest angle uses `u=-1` during SPINUP, then `u=+1` in KICK; negative rest mirrors it. Capture occurs only during KICK when `abs(theta)<capture`; coast for the handover tick, then reset controller and balance. Kick expiry aborts immediately; there is no delayed capture after expiry. Spin timeout 1.5 s, total jump timeout 2 s, jump angle abort 25 deg, balance fall angle 20 deg.
- Balance law/gains match firmware: Kp=1137.4, Kd=500, Kw=1, Ki=0.0387; inverse voltage mapping, integral clamp 1, acceleration clamp 4000, 85% speed guard, 0.3 rad/s-equivalent deadband. Control/RK4 step 2 ms; supplemental RK4 substeps 0.5 ms keep control timing fixed.
- Main feedback is ideal angle/rate/signed wheel speed. Firmware uses filtered IMU and unsigned FG plus its existing estimator; main spin threshold is exact speed, whereas hardware waits for at least 3 fresh physical pulses. Pulse timing can overshoot the requested spin RPM. The sim does not claim exact tach/IMU emulation.
- `fg_dropout` stress case uses the existing 0.18 s applied-voltage estimator for sign. After DIR reversal, FG magnitude decays with pulse age (72 pulses/wheel revolution), then uses the model after 300 ms, with no further FG pulses. This deliberately explores persistent FG loss; it is not a fitted dropout model.
- Success: capture, no later stop impact, still BALANCING **3 s after capture**, with `abs(theta)<2 deg` and `abs(theta_dot)<5 deg/s` throughout the final 0.5 s. Time to upright is first zero crossing from the manual command (null if never crossed). Overshoot is maximum angle on the opposite side over the observation. Failed uncaptured runs are observed to 5.5 s.

## Condensed main sweep

Counts are successful grid combinations out of 198, **not hardware probabilities**. Minimum RPM is the lowest tested successful legal value; 100 means <=100 within this grid, not an exact physical lower bound. Capture/kick vary to find that minimum.

| Iw multiplier (kg m^2) | Nominal count; min RPM | Half torque +20 ms | Half torque +50 ms |
|---|---|---|---|
| 0.5 (0.0012250) | 19/198; 250 | 0/198; none | 0/198; none |
| 0.75 (0.0018375) | 46/198; 100 | 20/198; 100 | 12/198; 100 |
| 1 (0.0024500) | 71/198; 100 | 53/198; 100 | 35/198; 100 |
| 1.25 (0.0030625) | 96/198; 100 | 76/198; 100 | 51/198; 100 |
| 1.5 (0.0036750) | 99/198; 100 | 90/198; 100 | 68/198; 100 |

At 0.5x inertia the first nominal success is **250 rpm / 350 ms / 5 deg**. Neither pessimistic case succeeded anywhere in the sampled grid, including 580 rpm. At 0.75x, 100 rpm works with 5 deg capture and 200 ms nominal / 350 ms pessimistic. At >=1x, minimum tested speed is 100 rpm, with suitable kick/capture. These thresholds are not monotonic guarantees for every larger speed; high wheel momentum can impair capture.

## Recommended experiments

**Safe first try: `jump 100 20`, capture 10 deg (firmware defaults).** Nominal rise is 0.96 deg, reaching about 15.04 deg before returning to the same stop. At 1.5x inertia it rises 2.62 deg (to 13.38 deg). No sampled inertia/plugging case reaches capture or overshoots upright. 20-50 ms dead time can consume the entire short kick and produce no lift. "Safe" here means intentionally under-powered in the tested model, not mechanically certified.

**Expected working candidate: `jump 200 350`, capture 10 deg.** Use only after verifying the small test moves toward upright. At nominal inertia it passes all three plugging cases with ideal feedback and the persistent FG-loss stress case. Actual full-voltage kick lasts only 84-198 ms before capture; 350 ms is a ceiling, not an instruction to continue after capture. It is not robustly demonstrated across all unknown inertia values.

| Plugging case | Feedback | Success | Capture (s from command) | Upright (s) | Opposite overshoot (deg) |
|---|---|---|---|---|---|
| nominal | ideal | True | 0.160 | 1.496 | 1.13 |
| nominal | fg_dropout | True | 0.160 | 1.508 | 1.19 |
| half_20ms | ideal | True | 0.246 | 1.384 | 1.17 |
| half_20ms | fg_dropout | True | 0.246 | 1.470 | 1.63 |
| half_50ms | ideal | True | 0.274 | 1.366 | 0.87 |
| half_50ms | fg_dropout | True | 0.274 | 1.502 | 1.34 |

The earlier **100 rpm / 200 ms / 10 deg** candidate (`recommended_*.png`, a provisional filename retained with all results) passes ideal feedback but misses the strict settling criterion with persistent FG loss: final-window angle reaches 2.14-2.59 deg, despite remaining off the stops. The final 200 rpm candidate gives more margin in that stress case. A 5 deg capture at 100 rpm also works with a longer kick but can overshoot 7.6 deg under half torque +50 ms; it is not the recommended first working trial.

## Saved artifacts

- `sweep.csv`, `sweep.json`: every main run, full parameters, summary, and metrics.
- `examples.json`, `*.csv`, `*.png`: safe, provisional, minimum-speed, and final `expected_*` traces; angle, wheel RPM, duty, phase.
- `thresholds.png`: minimum tested successful legal RPM versus estimated inertia; missing points mean no success.
- `working_refinement.json`: 180 extra nominal-inertia runs, spins 100-550 / captures 5,10,15 / kick 350 / all three plugging cases / both feedback models.
- `sensitivity.json`: 54 runs checking 0.5 ms physics, FG loss, and inertia-dependent tau at 0.5/1/1.5x, safe and provisional commands.
- `expected_convergence.json`: all six final working cases at 2, 1, 0.5 and 0.25 ms physics steps, retaining 2 ms control. All 24 pass. The plugging law has a discontinuity at zero wheel speed, so 2 ms versus finer physics changes overshoot by up to 0.15 deg. At 1 ms versus 0.25 ms the difference is below 0.02 deg. Use **0.8-1.8 deg overshoot and 1.35-1.51 s to upright** as the rounded prediction including this sensitivity.
- `exploratory_v1/`: all initial sweep and example artifacts retained; this first pass applied balance control immediately at capture instead of the final one-tick coast. Success counts were unchanged.
- `run.log`: printed main summary/examples from the final extras run.

## Remaining risks

Plugging may be absent, delayed, current-limited, or asymmetric; FG disappearance alone does not prove strong braking. BRAKE is not assumed to supply the kick. Wheel inertia and frame COM dominate feasibility, and changing inertia physically also changes motor tau. An angle-only capture can hand over with excessive angular rate; faster/higher-energy commands are not automatically better. Ideal stops hide real impact forces and bounce. Guard the frame on the opposite side, keep hands away from the wheel, and retain immediate power isolation.

Firmware permits jump only by an explicit command at rest, requires real FG pulses before kick, aborts spin on FG loss (100 ms), aborts jump on any IMU read failure, and requires fresh manual authorization after a failed attempt. EEPROM balance settings remain untouched. No board upload or COM3 access was performed.

## Validation

- `cd sim; py -X utf8 -m unittest test_model`: 23 tests passed, including existing balance regressions and new jump physics, mirror, safe-default, capture, timing and numerical checks.
- `arduino-cli compile --fqbn arduino:avr:mega firmware/reaction_wheel_balance`: passed, 29,780 bytes flash / 1,787 bytes SRAM.
- Host C++ self-tests passed: existing balance controller and wheel estimator plus jump parsing, rest gate, missing FG, both signs, timeouts, rollover, overspeed and non-finite input.

Host runner command used from repo root (Zig compiler installed only under ignored `build/`):

```powershell
py -X utf8 -m pip install --target build/jump-host ziglang
build/jump-host/ziglang/zig.exe cc -x c++ -std=c++11 firmware/tests/jump_selftest.cpp -o build/jump_selftest.exe -lc -Wno-nullability-completeness
build/jump_selftest.exe
```

The equivalent runner can be built with a standard host C++11 compiler. No self-test was executed on the board.
