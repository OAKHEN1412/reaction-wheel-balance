# Nano-build replica: validation and findings (2026-09-22)

Produced by `py -X utf8 sim/nano_replica.py` from the repository root.
Everything in this directory comes from that one script; re-run it to regenerate.

## Validation scores first

**Gate 1 (steady balancing, Kp 1137.4 / Kd 450 / Kw 3 / Ki 0.0387): passed on amplitude, not on frequency.**

| quantity | real machine (Nano, `nano_kd_sweep.log`, 511 s at Kd 450/500) | replica (g_eff 28) |
|---|---|---|
| holds from 2 deg, wheel stopped | yes (>2 min) | yes (20 s, 60 s tests) |
| gyro SD | 1.18-1.25 dps | 1.19 dps |
| logged angle SD | 0.37-0.43 deg (0.27-0.34 on the Mega build) | 0.59 deg |
| duty mean / max / saturation | 0.14 / 0.24-0.27 / 0 % | 0.105 / 0.22 / 0 % |
| wheel speed | one-sided bias, -99..-19 rpm | one-sided bias, +-20..90 rpm |
| static recovery limit, wheel stopped | ~8 deg | 8 deg |
| dominant oscillation | 2.1 Hz (0.47 s), rising with Kd: 1.73/2.1/2.22 Hz at Kd 400/450/500 | ~1.1 Hz (0.92 s), no Kd dependence |
| Kd 600 | fell (wheel swung to -663 rpm, duty pinned 27 %) | stable |

The 0.2 s period in the task statement is the Mega build's zero-crossing count at Kd 500; on the
Nano log the spectral peak is 2.1 Hz. The replica does not reproduce that mode or the Kd-600 cliff.
It does reproduce the wobble amplitude because the wheel-wind-up (slow) mode is nearly marginal
(see findings), and it reproduces Kw = 1 being worse than Kw = 3.

**Gate 2 (classify the known launches from the handover triple): FAILED.**

Prediction = held for 3 s after handover, initial state = the logged (angle, rate, wheel) triple.

| model | campaign (8; 3 held) successes caught | failures caught | today left side (17; 1 held) successes | failures |
|---|---|---|---|---|
| g_eff 28 (Gate-1 consistent) | 3/3 | 0/5 | 1/1 | 0/16 |
| g_eff 30 (marginal) | 3/3 | 0/5 | 1/1 | 0/16 |
| g_eff 30, torque x0.9 | 2/3 | 2/5 | 1/1 | 5/16 |
| ranking by g_crit (largest g_eff the handover survives) | AUC 0.63 | | AUC 0.44 | |
| ranking by torque margin (lowest torque the handover survives) | AUC 0.60 | | AUC 0.69 | |

Files: `gate2_g28.*`, `gate2_g30_marginal.*`, `gate2_g30_torque0.9.*`, `margin_ranking.*`, `torque_margin.csv`.

The model predicts "held" for essentially everything, which is the mirror image of the previous
attempt (`sim/capture_set.py`, which predicted "fell" for everything). Neither is a model of the catch.
Two specific, decisive misses:

1. Today's launches 1, 2, 3, 6 (63-88 dps at handover with the wheel still at -205..-349 rpm) failed
   hardest in reality (37-48 % duty saturation, overshoot to +17 deg). In the replica they are the *most*
   forgiving handovers of all (g_crit 35-41, survive down to 50-59 % torque) because the modelled
   plugging torque from a fast wheel brakes the frame in a few degrees.
2. Today's launches 15, 18, 20, 22 (17-24 dps, wheel ~0, -5 to -6 deg) failed by stalling short of
   upright. Every model variant tried, down to 80 % torque, catches them: a frame at -5 deg with a
   stopped wheel is inside the 8 deg static limit. The late-batch machine could not do this.

**The recoverable-region grid (`margin_grid.*`) and the jump-parameter scan (`launch_scan.*`) are
therefore not deliverables.** They are left in place so the failure is inspectable, and they show
what the model believes: high wheel speed in the kick direction helps, angle 4-6 deg is better than 8-9,
rate 50-70 dps is best. The real data contradict the first of these outright.

## What was established (trust these)

1. **Motor model** (8 full-duty spin-ups in `nano_jump_campaign.log.gz`, frame on its stop):
   voltage-mode first order, tau 0.236 s (0.22-0.26 per launch), full scale 593 rpm, with the
   acceleration from rest flat at ~200 rad/s^2 for the first 100 ms (fit rms 2-3 rpm per launch).
   A pure first order fits worse (rms 4-8 rpm). No driver input lag (free fit tau_in = 0.001 s).
   The documented "40 rad/s^2 sustained" is exactly what this model gives once the wheel is at
   350-450 rpm: it is back-EMF, not a torque cap. `step_test.log.gz` (t63 485-595 ms) is the
   *weighted-wheel* run, not the light wheel.
2. **The logged angle is mostly not tilt.** In steady balancing the gyro-integrated tilt has SD 0.15 deg,
   the logged `theta_deg` has SD 0.42 deg and correlates 0.37 with it. The complementary filter
   (0.1 s) passes accelerometer contamination: frame acceleration through a lever arm of ~0.05-0.09 m,
   with the sign such that a hard deceleration of a rising frame reads as *extra* rise (run 3 handover:
   +4.3 deg logged jump in 21 ms against +0.7 deg of gyro). The 0.27-0.43 deg SD figures are a sensor
   property and should not be used as a plant target. Immediately after a catch the controller is
   acting on an angle that can be several degrees wrong for ~0.1-0.2 s.
3. **Friction offset** of ~0.035 duty (from the step-test steady speeds and the balancing duty/wheel
   pair) explains quantitatively why the wheel parks at ~-57 rpm with mean duty 0.14 at Kw 3, and at
   -178..-38 rpm at Kw 1 (the controller's mean acceleration command Kw*omega is what the friction eats).
4. **Plant ratios.** With relative-wheel dynamics theta_dd = g_eff*sin(theta) - r*omega_dot:
   r = 0.031-0.045 from the kick-phase gyro (valley, correlated with g_eff), and the ideal-loop fast
   mode r*Kd - Kw would put r at 0.035 for the observed 1.73/2.1/2.22 Hz. g_eff is only bounded:
   the steady loop is stable in the replica up to 30 and diverges at 31 (Routh condition
   (r*Kd - Kw)(0.75*r*Kp - g_eff) > Kw*g_eff, the 0.75 being the 0.18/0.24 tau mismatch), and the
   campaign's wind-up failures need >= 29. The machine really is on the edge of the slow mode,
   which is why Kw = 1 is worse (confirmed in the replica, 1/3 vs 3/3 successes) and why launch-to-launch
   variation flips outcomes.
5. **The real catch is a narrow energy window.** From the campaign's overshoot column: all three successes
   overshot upright to +4.6..+11.7 deg; the failures either stalled (<= +1.4 deg, then wound the wheel to
   the 495 rpm clamp) or overshot to +16.8..+18.1 deg. For a frame at 85 dps to reach +17 deg the
   braking authority must be <= ~3.5 rad/s^2 of frame deceleration; the ~8 deg static limit independently
   gives r*alpha_max ~ g_eff*sin(8 deg) ~ 3.5-4 rad/s^2. The uncapped-plugging model has 4-5x that.
6. **Contradiction that blocks the model.** A symmetric current cap that makes the catch weak enough
   (r*alpha ~ 4) cannot lift the frame in the kick (needs r*alpha >= g_eff*sin(17 deg) ~ 8): with
   r <= 0.03 and cap <= 140 rad/s^2 every launch aborts with "kick expired" (`weakscan`, in the
   session notes). So the kick and the catch see torque capabilities differing by >= 2x, and the
   50 Hz telemetry cannot say why. Candidates, none excluded: energy stored in the compliant stop
   released at the kick (the 0 -> 25 dps jump in the first 20 ms of every kick), supply sag after 0.7 s
   of full current, base sliding during the catch (the user saw it move), belt/gear slip under
   reversal, estimator sign errors after the DIR flip (the estimate is model-signed FG magnitude).
7. **Weakening across the batch.** As a lower bound in this model, today's late launches (n >= 13)
   would need >= 14-21 % less torque than nominal to fail (`torque_margin.csv`, mean torque margin
   0.81 vs 0.67 for the early ones). Because the model is wrong about the early fast launches
   (they would need >= 40-50 % loss, which is not credible), this number is only meaningful for the
   slow stalls, and there it says the machine had lost at least a fifth of its authority.

## What could not be established

- A model that separates successes from failures on either dataset. Campaign launches 1 (fell) and 6
  (held) are near-mirror states (6.3 deg / 40 dps / 48 rpm vs 6.8 deg / 41 dps / 19 rpm); no map of the
  handover triple can score both, so the deciding variable is not in the triple.
- The 2.1 Hz mode and the Kd-600 cliff. Delay, driver lag, accelerometer lever arm and g_eff were tried;
  none produces a lightly damped mode that scales with Kd. Backlash or compliance in the 6:1 reduction
  would, and is not measurable from the logs.
- The true wheel speed after a DIR flip. `wheel_rpm` in telemetry is the estimator; at the run-3 handover
  it moved 10 rpm in 21 ms while the frame lost 47 dps, which is inconsistent with any r found.
- Jump parameters to try next. Not delivered: both gates did not pass.

## What the next hardware session should measure instead

Each of these resolves one of the ambiguities above and is cheap:

1. Log the raw FG magnitude, the fresh flag and the estimator at 100-500 Hz through one catch.
2. Static authority map: while balancing, command a 100 ms full-duty pulse each way with the wheel
   parked at 0, -200 and -400 rpm and log the gyro. That is r*alpha directly, in the catch regime.
3. Battery voltage on an analog pin during spin-up, kick and catch.
4. Clamp the base to the table for one batch.
5. Repeat the launch that succeeded (`550 450 8 55`, handover ~6-7 deg, 40-60 dps, |wheel| < 100 rpm)
   until the machine fails, then let it cool and repeat: the empirical window from all four successes
   is 6.1-7.4 deg, 37-60 dps, |wheel| 19-94 rpm, but 7 failures sit in the same window.

## Parameter set used (`nano_replica.Params`)

g_eff 28 (bounded 25..30), r 0.036, tau_m 0.22 s, drive cap 200 rad/s^2, plugging uncapped,
friction 0.035 duty, IMU lever arm 0.08 m, 2-sample sensor delay, full scale 583 rpm, guard 0.85,
gains 1137.4/450/3/0.0387 with tau_est 0.18, capture blend 350 ms from 25 % Kp.
