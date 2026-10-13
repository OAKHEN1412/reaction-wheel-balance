#pragma once
// =============================================================================
// wheel_speed_estimator.h -- pure logic, NO Arduino dependency.
//
// The FG tach only reports an UNSIGNED speed magnitude. Naively signing that
// magnitude by Motor::currentDirectionSign() (the direction pin the motor is
// CURRENTLY being commanded to) is wrong immediately after every direction
// flip: the wheel is still coasting in its OLD direction by inertia for some
// time after the command flips, so the raw commanded-direction sign is
// briefly the opposite of the true physical sign. Since balancing involves
// omega_cmd crossing zero constantly, that would turn the Kw feedback term
// into positive feedback exactly at every zero crossing.
//
// Model: track a first-order lag of the commanded wheel speed (the physical
// wheel speed can't change instantaneously; it approaches whatever is
// commanded with time constant tau, dominated by wheel inertia + available
// motor torque):
//   omega_est += (omega_cmd - omega_est) * dt / tau
// sign(omega_est) is then a trustworthy (lagged, but not falsely flipped)
// sign. Fresh FG readings correct the model's magnitude (gated against
// outliers, see update()); without FG the model runs open-loop.
// =============================================================================
#include <math.h>

class WheelSpeedEstimator {
public:
  explicit WheelSpeedEstimator(float tauS = 0.08f) : tau_(tauS), omegaEst_(0.0f) {}

  void setTau(float tauS) { tau_ = tauS; }
  void reset(float omega = 0.0f) { omegaEst_ = omega; disagreeS_ = 0.0f; minFgOpposed_ = kNoMin; }

  // omegaCmd: rad/s, the wheel speed currently being commanded (drives the lag).
  // fgMagnitude: rad/s, unsigned magnitude from the FG tach.
  // fgFresh: true if the FG reading has not timed out.
  // dt: s.
  // Returns the signed wheel speed estimate (rad/s).
  float update(float omegaCmd, float fgMagnitude, bool fgFresh, float dt) {
    if (tau_ > 0.0f) {
      omegaEst_ += (omegaCmd - omegaEst_) * (dt / tau_);
    } else {
      omegaEst_ = omegaCmd;
    }
    // FG corrects the model's magnitude instead of replacing it. On hardware
    // (2026-09-20) FG read -436, -215, -429, -178, -21 rpm within ~100 ms at
    // full duty (missed pulses), and using it raw turned the voltage command
    // into a brake. Readings far from the model are ignored unless they
    // persist for kResyncS, so a real model error still gets corrected.
    // Zero crossing (hardware 2026-10-13): while the applied command points
    // against the estimate, the real wheel can only be slowing down, so FG
    // must keep falling. If it climbs back clearly above the lowest value seen
    // since then, the wheel has already crossed zero -- it reverses faster
    // than tau predicts -- and is speeding up the commanded way. Take the
    // command's sign. Without this the magnitude correction below kept the
    // old sign and drove the estimate to -210 rpm with the wheel at +330 rpm.
    // The margin grows with speed so FG period jitter at high speed can't flip it.
    bool opposed = crossingFix && omegaCmd * omegaEst_ < 0.0f && fabsf(omegaCmd) > kOpposeMinRadS;
    if (!opposed) {
      minFgOpposed_ = kNoMin;
    } else if (fgFresh) {
      if (fgMagnitude < minFgOpposed_) minFgOpposed_ = fgMagnitude;
      if (fgMagnitude > minFgOpposed_ * (1.0f + kCrossMarginFrac) + kCrossMarginRadS) {
        omegaEst_ = (omegaCmd > 0.0f) ? fgMagnitude : -fgMagnitude;
        minFgOpposed_ = kNoMin;
        disagreeS_ = 0.0f;
        return omegaEst_;
      }
    }

    if (fgFresh) {
      float sign = (omegaEst_ >= 0.0f) ? 1.0f : -1.0f;
      float m = fabsf(omegaEst_);
      float err = fgMagnitude - m;
      float gate = fmaxf(kGateMinRadS, kGateFrac * m);
      bool inGate = fabsf(err) <= gate;
      // Once a disagreement has persisted, keep accepting until back in gate.
      if (inGate || disagreeS_ >= kResyncS) {
        float g = dt / kCorrTauS;
        if (g > 1.0f) g = 1.0f;
        omegaEst_ += sign * err * g;
      }
      disagreeS_ = inGate ? 0.0f : disagreeS_ + dt;
    }
    return omegaEst_;
  }
  static constexpr float kCorrTauS = 0.03f;    // FG correction time constant
  static constexpr float kGateMinRadS = 15.0f; // ~140 wheel rpm
  static constexpr float kGateFrac = 0.35f;
  static constexpr float kResyncS = 0.10f;
  static constexpr float kOpposeMinRadS = 2.0f;    // |command| below this is not "driving the other way"
  static constexpr float kCrossMarginRadS = 3.0f;  // ~29 wheel rpm above the minimum ...
  static constexpr float kCrossMarginFrac = 0.30f; // ... plus 30% of it
  static constexpr float kNoMin = 1.0e9f;

  // Zero-crossing check on/off. Set false for the pre-2026-10-13 behaviour.
  // The A/B build toggled it over serial (`estfix 0|1`, not kept in the
  // firmware); result in tuning/tools/estfix_ab.py.
  bool crossingFix = true;

  float raw() const { return omegaEst_; }

private:
  float tau_;
  float omegaEst_;
  float disagreeS_ = 0.0f;
  float minFgOpposed_ = kNoMin;
};

// ---------------------------------------------------------------------------
// Self-test (no Arduino dependency): confirms the estimator does NOT flip
// sign instantly on a step command change (the whole point of this module),
// and that it does eventually track a sustained new command.
// ---------------------------------------------------------------------------
inline bool wheelSpeedEstimatorSelfTest(const char **failMsg = nullptr) {
  auto fail = [&](const char *m) -> bool { if (failMsg) *failMsg = m; return false; };

  WheelSpeedEstimator est(0.08f);
  est.reset(100.0f); // was spinning at +100 rad/s
  const float dt = 0.01f;

  // Command flips hard to -100 rad/s. FG is stale (no fresh reading right
  // after a flip is a realistic case). The estimate must stay positive
  // immediately after the flip (it's still coasting the old way).
  float first = est.update(-100.0f, 100.0f, /*fgFresh=*/false, dt);
  if (!(first > 0.0f)) return fail("estimated sign flipped instantly on a step command change");

  // After several time constants (2s >> tau=0.08s) of a sustained new
  // command, the estimate should have decayed through zero and now track it.
  float last = 0.0f;
  for (int i = 0; i < 200; ++i) last = est.update(-100.0f, 100.0f, false, dt);
  if (!(last < 0.0f)) return fail("estimated sign failed to eventually track a sustained new command");

  // FG consistent with the model pulls the magnitude toward it.
  WheelSpeedEstimator e2(0.18f);
  e2.reset(50.0f);
  float v = 0.0f;
  for (int i = 0; i < 50; ++i) v = e2.update(50.0f, 55.0f, true, 0.002f);
  if (!(v > 53.0f && v < 56.0f)) return fail("consistent FG did not correct the model magnitude");
  // A single wild FG reading (missed pulses) is ignored.
  v = e2.update(50.0f, 2.0f, true, 0.002f);
  if (!(v > 50.0f)) return fail("outlier FG reading was not rejected");
  // A persistent disagreement is eventually accepted (resync).
  for (int i = 0; i < 100; ++i) v = e2.update(v, 20.0f, true, 0.002f);
  if (!(v < 30.0f)) return fail("persistent FG disagreement never resynced");

  // 4) Zero crossing under a reversed command (hardware 2026-10-13, 12 of 14
  //    push-test falls). The wheel idles at about -65 rpm on small negative
  //    duty, then duty swings positive and ramps to full over 0.6 s. The real
  //    wheel reverses faster than the model's tau (a replay of the logged duty
  //    reproduces the hardware trace with a real tau of 0.05-0.08 s, not with
  //    0.12-0.18 s). FG reports only |omega|, so the magnitude correction kept
  //    the model's old sign and dragged the estimate to -110..-210 rpm while
  //    the wheel really turned +100..+330 rpm. Voltage mode then subtracted
  //    that from the duty for the whole recovery.
  {
    WheelSpeedEstimator e3(0.18f);
    const float fullScale = 61.1f;          // MOTOR_FULL_SCALE_WHEEL_RADPS
    const float h = 0.002f, tauReal = 0.07f;
    float truth = -6.8f;                    // -65 wheel rpm
    e3.reset(truth);
    float est = truth, worstWrongSign = 0.0f;
    for (int i = 1; i <= 550; ++i) {        // 1.1 s at 500 Hz
      float t = i * h;
      float duty = (t < 0.5f) ? -0.06f : 0.17f + (t - 0.5f) * 1.4f;
      if (duty > 1.0f) duty = 1.0f;
      float cmd = duty * fullScale;
      truth += (cmd - truth) * (h / tauReal);
      bool fresh = fabsf(truth) > 1.0f;     // FG goes stale near standstill
      est = e3.update(cmd, fabsf(truth), fresh, h);
      if (truth > 5.0f && est < 0.0f && -est > worstWrongSign) worstWrongSign = -est;
    }
    if (!(worstWrongSign < 3.0f)) return fail("estimate grew the wrong way while the wheel ran the other way");
    if (!(est > 0.0f && fabsf(est - truth) < 0.2f * truth)) return fail("estimate did not follow the wheel across zero");
  }

  // 5) The zero-crossing check must not fire early: braking from +400 rpm with
  //    FG jittering +-8% (one-period measurement), the estimate keeps the old
  //    sign until the wheel really has crossed zero.
  {
    WheelSpeedEstimator e4(0.18f);
    const float h = 0.002f, tauReal = 0.07f;
    float truth = 41.9f;                    // +400 wheel rpm
    e4.reset(truth);
    const float cmd = -0.6f * 61.1f;
    for (int i = 1; i <= 400; ++i) {
      truth += (cmd - truth) * (h / tauReal);
      float jitter = (i % 2) ? 1.08f : 0.92f;
      float mag = fabsf(truth) * jitter;
      float est = e4.update(cmd, mag, fabsf(truth) > 1.0f, h);
      if (truth > 4.0f && !(est > 0.0f)) return fail("estimate flipped sign while the wheel was still braking");
    }
  }

  return true;
}
