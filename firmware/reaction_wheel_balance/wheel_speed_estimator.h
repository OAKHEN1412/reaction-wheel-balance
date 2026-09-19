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
  void reset(float omega = 0.0f) { omegaEst_ = omega; disagreeS_ = 0.0f; }

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

  float raw() const { return omegaEst_; }

private:
  float tau_;
  float omegaEst_;
  float disagreeS_ = 0.0f;
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

  return true;
}
