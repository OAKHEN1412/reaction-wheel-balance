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
// sign. The FG's own magnitude is used when it has a fresh reading (more
// accurate than the model's magnitude); otherwise the model's magnitude is
// used as a fallback (e.g. very low speed where FG pulses are sparse).
// =============================================================================
#include <cmath>

class WheelSpeedEstimator {
public:
  explicit WheelSpeedEstimator(float tauS = 0.08f) : tau_(tauS), omegaEst_(0.0f) {}

  void setTau(float tauS) { tau_ = tauS; }
  void reset(float omega = 0.0f) { omegaEst_ = omega; }

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
    float sign = (omegaEst_ >= 0.0f) ? 1.0f : -1.0f;
    float mag = fgFresh ? fgMagnitude : std::fabs(omegaEst_);
    return sign * mag;
  }

  float raw() const { return omegaEst_; }

private:
  float tau_;
  float omegaEst_;
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

  return true;
}
