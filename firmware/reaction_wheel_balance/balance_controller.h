#pragma once
// =============================================================================
// balance_controller.h -- pure logic, NO Arduino dependency.
//
// CONTROL LAW
// -----------
// The reaction wheel produces a torque on the frame that is (Newton's third
// law) equal and opposite to the torque the motor applies to the wheel, i.e.
// proportional to the wheel's ANGULAR ACCELERATION, not its angular speed or
// position. So we cannot just map tilt angle -> PWM duty; instead we compute
// a desired wheel angular ACCELERATION (torque) command:
//
//     a = controlSign * ( Kp*theta + Kd*theta_dot + Kw*omega_w + Ki*integral(theta) )
//
// and integrate that into a wheel speed command:
//
//     omega_cmd += a * dt         (clamped to +-maxWheelSpeed)
//
// which is what actually gets handed to the motor driver as a speed/voltage
// setpoint (see motor.h). omega_cmd is the state variable that carries the
// wheel's momentum budget forward between control steps.
//
// SIGN CONVENTION (must be verified against real hardware -- see
// firmware/README.md "tuning procedure" and config.h DEFAULT_CONTROL_SIGN):
//   theta      : rad, 0 = upright, sign/axis chosen in config.h (IMU_*).
//   theta_dot  : rad/s, gyro rate about the same axis, same sign convention.
//   omega_w    : rad/s, wheel speed signed by the CURRENTLY COMMANDED motor
//                direction (not independently sensed direction).
//   We DEFINE "positive" wheel-command direction such that, with positive
//   Kp, a positive theta (frame falling toward +theta) commands the wheel to
//   accelerate in the +omega direction, and by the reaction-torque pairing
//   this is assumed to push the frame back toward theta=0. Whether that
//   assumption matches the real mechanism depends on: (a) which physical
//   direction was wired to DIR_CW_LEVEL="positive" in config.h, and (b)
//   which physical tilt direction was chosen as "positive theta" in
//   config.h's IMU_* axis selection. Both are independently unconfirmed on
//   this hardware. Rather than trying to derive the combined sign from two
//   unverified choices, config.h exposes a single DEFAULT_CONTROL_SIGN
//   (+1/-1) that flips the whole control law output. If the frame falls
//   *faster* once BALANCING is engaged (in a repeatable way, not just
//   "it never balances"), flip CONTROL_SIGN -- do not try to reverse the
//   individual axis/motor polarity constants for this.
//
// The Kw*omega_w term exists so the wheel doesn't just spin up compensating
// for small steady-state tilt/offset errors and saturate, uselessly. Its
// SIGN IS NOT ASSERTED HERE: for a reaction-wheel inverted pendulum the
// correct sign of the wheel-speed feedback gain is not intuitive (an LQR
// design for this plant can legitimately come out either way, and "slowing
// the wheel down" may require the controller to first lean the frame the
// other way to do it). The sign and magnitude of Kw should come from the
// simulation/LQR design -- see sim/README.md -- not from an assumption made
// here.
// =============================================================================
#include <cmath>
#include <cstdint>
#include "motor_logic.h"

struct ControllerGains {
  // TODO(sim): all placeholders below -- replace with values from the
  // Python simulation worker before real hardware tuning. See config.h.
  float kp = 0.0f;
  float kd = 0.0f;
  float kw = 0.0f;
  float ki = 0.0f;             // optional; leave 0 to disable the integral term entirely
  float controlSign = 1.0f;    // +1 or -1, see convention note above
};

struct ControllerLimits {
  float maxWheelSpeed = 300.0f;        // rad/s, SAFETY CLAMP on omega_cmd (<= fullScaleWheelSpeed)
  float fullScaleWheelSpeed = 300.0f;  // rad/s, wheel speed at duty fraction = 1.0 -- the SCALING
                                        // reference for toMotorFraction(), deliberately separate from
                                        // maxWheelSpeed (see config.h MOTOR_FULL_SCALE_WHEEL_RADPS).
  float maxAccelCmd = 4000.0f;         // rad/s^2, clamp on 'a' before integrating
  float integralClamp = 1.0f;          // rad*s, anti-windup clamp (only relevant if ki != 0)
  float deadbandFraction = 0.02f;      // fraction of fullScaleWheelSpeed treated as exactly zero
  float minDutyFraction = 0.0f;        // duty fraction at which the motor actually starts turning
                                        // (measure with motor_test); nonzero commands above the
                                        // deadband are mapped linearly into [minDutyFraction, 1]
};

class BalanceController {
public:
  BalanceController() {}

  void setGains(const ControllerGains &g) { gains_ = g; }
  const ControllerGains &gains() const { return gains_; }

  void setLimits(const ControllerLimits &l) { limits_ = l; }
  const ControllerLimits &limits() const { return limits_; }

  // Resets the integral term and the wheel speed command (call on entering
  // BALANCING, and on leaving it e.g. FALLEN).
  void reset() { integral_ = 0.0f; omegaCmd_ = 0.0f; }

  // theta: rad, upright = 0. thetaDot: rad/s. omegaWheel: rad/s, signed by
  // the currently commanded motor direction. dt: s.
  // Returns the wheel speed command omega_cmd (rad/s), clamped.
  float update(float theta, float thetaDot, float omegaWheel, float dt) {
    if (gains_.ki != 0.0f) {
      integral_ += theta * dt;
      if (integral_ > limits_.integralClamp) integral_ = limits_.integralClamp;
      if (integral_ < -limits_.integralClamp) integral_ = -limits_.integralClamp;
    }

    float a = gains_.kp * theta + gains_.kd * thetaDot + gains_.kw * omegaWheel + gains_.ki * integral_;
    a *= gains_.controlSign;

    if (a > limits_.maxAccelCmd) a = limits_.maxAccelCmd;
    if (a < -limits_.maxAccelCmd) a = -limits_.maxAccelCmd;

    omegaCmd_ += a * dt;
    if (omegaCmd_ > limits_.maxWheelSpeed) omegaCmd_ = limits_.maxWheelSpeed;
    if (omegaCmd_ < -limits_.maxWheelSpeed) omegaCmd_ = -limits_.maxWheelSpeed;
    return omegaCmd_;
  }

  float omegaCmd() const { return omegaCmd_; }
  float integral() const { return integral_; }

  // Maps the internal signed wheel speed command to a signed duty fraction
  // [-1,1] for the motor driver. Scaling uses fullScaleWheelSpeed (duty=1.0
  // <-> the wheel speed the motor can actually reach at full duty), NOT
  // maxWheelSpeed (which is just a lower safety clamp on omega_cmd and would
  // under-scale the mapping if used here). Below the deadband the fraction
  // is exactly zero; above it, magnitude is mapped linearly into
  // [minDutyFraction, 1] so small commands aren't lost below the motor's
  // real starting duty.
  float toMotorFraction() const {
    float raw = (limits_.fullScaleWheelSpeed > 0.0f) ? (omegaCmd_ / limits_.fullScaleWheelSpeed) : 0.0f;
    if (raw > 1.0f) raw = 1.0f;
    if (raw < -1.0f) raw = -1.0f;

    float mag = std::fabs(raw);
    if (mag < limits_.deadbandFraction) return 0.0f;

    float mappedMag = limits_.minDutyFraction + (1.0f - limits_.minDutyFraction) * mag;
    if (mappedMag > 1.0f) mappedMag = 1.0f;
    return (raw < 0.0f) ? -mappedMag : mappedMag;
  }

private:
  ControllerGains gains_;
  ControllerLimits limits_;
  float integral_ = 0.0f;
  float omegaCmd_ = 0.0f;
};

// ---------------------------------------------------------------------------
// Self-test (no Arduino dependency): sanity-checks the sign convention,
// deadband mapping, wheel-speed clamping, duty-fraction speed scaling, and
// the motor direction-flip safety rule described above. Returns true if all
// checks pass; on failure
// *failMsg (if non-null) is set to a short static string describing which
// check failed. Intended to be invoked from the sketch's `selftest` serial
// command.
// ---------------------------------------------------------------------------
inline bool balanceControllerSelfTest(const char **failMsg = nullptr) {
  auto fail = [&](const char *m) -> bool { if (failMsg) *failMsg = m; return false; };

  // 1) Positive theta with controlSign=+1 and positive Kp must command the
  //    wheel to move in the +direction (the assumed restoring direction per
  //    the documented convention above).
  {
    BalanceController c;
    ControllerGains g; g.kp = 10.0f; g.controlSign = 1.0f;
    ControllerLimits l; l.maxWheelSpeed = 100.0f; l.maxAccelCmd = 10000.0f;
    c.setGains(g); c.setLimits(l);
    float cmd = c.update(0.1f, 0.0f, 0.0f, 0.01f);
    if (!(cmd > 0.0f)) return fail("positive theta did not produce positive wheel-speed command");
  }

  // 2) controlSign = -1 flips the sign of the output.
  {
    BalanceController c;
    ControllerGains g; g.kp = 10.0f; g.controlSign = -1.0f;
    ControllerLimits l; l.maxWheelSpeed = 100.0f; l.maxAccelCmd = 10000.0f;
    c.setGains(g); c.setLimits(l);
    float cmd = c.update(0.1f, 0.0f, 0.0f, 0.01f);
    if (!(cmd < 0.0f)) return fail("controlSign=-1 did not flip the output sign");
  }

  // 3) Kw*omega_w arithmetic: this only checks that the term is included
  //    with the correct sign/magnitude in the sum -- it does NOT assert
  //    which sign Kw *should* be for this plant (that comes from the
  //    sim/LQR design, see sim/README.md; the sign is not intuitive for a
  //    reaction-wheel pendulum). With theta = theta_dot = 0 and a given
  //    (arbitrary, here negative) Kw, Kw*omega_w must show up in 'a' with
  //    the expected sign: Kw<0 and omega_w>0 => contribution < 0.
  {
    BalanceController c;
    ControllerGains g; g.kp = 0; g.kd = 0; g.ki = 0; g.kw = -0.5f; g.controlSign = 1.0f;
    ControllerLimits l; l.maxWheelSpeed = 1000.0f; l.fullScaleWheelSpeed = 1000.0f; l.maxAccelCmd = 10000.0f;
    c.setGains(g); c.setLimits(l);
    float cmd = c.update(0.0f, 0.0f, 50.0f, 0.01f);
    if (!(cmd < 0.0f)) return fail("Kw*omega_w term arithmetic incorrect (expected negative contribution for Kw<0, omega_w>0)");
  }

  // 4) omega_cmd clamps to +-maxWheelSpeed under sustained large input.
  {
    BalanceController c;
    ControllerGains g; g.kp = 1000.0f;
    ControllerLimits l; l.maxWheelSpeed = 50.0f; l.maxAccelCmd = 1.0e6f;
    c.setGains(g); c.setLimits(l);
    float cmd = 0.0f;
    for (int i = 0; i < 1000; ++i) cmd = c.update(1.0f, 0.0f, 0.0f, 0.01f);
    if (!(cmd <= l.maxWheelSpeed + 1e-3f)) return fail("wheel-speed command failed to clamp to maxWheelSpeed");
  }

  // 5) Deadband: a small command well inside the deadband maps to an exact
  //    zero fraction; a larger one does not.
  {
    BalanceController c;
    ControllerGains g; g.kp = 400.0f;
    ControllerLimits l; l.maxWheelSpeed = 100.0f; l.fullScaleWheelSpeed = 100.0f; l.maxAccelCmd = 1.0e6f; l.deadbandFraction = 0.05f;
    c.setGains(g); c.setLimits(l);
    c.update(0.001f, 0.0f, 0.0f, 0.01f); // omega_cmd = 400*0.001*0.01 = 0.004 rad/s -> frac 0.00004
    if (c.toMotorFraction() != 0.0f) return fail("small command inside deadband did not map to exact zero fraction");

    BalanceController c2;
    c2.setGains(g); c2.setLimits(l);
    for (int i = 0; i < 50; ++i) c2.update(0.1f, 0.0f, 0.0f, 0.01f); // drives omega_cmd well above threshold
    if (c2.toMotorFraction() == 0.0f) return fail("command well outside deadband incorrectly mapped to zero fraction");
  }

  // 6) Speed scaling (fullScaleWheelSpeed / minDutyFraction): toMotorFraction
  //    must scale by fullScaleWheelSpeed (duty=1 <-> the motor's actual
  //    full-duty wheel speed), NOT by the (generally lower/different) safety
  //    clamp maxWheelSpeed, and must map the magnitude linearly into
  //    [minDutyFraction, 1] above the deadband.
  {
    BalanceController c;
    ControllerGains g; g.kp = 10000.0f; g.controlSign = 1.0f;
    ControllerLimits l;
    l.fullScaleWheelSpeed = 200.0f;   // duty=1.0 <-> 200 rad/s at the wheel
    l.maxWheelSpeed = 200.0f;         // clamp not binding for this check
    l.maxAccelCmd = 1.0e7f;
    l.deadbandFraction = 0.02f;
    l.minDutyFraction = 0.1f;
    c.setGains(g); c.setLimits(l);
    c.update(1.0f, 0.0f, 0.0f, 0.01f); // omega_cmd = 10000*1*0.01 = 100 rad/s = half of fullScaleWheelSpeed
    float frac = c.toMotorFraction();
    float expected = 0.1f + 0.9f * 0.5f; // minDuty + (1-minDuty)*0.5 = 0.55
    if (std::fabs(frac - expected) > 1e-3f) return fail("speed scaling (fullScaleWheelSpeed/minDutyFraction mapping) incorrect");
  }

  // 7) Motor direction-flip safety (motor_logic.h): flipping direction while
  //    duty is still nonzero must be deferred; only once duty has reached
  //    zero is the direction pin allowed to change, and never in the same
  //    step as applying nonzero duty in the new direction.
  {
    MotorState st;
    st.dirSign = 1;
    st.appliedFraction = 0.5f; // motor currently running forward at 50%

    MotorDecision d1 = decideMotorStep(st, -0.3f, 0.02f);
    if (d1.changeDir) return fail("direction flip was not deferred while duty was still nonzero");
    if (d1.dutyFraction != 0.0f) return fail("duty was not forced to zero before a deferred direction flip");

    MotorDecision d2 = decideMotorStep(st, -0.3f, 0.02f);
    if (!d2.changeDir || d2.newDirSign != -1) return fail("direction flip did not occur once duty reached zero");
    if (d2.dutyFraction <= 0.0f) return fail("duty was not applied after the direction flip completed");
  }

  return true;
}
