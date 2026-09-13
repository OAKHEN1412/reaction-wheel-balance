#pragma once
// =============================================================================
// motor_logic.h -- pure logic, NO Arduino dependency.
// Decides how a desired signed speed fraction [-1,1] should be turned into a
// direction sign + PWM duty fraction, enforcing the rule: the F/R direction
// pin is never changed while nonzero duty is (or was, last step) applied.
// motor.cpp (Arduino-dependent) calls decideMotorStep() and only then touches
// real pins/PWM, so this decision logic can be exercised standalone by
// balanceControllerSelfTest() in balance_controller.h.
// =============================================================================
#include <cstdint>
#include <cmath>

struct MotorState {
  int8_t dirSign = 0;            // +1, -1, or 0 (undecided / stopped)
  float appliedFraction = 0.0f;  // last applied |duty| fraction, 0..1
};

struct MotorDecision {
  bool changeDir = false;    // true if the direction pin should be written this step
  int8_t newDirSign = 0;     // direction to write if changeDir is true
  float dutyFraction = 0.0f; // |duty| fraction to apply this step, 0..1
};

// state is updated in place to reflect the decision taken.
// desiredFraction: signed target speed fraction, will be clamped to [-1,1].
// deadbandFraction: |desiredFraction| below this collapses to zero duty.
inline MotorDecision decideMotorStep(MotorState &state, float desiredFraction, float deadbandFraction) {
  if (desiredFraction > 1.0f) desiredFraction = 1.0f;
  if (desiredFraction < -1.0f) desiredFraction = -1.0f;

  MotorDecision d;
  float mag = std::fabs(desiredFraction);

  if (mag < deadbandFraction) {
    // Inside deadband: force duty to zero. Direction pin is left untouched
    // (it's safe either way since duty is/will-be zero).
    d.changeDir = false;
    d.dutyFraction = 0.0f;
    state.appliedFraction = 0.0f;
    return d;
  }

  int8_t desiredDir = (desiredFraction > 0.0f) ? 1 : -1;

  if (state.dirSign == 0) {
    // No direction latched yet (e.g. just after boot/coast): safe to set it now.
    d.changeDir = true;
    d.newDirSign = desiredDir;
    state.dirSign = desiredDir;
  } else if (desiredDir != state.dirSign) {
    // A direction flip is requested.
    if (state.appliedFraction > 0.0f) {
      // Not safe: duty is still nonzero from the previous step. Force it to
      // zero THIS step and defer the flip to a later call once duty has
      // actually reached zero.
      d.changeDir = false;
      d.dutyFraction = 0.0f;
      state.appliedFraction = 0.0f;
      return d;
    }
    // Duty already zero: safe to flip now.
    d.changeDir = true;
    d.newDirSign = desiredDir;
    state.dirSign = desiredDir;
  }

  d.dutyFraction = mag;
  state.appliedFraction = mag;
  return d;
}
