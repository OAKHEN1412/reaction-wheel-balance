#include "motor.h"
#include <Arduino.h>
#include "config.h"
#include "motor_logic.h"

namespace Motor {

namespace {
MotorState state;
bool dirPinInitialized = false;

void writeDutyRaw(float fraction) {
  uint32_t duty = (uint32_t)(fraction * PWM_MAX_DUTY + 0.5f);
  if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
  uint32_t applied = PWM_INVERT ? (PWM_MAX_DUTY - duty) : duty;
  ledcWrite(PIN_MOTOR_PWM, applied);
}

void writeDirPin(int8_t dirSign) {
  digitalWrite(PIN_MOTOR_DIR, (dirSign > 0) ? DIR_CW_LEVEL : !DIR_CW_LEVEL);
}
} // namespace

void begin() {
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  pinMode(PIN_MOTOR_BRAKE, OUTPUT);
  ledcAttach(PIN_MOTOR_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);

  state = MotorState();
  dirPinInitialized = false;
  writeDutyRaw(0.0f);
  brake(); // safe default state at boot
}

void setSpeed(float signedFraction) {
  MotorDecision d = decideMotorStep(state, signedFraction, MOTOR_DEADBAND_FRACTION);

  // setSpeed() is the only place that is supposed to actively drive the
  // wheel, so it must release the brake whenever it is about to apply
  // nonzero duty -- otherwise any caller that reaches setSpeed() without a
  // preceding coast() (e.g. `jump` issued during FALLEN's brief brake
  // window, or BALANCING resuming right after a FALLEN recovery) would be
  // fighting an engaged brake.
  if (d.dutyFraction > 0.0f) {
    digitalWrite(PIN_MOTOR_BRAKE, !BRAKE_ACTIVE_LEVEL);
  }

  if (d.changeDir) {
    writeDirPin(d.newDirSign);
    dirPinInitialized = true;
  }
  writeDutyRaw(d.dutyFraction);
}

void brake() {
  digitalWrite(PIN_MOTOR_BRAKE, BRAKE_ACTIVE_LEVEL);
  writeDutyRaw(0.0f);
  state.appliedFraction = 0.0f;
}

void coast() {
  digitalWrite(PIN_MOTOR_BRAKE, !BRAKE_ACTIVE_LEVEL);
  writeDutyRaw(0.0f);
  state.appliedFraction = 0.0f;
}

int8_t currentDirectionSign() {
  return state.dirSign;
}

float currentDutyFraction() {
  return state.appliedFraction;
}

} // namespace Motor
