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
  OCR1A = (uint16_t)applied;
}

// Timer1 fast PWM, 10-bit (TOP = 0x3FF), no prescaler -> 16 MHz / 1024 =
// ~15.6 kHz on OC1A. Both chips run at 16 MHz so the frequency is identical;
// only the pin OC1A comes out on differs (D11 on the Mega, D9 on the 328P).
void setupPwmTimer() {
#if defined(__AVR_ATmega2560__)
  static_assert(PIN_MOTOR_PWM == 11, "Timer1 OC1A is D11 on the Mega 2560");
#else
  static_assert(PIN_MOTOR_PWM == 9, "Timer1 OC1A is D9 on the ATmega328P");
#endif
  static_assert(PWM_RESOLUTION_BITS == 10, "Timer1 is set up for 10-bit fast PWM");
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);
}

void writeDirPin(int8_t dirSign) {
  digitalWrite(PIN_MOTOR_DIR, (dirSign > 0) ? DIR_CW_LEVEL : !DIR_CW_LEVEL);
}
} // namespace

void begin() {
  // PWM first: with PWM_INVERT the driver runs flat out while the pin is LOW,
  // so get it to the "stopped" level as early as possible after reset.
  setupPwmTimer();
  writeDutyRaw(0.0f);
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  pinMode(PIN_MOTOR_BRAKE, OUTPUT);

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
