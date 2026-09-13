#pragma once
// =============================================================================
// motor.h -- Arduino-dependent motor driver wrapper. Applies the invert /
// polarity flags from config.h and the direction-flip safety rule from
// motor_logic.h. See motor_logic.h for the pure decision logic (testable
// without Arduino) that this module drives.
// =============================================================================
#include <stdint.h>

namespace Motor {

// Initializes pins and PWM channel. Leaves the motor braked (safe state).
void begin();

// Commands a signed speed fraction in [-1, 1]. Positive = the direction
// wired to DIR_CW_LEVEL in config.h (see balance_controller.h for how this
// sign is used by the control law). Internally enforces: the direction pin
// is never changed while nonzero duty is/was being applied -- flips ramp
// through a zero-duty step first.
void setSpeed(float signedFraction);

// Engages the brake and sets duty to zero immediately.
void brake();

// Releases the brake and sets duty to zero (idle/freewheel).
void coast();

// Currently latched direction: +1, -1, or 0 if none latched yet (e.g. after coast()).
int8_t currentDirectionSign();

// Last applied |duty| fraction, 0..1 (for telemetry).
float currentDutyFraction();

} // namespace Motor
