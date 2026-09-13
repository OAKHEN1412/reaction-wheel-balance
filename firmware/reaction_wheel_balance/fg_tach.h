#pragma once
// =============================================================================
// fg_tach.h -- Arduino-dependent FG (speed pulse) tachometer reader.
// Counts open-collector pulses from the motor's FG output via an ISR and
// derives unsigned motor-shaft RPM. FG_PULSES_PER_REV and GEAR_RATIO (in
// config.h) convert this to wheel RPM elsewhere; this module only knows
// about the motor shaft.
// =============================================================================
#include <stdint.h>

namespace FgTach {

// Attaches the interrupt on the given pin (INPUT_PULLUP, FALLING edge).
void begin(uint8_t pin);

// Call periodically (e.g. once per control loop iteration) to let the RPM
// estimate time out to zero if no pulses have arrived recently.
void update();

// Unsigned motor-shaft RPM, 0 if no pulses within the timeout window.
float getRpm();

// True if the last FG pulse arrived within the timeout window, i.e.
// getRpm()'s magnitude can be trusted as "live" rather than "timed out to
// zero". Used by wheel_speed_estimator.h to decide whether to trust FG
// magnitude or fall back to the lag model's own estimate.
bool isFresh();

// Raw cumulative pulse count since begin()/resetPulseCount() -- useful for
// motor_test to discover FG_PULSES_PER_REV by hand-counting one revolution.
uint32_t getPulseCount();
void resetPulseCount();

} // namespace FgTach
