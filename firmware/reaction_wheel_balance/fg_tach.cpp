#include "fg_tach.h"
#include <Arduino.h>
#include "config.h"

namespace FgTach {

namespace {
constexpr uint32_t kTimeoutUs = 300000UL; // no pulse for this long -> report 0 rpm

volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseMicros = 0;
volatile uint32_t lastPeriodUs = 0;

void onFgPulse() {
  uint32_t now = micros();
  uint32_t period = now - lastPulseMicros; // wraps correctly even across micros() overflow
  // Glitch filter: the first balance run (2026-09-19) saw single spurious
  // edges ~25-35 us apart around DIR flips, read as 30,000+ wheel rpm; in
  // voltage mode that drove duty to 100% the wrong way for ~300 ms. A real
  // pulse can't arrive faster than FG_MIN_PERIOD_US, so drop it entirely.
  if (period < FG_MIN_PERIOD_US) return;
  lastPeriodUs = period;
  lastPulseMicros = now;
  pulseCount++;
}
} // namespace

void begin(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  pulseCount = 0;
  lastPulseMicros = micros();
  lastPeriodUs = 0;
  attachInterrupt(digitalPinToInterrupt(pin), onFgPulse, FALLING);
}

void update() {
  // Nothing to do continuously; timeout handling lives in getRpm() so it
  // always reflects "now", not just "as of the last update() call".
}

float getRpm() {
  uint32_t nowMicros = micros();
  uint32_t lastPulse, period;
  noInterrupts();
  lastPulse = lastPulseMicros;
  period = lastPeriodUs;
  interrupts();

  if (period == 0) return 0.0f;
  uint32_t sinceLast = nowMicros - lastPulse;
  if (sinceLast > kTimeoutUs) return 0.0f;
  // No pulse for longer than the last period means the wheel has slowed at
  // least that much -- decay instead of holding a stale (possibly high) value.
  if (sinceLast > period) period = sinceLast;

  float revsPerSec = (1000000.0f / (float)period) / FG_PULSES_PER_REV;
  return revsPerSec * 60.0f;
}

bool isFresh() {
  uint32_t nowMicros = micros();
  uint32_t lastPulse, period;
  noInterrupts();
  lastPulse = lastPulseMicros;
  period = lastPeriodUs;
  interrupts();

  if (period == 0) return false;
  // FG drops out at high wheel speed (hardware 2026-09-20: estimate fell from
  // -450 to -4 rpm while the wheel was still spinning, turning the voltage
  // command into a brake). Once no pulse has come for 4x the last period
  // (min 15 ms) the magnitude is not trusted and WheelSpeedEstimator falls
  // back to its applied-duty model.
  uint32_t sinceLast = nowMicros - lastPulse;
  uint32_t staleUs = period * 4UL;
  if (staleUs < 15000UL) staleUs = 15000UL;
  return sinceLast <= staleUs && sinceLast <= kTimeoutUs;
}

uint32_t getPulseCount() {
  noInterrupts();
  uint32_t c = pulseCount;
  interrupts();
  return c;
}

void resetPulseCount() {
  noInterrupts();
  pulseCount = 0;
  interrupts();
}

} // namespace FgTach
