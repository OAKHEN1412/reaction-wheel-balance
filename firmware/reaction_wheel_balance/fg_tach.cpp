#include "fg_tach.h"
#include <Arduino.h>
#include "config.h"

namespace FgTach {

namespace {
constexpr uint32_t kTimeoutUs = 300000UL; // no pulse for this long -> report 0 rpm

volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseMicros = 0;
volatile uint32_t lastPeriodUs = 0;

void IRAM_ATTR onFgPulse() {
  uint32_t now = micros();
  uint32_t period = now - lastPulseMicros; // wraps correctly even across micros() overflow
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
  if ((uint32_t)(nowMicros - lastPulse) > kTimeoutUs) return 0.0f;

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
  return (uint32_t)(nowMicros - lastPulse) <= kTimeoutUs;
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
