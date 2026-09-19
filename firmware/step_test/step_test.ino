// =============================================================================
// step_test.ino -- measures the wheel's open-loop speed response to a duty step,
// to get the two numbers the balance controller depends on most:
//   * steady wheel rpm vs duty  -> real MOTOR_MAX_RPM (duty -> speed scaling)
//   * time to 63% of steady     -> real MOTOR_TAU_S / sim tau_m
// Added 2026-09-19 after the first balance attempt oscillated with growing
// amplitude while the measured wheel speed lagged far behind omega_cmd.
//
// Setup: frame laid down or held firmly so it can't move; wheel free to spin.
// Self-contained; constants mirror firmware/reaction_wheel_balance/config.h.
//
// SAFETY: 12V OFF while uploading/resetting (PWM is inverted). Turn 12V on only
// after "READY" is printed.
//
// Serial @115200, newline. Commands:
//   s <duty>   step 0 -> duty (0-1023) for 2 s, then coast 1.5 s; logs + summary
//   help
// =============================================================================
#include <Arduino.h>

// --- mirror of config.h --------------------------------------------------------
static const uint8_t PIN_PWM = 11;
static const uint8_t PIN_DIR = 7;
static const uint8_t PIN_BRAKE = 6;
static const uint8_t PIN_FG = 2;
static const bool PWM_INVERT = true;
static const uint8_t DIR_CW_LEVEL = LOW;
static const uint8_t BRAKE_ACTIVE_LEVEL = LOW;
static const float FG_PULSES_PER_WHEEL_REV = 70.0f;
// -------------------------------------------------------------------------------

static const int PWM_MAX_DUTY = 1023;
static const uint16_t STEP_MS = 2000;
static const uint16_t COAST_MS = 1500;
static const uint16_t LOG_MS = 10;
static const uint16_t WINDOW_MS = 50;   // speed = pulses over the last 50 ms (5 log slots)

volatile uint32_t pulseCount = 0;
void onFg() { pulseCount++; }

void pwmWrite(int duty) {
  duty = constrain(duty, 0, PWM_MAX_DUTY);
  OCR1A = PWM_INVERT ? (PWM_MAX_DUTY - duty) : duty;
}

void setupPwm() {
  pinMode(PIN_PWM, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  pwmWrite(0);
}

uint32_t readCount() {
  noInterrupts();
  uint32_t c = pulseCount;
  interrupts();
  return c;
}

// Logs wheel rpm every LOG_MS for durationMs; returns samples in rpmOut.
// Speed is from a sliding WINDOW_MS pulse count, so it lags ~25 ms -- the
// summary corrects t63 for that.
static const uint8_t HIST = WINDOW_MS / LOG_MS;
uint16_t runPhase(uint32_t t0, uint16_t durationMs, float *rpmOut, uint16_t maxOut, uint16_t idx0) {
  uint32_t hist[HIST];
  uint32_t c = readCount();
  for (uint8_t i = 0; i < HIST; i++) hist[i] = c;
  uint8_t h = 0;
  uint16_t n = 0;
  uint32_t start = millis();
  uint32_t next = start + LOG_MS;
  while (millis() - start < durationMs) {
    while ((int32_t)(millis() - next) < 0) {}
    next += LOG_MS;
    uint32_t now = readCount();
    uint32_t pulses = now - hist[h];
    hist[h] = now;
    h = (h + 1) % HIST;
    float rpm = pulses / FG_PULSES_PER_WHEEL_REV * (60000.0f / WINDOW_MS);
    if (idx0 + n < maxOut) rpmOut[idx0 + n] = rpm;
    n++;
    Serial.print(millis() - t0);
    Serial.print(',');
    Serial.println(rpm, 1);
  }
  return n;
}

static const uint16_t MAX_SAMPLES = (STEP_MS + COAST_MS) / LOG_MS + 10;
float rpmLog[MAX_SAMPLES];

void runStep(int duty) {
  duty = constrain(duty, 0, 800);
  Serial.print(F("step: duty="));
  Serial.println(duty);
  Serial.println(F("t_ms,wheel_rpm"));

  digitalWrite(PIN_DIR, DIR_CW_LEVEL);
  digitalWrite(PIN_BRAKE, !BRAKE_ACTIVE_LEVEL);
  delay(5);
  uint32_t t0 = millis();
  pwmWrite(duty);
  uint16_t nStep = runPhase(t0, STEP_MS, rpmLog, MAX_SAMPLES, 0);
  pwmWrite(0);  // coast (brake released) -- shows how fast it can slow down on its own
  uint16_t nCoast = runPhase(t0, COAST_MS, rpmLog, MAX_SAMPLES, nStep);
  digitalWrite(PIN_BRAKE, BRAKE_ACTIVE_LEVEL);
  delay(800);
  digitalWrite(PIN_BRAKE, !BRAKE_ACTIVE_LEVEL);

  // steady = mean of the last 500 ms of the step
  uint16_t k = 500 / LOG_MS;
  float steady = 0;
  for (uint16_t i = nStep - k; i < nStep; i++) steady += rpmLog[i];
  steady /= k;
  int t63 = -1, tCoast37 = -1;
  for (uint16_t i = 0; i < nStep; i++) {
    if (rpmLog[i] >= 0.632f * steady) { t63 = (i + 1) * LOG_MS - WINDOW_MS / 2; break; }
  }
  for (uint16_t i = nStep; i < nStep + nCoast; i++) {
    if (rpmLog[i] <= 0.368f * steady) { tCoast37 = (i - nStep + 1) * LOG_MS - WINDOW_MS / 2; break; }
  }
  Serial.print(F("RESULT duty="));
  Serial.print(duty);
  Serial.print(F(" frac="));
  Serial.print(duty / 1023.0f, 3);
  Serial.print(F(" steady_wheel_rpm="));
  Serial.print(steady, 1);
  Serial.print(F(" t63_ms="));
  Serial.print(t63);
  Serial.print(F(" coast_t37_ms="));
  Serial.println(tCoast37);
}

void setup() {
  setupPwm();  // first: inverted driver runs flat out while the pin is LOW
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_BRAKE, BRAKE_ACTIVE_LEVEL);
  pinMode(PIN_FG, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FG), onFg, FALLING);

  Serial.begin(115200);
  delay(300);
  Serial.println(F("step_test: wheel speed step response"));
  Serial.println(F("commands: s <duty 0-800>"));
  Serial.println(F("READY -- lay the frame down / hold it, switch 12V on, send 's 300'"));
  digitalWrite(PIN_BRAKE, !BRAKE_ACTIVE_LEVEL);
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.startsWith("s ")) runStep(line.substring(2).toInt());
  else Serial.println(F("commands: s <duty 0-800>"));
}
