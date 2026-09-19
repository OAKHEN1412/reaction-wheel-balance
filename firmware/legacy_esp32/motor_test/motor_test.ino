// =============================================================================
// motor_test.ino -- standalone interactive tool to discover BLDC-3640 driver
// polarity: PWM_INVERT, DIR_CW_LEVEL, BRAKE_ACTIVE_LEVEL, FG_PULSES_PER_REV.
//
// SAFETY: secure/clamp the wheel or keep it clear of anything it could hit
// before commanding nonzero duty -- direction/brake polarity is unknown at
// this point, so behaviour is unpredictable until you've characterized it.
//
// Serial @115200. Commands:
//   duty <0-1023>   motor speed, 0 = stop (PWM_INVERT below is applied, so
//                    this is NOT the raw pin duty when PWM_INVERT is true).
//   freq <Hz>        change PWM frequency (default 20000) -- try 1000 if the
//                    driver ignores 20 kHz.
//   dir <0|1>        raw level on PIN_DIR (note which physical rotation
//                    direction each level produces -- that tells you
//                    DIR_CW_LEVEL).
//   brake <0|1>      raw level on PIN_BRAKE (find which level actually
//                    stops/holds the shaft -- that's BRAKE_ACTIVE_LEVEL).
//   stop             sets duty to 0 (keeps dir/brake levels as-is).
//   count            print FG pulses since the last reset, then reset the
//                    counter -- rotate the wheel BY HAND exactly one full
//                    revolution, then run this to read off FG_PULSES_PER_REV.
//   resetcount       reset the FG pulse counter without printing.
//   status           print current dir/brake levels, duty, live FG rpm.
//   coastdown <duty> <brakeLevelForPhaseC>
//                    Spins up to <duty>, then measures how long the FG rpm
//                    takes to drop to 50% of its spun-up value under three
//                    conditions: (a) duty=0 coast, (b) duty halved, (c) the
//                    brake pin driven to <brakeLevelForPhaseC> (run this
//                    AFTER you already know which brake level stops the
//                    shaft, from the 'brake' command). Use the result to
//                    decide whether to enable ACTIVE_DECEL_BRAKE in the main
//                    firmware's config.h -- if (a)/(b) are much slower than
//                    (c), the driver can't brake actively via PWM alone.
//   help             this message.
//
// Live FG rpm + pulse count are also printed automatically every 500ms.
// =============================================================================
#include <Arduino.h>

// ---- pins (must match the main firmware's config.h) ----
static const int PIN_PWM = 1;
static const int PIN_DIR = 2;
static const int PIN_BRAKE = 3;
static const int PIN_FG = 4;

static const int PWM_FREQ_HZ = 20000;
static const int PWM_RES_BITS = 10;
static const int PWM_MAX_DUTY = (1 << PWM_RES_BITS) - 1;

// Measured on the real BLDC-3640 (2026-09-19): the speed input is INVERTED --
// raw pin HIGH (duty 1023) = stopped, raw LOW (duty 0) = full speed. With this
// true, every "duty" in this sketch means speed (0 = stop) and boot is safe.
static const bool PWM_INVERT = true;

volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseMicros = 0;
volatile uint32_t lastPeriodUs = 0;

void IRAM_ATTR onFgPulse() {
  uint32_t now = micros();
  lastPeriodUs = now - lastPulseMicros;
  lastPulseMicros = now;
  pulseCount++;
}

int currentDuty = 0;

void pwmWrite(int duty) {
  ledcWrite(PIN_PWM, PWM_INVERT ? (PWM_MAX_DUTY - duty) : duty);
}
int currentDirLevel = 0;
int currentBrakeLevel = 0;

float readRpmRaw() {
  uint32_t nowMicros = micros();
  uint32_t lastPulse, period;
  noInterrupts();
  lastPulse = lastPulseMicros;
  period = lastPeriodUs;
  interrupts();
  if (period == 0) return 0.0f;
  if ((uint32_t)(nowMicros - lastPulse) > 300000UL) return 0.0f;
  // Reports raw electrical pulse rate in "pulses/min" divided by a nominal
  // guess of 6 pulses/rev -- ONLY for a rough live sanity check; use the
  // 'count' command + hand-turning one revolution for the real PPR.
  float pulsesPerSec = 1000000.0f / (float)period;
  return (pulsesPerSec * 60.0f) / 6.0f;
}

void printStatus() {
  noInterrupts();
  uint32_t count = pulseCount;
  interrupts();
  Serial.printf("status: duty=%d/%d dir=%d brake=%d fg_pulses=%lu fg_rpm(assuming 6ppr)=%.1f\n",
                currentDuty, PWM_MAX_DUTY, currentDirLevel, currentBrakeLevel,
                (unsigned long)count, readRpmRaw());
}

// Spins up to testDuty (duty must already be safe to run -- direction is
// whatever was last set via 'dir'), waits to stabilize, and returns the
// resulting FG rpm reading as a decay baseline.
float coastdownSpinUpAndBaseline(int testDuty) {
  digitalWrite(PIN_BRAKE, currentBrakeLevel);
  pwmWrite(testDuty);
  delay(1500); // let speed stabilize
  return readRpmRaw();
}

// Polls FG rpm until it drops to <= baseline*0.5, or times out. Returns
// elapsed ms, or -1 on timeout.
long coastdownMeasureDecayMs(float baseline, uint32_t timeoutMs) {
  if (baseline <= 0.0f) return -1;
  float target = baseline * 0.5f;
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < timeoutMs) {
    if (readRpmRaw() <= target) return (long)(millis() - t0);
    delay(20);
  }
  return -1;
}

void runCoastdown(int testDuty, int brakeLevelC) {
  testDuty = constrain(testDuty, 1, PWM_MAX_DUTY);
  Serial.printf("coastdown: spin to duty=%d, testing decay-to-50%% under coast / half-duty / brake(level=%d)\n",
                testDuty, brakeLevelC);

  // (a) coast: duty -> 0
  float base = coastdownSpinUpAndBaseline(testDuty);
  Serial.printf("  baseline rpm(6ppr,est)=%.1f\n", base);
  pwmWrite(0);
  currentDuty = 0;
  long tCoast = coastdownMeasureDecayMs(base, 5000);
  Serial.printf("  A) coast (duty=0):            %s\n",
                tCoast < 0 ? "did not reach 50%% within 5s" : (String("t=") + tCoast + "ms").c_str());

  // (b) half duty
  base = coastdownSpinUpAndBaseline(testDuty);
  int halfDuty = testDuty / 2;
  pwmWrite(halfDuty);
  currentDuty = halfDuty;
  long tHalf = coastdownMeasureDecayMs(base, 5000);
  Serial.printf("  B) half duty (duty=%d):        %s\n", halfDuty,
                tHalf < 0 ? "did not reach 50%% within 5s" : (String("t=") + tHalf + "ms").c_str());

  // (c) brake
  base = coastdownSpinUpAndBaseline(testDuty);
  pwmWrite(0);
  currentDuty = 0;
  digitalWrite(PIN_BRAKE, brakeLevelC);
  currentBrakeLevel = brakeLevelC;
  long tBrake = coastdownMeasureDecayMs(base, 5000);
  Serial.printf("  C) brake (level=%d):            %s\n", brakeLevelC,
                tBrake < 0 ? "did not reach 50%% within 5s" : (String("t=") + tBrake + "ms").c_str());

  Serial.println(F("coastdown: done, motor left braked/stopped"));
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  int spaceIdx = line.indexOf(' ');
  String cmd = (spaceIdx < 0) ? line : line.substring(0, spaceIdx);
  String argStr = (spaceIdx < 0) ? String("") : line.substring(spaceIdx + 1);
  argStr.trim();
  int argVal = argStr.toInt();

  if (cmd == "duty") {
    currentDuty = constrain(argVal, 0, PWM_MAX_DUTY);
    pwmWrite(currentDuty);
    Serial.printf("duty=%d/%d\n", currentDuty, PWM_MAX_DUTY);
  } else if (cmd == "dir") {
    currentDirLevel = (argVal != 0) ? 1 : 0;
    digitalWrite(PIN_DIR, currentDirLevel);
    Serial.printf("dir=%d (observe rotation direction)\n", currentDirLevel);
  } else if (cmd == "brake") {
    currentBrakeLevel = (argVal != 0) ? 1 : 0;
    digitalWrite(PIN_BRAKE, currentBrakeLevel);
    Serial.printf("brake=%d (observe whether shaft stops/holds)\n", currentBrakeLevel);
  } else if (cmd == "freq") {
    // Some built-in drivers only accept low PWM frequencies (e.g. 1-5 kHz).
    int hz = constrain(argVal, 100, 40000);
    ledcDetach(PIN_PWM);
    ledcAttach(PIN_PWM, hz, PWM_RES_BITS);
    pwmWrite(currentDuty);
    Serial.printf("pwm freq=%d Hz (duty kept at %d)\n", hz, currentDuty);
  } else if (cmd == "od") {
    // Open-drain drive for 5V-logic driver inputs: "od <gpio> 0" pulls the
    // line LOW, "od <gpio> 1" RELEASES it (pin becomes input) so the driver's
    // own pull-up takes it to its logic HIGH. Only GPIO2 / GPIO3 allowed.
    int sp = argStr.indexOf(' ');
    int gpio = (sp < 0) ? -1 : argStr.substring(0, sp).toInt();
    int level = (sp < 0) ? 0 : argStr.substring(sp + 1).toInt();
    if (gpio != PIN_DIR && gpio != PIN_BRAKE) {
      Serial.println(F("od: gpio must be 2 or 3"));
      return;
    }
    if (level) {
      pinMode(gpio, INPUT);
    } else {
      pinMode(gpio, OUTPUT);
      digitalWrite(gpio, LOW);
    }
    Serial.printf("od gpio%d = %s\n", gpio, level ? "released (driver pull-up)" : "pulled LOW");
  } else if (cmd == "probe") {
    // Poor-man's multimeter: float GPIO1-4 as inputs with a weak pull-DOWN and
    // report what the motor driver is doing on each line. A line the powered
    // driver pulls up reads HIGH (clamped ~3.3V); an unpowered/floating line
    // reads LOW. Restores PWM/DIR/BRAKE outputs afterwards (duty forced to 0).
    pwmWrite(0);
    currentDuty = 0;
    detachInterrupt(digitalPinToInterrupt(PIN_FG));
    // PWM pin is NOT probed: floating it would read as "full speed" on an
    // inverted driver. It stays driven at the stop level.
    const int pins[3] = {PIN_DIR, PIN_BRAKE, PIN_FG};
    const char *names[3] = {"GPIO2(DIR)", "GPIO3(BRAKE)", "GPIO4(FG)"};
    for (int i = 0; i < 3; i++) pinMode(pins[i], INPUT_PULLDOWN);
    delay(20);
    for (int i = 0; i < 3; i++) {
      int mv = analogReadMilliVolts(pins[i]);
      Serial.printf("probe %-13s digital=%d  ~%d mV\n", names[i], digitalRead(pins[i]), mv);
    }
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_BRAKE, OUTPUT);
    digitalWrite(PIN_DIR, currentDirLevel);
    digitalWrite(PIN_BRAKE, currentBrakeLevel);
    pinMode(PIN_FG, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FG), onFgPulse, FALLING);
    Serial.println(F("probe done (duty=0)"));
  } else if (cmd == "stop") {
    currentDuty = 0;
    pwmWrite(0);
    Serial.println(F("duty=0"));
  } else if (cmd == "count") {
    noInterrupts();
    uint32_t c = pulseCount;
    pulseCount = 0;
    interrupts();
    Serial.printf("FG pulses since last reset: %lu  (this should equal FG_PULSES_PER_REV if you turned exactly one revolution by hand)\n", (unsigned long)c);
  } else if (cmd == "resetcount") {
    noInterrupts();
    pulseCount = 0;
    interrupts();
    Serial.println(F("pulse count reset"));
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "coastdown") {
    int sp = argStr.indexOf(' ');
    int testDuty = (sp < 0) ? argStr.toInt() : argStr.substring(0, sp).toInt();
    int brakeLevelC = (sp < 0) ? 1 : argStr.substring(sp + 1).toInt();
    runCoastdown(testDuty, brakeLevelC);
  } else if (cmd == "help") {
    Serial.println(F("duty <0-1023> | freq <Hz> | probe | dir <0|1> | brake <0|1> | stop | count | resetcount | status | coastdown <duty> <brakeLevelForPhaseC> | help"));
  } else {
    Serial.println(F("unknown command, try 'help'"));
  }
}

String serialBuf;

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) {
        handleCommand(serialBuf);
        serialBuf = "";
      }
    } else {
      serialBuf += c;
      if (serialBuf.length() > 63) serialBuf = "";
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("motor_test: BLDC polarity discovery tool"));
  Serial.println(F("Secure/clamp the wheel before commanding nonzero duty. Type 'help'."));

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  pinMode(PIN_FG, INPUT_PULLUP);
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_BRAKE, LOW);

  ledcAttach(PIN_PWM, PWM_FREQ_HZ, PWM_RES_BITS);
  pwmWrite(0);

  attachInterrupt(digitalPinToInterrupt(PIN_FG), onFgPulse, FALLING);
}

void loop() {
  pollSerial();

  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 500) {
    lastPrint = now;
    printStatus();
  }
}
