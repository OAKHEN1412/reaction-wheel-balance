// =============================================================================
// motor_test.ino -- interactive BLDC-3640 bring-up tool for the Arduino Mega
// 2560 (the project controller since 2026-09-19). The driver's DIR/BRAKE inputs
// are 5V logic with strong (~1k) pull-ups, which the Mega's 5V pins drive
// directly. (The old ESP32-C3 version is in firmware/legacy_esp32/.)
//
// Findings on the real motor (2026-09-19), wire colour -> function:
//   red   +12V            black  GND
//   blue  PWM speed       INVERTED: pin HIGH = stopped, starts at ~10% duty
//   yellow FG             open-collector speed pulses
//   white DIR             HIGH/floating = CCW, LOW = CW (seen from wheel face);
//                         can change while powered (motor stopped first)
//   green BRAKE           HIGH/floating = run, LOW = brake/stop
//
// Wiring (Mega):
//   D11 -> blue (PWM, Timer1 OC1A, ~15.6 kHz 10-bit)
//   D7  -> white (DIR)      D6 -> green (BRAKE)
//   D2  <- yellow (FG, INT0, internal pull-up to 5V)
//   GND -> common GND (battery -, motor black)
//
// SAFETY: switch the 12V supply OFF before uploading/resetting -- during reset
// the PWM pin floats and the driver may run the motor.
//
// Serial @115200 (newline). Commands:
//   duty <0-1023>  speed, 0 = stop (inversion handled here)
//   dir <0|1>      raw level on DIR (0 = LOW = CW, 1 = HIGH = CCW)
//   brake <0|1>    raw level on BRAKE (0 = LOW = brake, 1 = HIGH = run)
//   stop           duty 0
//   count          print + reset FG pulse count (turn wheel 1 rev by hand)
//   resetcount     reset FG pulse count
//   status         print state (also auto every 500 ms)
//   help
// =============================================================================
#include <Arduino.h>

static const uint8_t PIN_PWM = 11;   // OC1A
static const uint8_t PIN_DIR = 7;
static const uint8_t PIN_BRAKE = 6;
static const uint8_t PIN_FG = 2;     // INT0

static const int PWM_MAX_DUTY = 1023;
static const bool PWM_INVERT = true;

volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseMicros = 0;
volatile uint32_t lastPeriodUs = 0;

int currentDuty = 0;
int currentDirLevel = 1;    // CCW, same as the floating default
int currentBrakeLevel = 1;  // brake released

void onFgPulse() {
  uint32_t now = micros();
  lastPeriodUs = now - lastPulseMicros;
  lastPulseMicros = now;
  pulseCount++;
}

void pwmWrite(int duty) {
  duty = constrain(duty, 0, PWM_MAX_DUTY);
  OCR1A = PWM_INVERT ? (PWM_MAX_DUTY - duty) : duty;
}

void setupPwm() {
  // Timer1 fast PWM, 10-bit (TOP = 0x3FF), no prescaler -> 16 MHz / 1024 = 15.6 kHz
  pinMode(PIN_PWM, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  pwmWrite(0);
}

float readRpmRaw() {
  noInterrupts();
  uint32_t lastPulse = lastPulseMicros;
  uint32_t period = lastPeriodUs;
  interrupts();
  if (period == 0) return 0.0f;
  if ((uint32_t)(micros() - lastPulse) > 300000UL) return 0.0f;
  // Rough live check assuming 6 pulses/rev -- use 'count' for the real PPR.
  return (1000000.0f / (float)period) * 60.0f / 6.0f;
}

void printStatus() {
  noInterrupts();
  uint32_t count = pulseCount;
  interrupts();
  Serial.print(F("status: duty="));
  Serial.print(currentDuty);
  Serial.print(F("/1023 dir="));
  Serial.print(currentDirLevel);
  Serial.print(F(" brake="));
  Serial.print(currentBrakeLevel);
  Serial.print(F(" fg_pulses="));
  Serial.print(count);
  Serial.print(F(" fg_rpm(assuming 6ppr)="));
  Serial.println(readRpmRaw(), 1);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  int spaceIdx = line.indexOf(' ');
  String cmd = (spaceIdx < 0) ? line : line.substring(0, spaceIdx);
  int argVal = (spaceIdx < 0) ? 0 : line.substring(spaceIdx + 1).toInt();

  if (cmd == "duty") {
    currentDuty = constrain(argVal, 0, PWM_MAX_DUTY);
    pwmWrite(currentDuty);
    Serial.print(F("duty="));
    Serial.println(currentDuty);
  } else if (cmd == "dir") {
    currentDirLevel = argVal ? 1 : 0;
    digitalWrite(PIN_DIR, currentDirLevel);
    Serial.print(F("dir="));
    Serial.println(currentDirLevel ? F("1 (HIGH, expect CCW)") : F("0 (LOW, expect CW)"));
  } else if (cmd == "brake") {
    currentBrakeLevel = argVal ? 1 : 0;
    digitalWrite(PIN_BRAKE, currentBrakeLevel);
    Serial.print(F("brake="));
    Serial.println(currentBrakeLevel ? F("1 (HIGH, released)") : F("0 (LOW, braking)"));
  } else if (cmd == "stop") {
    currentDuty = 0;
    pwmWrite(0);
    Serial.println(F("duty=0"));
  } else if (cmd == "count") {
    noInterrupts();
    uint32_t c = pulseCount;
    pulseCount = 0;
    interrupts();
    Serial.print(F("FG pulses since last reset: "));
    Serial.println(c);
  } else if (cmd == "resetcount") {
    noInterrupts();
    pulseCount = 0;
    interrupts();
    Serial.println(F("pulse count reset"));
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "help") {
    Serial.println(F("duty <0-1023> | dir <0|1> | brake <0|1> | stop | count | resetcount | status | help"));
  } else {
    Serial.println(F("unknown command, try 'help'"));
  }
}

String serialBuf;

void setup() {
  setupPwm();  // first, so the motor is commanded to stop as early as possible
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_DIR, currentDirLevel);
  digitalWrite(PIN_BRAKE, currentBrakeLevel);
  pinMode(PIN_FG, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FG), onFgPulse, FALLING);

  Serial.begin(115200);
  Serial.println(F("motor_test: BLDC-3640 bring-up tool. Type 'help'."));
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) {
        handleCommand(serialBuf);
        serialBuf = "";
      }
    } else if (serialBuf.length() < 63) {
      serialBuf += c;
    }
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    printStatus();
  }
}
