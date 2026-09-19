// =============================================================================
// sign_test.ino -- decides DEFAULT_CONTROL_SIGN in reaction_wheel_balance/config.h
// by measuring which way the FRAME kicks when the wheel is spun up.
//
// Physics: accelerating the wheel in +omega applies the opposite torque to the
// frame. The controller assumes (controlSign = +1) that a +theta tilt is fixed
// by accelerating the wheel in +omega, i.e. that a +omega spin-up pushes the
// frame toward NEGATIVE theta. So:
//   +omega pulse -> frame d(theta) < 0   => DEFAULT_CONTROL_SIGN = +1
//   +omega pulse -> frame d(theta) > 0   => DEFAULT_CONTROL_SIGN = -1
// d(theta) is integrated from the gyro only over the spin-up window, so the
// accelerometer's sensitivity to the kick doesn't matter.
//
// Setup: frame standing on its pivot edge, held LOOSELY near upright (fingertips,
// not a grip) so it can rock a little. Wheel must be free to spin.
//
// Self-contained on purpose (Arduino copies the sketch folder before building,
// so ../ includes break). Constants below MUST match
// firmware/reaction_wheel_balance/config.h -- update both if either changes.
//
// SAFETY: 12V OFF while uploading/resetting (PWM is inverted: a floating/LOW pin
// runs the motor at full speed). Turn 12V on only after "READY" is printed.
//
// Serial @115200, newline. Commands:
//   p [duty]   +omega pulse (default duty 300/1023), 250 ms spin-up, then brake
//   n [duty]   -omega pulse (should give the opposite d(theta) -- cross-check)
//   cal        re-measure gyro bias (hold still)
//   help
// =============================================================================
#include <Arduino.h>
#include <Wire.h>

// --- mirror of config.h --------------------------------------------------------
static const uint8_t PIN_PWM = 11;     // PIN_MOTOR_PWM (Timer1 OC1A)
static const uint8_t PIN_DIR = 7;      // PIN_MOTOR_DIR
static const uint8_t PIN_BRAKE = 6;    // PIN_MOTOR_BRAKE
static const bool PWM_INVERT = true;
static const uint8_t DIR_CW_LEVEL = LOW;      // +omega direction
static const uint8_t BRAKE_ACTIVE_LEVEL = LOW;
static const uint8_t IMU_ACCEL_AXIS_NUM = 1;
static const uint8_t IMU_ACCEL_AXIS_DEN = 2;
static const float IMU_ACCEL_ANGLE_SIGN = 1.0f;
static const uint8_t IMU_GYRO_AXIS = 0;
static const float IMU_GYRO_SIGN = 1.0f;
// -------------------------------------------------------------------------------

static const int PWM_MAX_DUTY = 1023;
static const int DEFAULT_PULSE_DUTY = 300;
static const int MAX_PULSE_DUTY = 600;
static const uint16_t SPINUP_MS = 250;
static const uint16_t BRAKE_MS = 1500;
static const uint16_t SAMPLE_US = 5000;   // 200 Hz

static const uint8_t MPU_ADDR = 0x68;
static const float GYRO_LSB_PER_DPS = 65.5f;   // +-500 dps

float gyroBias[3] = {0, 0, 0};

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

void brakeOn() {
  pwmWrite(0);
  digitalWrite(PIN_BRAKE, BRAKE_ACTIVE_LEVEL);
}

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readImu(float accel[3], float gyroDps[3]) {
  uint8_t buf[14];
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14, (int)true) != 14) return false;
  for (uint8_t i = 0; i < 14; i++) buf[i] = Wire.read();
  for (uint8_t i = 0; i < 3; i++) {
    accel[i] = (int16_t)((buf[2 * i] << 8) | buf[2 * i + 1]);
    gyroDps[i] = (int16_t)((buf[8 + 2 * i] << 8) | buf[9 + 2 * i]) / GYRO_LSB_PER_DPS;
  }
  return true;
}

float thetaDeg(const float accel[3]) {
  return IMU_ACCEL_ANGLE_SIGN * atan2(accel[IMU_ACCEL_AXIS_NUM], accel[IMU_ACCEL_AXIS_DEN]) * 180.0f / PI;
}

float thetaDotDps(const float gyroDps[3]) {
  return IMU_GYRO_SIGN * (gyroDps[IMU_GYRO_AXIS] - gyroBias[IMU_GYRO_AXIS]);
}

void calibrateGyro() {
  Serial.println(F("cal: hold still..."));
  float sum[3] = {0, 0, 0};
  float a[3], g[3];
  int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) {
    if (readImu(a, g)) {
      for (uint8_t i = 0; i < 3; i++) sum[i] += g[i];
      n++;
    }
    delay(5);
  }
  if (n == 0) {
    Serial.println(F("cal: IMU read failed"));
    return;
  }
  for (uint8_t i = 0; i < 3; i++) gyroBias[i] = sum[i] / n;
  Serial.print(F("cal: bias dps = "));
  Serial.print(gyroBias[0], 2); Serial.print(',');
  Serial.print(gyroBias[1], 2); Serial.print(',');
  Serial.println(gyroBias[2], 2);
}

// One pulse: record d(theta) from the gyro over the spin-up window only.
void runPulse(int8_t omegaSign, int duty) {
  duty = constrain(duty, 120, MAX_PULSE_DUTY);
  float a[3], g[3];
  if (!readImu(a, g)) {
    Serial.println(F("pulse: IMU read failed, aborted"));
    return;
  }
  float theta0 = thetaDeg(a);

  Serial.print(F("pulse: omega "));
  Serial.print(omegaSign > 0 ? '+' : '-');
  Serial.print(F(" duty="));
  Serial.print(duty);
  Serial.print(F(" theta0="));
  Serial.println(theta0, 1);
  Serial.println(F("t_ms,theta_dot_dps,dtheta_gyro_deg"));

  digitalWrite(PIN_DIR, omegaSign > 0 ? DIR_CW_LEVEL : !DIR_CW_LEVEL);
  delay(5);
  digitalWrite(PIN_BRAKE, !BRAKE_ACTIVE_LEVEL);
  pwmWrite(duty);

  float dtheta = 0.0f, peakRate = 0.0f;
  uint32_t start = micros(), last = start;
  while ((uint32_t)(micros() - start) < (uint32_t)SPINUP_MS * 1000UL) {
    while ((uint32_t)(micros() - last) < SAMPLE_US) {}
    uint32_t now = micros();
    float dt = (now - last) * 1e-6f;
    last = now;
    if (!readImu(a, g)) continue;
    float rate = thetaDotDps(g);
    dtheta += rate * dt;
    if (fabs(rate) > fabs(peakRate)) peakRate = rate;
    Serial.print((now - start) / 1000UL);
    Serial.print(',');
    Serial.print(rate, 1);
    Serial.print(',');
    Serial.println(dtheta, 2);
  }
  brakeOn();

  Serial.print(F("RESULT omega"));
  Serial.print(omegaSign > 0 ? '+' : '-');
  Serial.print(F(": dtheta="));
  Serial.print(dtheta, 2);
  Serial.print(F(" deg  peak_rate="));
  Serial.print(peakRate, 1);
  Serial.println(F(" dps"));
  if (fabs(dtheta) < 0.3f) {
    Serial.println(F("  -> too small to judge: hold the frame looser or raise duty (e.g. p 400)"));
  } else {
    // For a +omega pulse the expected (controlSign=+1) response is dtheta<0;
    // for a -omega pulse it is dtheta>0.
    bool matchesPlus = (omegaSign > 0) ? (dtheta < 0) : (dtheta > 0);
    Serial.print(F("  -> suggests DEFAULT_CONTROL_SIGN = "));
    Serial.println(matchesPlus ? F("+1") : F("-1"));
  }
  delay(BRAKE_MS);
}

void printHelp() {
  Serial.println(F("commands: p [duty] (+omega pulse), n [duty] (-omega pulse), cal, help"));
}

void setup() {
  // PWM first: inverted driver runs flat out while the pin is LOW.
  setupPwm();
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  brakeOn();

  Serial.begin(115200);
  delay(300);
  Serial.println(F("sign_test: which way does the frame kick on a +omega spin-up?"));

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  writeReg(0x6B, 0x00);  // wake
  delay(10);
  writeReg(0x1A, 0x03);  // DLPF ~44 Hz
  writeReg(0x1B, 0x08);  // +-500 dps
  writeReg(0x1C, 0x08);  // +-4 g

  calibrateGyro();
  printHelp();
  Serial.println(F("READY -- now switch 12V on, hold frame loosely upright, send 'p'"));
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  int arg = sp < 0 ? DEFAULT_PULSE_DUTY : line.substring(sp + 1).toInt();

  if (cmd == "p") runPulse(+1, arg);
  else if (cmd == "n") runPulse(-1, arg);
  else if (cmd == "cal") calibrateGyro();
  else printHelp();
}
