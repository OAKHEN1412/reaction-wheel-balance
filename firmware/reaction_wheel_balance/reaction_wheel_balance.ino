// =============================================================================
// Reaction Wheel Balance -- main firmware
//
// State machine: CALIBRATING -> WAIT_UPRIGHT -> BALANCING <-> FALLEN
//                                     ^--------------------------|
// plus an experimental JUMP_UP mode (see config.h ENABLE_JUMP_UP).
//
// See balance_controller.h for the control law and sign convention, and
// firmware/README.md for the bring-up / tuning procedure and serial command
// reference.
// =============================================================================
#include <Arduino.h>
#include <EEPROM.h>

#include "config.h"
#include "complementary_filter.h"
#include "balance_controller.h"
#include "wheel_speed_estimator.h"
#include "jump_controller.h"
#include "motor.h"
#include "fg_tach.h"
#include "imu.h"

namespace {

const float kDeg2Rad = PI / 180.0f;
const float kRad2Deg = 180.0f / PI;

enum class SystemState { CALIBRATING, WAIT_UPRIGHT, BALANCING, FALLEN, JUMP_UP };

SystemState state = SystemState::CALIBRATING;
JumpController jumpController;
JumpRestGate jumpRestGate;
uint32_t lastImuOkMs = 0, jumpStartPulses = 0;
uint32_t jumpLastPulses = 0, jumpLastPulseMs = 0;
float jumpSpinRpm = JUMP_SPIN_RPM;
uint16_t jumpKickMs = JUMP_KICK_MS;
float jumpCaptureDeg = JUMP_CAPTURE_DEG;
float jumpTargetRateDps = JUMP_TARGET_RATE_DPS;

ComplementaryFilter tiltFilter(0.98f);
BalanceController controller;
WheelSpeedEstimator wheelEst(MOTOR_TAU_S);

float uprightOffsetRad = 0.0f;
bool telemetryEnabled = false;
bool runEnabled = true;
uint32_t imuFailCount = 0;
bool imuFaultReported = false;
uint32_t loopOverrunCount = 0;

bool uprightHoldActive = false;
uint32_t uprightHoldStartMs = 0;

uint32_t fallenBrakeUntilMs = 0;

float lastThetaDeg = 0.0f;
float lastThetaDotDps = 0.0f;
float lastWheelRpmSigned = 0.0f;
float lastCmdRpm = 0.0f;

String serialBuf;

const char *stateName(SystemState s) {
  switch (s) {
    case SystemState::CALIBRATING: return "CALIBRATING";
    case SystemState::WAIT_UPRIGHT: return "WAIT_UPRIGHT";
    case SystemState::BALANCING: return "BALANCING";
    case SystemState::FALLEN: return "FALLEN";
    case SystemState::JUMP_UP: return "JUMP_UP";
  }
  return "?";
}

// Persisted settings. The magic word marks a valid record; a blank/foreign
// EEPROM (all 0xFF) falls back to the config.h defaults.
struct StoredPrefs {
  uint16_t magic;
  float kp, kd, kw, ki, sign, offset;
};

void loadPrefs() {
  StoredPrefs sp;
  EEPROM.get(EEPROM_ADDR, sp);
  ControllerGains g;
  if (sp.magic == EEPROM_MAGIC) {
    g.kp = sp.kp;
    g.kd = sp.kd;
    g.kw = sp.kw;
    g.ki = sp.ki;
    g.controlSign = sp.sign;
    uprightOffsetRad = sp.offset;
  } else {
    g.kp = DEFAULT_KP;
    g.kd = DEFAULT_KD;
    g.kw = DEFAULT_KW;
    g.ki = DEFAULT_KI;
    g.controlSign = DEFAULT_CONTROL_SIGN;
    uprightOffsetRad = 0.0f;
  }
  controller.setGains(g);
}

void savePrefs() {
  const ControllerGains &g = controller.gains();
  StoredPrefs sp = {EEPROM_MAGIC, g.kp, g.kd, g.kw, g.ki, g.controlSign, uprightOffsetRad};
  EEPROM.put(EEPROM_ADDR, sp); // put() only rewrites bytes that changed
}

// AVR printf has no %f, so floats go through Serial.print(value, digits).
void printKV(const __FlashStringHelper *key, float value, uint8_t digits) {
  Serial.print(key);
  Serial.println(value, digits);
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  kp [v]     get/set proportional gain"));
  Serial.println(F("  kd [v]     get/set derivative (tilt-rate) gain"));
  Serial.println(F("  kw [v]     get/set wheel-speed feedback gain"));
  Serial.println(F("  ki [v]     get/set integral gain (0 = disabled)"));
  Serial.println(F("  sign [v]   get/set overall control sign (+1/-1)"));
  Serial.println(F("  zero       set current filtered angle as upright offset"));
  Serial.println(F("  save       store gains + offset to EEPROM"));
  Serial.println(F("  start      enable balancing (re-arms WAIT_UPRIGHT gate)"));
  Serial.println(F("  stop       disable balancing, brake+coast motor"));
  Serial.println(F("  jump [rpm] [kick_ms] [capture_deg] [rate_dps]  manual jump (rate = handover tilt rate)"));
  Serial.println(F("  jumpcfg    print jump parameters and rest gate"));
  Serial.println(F("  tel 0|1    telemetry CSV stream on/off"));
  Serial.println(F("  selftest   run pure-logic self-checks (balance_controller + wheel_speed_estimator)"));
  Serial.println(F("  get        print current gains/state"));
  Serial.println(F("  help       this message"));
}

void printGet() {
  const ControllerGains &g = controller.gains();
  Serial.print(F("state="));
  Serial.print(stateName(state));
  Serial.print(F(" kp="));
  Serial.print(g.kp, 4);
  Serial.print(F(" kd="));
  Serial.print(g.kd, 4);
  Serial.print(F(" kw="));
  Serial.print(g.kw, 4);
  Serial.print(F(" ki="));
  Serial.print(g.ki, 4);
  Serial.print(F(" sign="));
  Serial.print(g.controlSign, 0);
  Serial.print(F(" offset_deg="));
  Serial.print(uprightOffsetRad * kRad2Deg, 3);
  Serial.print(F(" tel="));
  Serial.print(telemetryEnabled ? 1 : 0);
  Serial.print(F(" run="));
  Serial.print(runEnabled ? 1 : 0);
  Serial.print(F(" loop_overruns="));
  Serial.println(loopOverrunCount);
}

void enterFallen() {
  state = SystemState::FALLEN;
  Motor::brake();
  fallenBrakeUntilMs = millis() + 200;
  controller.reset();
  uprightHoldActive = false;
}

void handleUprightHoldAndMaybeAdvance(float theta, SystemState nextState) {
  if (fabsf(theta) < UPRIGHT_HOLD_DEG * kDeg2Rad) {
    if (!uprightHoldActive) {
      uprightHoldActive = true;
      uprightHoldStartMs = millis();
    } else if ((uint32_t)(millis() - uprightHoldStartMs) >= (uint32_t)UPRIGHT_HOLD_MS) {
      state = nextState;
      controller.reset();
      uprightHoldActive = false;
    }
  } else {
    uprightHoldActive = false;
  }
}

void printJumpConfig() {
  printKV(F("jump_spin_rpm="), jumpSpinRpm, 1);
  printKV(F("jump_kick_ms="), jumpKickMs, 0);
  printKV(F("jump_capture_deg="), jumpCaptureDeg, 1);
  printKV(F("jump_handover_rate_dps="), jumpTargetRateDps, 1);
  printKV(F("jump_default_spin_rpm="), JUMP_SPIN_RPM, 1);
  printKV(F("jump_default_kick_ms="), JUMP_KICK_MS, 0);
  Serial.println(F("jump: manual only; max=550rpm/500ms; rest=10..22deg <=3dps <=10rpm for 500ms"));
  Serial.println(F("jump: spin timeout=1500ms total=2000ms abort=25deg; spin FG loss=100ms"));
}

void runJumpStep(float thetaRad, float rateRad, float fgWheelRpm, bool fgFresh) {
  uint32_t pulses = FgTach::getPulseCount();
  if (pulses != jumpLastPulses) { jumpLastPulses = pulses; jumpLastPulseMs = millis(); }
  bool verified = fgFresh && (uint32_t)(pulses-jumpStartPulses) >= 3 &&
                  (uint32_t)(millis()-jumpLastPulseMs) < 100;
  float duty = jumpController.step(millis(), thetaRad*kRad2Deg, rateRad*kRad2Deg, fgWheelRpm, verified);
  if (!runEnabled || jumpController.phase() == JumpController::Phase::ABORTED) {
    enterFallen();
    runEnabled = false; // failed manual attempt cannot automatically re-arm balancing
    lastCmdRpm = 0;
    Serial.print(F("jump: aborted ("));
    Serial.print(JumpController::reasonText(jumpController.abortReason()));
    Serial.print(F(") theta="));
    Serial.print(thetaRad * kRad2Deg, 2);
    Serial.print(F(" rate="));
    Serial.print(rateRad * kRad2Deg, 1);
    Serial.print(F(" fg_rpm="));
    Serial.println(fgWheelRpm, 1);
    Serial.println(F("jump: issue a fresh jump or start command to re-arm"));
  } else if (jumpController.phase() == JumpController::Phase::CAPTURED) {
    Motor::coast();
    lastCmdRpm = 0;
    controller.reset(); // preserve the signed wheel estimator through capture
    state = SystemState::BALANCING;
  } else {
    Motor::setSpeed(duty);
    lastCmdRpm = duty * MOTOR_FULL_SCALE_WHEEL_RADPS * (60.0f/(2.0f*PI));
  }
}

// Applies u*omega_fs (voltage mode) or legacy speed command. If
// ACTIVE_DECEL_BRAKE is enabled and this step is a large same-direction
// deceleration (the driver likely can't coast down that fast on its own),
// pulses the physical brake for this control step instead of just lowering
// duty. See config.h ACTIVE_DECEL_BRAKE and motor_test's 'coastdown'.
void applyMotorCommand(float omegaCmdNew, float omegaEstSigned) {
  if (ACTIVE_DECEL_BRAKE) {
    bool sameSign = (omegaCmdNew * omegaEstSigned) >= 0.0f;
    float decel = fabsf(omegaEstSigned) - fabsf(omegaCmdNew);
    if (sameSign && decel > DECEL_BRAKE_MARGIN_RADPS) {
      Motor::brake();
      return;
    }
  }
  Motor::setSpeed(controller.toMotorFraction());
}

void runControlStep(float dt) {
  ImuSample sample;
  bool ok = Imu::read(sample);
  if (!ok) {
    imuFailCount++;
    jumpRestGate.reset();
    if (state == SystemState::JUMP_UP) {
      enterFallen(); // no open-loop kick on stale tilt, even for one missed read
      runEnabled = false;
      lastCmdRpm = 0;
      Serial.println(F("jump: aborted (IMU read failed)"));
    }
    if (imuFailCount >= IMU_FAIL_LIMIT) {
      if (!imuFaultReported) {
        Serial.println(F("ERROR: IMU read failed repeatedly -- forcing FALLEN and braking motor."));
        imuFaultReported = true;
      }
      if (state != SystemState::FALLEN) enterFallen();
      Motor::brake();
    }
    return;
  }
  imuFailCount = 0;
  imuFaultReported = false;
  lastImuOkMs = millis();

  float accelAngle = IMU_ACCEL_ANGLE_SIGN * accelTiltAngle(sample.accel[IMU_ACCEL_AXIS_NUM], sample.accel[IMU_ACCEL_AXIS_DEN]);
  float gyroRateRadPerSec = IMU_GYRO_SIGN * sample.gyro[IMU_GYRO_AXIS] * kDeg2Rad;

  float filteredAngle = accelLooksLikeGravity(sample.accel[0], sample.accel[1], sample.accel[2], ACCEL_GRAVITY_TOL_G)
      ? tiltFilter.update(accelAngle, gyroRateRadPerSec, dt)
      : tiltFilter.updateGyroOnly(gyroRateRadPerSec, dt);
  float theta = filteredAngle - uprightOffsetRad;
  float thetaDot = gyroRateRadPerSec;

  float motorRpm = FgTach::getRpm();
  bool fgFresh = FgTach::isFresh();
  float wheelRpm = motorRpm / GEAR_RATIO;
  float wheelRadPerSecMagnitude = wheelRpm * (2.0f * PI / 60.0f);

  // Signed wheel speed estimate: NOT Motor::currentDirectionSign() (that
  // flips the instant a new direction is commanded, while the wheel is
  // still coasting the old way by inertia -- see wheel_speed_estimator.h).
  // Predict from PREVIOUS applied signed duty, including reversal blanks,
  // brake/coast and jump mode; requested controller output may differ.
  float omegaWheelSigned = wheelEst.update(Motor::currentDirectionSign() * Motor::currentDutyFraction() * MOTOR_FULL_SCALE_WHEEL_RADPS, wheelRadPerSecMagnitude, fgFresh, dt);

  lastThetaDeg = theta * kRad2Deg;
  lastThetaDotDps = thetaDot * kRad2Deg;
  lastWheelRpmSigned = omegaWheelSigned * (60.0f / (2.0f * PI));

  jumpRestGate.update(millis(), lastThetaDeg, lastThetaDotDps,
      fmaxf(fabsf(lastWheelRpmSigned), wheelRpm),
      state == SystemState::WAIT_UPRIGHT || state == SystemState::FALLEN);

  switch (state) {
    case SystemState::CALIBRATING:
      // Handled synchronously in setup(); nothing to do here.
      break;

    case SystemState::WAIT_UPRIGHT:
      Motor::coast();
      lastCmdRpm = 0.0f;
      handleUprightHoldAndMaybeAdvance(theta, SystemState::BALANCING);
      break;

    case SystemState::BALANCING: {
      if (!runEnabled) {
        Motor::coast();
        controller.reset();
        state = SystemState::WAIT_UPRIGHT;
        break;
      }
      if (fabsf(theta) > FALL_ANGLE_DEG * kDeg2Rad) {
        enterFallen();
        break;
      }
      // The frame's rest stop is at ~16-18 deg, inside FALL_ANGLE_DEG, so a
      // frame lying on its stop never tripped the check above and the motor
      // ran at full duty for ~8 s (hardware 2026-09-20). Also call it a fall if
      // the tilt stays beyond FALL_HOLD_DEG for FALL_HOLD_MS.
      {
        static uint32_t beyondSinceMs = 0;
        if (fabsf(theta) > FALL_HOLD_DEG * kDeg2Rad) {
          if (beyondSinceMs == 0) beyondSinceMs = millis() | 1;
          else if ((uint32_t)(millis() - beyondSinceMs) >= FALL_HOLD_MS) {
            beyondSinceMs = 0;
            enterFallen();
            break;
          }
        } else {
          beyondSinceMs = 0;
        }
      }
      float omegaCmd = controller.update(theta, thetaDot, omegaWheelSigned, dt);
      applyMotorCommand(omegaCmd, omegaWheelSigned);
      lastCmdRpm = omegaCmd * (60.0f / (2.0f * PI));
      break;
    }

    case SystemState::FALLEN:
      if (millis() < fallenBrakeUntilMs) {
        Motor::brake();
      } else {
        Motor::coast();
      }
      lastCmdRpm = 0.0f;
      handleUprightHoldAndMaybeAdvance(theta, SystemState::WAIT_UPRIGHT);
      break;

    case SystemState::JUMP_UP:
      if (!ENABLE_JUMP_UP) {
        state = SystemState::WAIT_UPRIGHT;
        break;
      }
      runJumpStep(theta, thetaDot, wheelRpm, fgFresh);
      break;
  }
}

void printTelemetry() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < (uint32_t)(1000 / TELEMETRY_RATE_HZ)) return;
  lastMs = now;
  // CSV: ms,state,theta_deg,theta_dot_dps,wheel_rpm,cmd_rpm,duty
  Serial.print(now);
  Serial.print(',');
  Serial.print(stateName(state));
  Serial.print(',');
  Serial.print(lastThetaDeg, 3);
  Serial.print(',');
  Serial.print(lastThetaDotDps, 3);
  Serial.print(',');
  Serial.print(lastWheelRpmSigned, 2);
  Serial.print(',');
  Serial.print(lastCmdRpm, 2);
  Serial.print(',');
  Serial.println(Motor::currentDutyFraction(), 3);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  int spaceIdx = line.indexOf(' ');
  String cmd = (spaceIdx < 0) ? line : line.substring(0, spaceIdx);
  String argStr = (spaceIdx < 0) ? String("") : line.substring(spaceIdx + 1);
  argStr.trim();
  bool hasArg = argStr.length() > 0;
  float argVal = hasArg ? argStr.toFloat() : 0.0f;

  if (state == SystemState::JUMP_UP && cmd != "stop" && cmd != "tel") {
    // Ignore other commands during the short jump. Long help/config replies can
    // block the AVR serial buffer and extend an open-loop full-voltage kick.
    return;
  }
  ControllerGains g = controller.gains();

  if (cmd == "kp") {
    if (hasArg) { g.kp = argVal; controller.setGains(g); }
    printKV(F("kp="), controller.gains().kp, 4);
  } else if (cmd == "kd") {
    if (hasArg) { g.kd = argVal; controller.setGains(g); }
    printKV(F("kd="), controller.gains().kd, 4);
  } else if (cmd == "kw") {
    if (hasArg) { g.kw = argVal; controller.setGains(g); }
    printKV(F("kw="), controller.gains().kw, 4);
  } else if (cmd == "ki") {
    if (hasArg) { g.ki = argVal; controller.setGains(g); }
    printKV(F("ki="), controller.gains().ki, 4);
  } else if (cmd == "sign") {
    if (hasArg) { g.controlSign = (argVal < 0) ? -1.0f : 1.0f; controller.setGains(g); }
    printKV(F("sign="), controller.gains().controlSign, 0);
  } else if (cmd == "zero") {
    uprightOffsetRad = tiltFilter.angle();
    Serial.print(F("zero: upright offset set to "));
    Serial.print(uprightOffsetRad * kRad2Deg, 3);
    Serial.println(F(" deg"));
  } else if (cmd == "save") {
    savePrefs();
    Serial.println(F("saved gains + offset to EEPROM"));
  } else if (cmd == "start") {
    runEnabled = true;
    if (state == SystemState::FALLEN || state == SystemState::JUMP_UP) state = SystemState::WAIT_UPRIGHT;
    uprightHoldActive = false;
    Serial.println(F("start: armed, waiting for upright hold"));
  } else if (cmd == "stop") {
    runEnabled = false;
    Motor::brake();
    Motor::coast();
    controller.reset();
    if (state != SystemState::CALIBRATING) state = SystemState::WAIT_UPRIGHT;
    uprightHoldActive = false;
    Serial.println(F("stop: motor disabled"));
  } else if (cmd == "jumpcfg") {
    printJumpConfig();
  } else if (cmd == "jump") {
    float rpm = JUMP_SPIN_RPM;
    uint16_t kick = JUMP_KICK_MS;
    float capture = JUMP_CAPTURE_DEG;
    float targetRate = jumpTargetRateDps;
    // Optional 3rd arg: capture angle (deg). Split it off before the 2-arg parser.
    String firstTwo = argStr;
    bool captureOk = true;
    {
      int sp1 = argStr.indexOf(' ');
      int sp2 = (sp1 < 0) ? -1 : argStr.indexOf(' ', sp1 + 1);
      int sp3 = (sp2 < 0) ? -1 : argStr.indexOf(' ', sp2 + 1);
      if (sp2 >= 0) {
        String capStr = (sp3 < 0) ? argStr.substring(sp2 + 1) : argStr.substring(sp2 + 1, sp3);
        capStr.trim();
        firstTwo = argStr.substring(0, sp2);
        char *end = nullptr;
        double v = strtod(capStr.c_str(), &end);
        captureOk = end != capStr.c_str() && *end == ' ' && isfinite(v) && v >= 3.0 && v <= 15.0;
        if (captureOk) capture = (float)v;
      }
      // Optional 4th arg: handover tilt rate (dps).
      if (sp3 >= 0) {
        String kStr = argStr.substring(sp3 + 1);
        kStr.trim();
        char *end = nullptr;
        double v = strtod(kStr.c_str(), &end);
        if (end != kStr.c_str() && *end == ' ' && isfinite(v) && v >= 30.0 && v <= 150.0) {
          targetRate = (float)v;
        } else {
          captureOk = false;
        }
      }
    }
    if (!ENABLE_JUMP_UP || !CONTROL_MODE_VOLTAGE) {
      Serial.println(F("jump: disabled or not in voltage mode"));
    } else if (!captureOk || !parseJumpArgs(firstTwo.c_str(), rpm, kick)) {
      Serial.println(F("jump: invalid args; 0<rpm<=550, 1<=kick_ms<=500, 3<=capture_deg<=15, 30<=rate_dps<=150"));
    } else if ((state != SystemState::WAIT_UPRIGHT && state != SystemState::FALLEN) ||
               imuFailCount || (uint32_t)(millis()-lastImuOkMs) > 20 || !jumpRestGate.ready(millis())) {
      Serial.println(F("jump: requires fresh IMU, stationary frame at 10..22deg and stopped wheel for 500ms"));
    } else if (controller.gains().controlSign != 1.0f) {
      Serial.println(F("jump: requires verified control sign +1"));
    } else {
      jumpSpinRpm = rpm; jumpKickMs = kick;
      jumpStartPulses = FgTach::getPulseCount();
      jumpLastPulses = jumpStartPulses;
      jumpLastPulseMs = millis();
      jumpCaptureDeg = capture;
      jumpTargetRateDps = targetRate;
      jumpController.setTargetRateDps(targetRate);
      jumpController.begin(millis(), lastThetaDeg, rpm, kick, capture);
      jumpRestGate.reset();
      uprightHoldActive = false;
      controller.reset();
      runEnabled = true; // this explicit command also authorizes balance capture
      state = SystemState::JUMP_UP;
      Serial.println(F("jump: manual SPINUP then reverse-voltage KICK"));
    }
  } else if (cmd == "tel") {
    telemetryEnabled = hasArg ? (argVal != 0.0f) : !telemetryEnabled;
    Serial.println(telemetryEnabled ? F("telemetry on") : F("telemetry off"));
  } else if (cmd == "selftest") {
    const char *msg = nullptr;
    bool passed = balanceControllerSelfTest(&msg);
    if (passed) {
      Serial.println(F("selftest: balance_controller PASS"));
    } else {
      Serial.print(F("selftest: balance_controller FAIL ("));
      Serial.print(msg ? msg : "unknown");
      Serial.println(')');
    }
    msg = nullptr;
    bool passed2 = wheelSpeedEstimatorSelfTest(&msg);
    if (passed2) {
      Serial.println(F("selftest: wheel_speed_estimator PASS"));
    } else {
      Serial.print(F("selftest: wheel_speed_estimator FAIL ("));
      Serial.print(msg ? msg : "unknown");
      Serial.println(')');
    }
    Serial.println(jumpControllerSelfTest() ? F("selftest: jump_controller PASS") : F("selftest: jump_controller FAIL"));
  } else if (cmd == "get") {
    printGet();
  } else if (cmd == "help") {
    printHelp();
  } else {
    Serial.println(F("unknown command, try 'help'"));
  }
}

void pollSerial() {
  static bool discardLine = false;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (!discardLine && serialBuf.length() > 0) {
        handleCommand(serialBuf);
      }
      serialBuf = "";
      discardLine = false;
    } else if (!discardLine) {
      serialBuf += c;
      if (serialBuf.length() > 63) {
        serialBuf = "";
        discardLine = true; // never interpret an overlong line's tail as "jump"
      }
    }
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  // On ESP32-C3 with USB-CDC, Serial writes can block indefinitely (stalling
  // the 500Hz control loop) if no host is reading -- e.g. telemetry left on
  // with nobody attached to the serial monitor. Disable the TX blocking
  // timeout so writes drop instead of blocking.
  Serial.setTxTimeoutMs(0);
#endif
  delay(200);
  Serial.println(F("Reaction Wheel Balance firmware starting..."));

  Motor::begin(); // leaves motor braked
  FgTach::begin(PIN_MOTOR_FG);

  loadPrefs();
  ControllerLimits limits;
  limits.voltageMode = CONTROL_MODE_VOLTAGE;
  limits.motorTauS = MOTOR_TAU_S;
  limits.maxWheelSpeed = MAX_WHEEL_SPEED_RADPS;
  limits.fullScaleWheelSpeed = MOTOR_FULL_SCALE_WHEEL_RADPS;
  limits.minDutyFraction = MOTOR_MIN_DUTY_FRACTION;
  limits.maxAccelCmd = MAX_ACCEL_CMD_RADPS2;
  limits.integralClamp = INTEGRAL_CLAMP;
  limits.deadbandFraction = MOTOR_DEADBAND_FRACTION;
  controller.setLimits(limits);

  bool imuOk = Imu::begin();
  if (!imuOk) {
    Serial.println(F("WARNING: IMU did not respond as expected at begin() -- check wiring/address."));
  }
  Serial.print(F("IMU WHO_AM_I=0x"));
  Serial.println(Imu::lastWhoAmI(), HEX);

  Serial.println(F("Calibrating gyro bias -- keep the frame still..."));
  Imu::calibrateGyroBias(GYRO_CAL_DURATION_MS);
  Serial.println(F("Calibration done."));

  state = SystemState::WAIT_UPRIGHT;
  Serial.println(F("WAIT_UPRIGHT: hold the frame upright to begin balancing."));
  Serial.println(F("Type 'help' for the serial command list."));
}

void loop() {
  static uint32_t lastMicros = 0;
  static bool initialized = false;
  if (!initialized) {
    lastMicros = micros();
    initialized = true;
  }

  uint32_t now = micros();
  uint32_t elapsed = (uint32_t)(now - lastMicros);
  if (elapsed >= LOOP_PERIOD_US) {
    if (elapsed > 2UL * LOOP_PERIOD_US) loopOverrunCount++;
    if (elapsed > 10UL * LOOP_PERIOD_US) {
      // Fell far behind (e.g. a blocked/slow Serial write) -- resync to now
      // instead of bursting through many catch-up control steps back to
      // back on stale sensor data.
      lastMicros = now;
    } else {
      lastMicros += LOOP_PERIOD_US;
    }
    runControlStep(LOOP_PERIOD_US / 1000000.0f);
    if (telemetryEnabled) printTelemetry();
  }

  pollSerial();
}
