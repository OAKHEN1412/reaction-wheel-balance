#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

namespace Imu {

namespace {
// AD0 selects the address: tied to GND it is 0x68, pulled high it is 0x69.
// The wiring ties AD0 to GND, but a broken or missing AD0 wire leaves the pin
// floating, and then the part answers on either address -- and can change its
// mind between resets. Probing both at begin() turns that from a dead machine
// into a warning, without hiding the fault.
uint8_t kAddr = 0x68;
constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegConfig = 0x1A;     // DLPF_CFG
constexpr uint8_t kRegGyroConfig = 0x1B; // FS_SEL
constexpr uint8_t kRegAccelConfig = 0x1C; // AFS_SEL
constexpr uint8_t kRegAccelXoutH = 0x3B;

// AFS_SEL=1 -> +-4g -> 8192 LSB/g. FS_SEL=1 -> +-500dps -> 65.5 LSB/(deg/s).
constexpr float kAccelLsbPerG = 8192.0f;
constexpr float kGyroLsbPerDps = 65.5f;

uint8_t whoAmIValue = 0;
float gyroBiasDps[3] = {0.0f, 0.0f, 0.0f};

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(kAddr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start, keep bus held
  uint8_t got = Wire.requestFrom((int)kAddr, (int)len, (int)true);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool readRawAccelGyro(int16_t accelRaw[3], int16_t gyroRaw[3]) {
  uint8_t buf[14];
  if (!readRegisters(kRegAccelXoutH, buf, 14)) return false;
  accelRaw[0] = (int16_t)((buf[0] << 8) | buf[1]);
  accelRaw[1] = (int16_t)((buf[2] << 8) | buf[3]);
  accelRaw[2] = (int16_t)((buf[4] << 8) | buf[5]);
  // buf[6],buf[7] = temperature, unused
  gyroRaw[0] = (int16_t)((buf[8] << 8) | buf[9]);
  gyroRaw[1] = (int16_t)((buf[10] << 8) | buf[11]);
  gyroRaw[2] = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}
} // namespace

bool begin() {
  Wire.begin(); // Mega: SDA = D20, SCL = D21
  // 200 kHz rather than 400 kHz: slower edges tolerate the motor's switching
  // noise better, and a 14-byte read still costs well under 1 ms.
  Wire.setClock(200000);
  // AVR Wire can otherwise hang forever on a stuck bus (e.g. a loose SDA/SCL
  // wire) and freeze the control loop with the motor still running.
  Wire.setWireTimeout(3000 /* us */, true);

  // Pick whichever address answers, preferring the wired-for 0x68.
  static const uint8_t kCandidates[2] = {0x68, 0x69};
  for (uint8_t i = 0; i < 2; i++) {
    Wire.beginTransmission(kCandidates[i]);
    if (Wire.endTransmission() == 0) {
      kAddr = kCandidates[i];
      break;
    }
  }
  if (kAddr != 0x68) {
    Serial.println(F("WARNING: IMU answered on 0x69 -- AD0 is not tied to GND. Fix the wiring."));
  }

  // Wake the device (PWR_MGMT_1 default has SLEEP bit set).
  bool ok = writeRegister(kRegPwrMgmt1, 0x00);
  delay(10);

  uint8_t who = 0;
  Wire.beginTransmission(kAddr);
  Wire.write(kRegWhoAmI);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)kAddr, 1, (int)true) == 1) {
    who = Wire.read();
  }
  whoAmIValue = who;

  ok &= writeRegister(kRegConfig, 0x03);       // DLPF_CFG=3 -> accel ~44Hz, gyro ~42Hz
  ok &= writeRegister(kRegGyroConfig, 0x08);   // FS_SEL=1 -> +-500 dps
  ok &= writeRegister(kRegAccelConfig, 0x08);  // AFS_SEL=1 -> +-4 g

  gyroBiasDps[0] = gyroBiasDps[1] = gyroBiasDps[2] = 0.0f;

  // WHO_AM_I is 0x68 on genuine MPU-6050 parts; some clones/variants differ,
  // so this is reported but not treated as a hard failure on its own.
  return ok;
}

bool read(ImuSample &out) {
  int16_t accelRaw[3], gyroRaw[3];
  // One immediate retry: the motor's current spikes corrupt the odd I2C
  // transfer (hardware 2026-09-21 -- repeated failures aborted good jumps).
  // A retry costs ~0.4 ms at 200 kHz, well inside the 2 ms control period.
  if (!readRawAccelGyro(accelRaw, gyroRaw) && !readRawAccelGyro(accelRaw, gyroRaw)) {
    out.ok = false;
    return false;
  }
  for (int i = 0; i < 3; i++) {
    out.accel[i] = (float)accelRaw[i] / kAccelLsbPerG;
    out.gyro[i] = (float)gyroRaw[i] / kGyroLsbPerDps - gyroBiasDps[i];
  }
  out.ok = true;
  return true;
}

void calibrateGyroBias(uint16_t durationMs) {
  double sum[3] = {0.0, 0.0, 0.0};
  uint32_t count = 0;
  uint32_t startMs = millis();
  while ((uint32_t)(millis() - startMs) < durationMs) {
    int16_t accelRaw[3], gyroRaw[3];
    if (readRawAccelGyro(accelRaw, gyroRaw)) {
      sum[0] += (double)gyroRaw[0] / kGyroLsbPerDps;
      sum[1] += (double)gyroRaw[1] / kGyroLsbPerDps;
      sum[2] += (double)gyroRaw[2] / kGyroLsbPerDps;
      count++;
    }
    delay(2);
  }
  if (count > 0) {
    gyroBiasDps[0] = (float)(sum[0] / count);
    gyroBiasDps[1] = (float)(sum[1] / count);
    gyroBiasDps[2] = (float)(sum[2] / count);
  }
}

void getGyroBias(float biasOut[3]) {
  biasOut[0] = gyroBiasDps[0];
  biasOut[1] = gyroBiasDps[1];
  biasOut[2] = gyroBiasDps[2];
}

uint8_t lastWhoAmI() {
  return whoAmIValue;
}

} // namespace Imu
