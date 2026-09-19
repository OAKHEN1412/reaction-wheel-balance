// =============================================================================
// imu_test.ino -- standalone discovery tool for MPU-6050 axis/sign selection.
//
// Purpose: this firmware does NOT know which physical direction the frame
// tilts corresponds to which IMU axis, because the module's mounting
// orientation is unknown. Flash this sketch, open the serial monitor at
// 115200, and tilt the frame BY HAND about the intended balance axis while
// watching the printed candidate angles. Pick whichever (num,den) axis pair
// and sign gives: near 0 deg when upright, and a clean, monotonic,
// correctly-signed reading as you tilt it the way you want to call
// "positive". Then copy those choices into
// firmware/reaction_wheel_balance/config.h:
//   IMU_ACCEL_AXIS_NUM, IMU_ACCEL_AXIS_DEN, IMU_ACCEL_ANGLE_SIGN,
//   IMU_GYRO_AXIS, IMU_GYRO_SIGN
//
// This sketch is intentionally self-contained (no shared headers with the
// main firmware) so it can be flashed before any config.h values are known.
// =============================================================================
#include <Arduino.h>
#include <Wire.h>

// ---- pins (must match the main firmware's config.h) ----
static const int PIN_SDA = 8;
static const int PIN_SCL = 9;

static const uint8_t MPU_ADDR = 0x68;
static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

static const float ACCEL_LSB_PER_G = 8192.0f;   // AFS_SEL=1 (+-4g)
static const float GYRO_LSB_PER_DPS = 65.5f;    // FS_SEL=1 (+-500dps)

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)len, (int)true) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("imu_test: raw MPU-6050 axis/sign discovery"));

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  writeReg(REG_PWR_MGMT_1, 0x00); // wake
  delay(10);

  uint8_t who = 0;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)MPU_ADDR, 1, (int)true) == 1) {
    who = Wire.read();
  }
  Serial.printf("WHO_AM_I = 0x%02X (genuine MPU-6050 = 0x68; some clones differ)\n", who);

  writeReg(REG_CONFIG, 0x03);       // DLPF ~44Hz
  writeReg(REG_GYRO_CONFIG, 0x08);  // +-500 dps
  writeReg(REG_ACCEL_CONFIG, 0x08); // +-4 g

  Serial.println(F("Hold the frame UPRIGHT and still, then start tilting it by hand"));
  Serial.println(F("about the intended balance axis. Watch which angle column"));
  Serial.println(F("behaves the way you want (0 at upright, clean sign vs. direction)."));
  Serial.println();
  Serial.println(F("ax,ay,az(g)  gx,gy,gz(dps)  |  atan2(ay,az)  atan2(az,ay)  atan2(ax,az)  atan2(az,ax)  atan2(ax,ay)  atan2(ay,ax)  [deg]"));
}

void loop() {
  uint8_t buf[14];
  if (!readRegs(REG_ACCEL_XOUT_H, buf, 14)) {
    Serial.println(F("I2C read failed -- check wiring (SDA=GPIO8, SCL=GPIO9, addr 0x68)"));
    delay(200);
    return;
  }

  int16_t axr = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ayr = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t azr = (int16_t)((buf[4] << 8) | buf[5]);
  int16_t gxr = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t gyr = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t gzr = (int16_t)((buf[12] << 8) | buf[13]);

  float ax = axr / ACCEL_LSB_PER_G, ay = ayr / ACCEL_LSB_PER_G, az = azr / ACCEL_LSB_PER_G;
  float gx = gxr / GYRO_LSB_PER_DPS, gy = gyr / GYRO_LSB_PER_DPS, gz = gzr / GYRO_LSB_PER_DPS;

  float aYZ = atan2(ay, az) * 180.0f / PI;
  float aZY = atan2(az, ay) * 180.0f / PI;
  float aXZ = atan2(ax, az) * 180.0f / PI;
  float aZX = atan2(az, ax) * 180.0f / PI;
  float aXY = atan2(ax, ay) * 180.0f / PI;
  float aYX = atan2(ay, ax) * 180.0f / PI;

  Serial.printf("a=%.2f,%.2f,%.2f  g=%.1f,%.1f,%.1f  |  %.1f  %.1f  %.1f  %.1f  %.1f  %.1f\n",
                ax, ay, az, gx, gy, gz, aYZ, aZY, aXZ, aZX, aXY, aYX);

  delay(100);
}
