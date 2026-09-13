#pragma once
// =============================================================================
// imu.h -- Arduino-dependent MPU-6050 raw register access via Wire. No
// Adafruit/third-party IMU library required. Axis SELECTION (which of these
// raw axes forms the tilt angle) is a config.h concern, handled by the
// caller -- this module only reports all 6 axes in physical units.
// =============================================================================
#include <stdint.h>

// accel[]/gyro[] are indexed 0=X, 1=Y, 2=Z, matching the sensor's own axis
// labeling (see the MPU-6050 datasheet / imu_test.ino for which is which on
// your specific module orientation).
struct ImuSample {
  float accel[3]; // g
  float gyro[3];  // deg/s, gyro bias already subtracted (see calibrateGyroBias)
  bool ok;         // false if the I2C transaction failed
};

namespace Imu {

// Wakes the sensor, configures DLPF (~44 Hz), gyro range (+-500 dps), accel
// range (+-4 g). Returns false if the I2C bus/device did not respond as
// expected (WHO_AM_I mismatch or transaction failure); the caller may still
// choose to retry.
bool begin();

// Reads and converts one sample. Returns false (and leaves out.ok = false)
// on I2C failure; physical values are only valid when the return is true.
bool read(ImuSample &out);

// Blocking: averages gyro readings for durationMs (device must be held
// still) and stores the result as the bias subtracted by read() from then
// on. Call once at startup after begin().
void calibrateGyroBias(uint16_t durationMs);

// Returns the currently stored gyro bias (deg/s), e.g. for telemetry/debug.
void getGyroBias(float biasOut[3]);

// WHO_AM_I register value from the last begin() call, for diagnostics
// (imu_test prints this; genuine MPU-6050 parts read 0x68).
uint8_t lastWhoAmI();

} // namespace Imu
