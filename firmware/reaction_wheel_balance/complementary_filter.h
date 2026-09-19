#pragma once
// =============================================================================
// complementary_filter.h -- pure logic, NO Arduino dependency.
// Fuses an accelerometer-derived tilt angle (drifts less, noisy at short
// timescales) with a gyro rate integration (smooth short-term, drifts long
// term) into a single tilt angle estimate.
// =============================================================================
#include <math.h>

class ComplementaryFilter {
public:
  explicit ComplementaryFilter(float alpha = 0.98f) : alpha_(alpha), angle_(0.0f) {}

  // alpha in [0,1]: weight given to the gyro-integrated angle each update.
  // Higher alpha = trust gyro more / accel less (less noise, more drift).
  void setAlpha(float alpha) { alpha_ = alpha; }
  float alpha() const { return alpha_; }

  void reset(float angle = 0.0f) { angle_ = angle; }
  float angle() const { return angle_; }

  // accelAngle: angle computed from accelerometer via atan2 (rad).
  // gyroRate: angular rate about the tilt axis (rad/s), bias already removed.
  // dt: timestep (s).
  float update(float accelAngle, float gyroRate, float dt) {
    float gyroAngle = angle_ + gyroRate * dt;
    angle_ = alpha_ * gyroAngle + (1.0f - alpha_) * accelAngle;
    return angle_;
  }

private:
  float alpha_;
  float angle_;
};

// Angle (rad) of the tilt axis from two accelerometer readings that span the
// tilt plane, via atan2(num, den). When upright, den should read close to
// +1 g and num close to 0 g.
inline float accelTiltAngle(float num, float den) {
  return atan2f(num, den);
}
