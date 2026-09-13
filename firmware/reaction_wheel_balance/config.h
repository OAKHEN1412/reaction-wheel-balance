#pragma once
// =============================================================================
// config.h -- all board wiring, calibration constants, and tunable defaults
// for the Reaction Wheel Balance firmware live here. Nothing else in the
// sketch should hardcode a pin number, gain, or physical-unit constant.
// =============================================================================

// ----------------------------------------------------------------------------
// Pins (fixed by hardware -- see project-brief.md)
// ----------------------------------------------------------------------------
#define PIN_I2C_SDA     8
#define PIN_I2C_SCL     9
#define PIN_IMU_INT     10   // optional; not currently used by the firmware logic

#define PIN_MOTOR_PWM    1   // speed command (LEDC PWM output)
#define PIN_MOTOR_DIR    2   // F/R direction level
#define PIN_MOTOR_BRAKE  3   // brake level
#define PIN_MOTOR_FG     4   // FG tach pulses, open-collector -> INPUT_PULLUP

// ----------------------------------------------------------------------------
// Motor driver polarity -- ALL UNCONFIRMED. Determine with firmware/motor_test
// and fill in real values here before flashing the main sketch.
// ----------------------------------------------------------------------------
// If true, the driver's PWM input is inverted, i.e. duty=0 -> full speed and
// duty=PWM_MAX_DUTY -> stopped. Some built-in ESC/driver boards behave this way.
#define PWM_INVERT              false

// Logic level written to PIN_MOTOR_DIR that makes the WHEEL spin in the
// direction we call "positive" throughout this firmware (see balance_controller.h
// for how that sign is used in the control law). Pick either HIGH or LOW here;
// motor_test will tell you which level corresponds to which physical rotation
// direction. The label "CW" is just a name -- what matters is that this
// firmware is internally consistent about it.
#define DIR_CW_LEVEL            HIGH

// Logic level on PIN_MOTOR_BRAKE that ENGAGES the brake.
#define BRAKE_ACTIVE_LEVEL      HIGH

// FG tach pulses per one mechanical revolution of the MOTOR shaft (not the
// wheel). Common for small sensorless BLDC modules is 6; verify by rotating
// the motor shaft by hand exactly one turn and counting pulses in motor_test.
#define FG_PULSES_PER_REV       6.0f

// Reduction ratio between motor shaft and wheel: GEAR_RATIO = (motor shaft
// revolutions) / (wheel revolutions). GEAR_RATIO = 1.0 if direct-drive.
#define GEAR_RATIO               2.5f   // MEASURE: DEFAULT_K* gains were designed for 2.5; sim says >= 3 is safer

// Motor shaft no-load speed at full duty (datasheet value for BLDC-3640,
// ideally re-measured empirically via FG in motor_test/imu_test once
// polarity is known, since real no-load speed can differ from datasheet
// under the actual supply voltage).
#define MOTOR_MAX_RPM            4000.0f

// Duty fraction (0..1) at which the motor actually starts turning (stiction
// / minimum drive threshold) -- measure with motor_test by slowly raising
// duty from 0 and noting where FG pulses begin. Nonzero commanded fractions
// are mapped linearly into [MOTOR_MIN_DUTY_FRACTION, 1] so that small
// commands don't get lost below the motor's real starting duty. Leave at
// 0.0 until measured.
#define MOTOR_MIN_DUTY_FRACTION  0.0f

// Wheel angular speed (rad/s) that corresponds to duty fraction = 1.0 (i.e.
// the motor's no-load speed reflected through the gear/belt ratio to the
// wheel). This is the SCALING reference for mapping omega_cmd -> duty
// fraction -- it is deliberately a different constant from
// MAX_WHEEL_SPEED_RADPS below, which is a lower safety CLAMP on omega_cmd.
#define MOTOR_FULL_SCALE_WHEEL_RADPS ((MOTOR_MAX_RPM / GEAR_RATIO) * (2.0f * 3.14159265358979323846f / 60.0f))

// ----------------------------------------------------------------------------
// PWM configuration
// ----------------------------------------------------------------------------
#define PWM_FREQ_HZ              20000
#define PWM_RESOLUTION_BITS      10
#define PWM_MAX_DUTY             ((1 << PWM_RESOLUTION_BITS) - 1)

// Fraction (0..1) of full-scale speed command below which the motor is
// commanded to exactly zero duty. Prevents buzzing near zero and gives the
// direction-flip logic a clean "duty is zero" state to latch onto.
// Sim showed a large deadband (~2 rad/s) delays the response enough to fall;
// ~0.3 rad/s at the wheel works.
#define MOTOR_DEADBAND_FRACTION  (0.3f / MOTOR_FULL_SCALE_WHEEL_RADPS)

// ----------------------------------------------------------------------------
// IMU axis selection -- UNCONFIRMED, mounting-orientation dependent. Use
// firmware/imu_test to look at raw accel/gyro for each axis while tilting
// the frame by hand about the balance axis, then fill these in.
//
// Tilt angle theta is computed as:
//   theta_raw = IMU_ACCEL_ANGLE_SIGN * atan2(accel[IMU_ACCEL_AXIS_NUM], accel[IMU_ACCEL_AXIS_DEN])
// Tilt rate is:
//   theta_dot = IMU_GYRO_SIGN * gyro[IMU_GYRO_AXIS]   (rad/s, after bias removal)
//
// Axis indices: 0=X, 1=Y, 2=Z (matching ImuSample.accel[]/gyro[] in imu.h).
// ----------------------------------------------------------------------------
#define IMU_ACCEL_AXIS_NUM       1     // placeholder -- confirm with imu_test
#define IMU_ACCEL_AXIS_DEN       2     // placeholder -- confirm with imu_test
#define IMU_ACCEL_ANGLE_SIGN     1.0f  // +1 or -1
#define IMU_GYRO_AXIS             0     // placeholder -- confirm with imu_test
#define IMU_GYRO_SIGN             1.0f  // +1 or -1

// Gyro bias calibration duration at boot (device must be held still).
#define GYRO_CAL_DURATION_MS      2000

// Number of consecutive failed IMU reads before the firmware force-stops the
// motor as a safety measure.
#define IMU_FAIL_LIMIT             10

// ----------------------------------------------------------------------------
// Loop timing
// ----------------------------------------------------------------------------
#define LOOP_RATE_HZ              500
#define LOOP_PERIOD_US            (1000000UL / LOOP_RATE_HZ)
#define TELEMETRY_RATE_HZ          50

// ----------------------------------------------------------------------------
// State machine thresholds
// ----------------------------------------------------------------------------
#define UPRIGHT_HOLD_DEG           3.0f    // must be held within this many degrees...
#define UPRIGHT_HOLD_MS            500     // ...for this long, to leave WAIT_UPRIGHT
#define FALL_ANGLE_DEG             20.0f   // |theta| beyond this -> FALLEN

// ----------------------------------------------------------------------------
// EXPERIMENTAL jump-up-from-rest mode. Disabled by default -- mechanically
// dependent on the specific frame/wheel/motor and may not work at all, or may
// damage the frame if the wheel torque is too high. Only enable once BALANCING
// works reliably and you understand the risk. See firmware/README.md.
// ----------------------------------------------------------------------------
#define ENABLE_JUMP_UP             false
#define JUMP_SPEED_FRACTION        0.9f    // fraction of max wheel speed to spin up to before braking
#define JUMP_SPINUP_MS             1500    // time allotted to reach JUMP_SPEED_FRACTION
#define JUMP_CAPTURE_DEG           15.0f   // |theta| below this after the jump -> hand over to BALANCING
#define JUMP_TIMEOUT_MS            2000    // give up and go to FALLEN (brakes, then waits for upright) if not captured within this time

// ----------------------------------------------------------------------------
// Default control gains -- LQR starting point from sim/design_gains.py for the
// NOMINAL (estimated) parameters in sim/params.py: m_b 0.7 kg, l_b 0.11 m,
// m_w 0.35 kg, l_w 0.14 m, gear 2.5, tau_m 0.06 s. Units: theta rad,
// theta_dot rad/s, omega_w rad/s (wheel rel. frame) -> a rad/s^2.
// Minimum stabilizing Kp scales with (m*g*l)/I_w, so after measuring the real
// build re-derive with: py -X utf8 sim/design_gains.py --set m_b=... (see sim/README.md)
// Kd, Kw and Ki all take the same sign as Kp (from LQR).
// ----------------------------------------------------------------------------
#define DEFAULT_KP                 1137.4f
#define DEFAULT_KD                 144.3f
#define DEFAULT_KW                 1.0f
#define DEFAULT_KI                 0.0387f
#define DEFAULT_CONTROL_SIGN       1.0f   // +1 or -1 -- flip if hardware reaction torque sign is opposite
                                           // of the assumed convention (see balance_controller.h). Determine
                                           // empirically: if enabling BALANCING makes the fall visibly worse
                                           // in a consistent direction, flip this to -1.

// ----------------------------------------------------------------------------
// Safety limits (physical units)
// ----------------------------------------------------------------------------
// Safety CLAMP on omega_cmd -- distinct from MOTOR_FULL_SCALE_WHEEL_RADPS
// (the scaling reference for duty mapping). Kept below full scale so there
// is always some headroom left once mapped to duty. Override in config.h
// with a literal if you don't want it tied to MOTOR_MAX_RPM/GEAR_RATIO.
#define MAX_WHEEL_SPEED_RADPS      (0.85f * MOTOR_FULL_SCALE_WHEEL_RADPS)
#define MAX_ACCEL_CMD_RADPS2       4000.0f // clamp on the raw acceleration/torque command 'a'
#define INTEGRAL_CLAMP             1.0f    // anti-windup clamp on the (optional) integral term

// First-order lag time constant (s) used by wheel_speed_estimator.h to
// estimate the SIGNED wheel speed from the commanded speed + FG magnitude.
// Larger = trusts the commanded target longer / slower to reflect reality;
// smaller = tracks FG magnitude changes faster but is noisier right after a
// direction flip. Not measured yet -- a reasonable starting guess.
#define MOTOR_TAU_S                0.06f

// Many cheap built-in BLDC driver modules cannot produce active braking
// torque by simply lowering PWM duty (the wheel just coasts down under
// friction, much slower than the controller's requested decel), which makes
// the plant asymmetric (fast to accelerate, slow to decelerate). If
// motor_test's 'coastdown' command shows coast/half-duty decay times are
// much longer than the brake decay time, enable this so the main firmware
// pulses the physical brake for a control step instead of just lowering
// duty when a large same-direction deceleration is commanded. See
// firmware/README.md.
#define ACTIVE_DECEL_BRAKE         false
#define DECEL_BRAKE_MARGIN_RADPS   20.0f   // |omega_est|-|omega_cmd| (same sign) needed to trigger a brake pulse

// ----------------------------------------------------------------------------
// NVS (Preferences) storage
// ----------------------------------------------------------------------------
#define NVS_NAMESPACE               "rwbal"
