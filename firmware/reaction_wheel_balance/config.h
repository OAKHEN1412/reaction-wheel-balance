#pragma once
// =============================================================================
// config.h -- all board wiring, calibration constants, and tunable defaults
// for the Reaction Wheel Balance firmware live here. Nothing else in the
// sketch should hardcode a pin number, gain, or physical-unit constant.
// =============================================================================

// ----------------------------------------------------------------------------
// Pins -- builds for both the Arduino Mega 2560 (bench/dev board) and the
// Arduino Nano / ATmega328P (the on-frame board, fitted 2026-09-21). Switched
// off the ESP32-C3 on 2026-09-19 because the BLDC-3640 driver inputs are 5V
// logic with strong pull-ups, which both AVR boards drive directly.
// See hardware/wiring.md.
// ----------------------------------------------------------------------------
// I2C is fixed by the chip and Wire.begin() takes no pins:
//   Mega 2560 -> SDA = D20, SCL = D21
//   Nano/328P -> SDA = A4,  SCL = A5
#define PIN_IMU_INT      3   // optional (INT1); not currently used by the firmware logic

// The PWM pin is NOT free to choose: motor.cpp drives Timer1 OC1A directly to
// get 15.6 kHz, and that output sits on a different pin on each chip.
#if defined(__AVR_ATmega2560__)
#define PIN_MOTOR_PWM   11   // Timer1 OC1A on the Mega 2560
#else
#define PIN_MOTOR_PWM    9   // Timer1 OC1A on the ATmega328P (Nano/Uno)
#endif
#define PIN_MOTOR_DIR    7   // F/R direction level (white wire)
#define PIN_MOTOR_BRAKE  6   // brake level (green wire)
#define PIN_MOTOR_FG     2   // FG tach pulses (yellow wire), INT0, open-collector -> INPUT_PULLUP

// Momentary pushbutton wired between the pin and GND (INPUT_PULLUP, so the pin
// reads LOW while pressed). One button, two jobs: it launches a jump when the
// frame is resting on either stop, and stops the machine while it is running,
// which is the only way to stop it once the USB cable is off. Set
// ENABLE_JUMP_BUTTON to 0 to compile it out if the button is not fitted.
#define ENABLE_JUMP_BUTTON  1
#define PIN_JUMP_BUTTON     4
#define BUTTON_DEBOUNCE_MS 30UL

// ----------------------------------------------------------------------------
// Build size -- the `selftest` command's failure strings are plain (non-PROGMEM)
// literals, so they cost about 1.1 kB of SRAM on top of ~8 kB of flash. The
// Mega swallows that; the Nano (32 kB flash / 2 kB SRAM) does not, and the
// tests are pure logic that does not depend on the board, so they are compiled
// out there. Run `selftest` on the Mega before flashing the Nano.
// ----------------------------------------------------------------------------
#ifndef ENABLE_SELFTEST
#if defined(__AVR_ATmega2560__)
#define ENABLE_SELFTEST 1
#else
#define ENABLE_SELFTEST 0
#endif
#endif

// ----------------------------------------------------------------------------
// Motor driver polarity -- MEASURED on the real BLDC-3640 on 2026-09-19 with
// firmware/motor_test (+ motor_test_mega). Real wire colours differ from the
// slides: red +12V, black GND, blue PWM, yellow FG, white DIR, green BRAKE.
// DIR (white) and BRAKE (green) are 5V-logic inputs with strong (~1k) pull-ups;
// the Mega's 5V outputs drive them directly (no transistor), so the levels
// below are the wire levels themselves.
// ----------------------------------------------------------------------------
// If true, the driver's PWM input is inverted, i.e. duty=0 -> full speed and
// duty=PWM_MAX_DUTY -> stopped. Some built-in ESC/driver boards behave this way.
#define PWM_INVERT              true   // measured: raw pin HIGH = stopped

// Logic level written to PIN_MOTOR_DIR that makes the WHEEL spin in the
// direction we call "positive" throughout this firmware (see balance_controller.h
// for how that sign is used in the control law). Pick either HIGH or LOW here;
// motor_test will tell you which level corresponds to which physical rotation
// direction. The label "CW" is just a name -- what matters is that this
// firmware is internally consistent about it.
// Measured: white wire LOW = CW, HIGH/floating = CCW (seen from the wheel
// face); switchable while powered.
#define DIR_CW_LEVEL            LOW

// Logic level on PIN_MOTOR_BRAKE that ENGAGES the brake. Measured: green wire
// LOW = brake, HIGH/floating = run. Note: duty 0 already stops
// the wheel about as fast as the brake does (coast test), so the brake is only
// needed for the FALLEN/emergency state.
#define BRAKE_ACTIVE_LEVEL      LOW

// FG tach pulses per one mechanical revolution of the MOTOR shaft (not the
// wheel). The driver only emits FG while it is driving the motor (turning it
// by hand with duty 0 gives no pulses), so this was measured by spinning at
// low duty: 269 pulses over ~3.85 wheel revolutions = ~70 pulses per WHEEL
// revolution. Firmware wheel speed only depends on FG_PULSES_PER_REV *
// GEAR_RATIO (= 70), so this is 70 / GEAR_RATIO; re-derive it if GEAR_RATIO
// changes. With GEAR_RATIO = 6 this is ~11.7 pulses/motor rev -- the true
// value is probably 12 (i.e. ~72/wheel rev); recount FG over 10 wheel revs if
// wheel speed looks ~3% off.
#define FG_PULSES_PER_WHEEL_REV  70.0f
#define FG_PULSES_PER_REV       (FG_PULSES_PER_WHEEL_REV / GEAR_RATIO)
// FG edges closer than this are rejected as noise (see fg_tach.cpp). Full
// duty is ~620 wheel rpm = ~1.4 ms between pulses; 700 us allows ~2x headroom.
#define FG_MIN_PERIOD_US         700UL

// Reduction ratio between motor shaft and wheel: GEAR_RATIO = (motor shaft
// revolutions) / (wheel revolutions). GEAR_RATIO = 1.0 if direct-drive.
#define GEAR_RATIO               6.0f   // measured 2026-09-19: 1 wheel rev = 6 motor revs (hand count)

// Motor shaft no-load speed at full duty (datasheet value for BLDC-3640,
// ideally re-measured empirically via FG in motor_test/imu_test once
// polarity is known, since real no-load speed can differ from datasheet
// under the actual supply voltage).
// Measured 2026-09-19 (firmware/step_test, 12V): steady wheel rpm 153/279/390 at
// duty 0.29/0.49/0.68 -> ~585 wheel rpm (~3500 motor rpm) at full duty.
#define MOTOR_MAX_RPM            3500.0f

// Duty fraction (0..1) at which the motor actually starts turning (stiction
// / minimum drive threshold) -- measure with motor_test by slowly raising
// duty from 0 and noting where FG pulses begin. Nonzero commanded fractions
// are mapped linearly into [MOTOR_MIN_DUTY_FRACTION, 1] so that small
// commands don't get lost below the motor's real starting duty. Leave at
// 0.0 until measured.
#define MOTOR_MIN_DUTY_FRACTION  0.10f  // measured: starts turning at duty ~100/1023

// Wheel angular speed (rad/s) that corresponds to duty fraction = 1.0 (i.e.
// the motor's no-load speed reflected through the gear/belt ratio to the
// wheel). This is the SCALING reference for mapping omega_cmd -> duty
// fraction -- it is deliberately a different constant from
// MAX_WHEEL_SPEED_RADPS below, which is a lower safety CLAMP on omega_cmd.
#define MOTOR_FULL_SCALE_WHEEL_RADPS ((MOTOR_MAX_RPM / GEAR_RATIO) * (2.0f * 3.14159265358979323846f / 60.0f))

// ----------------------------------------------------------------------------
// PWM configuration
// ----------------------------------------------------------------------------
// Fixed by the Timer1 setup in motor.cpp: fast PWM, 10-bit, no prescaler ->
// 16 MHz / 1024 = ~15.6 kHz (the driver was verified at 20 kHz and at ~15.6 kHz).
#define PWM_FREQ_HZ              15625
#define PWM_RESOLUTION_BITS      10
#define PWM_MAX_DUTY             ((1 << PWM_RESOLUTION_BITS) - 1)

// Fraction (0..1) of full-scale speed command below which the motor is
// commanded to exactly zero duty. Prevents buzzing near zero and gives the
// direction-flip logic a clean "duty is zero" state to latch onto.
// Sim showed a large deadband (~2 rad/s) delays the response enough to fall;
// ~0.3 rad/s at the wheel works.
#define MOTOR_DEADBAND_FRACTION  (0.3f / MOTOR_FULL_SCALE_WHEEL_RADPS)

// ----------------------------------------------------------------------------
// IMU axis selection -- CONFIRMED 2026-09-19 on the new MPU-6050 with
// tools/imu_viewer.html (hand tilt: d(theta) +9.4 deg, rate correlation 0.74).
// Balance axis = sensor X: roll from atan2(ay, az), rate from gx, both +1
// (consistent with the right-hand rule). Re-run firmware/imu_test if the
// module is ever remounted.
//
// Tilt angle theta is computed as:
//   theta_raw = IMU_ACCEL_ANGLE_SIGN * atan2(accel[IMU_ACCEL_AXIS_NUM], accel[IMU_ACCEL_AXIS_DEN])
// Tilt rate is:
//   theta_dot = IMU_GYRO_SIGN * gyro[IMU_GYRO_AXIS]   (rad/s, after bias removal)
//
// Axis indices: 0=X, 1=Y, 2=Z (matching ImuSample.accel[]/gyro[] in imu.h).
// ----------------------------------------------------------------------------
#define IMU_ACCEL_AXIS_NUM       1     // Y  (measured)
#define IMU_ACCEL_AXIS_DEN       2     // Z  (measured)
#define IMU_ACCEL_ANGLE_SIGN     1.0f  // +1 (measured)
#define IMU_GYRO_AXIS             0     // X  (measured)
#define IMU_GYRO_SIGN             1.0f  // +1 (measured)

// Skip the accelerometer in the complementary filter (gyro-only step) when
// |accel| differs from 1 g by more than this -- impacts / jump kicks.
#define ACCEL_GRAVITY_TOL_G       0.25f

// Gyro bias calibration duration at boot (device must be held still).
#define GYRO_CAL_DURATION_MS      2000

// Number of consecutive failed IMU reads before the firmware force-stops the
// motor as a safety measure.
#define IMU_FAIL_LIMIT             10
// Consecutive failed IMU reads tolerated during a jump before aborting
// (2 ms each at 500 Hz). One is too strict: motor current spikes cause them.
#define JUMP_IMU_FAIL_LIMIT         5

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
#define FALL_HOLD_DEG              12.0f   // |theta| beyond this...
#define FALL_HOLD_MS               300     // ...for this long also counts as FALLEN (rest stop is ~16 deg)

// ----------------------------------------------------------------------------
// Manual experimental voltage kick. Never automatically entered.
// Defaults deliberately under-powered; see sim/out/jumpup and firmware/README.md.
// ----------------------------------------------------------------------------
#define ENABLE_JUMP_UP             true
#define JUMP_SPIN_RPM              550.0f  // wheel RPM; command rejects values above 550.
                                           // Hardware 2026-09-21: 550/450/8deg/75dps self-righted from ~-15 deg and
                                           // kept balancing 3/3 tries (earlier fixed-angle handover: 3/10).
#define JUMP_KICK_MS               450     // kick deadline; capture at JUMP_CAPTURE_DEG ends it (~190 ms at 500 rpm)
#define JUMP_CAPTURE_DEG           8.0f   // capture only during reverse-voltage KICK
// Handover tilt rate (deg/s toward upright). The KICK pushes until the frame
// is rising at this rate near upright, coasting if it exceeds rate+15 and
// kicking again below rate-15. Measured on hardware: 68-76 dps at ~7.6 deg
// balanced; 50 dps stalled short; 100-195 dps overshot and fell the other way.
#define JUMP_TARGET_RATE_DPS       75.0f
// Pure jump logic: 1500 ms spin timeout, 2000 ms total, 25 deg abort,
// 500 ms rest hold (10..22 deg, <=3 deg/s, <=10 wheel rpm).

// ----------------------------------------------------------------------------
// Default control gains -- LQR starting point from sim/design_gains.py for the
// NOMINAL (estimated) parameters in sim/params.py: m_b 0.7 kg, l_b 0.11 m,
// m_w 0.35 kg, l_w 0.14 m, gear 6 (measured), tau_m 0.18 s, voltage mode.
// Re-run 2026-09-19 with gear 6: same gains (they are in wheel-accel units);
// See sim/voltage_results.json for the measured-tau sweep. Units: theta rad,
// theta_dot rad/s, omega_w rad/s (wheel rel. frame) -> a rad/s^2.
// Minimum stabilizing Kp scales with (m*g*l)/I_w, so after measuring the real
// build re-derive with: py -X utf8 sim/design_gains.py --set m_b=... (see sim/README.md)
// Kd, Kw and Ki all take the same sign as Kp (from LQR).
// ----------------------------------------------------------------------------
#define DEFAULT_KP                 1137.4f
#define DEFAULT_KD                 450.0f   // re-tuned 2026-09-22 on the Nano build (the board moved onto the frame,
                                           // so the mass and CoG changed). 400 -> SD 0.55, 450 -> 0.42/0.51, 500 -> 0.45,
                                           // 600 -> 3.32 deg with duty saturating: 400..500 are within measurement noise
                                           // of each other, 450 just sits furthest from the 600 cliff.
                                           // Was 500.0f, tuned 2026-09-19 on the Mega build with the board off-frame:
                                           // 144.3 (LQR) fell; 300 -> SD 1.6, 350 -> 0.71, 400 -> 0.49, 500 -> 0.34 deg.
#define DEFAULT_KW                 3.0f   // raised from 1.0 on 2026-09-21: at Kw=1 the wheel wound up
                                           // one way during a long balance (-178..-38 rpm); Kw=3 keeps it
                                           // bounded (55..139 rpm) for +0.04 deg of tilt SD. Re-check after
                                           // the Nano rebuild -- this was measured on the bench frame.
#define DEFAULT_KI                 0.0387f
#define DEFAULT_CONTROL_SIGN       1.0f   // MEASURED +1 (2026-09-19, firmware/sign_test): +omega spin-up kicks
                                           // the frame to -theta (-10.3 deg), -omega to +theta (+7.0 deg).
                                           // Was: flip if hardware reaction torque sign is opposite
                                           // of the assumed convention (see balance_controller.h). Determine
                                           // empirically: if enabling BALANCING makes the fall visibly worse
                                           // in a consistent direction, flip this to -1.

// ----------------------------------------------------------------------------
// Safety limits (physical units)
// ----------------------------------------------------------------------------
// Safety CLAMP on omega_cmd -- distinct from MOTOR_FULL_SCALE_WHEEL_RADPS
// (the scaling reference for duty mapping). Kept below full scale so there
// is always some headroom left in legacy speed mode. In voltage mode this
// guards measured wheel speed against further outward acceleration; voltage
// may still reach full duty for torque. Override in config.h
// with a literal if you don't want it tied to MOTOR_MAX_RPM/GEAR_RATIO.
#define MAX_WHEEL_SPEED_RADPS      (0.85f * MOTOR_FULL_SCALE_WHEEL_RADPS)
#define MAX_ACCEL_CMD_RADPS2       4000.0f // clamp on the raw acceleration/torque command 'a'
#define INTEGRAL_CLAMP             1.0f    // anti-windup clamp on the (optional) integral term

// Measured light wheel (no rim nuts): t63=0.155/0.175/0.215 s.
// Mechanical voltage-mode time constant, used for inverse torque command and
// signed-speed prediction. With rim nuts this rises to 0.45-0.59 s: remeasure.
// false selects the legacy integrated-speed controller for comparison.
#define CONTROL_MODE_VOLTAGE       true
#define MOTOR_TAU_S                0.18f

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
// EEPROM storage (gains + upright offset, see reaction_wheel_balance.ino)
// ----------------------------------------------------------------------------
#define EEPROM_ADDR                 0
#define EEPROM_MAGIC                0x5242u   // "RB" -- bump if the stored layout changes
