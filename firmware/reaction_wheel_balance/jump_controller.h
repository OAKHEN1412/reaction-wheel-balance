#pragma once
// Pure manual-jump logic. No changes to the balance controller or motor driver.
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>

inline bool parseJumpArgs(const char *text, float &rpm, uint16_t &kickMs) {
  // Validate defaults too: editing config must never bypass the hard limits.
  if (!isfinite(rpm) || rpm <= 0 || rpm > 550 || kickMs < 1 || kickMs > 500) return false;
  while (isspace(*text)) ++text;
  if (!*text) return true;
  char *end;
  double value = strtod(text, &end);
  if (end == text || !isfinite(value) || value <= 0 || value > 550) return false;
  rpm = value;
  if (*end && !isspace(*end)) return false;
  text = end;
  while (isspace(*text)) ++text;
  if (!*text) return true;
  value = strtod(text, &end);
  if (end == text || !isfinite(value) || value < 1 || value > 500 || floor(value) != value) return false;
  while (isspace(*end)) ++end;
  if (*end) return false;
  kickMs = (uint16_t)value;
  return true;
}

class JumpRestGate {
public:
  void reset() { holding_ = false; }
  void update(uint32_t now, float thetaDeg, float rateDps, float wheelRpm, bool eligible) {
    if ((uint32_t)(now-last_) > 20) reset();
    last_ = now;
    if (!eligible || !isfinite(thetaDeg) || !isfinite(rateDps) || !isfinite(wheelRpm) ||
        fabsf(thetaDeg) < 10 || fabsf(thetaDeg) > 22 ||
        fabsf(rateDps) > 3 || fabsf(wheelRpm) > 10) { reset(); return; }
    if (!holding_ || fabsf(thetaDeg - anchor_) > 1) {
      holding_ = true; since_ = now; anchor_ = thetaDeg;
    }
  }
  bool ready(uint32_t now) const { return holding_ && (uint32_t)(now-since_) >= 500; }
private:
  bool holding_ = false;
  uint32_t since_ = 0, last_ = 0;
  float anchor_ = 0;
};

class JumpController {
public:
  // Spin-up voltage scales with the requested speed: 0.6 (gentle) for low rpm,
  // up to full duty so ~500 rpm is reachable (0.6 duty tops out near 340 rpm;
  // hardware 2026-09-20). kSpinDuty is the floor.
  static constexpr float kSpinDuty = 0.6f;
  static float spinDutyFor(float rpm) {
    float d = rpm / 450.0f;
    return d < kSpinDuty ? kSpinDuty : (d > 1.0f ? 1.0f : d);
  }
  enum class Phase { SPINUP, KICK, CAPTURED, ABORTED };
  void begin(uint32_t now, float thetaDeg, float rpm, uint16_t kickMs, float captureDeg) {
    phase_ = Phase::SPINUP; start_ = phaseStart_ = now; reason_ = 0;
    side_ = thetaDeg > 0 ? 1 : -1;
    restAngle_ = thetaDeg; rpm_ = rpm; kickMs_ = kickMs; captureDeg_ = captureDeg;
    spinProven_ = false;
  }
  // Open-loop wheel speed model for the spin-up (voltage mode, from rest):
  // full-scale ~583 wheel rpm (MOTOR_MAX_RPM / GEAR_RATIO), tau 0.20 s (slightly
  // slower than the measured 0.18 so it errs toward kicking late, not early).
  static constexpr float kFullScaleRpm = 583.0f;
  static constexpr float kModelTauS = 0.20f;
  float modelRpm(uint32_t now) const {
    float t = (uint32_t)(now - phaseStart_) * 0.001f;
    return kFullScaleRpm * spinDutyFor(rpm_) * (1.0f - expf(-t / kModelTauS));
  }
  // fgVerified: >=3 pulses since start, with a new pulse in the last 100 ms.
  float step(uint32_t now, float thetaDeg, float rateDps, float fgRpm, bool fgVerified) {
    if (!isfinite(thetaDeg) || !isfinite(rateDps) || !isfinite(fgRpm)) return abort(1);
    if (fabsf(thetaDeg) > 25) return abort(2);
    if ((uint32_t)(now-start_) >= 2000) return abort(3);
    if (phase_ == Phase::SPINUP) {
      if ((uint32_t)(now-phaseStart_) >= 1500) return abort(5);
      // Overspeed only matters while FG drives the spin-up. At the KICK's DIR
      // flip the FG emits a glitch (read 782 rpm on hardware 2026-09-20) and the
      // kick is open-loop anyway, so it is not checked there.
      if (fgRpm > 575) return abort(4);
      // 2026-09-19 hardware: a full-duty spin-up presses the frame into its
      // compliant stop at >10 dps, so the guards are loose enough for that.
      if (fabsf(thetaDeg-restAngle_) > 4) return abort(6);
      if (fabsf(rateDps) > 40) return abort(7);
      // FG must prove the wheel is really turning early on; after that it may
      // drop out (it did at ~240 rpm on hardware 2026-09-20), so the kick fires
      // on FG speed OR the open-loop model, whichever reaches the target first.
      if (fgVerified) spinProven_ = true;
      if ((uint32_t)(now-phaseStart_) >= 150 && !spinProven_) return abort(8);
      if ((fgVerified && fgRpm >= rpm_) || (spinProven_ && modelRpm(now) >= rpm_)) {
        phase_ = Phase::KICK; phaseStart_ = now;
      }
      else return -side_ * spinDutyFor(rpm_);
    }
    if (phase_ == Phase::KICK) {
      // Capture wins at the deadline, as in the simulation. Never capture in SPINUP.
      if (fabsf(thetaDeg) < captureDeg_) { phase_ = Phase::CAPTURED; return 0; }
      if ((uint32_t)(now-phaseStart_) >= kickMs_) return abort(9);
      return side_; // reverse VOLTAGE, motor_logic supplies the zero-duty interval
    }
    return 0;
  }
  Phase phase() const { return phase_; }
  // Why the last attempt aborted (0 = not aborted). Printed by the sketch so a
  // hardware run shows which guard fired.
  uint8_t abortReason() const { return reason_; }
  static const char *reasonText(uint8_t r) {
    switch (r) {
      case 1: return "non-finite input";
      case 2: return "|theta| > 25 deg";
      case 3: return "total timeout 2 s";
      case 4: return "FG > 575 rpm during spin-up";
      case 5: return "spin timeout 1.5 s";
      case 6: return "frame moved > 4 deg during spin-up";
      case 7: return "frame rate > 40 dps during spin-up";
      case 8: return "no FG pulses in first 150 ms of spin-up";
      case 9: return "kick time expired before capture";
      default: return "-";
    }
  }
private:
  float abort(uint8_t r) { phase_ = Phase::ABORTED; reason_ = r; return 0; }
  uint8_t reason_ = 0;
  bool spinProven_ = false;
  Phase phase_ = Phase::ABORTED;
  uint32_t start_ = 0, phaseStart_ = 0;
  float side_ = 1, restAngle_ = 16, rpm_ = 100, captureDeg_ = 10;
  uint16_t kickMs_ = 20;
};

inline bool jumpControllerSelfTest() {
  float rpm = 100; uint16_t ms = 20;
  if (!parseJumpArgs("550 200", rpm, ms) || rpm != 550 || ms != 200) return false;
  const char *bad[] = {"551", "nan", "inf", "0", "-1", "100x", "100 0", "100 501", "100 2.5", "100 20 extra"};
  for (const char *s : bad) {
    rpm = 100; ms = 20;
    if (parseJumpArgs(s, rpm, ms)) return false;
  }
  rpm = 551;
  if (parseJumpArgs("", rpm, ms)) return false;
  JumpRestGate gate;
  gate.update(0, 16, 0, 0, true);
  if (gate.ready(499) || !gate.ready(500)) return false;
  gate.update(501, 16, 4, 0, true);
  if (gate.ready(1001)) return false;
  for (int side = -1; side <= 1; side += 2) {
    JumpController c;
    c.begin(0, side*16, 100, 20, 10);
    if (c.step(0, side*16, 0, 0, false) != -side*JumpController::kSpinDuty) return false;
    if (c.step(98, side*16, 0, 110, false) != -side*JumpController::kSpinDuty) return false;
    if (c.step(102, side*16, 0, 110, true) != side) return false;
    c.step(110, side*9, -side*30, 50, false);
    if (c.phase() != JumpController::Phase::CAPTURED) return false;
    c.begin(0, side*16, 100, 20, 10);
    c.step(100, side*16, 0, 100, true);
    c.step(120, side*15, 0, 50, true);
    if (c.phase() != JumpController::Phase::ABORTED) return false;
  }
  JumpController c;
  c.begin(0, 16, 100, 20, 10); c.step(1500, 16, 0, 0, false);
  if (c.phase() != JumpController::Phase::ABORTED) return false;
  c.begin(0, 16, 100, 20, 10); c.step(2, 26, 0, 0, false);
  return c.phase() == JumpController::Phase::ABORTED;
}
