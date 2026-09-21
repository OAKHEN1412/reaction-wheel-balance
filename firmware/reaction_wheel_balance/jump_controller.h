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
  enum class Phase { SPINUP, KICK, COAST, CAPTURED, ABORTED };
  static constexpr float kDeg2RadF = 0.01745329252f;
  static constexpr float kRateBandDps = 15.0f;   // +/- around vTargetDps_
  void setTargetRateDps(float v) { vTargetDps_ = v; }
  float targetRateDps() const { return vTargetDps_; }
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
      // HANDOVER WINDOW (2026-09-21, from hardware): what decides the outcome is
      // the tilt RATE at handover, not the angle alone. Balanced runs handed over
      // at 7.5-7.7 deg with 68-76 dps; 50 dps stalled short of upright, while
      // 100-195 dps overshot and fell the other way. The theoretical energy
      // criterion did not match because the balance controller keeps pushing
      // after capture, so the rate window is used directly.
      float rising = -side_ * rateDps;                      // dps toward upright
      if (rising > vTargetDps_ + kRateBandDps) {             // too fast: stop pushing
        phase_ = Phase::COAST;
        return 0;
      }
      if (fabsf(thetaDeg) < captureDeg_ && rising >= vTargetDps_ - kRateBandDps) {
        phase_ = Phase::CAPTURED;
        return 0;
      }
      if ((uint32_t)(now-phaseStart_) >= kickMs_) return abort(9);
      return side_; // reverse VOLTAGE, motor_logic supplies the zero-duty interval
    }
    if (phase_ == Phase::COAST) {
      // Gravity bleeds off the excess while the wheel free-wheels; hand over as
      // soon as the frame is close enough, or kick again if it slowed too much.
      float rising = -side_ * rateDps;
      if (fabsf(thetaDeg) < captureDeg_) { phase_ = Phase::CAPTURED; return 0; }
      if (rising < vTargetDps_ - kRateBandDps) { phase_ = Phase::KICK; phaseStart_ = now; return side_; }
      return 0.0f; // coast
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
  float vTargetDps_ = 75.0f;  // handover tilt rate, see KICK
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
    c.step(110, side*9, -side*70, 50, false);  // 70 dps toward upright: inside the handover window
    if (c.phase() != JumpController::Phase::CAPTURED) return false;
    c.begin(0, side*16, 100, 20, 10);
    c.step(100, side*16, 0, 100, true);
    c.step(120, side*15, 0, 50, true);
    if (c.phase() != JumpController::Phase::ABORTED) return false;
  }
  {
    // Handover window: kick until the rate is in [vTarget-15, vTarget+15] dps
    // near upright; too fast -> COAST until the angle is small enough.
    JumpController ec;
    ec.setTargetRateDps(75.0f);
    ec.begin(0, -16, 100, 400, 8);
    ec.step(0, -16, 0, 0, false);
    ec.step(102, -16, 0, 110, true);          // -> KICK
    if (ec.phase() != JumpController::Phase::KICK) return false;
    ec.step(150, -12, 70, 50, false);         // inside the band but still 12 deg
    if (ec.phase() != JumpController::Phase::KICK) return false;
    ec.step(180, -6, 70, 50, false);          // 6 deg, 70 dps -> hand over
    if (ec.phase() != JumpController::Phase::CAPTURED) return false;

    JumpController fc;                         // too fast high up -> coast, then capture
    fc.setTargetRateDps(75.0f);
    fc.begin(0, -16, 100, 400, 8);
    fc.step(0, -16, 0, 0, false);
    fc.step(102, -16, 0, 110, true);
    if (fc.step(150, -12, 140, 50, false) != 0.0f) return false;
    if (fc.phase() != JumpController::Phase::COAST) return false;
    fc.step(200, -10, 100, 0, false);          // still coasting at 10 deg
    if (fc.phase() != JumpController::Phase::COAST) return false;
    fc.step(240, -7, 85, 0, false);
    if (fc.phase() != JumpController::Phase::CAPTURED) return false;

    JumpController sc;                         // coasted too slow -> kick again
    sc.setTargetRateDps(75.0f);
    sc.begin(0, -16, 100, 400, 8);
    sc.step(0, -16, 0, 0, false);
    sc.step(102, -16, 0, 110, true);
    sc.step(150, -12, 140, 50, false);
    if (sc.step(300, -11, 30, 0, false) != -1.0f) return false; // back to KICK, duty = side_ (= -1)
    if (sc.phase() != JumpController::Phase::KICK) return false;
  }

  JumpController c;
  c.begin(0, 16, 100, 20, 10); c.step(1500, 16, 0, 0, false);
  if (c.phase() != JumpController::Phase::ABORTED) return false;
  c.begin(0, 16, 100, 20, 10); c.step(2, 26, 0, 0, false);
  return c.phase() == JumpController::Phase::ABORTED;
}
