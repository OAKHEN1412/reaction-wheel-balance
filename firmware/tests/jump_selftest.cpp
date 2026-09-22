// Host-only runner: never accesses Arduino hardware or serial ports.
#include <stdio.h>
#include "../reaction_wheel_balance/balance_controller.h"
#include "../reaction_wheel_balance/wheel_speed_estimator.h"
#include "../reaction_wheel_balance/jump_controller.h"

#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); return 1; } } while (0)
int main() {
  const char *failure = nullptr;
  CHECK(balanceControllerSelfTest(&failure));
  CHECK(wheelSpeedEstimatorSelfTest(&failure));
  CHECK(jumpControllerSelfTest());
  JumpRestGate gate;
  for (uint32_t t=0; t<=500; t+=2) gate.update(t, 16, 0, 0, true);
  CHECK(gate.ready(500));
  gate.update(1000, 16, 0, 0, true); // stale sample breaks stationary hold
  CHECK(!gate.ready(1000));
  float angles[] = {9.9f, 22.1f, 0.0f};
  for (float angle : angles) {
    gate.update(1002, angle, 0, 0, true); CHECK(!gate.ready(2000));
  }
  gate.update(2000, 16, 0, 11, true); CHECK(!gate.ready(3000));
  gate.update(3000, 16, 0, 0, false); CHECK(!gate.ready(4000));
  JumpController c;
  const uint32_t wrap = 0xffffff00UL;
  c.begin(wrap, -16, 100, 20, 10);
  // Spin-up duty scales with the requested speed and floors at kSpinDuty, so a
  // 100 rpm request spins at 0.6, not at full duty. This expected full duty and
  // had been failing at HEAD unnoticed -- the suite is not run often enough.
  CHECK(c.step(wrap+2, -16, 0, 0, false) == JumpController::kSpinDuty);
  c.step(wrap+1500, -16, 0, 0, false);
  CHECK(c.phase() == JumpController::Phase::ABORTED);
  c.begin(0, 16, 100, 500, 10);
  c.step(1498, 16, 0, 100, true);
  c.step(2000, 1, -10, 100, true); // total deadline beats late capture
  CHECK(c.phase() == JumpController::Phase::ABORTED);
  // Model speed alone never authorizes the kick: without a verified FG pulse
  // the spin-up must hold and then abort on the 150 ms FG deadline. This used
  // to assert ABORTED at 100 ms, which the guard cannot do, and had been
  // failing at HEAD alongside the check above.
  c.begin(0, 16, 100, 20, 10);
  c.step(100, 16, 0, 100, false);
  CHECK(c.phase() == JumpController::Phase::SPINUP);
  c.step(160, 16, 0, 100, false);
  CHECK(c.phase() == JumpController::Phase::ABORTED);
  CHECK(c.abortReason() == 8);
  // Overspeed now needs three consecutive readings. A single FG sample is not
  // evidence: on 2026-09-22 two launches in a batch of six were aborted here
  // reading 719 and 806 rpm off a frame sitting still with a stopped wheel.
  // fgVerified is false here, which is the case the guard exists for: the
  // glitches arrive before FG has proven itself, so the kick trigger (which
  // requires a verified FG) cannot consume them first.
  c.begin(0, 16, 550, 20, 10);
  c.step(2, 16, 0, 800, false);
  CHECK(c.phase() == JumpController::Phase::SPINUP);
  c.step(4, 16, 0, 800, false);
  c.step(6, 16, 0, 800, false);
  CHECK(c.phase() == JumpController::Phase::ABORTED);
  CHECK(c.abortReason() == 4);
  c.begin(0, 16, 100, 20, 10);
  c.step(2, NAN, 0, 0, false);
  CHECK(c.phase() == JumpController::Phase::ABORTED);
  puts("PASS: balance controller, wheel estimator, jump parser/gates/phases/timeouts/signs");
  return 0;
}
