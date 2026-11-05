// Host unit tests for the Water Rover v2 firmware cores (no Arduino needed).
//   bash firmware/tests/run_tests.sh
#include <cstdio>
#include <cmath>
#include "../waterdroid_esp32/src/power_core.h"
#include "../waterdroid_esp32/src/pump_cores.h"
#include "../waterdroid_esp32/src/motion_cores.h"
#include "../waterdroid_esp32/src/mission_core.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)
#define NEAR(a, b, e) CHECK(std::fabs((a) - (b)) <= (e))

// ---------------- Issue #1 intake ----------------
static void test_intake() {
  IntakeController in;
  uint32_t t = 0;
  float level = 0;
  in.start();
  in.tick(t, false, level, false);
  CHECK(in.want && in.state == IN_PUMPING);
  // relay ON, level rises 2 %/s -> fills, stops at full
  for (t = 20; t < 60000 && in.want; t += 20) { level += 0.04f; in.tick(t, true, level, false); }
  CHECK(in.state == IN_FULL && !in.want);
  CHECK(t < 50000);

  // dry run: relay ON but level flat -> DRY_RUN fault after dryCheckMs
  IntakeController dry; dry.start();
  for (t = 0; t <= 9000; t += 20) dry.tick(t, true, 5.0f, false);
  CHECK(dry.state == IN_FAULT_DRY && !dry.want);
  dry.start();                                 // operator retry clears it
  dry.tick(t, false, 5.0f, false);
  CHECK(dry.state == IN_PUMPING);

  // max-run cutoff (level rises too slowly to be "full")
  IntakeController slow; slow.cfg.maxRunMs = 20000; slow.start();
  float lv = 0;
  for (t = 0; t <= 21000; t += 20) { lv += 0.006f; slow.tick(t, true, lv, false); }
  CHECK(slow.state == IN_FAULT_TIMEOUT);

  // blocked by column overflow, resumes after
  IntakeController b; b.start();
  b.tick(0, false, 10, true);
  CHECK(!b.want && b.state == IN_BLOCKED);
  b.tick(20, false, 10, false);
  CHECK(b.want);
  // wait for relay does not count as dry-run
  IntakeController w; w.start();
  for (t = 0; t < 20000; t += 20) w.tick(t, false, 0, false);
  CHECK(w.state == IN_PUMPING && w.runMs == 0);
}

// ---------------- Issue #3 dosing ----------------
static void test_dosing() {
  DosingPump p; p.rateMlPerS = 1.0f;
  CHECK(p.start(2.0f) == 2.0f);
  uint32_t t = 0; p.tick(t, false);
  for (t = 20; t <= 1000; t += 20) p.tick(t, false);           // relay held off 1 s
  NEAR(p.totalMl, 0.0f, 1e-6f);
  CHECK(p.want);
  uint32_t onAt = t;
  for (; p.want && t < 10000; t += 20) p.tick(t, true);
  NEAR(p.totalMl, 2.0f, 0.001f);
  CHECK(t - onAt >= 1980 && t - onAt <= 2060);                 // 2 s = 2 mL
  CHECK(p.start(50) == 10.0f);                                 // max-dose clamp
  p.abort(); CHECK(!p.want);
  NEAR(DosingPump::doseFor(1000, 3.0f), 3.0f, 1e-6f);
  DosingPump low; low.stockMl = 0.5f;
  CHECK(low.start(2.0f) == 0.5f);                              // stock clamp
}

// ---------------- Issue #8 delivery ----------------
static void test_delivery() {
  DeliveryController d; d.flowMlPerS = 25;
  d.request(250);
  uint32_t t = 0;
  d.tick(t, false, true, false, 80);
  CHECK(d.state == DL_WAIT_ARM && !d.want);                    // arm not deployed yet
  d.tick(t += 20, false, true, true, 80);
  CHECK(d.state == DL_PUMPING && d.want);
  for (t += 20; d.want && t < 30000; t += 20) d.tick(t, true, true, true, 80);
  CHECK(d.state == DL_DONE && d.count == 1);
  NEAR(d.deliveredMl, 250, 0.5f);

  DeliveryController e; e.request(250);
  e.tick(0, false, true, true, 80);
  e.tick(20, true, true, true, 80);
  e.tick(40, true, true, false, 80);                            // arm moved away
  CHECK(e.state == DL_ABORTED && !e.want);
  DeliveryController f; f.request(250);
  f.tick(0, false, false, true, 80);
  CHECK(f.state == DL_ABORTED);                                // not at homestead
}

// ---------------- Issue #5 filtration ----------------
static void test_filtration() {
  FiltrationMonitor f;
  f.tick(0, 50, 20, 30, true);
  CHECK(f.allowIntake && f.health == FH_HEALTHY);
  f.tick(100, 50, 92, 30, true);
  CHECK(!f.allowIntake && f.overflow && f.health == FH_OVERFLOW);
  f.tick(200, 50, 70, 30, true);                               // hysteresis: still latched
  CHECK(f.overflow);
  f.tick(300, 50, 45, 30, true);
  CHECK(!f.overflow && f.allowIntake);
  f.tick(400, 50, 45, 96, true);
  CHECK(!f.allowIntake && f.health == FH_TANK_FULL);
  FiltrationMonitor c;
  c.tick(0, 50, 75, 40, true);
  c.tick(31000, 50, 75, 40.2f, true);
  CHECK(c.clogged && c.health == FH_CLOGGING);
  FiltrationMonitor e;
  e.tick(0, 0, 0, 0, false);
  CHECK(e.allowIntake);                                        // empty raw tank must not block a fill
  NEAR(levelPct(1200, 200, 2200), 50.0f, 1e-4f);
}

// ---------------- Issue #6 arm ----------------
static void test_arm() {
  ServoArm a;
  CHECK(a.command("extend"));
  uint32_t t = 0; a.tick(t);
  while (a.angle < 90 && t < 5000) a.tick(t += 20);
  CHECK(a.deployed());
  CHECK(a.angle == 90 && t >= 1700 && t <= 1900);              // 90 deg at 1 deg / 20 ms
  CHECK(a.command("ANGLE:45"));
  CHECK(!a.command("ANGLE:120"));
  CHECK(!a.command("ANGLE:4x"));
  CHECK(!a.command("ANGLE:"));
  CHECK(!a.command("WAVE"));
  CHECK(a.command("RETRACT\r\n"));
  a.freeze(true);
  CHECK(!a.command("EXTEND") && a.target == a.angle);
}

// ---------------- Issue #9 drive ----------------
static void test_drive() {
  DriveController d;
  uint32_t t = 0; d.tick(t);
  CHECK(d.named("forward", t));
  for (int i = 0; i < 30; i++) d.tick(t += 20);
  CHECK(d.left == 200 && d.right == 200);
  CHECK(d.out.in1 && !d.out.in2 && d.out.in3 && !d.out.in4 && d.out.ena == 200);
  d.named("left", t);
  for (int i = 0; i < 30; i++) d.tick(t += 20);
  CHECK(d.left < 0 && d.right > 0);                            // zero-radius turn
  CHECK(!d.out.in1 && d.out.in2 && d.out.in3 && !d.out.in4);
  for (int i = 0; i < 40; i++) d.tick(t += 20);                // no repeat -> dead-man stop
  CHECK(d.left == 0 && d.right == 0 && d.out.ena == 0 && d.out.enb == 0);
  d.set(40, 40, t);
  for (int i = 0; i < 10; i++) d.tick(t += 20);
  CHECK(d.out.ena == d.minDuty);                               // stall floor
  d.named("forward", t); d.inhibit = true; d.tick(t += 20);
  CHECK(d.left == 0 && d.out.ena == 0);
  CHECK(!d.named("jump", t));
}

// ---------------- Issue #10 power: pumps on relays ----------------
static void test_power_relays() {
  PowerSequencer s;
  uint32_t t = 0;
  s.setMode(PM_ACTIVE, t);
  for (int i = 0; i < 4; i++) s.setWant(i, true);
  int maxOnAtOnce = 0, prevCount = 0;
  for (; t < 5000; t += 20) {
    s.tick(t, 5.0f, true);
    int n = s.on[0] + s.on[1] + s.on[2] + s.on[3];
    CHECK(n - prevCount <= 1);                                 // never two switch-ONs at once
    prevCount = n; if (n > maxOnAtOnce) maxOnAtOnce = n;
  }
  CHECK(maxOnAtOnce == 4);
  // dosing pumps (CH2, CH3) start first and are shed last
  PowerSequencer r; r.setMode(PM_ACTIVE, 0);
  for (int i = 0; i < 4; i++) r.setWant(i, true);
  r.tick(0, 5.0f, true);
  CHECK(r.on[1] && !r.on[0] && !r.on[2] && !r.on[3]);
}

// ---------------- Issue #11 mission ----------------
static void test_mission() {
  MissionCore m;
  m.cfg.contactMs = 2000;
  MissionInputs in;
  uint32_t t = 0;
  m.start(t);
  CHECK(m.state == MS_EN_ROUTE_RIVER && m.stateId() == 1 && m.driveAllowed);
  m.arrivedRiver(t);
  m.tick(t += 20, in);
  CHECK(m.state == MS_PUMPING && m.act.startIntake && m.act.doseAlumMl > 2.9f && !m.driveAllowed);
  m.act.clear();
  in.intakeBusy = true; m.tick(t += 2000, in); CHECK(m.state == MS_PUMPING);
  in.intakeBusy = false; in.s1Pct = 60; m.tick(t += 20, in); CHECK(m.state == MS_FILTERING);
  m.tick(t += 1000, in); CHECK(m.state == MS_FILTERING);
  in.s1Pct = 2; in.s2Pct = 5; in.cleanMl = 900; m.tick(t += 20, in); CHECK(m.state == MS_DOSING_CHLORINE);
  m.tick(t += 20, in); NEAR(m.act.doseChlorineMl, 0.54f, 0.01f); m.act.clear();
  m.tick(t += 600, in); m.tick(t += 2100, in);
  CHECK(m.state == MS_EN_ROUTE_HOMESTEAD && m.stateId() == 3 && m.driveAllowed);

  for (int h = 0; h < 3; h++) {
    m.arrivedHomestead(t);
    CHECK(m.state == MS_DISPENSING && m.stateId() == 4);
    m.tick(t += 20, in); CHECK(m.act.armExtend); m.act.clear();
    in.armDeployed = true; in.armStowed = false;
    m.tick(t += 20, in); NEAR(m.act.deliverMl, in.cleanMl / (3 - h), 0.5f);
    in.cleanMl -= m.act.deliverMl; m.act.clear();
    in.deliveryBusy = false; m.tick(t += 600, in); CHECK(m.act.armRetract); m.act.clear();
    in.armDeployed = false; in.armStowed = true; m.tick(t += 20, in);
  }
  CHECK(m.state == MS_RETURNING);
  m.arrivedDepot(t); CHECK(m.state == MS_COMPLETE);

  // e-stop from any state, release -> paused, resume continues
  MissionCore e; e.start(0); e.arrivedRiver(0); e.tick(20, in); e.act.clear();
  e.setEstop(true, 40);
  CHECK(e.state == MS_EMERGENCY_STOP && e.stateId() == 5 && e.act.stopAll);
  e.tick(60, in); CHECK(!e.driveAllowed);
  e.setEstop(false, 80); CHECK(e.state == MS_PAUSED);
  e.resume(100); CHECK(e.state == MS_PUMPING);
  // arrival events in the wrong state are ignored
  MissionCore g; g.arrivedHomestead(0); CHECK(g.state == MS_IDLE);
}

int main() {
  test_intake(); test_dosing(); test_delivery(); test_filtration();
  test_arm(); test_drive(); test_power_relays(); test_mission();
  printf("%d checks, %d failed\n", checks, fails);
  if (fails == 0) printf("ALL TESTS PASSED\n");
  return fails ? 1 : 0;
}
