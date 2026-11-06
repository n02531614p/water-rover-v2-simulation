// Host unit tests for the Water Rover v2 firmware cores (no Arduino needed).
//   bash firmware/tests/run_tests.sh
#include <cstdio>
#include <cmath>
#include "../waterdroid_esp32/src/power_core.h"
#include "../waterdroid_esp32/src/pump_cores.h"
#include "../waterdroid_esp32/src/motion_cores.h"
#include "../waterdroid_esp32/src/mission_core.h"
#include "../waterdroid_esp32/src/nav_cores.h"
#include <cstdlib>

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
  while (a.angle < ServoArm::DEPLOY && t < 5000) a.tick(t += 20);
  CHECK(a.deployed());
  CHECK(a.angle == 45 && t >= 850 && t <= 950);                // 45 deg at 1 deg / 20 ms
  CHECK(a.command("ANGLE:30"));
  CHECK(!a.command("ANGLE:60"));                                // beyond the 45 deg range
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
  CHECK(m.state == MS_PUMPING && m.act.armExtend && !m.act.startIntake && !m.driveAllowed);   // lower intake first
  m.act.clear();
  m.tick(t += 20, in); CHECK(!m.act.startIntake);                                         // waits for the servo
  in.armDeployed = true; in.armStowed = false;
  m.tick(t += 20, in); CHECK(m.act.startIntake && m.act.transferMl == 0); m.act.clear();
  in.intakeBusy = true; m.tick(t += 2000, in); CHECK(m.state == MS_PUMPING && !m.act.armRetract);
  in.intakeBusy = false; in.s1Pct = 60; m.tick(t += 20, in); CHECK(m.act.armRetract); m.act.clear();
  CHECK(m.state == MS_PUMPING);                                                           // waits until raised
  in.armDeployed = false; in.armStowed = true; m.tick(t += 20, in); CHECK(m.state == MS_FILTERING);
  in.columnMl = 25; m.tick(t += 1000, in); CHECK(m.state == MS_FILTERING && m.act.transferMl == 25); m.act.clear();
  in.transferBusy = true; in.columnMl = 30; m.tick(t += 20, in); CHECK(m.act.transferMl == 0);   // no restart while running
  in.transferBusy = false; in.columnMl = 10; m.tick(t += 20, in); CHECK(m.act.transferMl == 0 && m.state == MS_FILTERING);
  in.s1Pct = 2; m.tick(t += 20, in); CHECK(m.act.transferMl == 10); m.act.clear();               // raw empty: move the rest
  in.columnMl = 0; in.cleanMl = 370; m.tick(t += 20, in); CHECK(m.state == MS_DOSING_CHLORINE);
  m.tick(t += 20, in); NEAR(m.act.doseChlorineMl, 1.11f, 0.01f); m.act.clear();
  m.tick(t += 600, in); m.tick(t += 2100, in);
  CHECK(m.state == MS_EN_ROUTE_HOMESTEAD && m.stateId() == 3 && m.driveAllowed);

  for (int h = 0; h < 3; h++) {
    m.arrivedHomestead(t);
    CHECK(m.state == MS_DISPENSING && m.stateId() == 4);
    m.tick(t += 20, in); NEAR(m.act.deliverMl, in.cleanMl / (3 - h), 0.5f); CHECK(!m.act.armExtend);
    in.cleanMl -= m.act.deliverMl; m.act.clear();
    in.deliveryBusy = false; m.tick(t += 600, in);
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


// ---------------- flow sensor, volume model, fixed route ----------------
static void test_flow_volume() {
  FlowMeter f; f.pulsesPerLitre = 450;
  uint32_t pulses = 0, t = 0;
  f.tick(t, pulses);
  for (t = 20; t <= 10000; t += 20) { pulses += 3; f.tick(t, pulses); }   // 150 Hz
  NEAR(f.totalMl, 500 * 3 * 1000.0f / 450, 1.0f);                        // 1500 pulses = 3333 mL
  NEAR(f.rateMlPerS, 150 * 1000.0f / 450, 5.0f);
  CHECK(!f.noFlowFor(t, 500));
  CHECK(f.noFlowFor(t + 3000, 2000));
  f.resetBatch(); CHECK(f.batchMl == 0 && f.totalMl > 3000);

  VolumeModel v; v.filterMlPerS = v.drainMlPerS = 5;   // gravity variant
  v.addRaw(380); v.tick(0);
  for (t = 100; t <= 400000; t += 100) v.tick(t);
  NEAR(v.cleanMl, 380, 0.5f); CHECK(!v.filtering());
  v.removeClean(2000); CHECK(v.cleanMl == 0);
  VolumeModel w; w.filterMlPerS = 5; w.addRaw(380);                      // transfer-pump variant (default)
  for (t = 0; t <= 200000; t += 100) w.tick(t);
  NEAR(w.columnMl, 100, 0.5f); NEAR(w.cleanMl, 0, 1e-4f); NEAR(w.rawMl, 280, 0.5f);   // column full, nothing moves without the pump
  w.moveColumnToClean(60); NEAR(w.columnMl, 40, 1e-4f); NEAR(w.cleanMl, 60, 1e-4f);
  w.moveColumnToClean(500); NEAR(w.columnMl, 0, 1e-4f); NEAR(w.cleanMl, 100, 1e-4f);
}

static void drive_sim(RouteNavigator& n, uint32_t& t, int ms, float exec = 1.0f) {
  for (int i = 0; i < ms; i += 20) { t += 20; n.tick(t, exec); }
}

static void test_route() {
  RouteNavigator n; n.speedMps = 0.3f; n.turnDegPerS = 90;
  CHECK(n.n == 6 && n.homes() == 3);
  uint32_t t = 0; n.tick(t, 0);
  n.go();
  RouteAction seen[8]; int k = 0;
  for (int step = 0; step < 200000 && k < 5; step++) {
    t += 20; n.tick(t, 1.0f);
    if (n.event != RA_NONE) { seen[k++] = n.event; n.event = RA_NONE; n.go(); }
  }
  CHECK(k == 5);
  CHECK(seen[0] == RA_RIVER && seen[1] == RA_HOME && seen[2] == RA_HOME && seen[3] == RA_HOME && seen[4] == RA_DEPOT);
  NEAR(n.x, 0, 0.08f); NEAR(n.y, 0, 0.08f);
  CHECK(t < 400000);                                   // ~40 m of route at 0.3 m/s + pivots
  // first leg timing: 9.6 m at 0.3 m/s = 32 s, no pivot needed
  RouteNavigator a; uint32_t u = 0; a.tick(u, 0); a.go();
  while (a.event == RA_NONE && u < 60000) { u += 20; a.tick(u, 1.0f); }
  CHECK(a.event == RA_RIVER && u >= 31700 && u <= 32400);
  // paused / inhibited (exec 0): pose must not move
  RouteNavigator p; uint32_t w = 0; p.tick(w, 0); p.go(); drive_sim(p, w, 2000);
  float x0 = p.x; drive_sim(p, w, 5000, 0.0f); NEAR(p.x, x0, 1e-4f);
  // operator re-sync at a homestead
  RouteNavigator s; CHECK(s.syncTo(RA_HOME)); CHECK(s.x == 6.0f && s.y == 3.0f && s.idx == 3 && s.phase == NV_ARRIVED);
  // 180 degree pivot after the river: reverse leg then turn towards homestead 1
  RouteNavigator r; uint32_t v = 0; r.tick(v, 0); r.go();
  while (r.event == RA_NONE) { v += 20; r.tick(v, 1.0f); }
  r.event = RA_NONE; r.go();
  CHECK(r.phase == NV_REVERSE);
  for (int i = 0; i < 200 && r.cmd != 2 && r.cmd != -2; i++) { v += 20; r.tick(v, 1.0f); }
  CHECK(r.cmd == 2 || r.cmd == -2);                    // pivoting
  NEAR(r.x, 9.0f, 0.03f);
}

// The ESP32 control step, reduced to the cores (no Arduino): proves the subsystems
// cooperate - pumps never run while driving, route events drive the mission.
static void run_cycle(float filterRate);
static void test_full_cycle() { run_cycle(10); run_cycle(25); }   // 25 mL/s: column fills faster than the intake

static void run_cycle(float filterRate) {
  FiltrationMonitor filt;
  PowerSequencer seq; IntakeController in; DosingPump tr, cl; DeliveryController dl;
  ServoArm arm; DriveController dr; MissionCore m; FlowMeter fl; VolumeModel vol; RouteNavigator nav;
  fl.pulsesPerLitre = 450;
  m.cfg.contactMs = 5000; m.cfg.homesteads = nav.homes();
  vol.filterMlPerS = filterRate; tr.rateMlPerS = 8; tr.maxDoseMl = 400; tr.stockMl = 1e9f;
  in.cfg.fullPct = 100; in.cfg.dryCheckMs = 4000; in.cfg.minRisePct = 1;
  seq.setMode(PM_ACTIVE, 0);
  uint32_t t = 0, pulses = 0; float target = 380, lf = 0, lt = 0, lc = 0, ld = 0;
  bool atHome = false; int overlap = 0, deliveries = 0; uint32_t pumpingMs = 0;
  m.start(0); nav.reset();
  for (; t < 3000000 && m.state != MS_COMPLETE; t += 20) {
    if (m.state == MS_PUMPING) pumpingMs += 20;
    if (seq.on[0] && (t / 20) % 6 == 0) pulses += 1;   // 450 pulses/L, 1 pulse / 120 ms = 18.5 mL/s (5 V pump)
    MissionInputs mi; mi.intakeBusy = in.requested; mi.transferBusy = tr.busy(); mi.columnMl = vol.columnMl; mi.chlorineBusy = cl.busy();
    mi.s1Pct = vol.rawPct(); mi.s2Pct = vol.columnPct(); mi.armDeployed = arm.deployed(); mi.armStowed = arm.stowed();
    mi.deliveryBusy = dl.state == DL_WAIT_ARM || dl.state == DL_PUMPING; mi.cleanMl = vol.cleanMl;
    m.tick(t, mi);
    MissionActions& a = m.act;
    if (a.startIntake) { target = 380 - (vol.rawMl + vol.columnMl + vol.cleanMl); fl.resetBatch(); in.start(); }
    if (a.transferMl > 0) tr.start(a.transferMl);
    if (a.doseChlorineMl > 0) cl.start(a.doseChlorineMl);
    if (a.armExtend) arm.command("EXTEND");
    if (a.armRetract) arm.command("RETRACT");
    if (a.deliverMl > 0) dl.request(a.deliverMl);
    a.clear();
    if (m.state == MS_EN_ROUTE_HOMESTEAD || m.state == MS_RETURNING) atHome = false;
    fl.tick(t, pulses);
    vol.addRaw(fl.totalMl - lf); lf = fl.totalMl; vol.moveColumnToClean(tr.totalMl - lt); lt = tr.totalMl;
    vol.addClean(cl.totalMl - lc); lc = cl.totalMl; vol.removeClean(dl.totalMl - ld); ld = dl.totalMl;
    vol.tick(t);
    filt.tick(t, vol.rawPct(), vol.columnPct(), vol.cleanPct(), seq.on[0]);
    in.tick(t, seq.on[0], target > 0 ? 100 * fl.batchMl / target : 100, !arm.deployed() || !filt.allowIntake);
    if (in.state == IN_FAULT_DRY || in.state == IN_FAULT_TIMEOUT) overlap += 1000;
    tr.tick(t, seq.on[1]); cl.tick(t, seq.on[2]);
    uint16_t before = dl.count;
    dl.tick(t, seq.on[3], atHome, true, vol.cleanPct());
    deliveries += dl.count - before;
    seq.setWant(0, in.want); seq.setWant(1, tr.want); seq.setWant(2, cl.want); seq.setWant(3, dl.want);
    seq.tick(t, 5.0f, true);
    bool travelling = m.state == MS_EN_ROUTE_RIVER || m.state == MS_EN_ROUTE_HOMESTEAD || m.state == MS_RETURNING;
    int8_t lc2 = nav.cmd; float duty = (std::abs(dr.left) + std::abs(dr.right)) / 2.0f;
    nav.tick(t, lc2 == 0 ? 0 : duty / ((lc2 == 1 || lc2 == -1) ? dr.cruise : dr.turn));
    if (travelling) nav.go(); else nav.halt();
    if (nav.event == RA_RIVER) m.arrivedRiver(t);
    if (nav.event == RA_HOME) { atHome = true; m.arrivedHomestead(t); }
    if (nav.event == RA_DEPOT) m.arrivedDepot(t);
    nav.event = RA_NONE;
    static const char* NM[] = {"right", "reverse", "stop", "forward", "left"};
    if (travelling) dr.named(NM[nav.cmd + 2], t);
    bool anyPump = seq.on[0] || seq.on[1] || seq.on[2] || seq.on[3];
    dr.inhibit = !m.driveAllowed || anyPump || !arm.stowed();
    dr.tick(t);
    if (anyPump && (dr.left || dr.right)) overlap++;
    if (!arm.stowed() && (dr.left || dr.right)) overlap++;            // never drive with the intake lowered
    if (seq.on[0] && !arm.deployed()) overlap++;                      // never pump with the intake raised
    arm.tick(t);
  }
  CHECK(m.state == MS_COMPLETE);
  CHECK(overlap == 0);                                 // + no dry-run / timeout fault
  CHECK(pumpingMs < 60000);                            // intake never stalls on a full column                                 // never drive + pump together
  CHECK(deliveries == 3);
  CHECK(dl.totalMl > 350 && dl.totalMl < 390);
  CHECK(tr.totalMl > 370 && tr.totalMl < 385);         // whole batch moved column -> delivery tank
  NEAR(vol.columnMl, 0, 2.0f);
  NEAR(nav.x, 0, 0.1f); NEAR(nav.y, 0, 0.1f);
  printf("full cycle: %.0f s, transferred %.0f mL, delivered %.0f mL in %d deliveries, chlorine %.2f mL\n",
         t / 1000.0, tr.totalMl, dl.totalMl, deliveries, cl.totalMl);
}

// ---------------- bench test mode ----------------
static void test_pump_test() {
  PumpTest pt;
  CHECK(!pt.start(4, 5, 0));                      // bad channel
  CHECK(!pt.start(0, 0, 0));                      // zero seconds
  CHECK(pt.start(0, 10, 0));
  CHECK(pt.tick(0, false, 100));                  // wants the relay, timer not started yet
  CHECK(pt.remainingS(0, 100) > 9.9f);
  CHECK(pt.tick(0, true, 2000));                  // relay came on at 2 s (servo was lowering)
  CHECK(pt.tick(0, true, 11900));                 // still within 10 s of ON
  CHECK(!pt.tick(0, true, 12000) && !pt.active[0]);  // 10 s after ON -> done
  CHECK(pt.start(1, 99, 0) && pt.runMs[1] == 30000); // capped at 30 s
  CHECK(!pt.tick(1, false, 15000) && !pt.active[1]); // relay never came on -> give up
  CHECK(pt.start(3, 5, 0)); pt.stopAll(); CHECK(!pt.any());
}

int main() {
  test_intake(); test_dosing(); test_delivery(); test_filtration();
  test_pump_test();
  test_arm(); test_drive(); test_power_relays(); test_mission();
  test_flow_volume(); test_route(); test_full_cycle();
  printf("%d checks, %d failed\n", checks, fails);
  if (fails == 0) printf("ALL TESTS PASSED\n");
  return fails ? 1 : 0;
}
