// Host unit tests for power_core.h.  Build & run:  bash run_tests.sh
#include <cstdio>
#include <cstring>
#include "../core/power_core.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while (0)

// run the sequencer for `ms` milliseconds in 10 ms steps
static void run(PowerSequencer& s, uint32_t& t, uint32_t ms, float v = 5.0f, bool valid = true) {
  for (uint32_t e = 0; e < ms; e += 10) { t += 10; s.tick(t, v, valid); }
}
static int count_on(const PowerSequencer& s) { int n = 0; for (int i = 0; i < 4; i++) n += s.on[i]; return n; }
static void active(PowerSequencer& s, uint32_t t) { s.setMode(PM_ACTIVE, t); }

static void test_boot_is_safe() {
  PowerSequencer s; uint32_t t = 0;
  for (int i = 0; i < 4; i++) s.setWant(i, true);
  run(s, t, 5000);
  CHECK(s.mode == PM_SLEEP); CHECK(count_on(s) == 0);   // wants ignored until ACTIVE
}

static void test_staggered_start_order() {
  PowerSequencer s; uint32_t t = 1000; active(s, t);
  for (int i = 0; i < 4; i++) s.setWant(i, true);
  int maxSimultaneousSwitchOns = 0; bool order[4] = {0}; int seq[4], n = 0;
  for (uint32_t e = 0; e < 4000; e += 10) {
    bool before[4]; for (int i = 0; i < 4; i++) before[i] = s.on[i];
    t += 10; s.tick(t, 5.0f, true);
    int switched = 0;
    for (int i = 0; i < 4; i++) if (s.on[i] && !before[i]) { switched++; if (n < 4) seq[n++] = i; }
    if (switched > maxSimultaneousSwitchOns) maxSimultaneousSwitchOns = switched;
  }
  (void)order;
  CHECK(maxSimultaneousSwitchOns == 1);            // never two ON edges in one tick
  CHECK(n == 4 && seq[0] == 1 && seq[1] == 2 && seq[2] == 0 && seq[3] == 3);  // L, R, pump, aux
  CHECK(count_on(s) == 4);
}

static void test_gap_between_starts() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  s.setWant(0, true); s.setWant(1, true);
  uint32_t first = 0, second = 0;
  for (uint32_t e = 0; e < 3000; e += 10) {
    bool b1 = s.on[1], b0 = s.on[0];
    t += 10; s.tick(t, 5.0f, true);
    if (s.on[1] && !b1) first = t;
    if (s.on[0] && !b0) second = t;
  }
  CHECK(first > 0 && second > 0 && (second - first) >= 600);   // default stagger = 600 ms
}

static void test_off_is_immediate_and_rest_time() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  s.setWant(0, true); run(s, t, 200); CHECK(s.on[0]);
  s.setWant(0, false); run(s, t, 10); CHECK(!s.on[0]);          // immediate
  s.setWant(0, true);  run(s, t, 500); CHECK(!s.on[0]);          // still resting (< minOffMs)
  run(s, t, 700); CHECK(s.on[0]);
}

static void test_sleep_and_emergency() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  for (int i = 0; i < 4; i++) s.setWant(i, true);
  run(s, t, 4000); CHECK(count_on(s) == 4);
  s.setMode(PM_SLEEP, t); run(s, t, 10); CHECK(count_on(s) == 0 && s.mode == PM_SLEEP);
  active(s, t); run(s, t, 4000); CHECK(count_on(s) == 4);
  s.setMode(PM_EMERGENCY, t); s.tick(t + 10, 5.0f, true); t += 10;
  CHECK(count_on(s) == 0 && s.mode == PM_EMERGENCY);
  active(s, t); run(s, t, 3000);                                  // ignored: latched
  CHECK(s.mode == PM_EMERGENCY && count_on(s) == 0);
  s.reset(t); CHECK(s.mode == PM_SLEEP && count_on(s) == 0);      // reset -> SLEEP, not ACTIVE
  active(s, t); run(s, t, 4000); CHECK(count_on(s) == 4);
}

static void test_brownout_sheds_least_important() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  for (int i = 0; i < 4; i++) s.setWant(i, true);
  run(s, t, 4000); CHECK(count_on(s) == 4);
  run(s, t, 200, 4.40f);                                           // sag
  CHECK(!s.on[3] && s.on[0] && s.on[1] && s.on[2]);                // aux (rank 3) shed first
  CHECK(s.brownouts == 1 && s.rail == RAIL_SAG);
  run(s, t, 6000, 5.0f);                                           // recovered, backoff over
  CHECK(s.on[3]);                                                  // aux restarts by itself
}

static void test_no_start_while_rail_low_or_lockout() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  s.setWant(1, true);
  run(s, t, 1000, 4.60f);                                          // below startMinV (4.75)
  CHECK(!s.on[1] && s.pending);
  run(s, t, 100, 5.0f); CHECK(s.on[1]);
}

static void test_critical_rail_latches_emergency() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  s.setWant(1, true); run(s, t, 1000); CHECK(s.on[1]);
  run(s, t, 1200, 4.0f);
  CHECK(s.mode == PM_EMERGENCY && count_on(s) == 0 && std::strstr(s.fault, "critical"));
  run(s, t, 3000, 5.0f); CHECK(s.mode == PM_EMERGENCY);            // stays latched
}

static void test_repeated_brownouts_escalate() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  for (int i = 0; i < 4; i++) s.setWant(i, true);
  run(s, t, 4000);
  run(s, t, 4000, 4.4f);                                           // sustained sag
  CHECK(s.mode == PM_EMERGENCY && std::strstr(s.fault, "brownout"));
}

static void test_sustained_sag_sheds_one_load_per_settle_interval() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  for (int i = 0; i < 4; i++) s.setWant(i, true);
  run(s, t, 4000);
  run(s, t, 160, 4.4f);   CHECK(count_on(s) == 3);                 // first shed after ~150 ms
  run(s, t, 600, 4.4f);   CHECK(count_on(s) == 3);                 // settling: no second shed yet
  run(s, t, 500, 4.4f);   CHECK(count_on(s) == 2 && s.mode == PM_ACTIVE);
  CHECK(!s.on[3] && !s.on[0] && s.on[1] && s.on[2]);               // aux, then pump; motors kept
}

static void test_streak_decays_when_healthy() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  for (int i = 0; i < 4; i++) s.setWant(i, true);
  run(s, t, 4000);
  run(s, t, 200, 4.4f); run(s, t, 40000, 5.0f);                    // one sag, long recovery
  run(s, t, 200, 4.4f); run(s, t, 40000, 5.0f);
  run(s, t, 200, 4.4f);
  CHECK(s.mode == PM_ACTIVE && s.brownouts == 3);                  // never escalated
}

static void test_streak_resets_on_reactivation() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  for (int i = 0; i < 4; i++) s.setWant(i, true);
  run(s, t, 4000);
  run(s, t, 200, 4.4f); run(s, t, 6000, 5.0f); run(s, t, 200, 4.4f);   // 2 sheds
  s.setMode(PM_SLEEP, t); run(s, t, 100, 5.0f); active(s, t); run(s, t, 8000, 5.0f);
  run(s, t, 200, 4.4f); run(s, t, 6000, 5.0f); run(s, t, 200, 4.4f);   // 2 more sheds
  CHECK(s.mode == PM_ACTIVE);
}

static void test_invalid_rail_reading_uses_timers_only() {
  PowerSequencer s; uint32_t t = 0; active(s, t);
  s.setWant(1, true); s.setWant(2, true);
  run(s, t, 2000, 0.0f, false);
  CHECK(s.on[1] && s.on[2] && s.rail == RAIL_UNKNOWN);
}

static void test_active_low_level() {
  PowerSequencer s; uint32_t t = 0; active(s, t); s.setWant(1, true); run(s, t, 100);
  CHECK(s.pinLevel(1, true) == false && s.pinLevel(1, false) == true);   // ON: LOW for active-low
  CHECK(s.pinLevel(0, true) == true);                                     // OFF: HIGH for active-low
}

static void test_millis_wraparound() {
  PowerSequencer s; uint32_t t = 0xFFFFFF00u; active(s, t);
  s.setWant(0, true); s.setWant(1, true);
  run(s, t, 3000);
  CHECK(s.on[0] && s.on[1]);
}

int main() {
  test_boot_is_safe(); test_staggered_start_order(); test_gap_between_starts();
  test_off_is_immediate_and_rest_time(); test_sleep_and_emergency();
  test_brownout_sheds_least_important(); test_no_start_while_rail_low_or_lockout();
  test_critical_rail_latches_emergency(); test_repeated_brownouts_escalate();
  test_sustained_sag_sheds_one_load_per_settle_interval(); test_streak_decays_when_healthy(); test_streak_resets_on_reactivation(); test_invalid_rail_reading_uses_timers_only();
  test_active_low_level(); test_millis_wraparound();
  if (failures == 0) std::printf("ALL TESTS PASSED\n");
  return failures ? 1 : 0;
}
