// relay-sequencer : staggered relay start-up, brownout load-shedding and power modes.
// Issue #10 - Power Distribution & Load Management (XOD node role).
// The class below is identical to core/power_core.h and esp32/power_management/power_core.h.

#include <stdint.h>

#define PC_CHANNELS 4

enum PowerMode : uint8_t { PM_ACTIVE = 0, PM_SLEEP = 1, PM_EMERGENCY = 2 };
enum RailState : uint8_t { RAIL_UNKNOWN = 0, RAIL_OK = 1, RAIL_SAG = 2, RAIL_CRITICAL = 3 };

struct PowerConfig {
  uint32_t staggerMs       = 600;    // min gap between two relay switch-ONs
  uint32_t minOffMs        = 1000;   // a channel must rest this long before re-start
  uint32_t shedDebounceMs  = 150;    // rail must stay low this long before the FIRST shed
  uint32_t shedSettleMs    = 1000;   // ...and this long after a shed before the NEXT one
  uint32_t shedBackoffMs   = 5000;   // no new starts for this long after a shed
  uint32_t criticalHoldMs  = 1000;   // rail < criticalV this long -> EMERGENCY
  uint32_t streakDecayMs   = 30000;  // healthy rail this long forgives earlier sags
  uint8_t  maxBrownouts    = 3;      // sheds in a row (without recovery) -> EMERGENCY
  float    startMinV       = 4.75f;  // do not start a load below this rail voltage
  float    shedV           = 4.50f;  // below this: shed one load
  float    criticalV       = 4.20f;  // below this: emergency shutdown
  // Start order / importance: lowest rank starts first and is shed last.
  // Default (CH1 intake pump, CH2 left motor, CH3 right motor, CH4 aux pump):
  uint8_t  rank[PC_CHANNELS] = {2, 0, 1, 3};
};

struct PowerSequencer {
  PowerConfig cfg;

  // ---- inputs (set by the caller) ----
  bool want[PC_CHANNELS] = {false, false, false, false};

  // ---- outputs (read by the caller) ----
  bool      on[PC_CHANNELS] = {false, false, false, false};
  PowerMode mode        = PM_SLEEP;     // boot-safe: everything off
  RailState rail        = RAIL_UNKNOWN;
  bool      pending     = false;        // a wanted channel is waiting to start
  uint16_t  brownouts   = 0;            // total sheds since boot
  const char* fault     = "";           // last emergency reason ("" = none)

  // ---- internal timing state ----
  bool     latched = false;             // emergency latch (cleared by reset())
  bool     anyStart = false;
  uint32_t lastStartMs = 0;
  bool     hasOff[PC_CHANNELS] = {false, false, false, false};
  uint32_t offAtMs[PC_CHANNELS] = {0, 0, 0, 0};
  bool     lockout = false;
  uint32_t lockoutAtMs = 0;
  bool     lowActive = false;   uint32_t lowSinceMs = 0;   bool justShed = false;
  bool     critActive = false;  uint32_t critSinceMs = 0;
  bool     okActive = false;    uint32_t okSinceMs = 0;
  uint8_t  streak = 0;

  static uint32_t since(uint32_t now, uint32_t t) { return now - t; }  // wrap-safe

  // ---- commands ----
  void setMode(PowerMode m, uint32_t nowMs) {
    if (m == PM_EMERGENCY) { enterEmergency("operator emergency shutdown", nowMs); return; }
    if (latched) return;                       // only reset() leaves EMERGENCY
    if (m == PM_ACTIVE && mode != PM_ACTIVE) streak = 0;
    mode = m;
  }

  // Clears the emergency latch. The system returns to SLEEP (all loads off);
  // the operator must then choose ACTIVE on purpose.
  void reset(uint32_t nowMs) {
    (void)nowMs;
    if (!latched) return;
    latched = false;
    mode = PM_SLEEP;
    fault = "";
    streak = 0;
    lockout = false;
    lowActive = critActive = false;
    justShed = false;
  }

  void setWant(uint8_t ch, bool v) { if (ch < PC_CHANNELS) want[ch] = v; }

  // ---- main update: call every 10-50 ms ----
  void tick(uint32_t nowMs, float railV, bool railValid) {
    updateRail(nowMs, railV, railValid);

    if (mode != PM_ACTIVE) {                   // SLEEP or EMERGENCY: all loads off
      for (uint8_t i = 0; i < PC_CHANNELS; i++) switchOff(i, nowMs);
      pending = false;
      return;
    }

    if (lockout && since(nowMs, lockoutAtMs) >= cfg.shedBackoffMs) lockout = false;

    for (uint8_t i = 0; i < PC_CHANNELS; i++)  // switching OFF is always immediate
      if (on[i] && !want[i]) switchOff(i, nowMs);

    // switching ON: at most ONE channel per call, lowest rank first
    int8_t pick = -1;
    for (uint8_t i = 0; i < PC_CHANNELS; i++)
      if (want[i] && !on[i] && (pick < 0 || cfg.rank[i] < cfg.rank[pick])) pick = i;

    pending = (pick >= 0);
    if (pick < 0) return;

    bool staggerOk = !anyStart || since(nowMs, lastStartMs) >= cfg.staggerMs;
    bool restOk    = !hasOff[pick] || since(nowMs, offAtMs[pick]) >= cfg.minOffMs;
    bool railOk    = !railValid || railV >= cfg.startMinV;
    if (staggerOk && restOk && railOk && !lockout) {
      on[pick] = true;
      anyStart = true;
      lastStartMs = nowMs;
      pending = false;
      for (uint8_t i = 0; i < PC_CHANNELS; i++)      // is another channel still waiting?
        if (want[i] && !on[i]) pending = true;
    }
  }

  // GPIO level to write for a channel (relay boards are normally active-LOW)
  bool pinLevel(uint8_t ch, bool activeLow) const { return activeLow ? !on[ch] : on[ch]; }

  static const char* modeName(PowerMode m) {
    return m == PM_ACTIVE ? "ACTIVE" : (m == PM_SLEEP ? "SLEEP" : "EMERGENCY");
  }
  static const char* railName(RailState r) {
    return r == RAIL_OK ? "OK" : (r == RAIL_SAG ? "SAG" : (r == RAIL_CRITICAL ? "CRITICAL" : "UNKNOWN"));
  }

 private:
  void switchOff(uint8_t i, uint32_t now) {
    if (!on[i]) return;
    on[i] = false;
    hasOff[i] = true;
    offAtMs[i] = now;
  }

  void enterEmergency(const char* why, uint32_t now) {
    latched = true;
    mode = PM_EMERGENCY;
    fault = why;
    for (uint8_t i = 0; i < PC_CHANNELS; i++) switchOff(i, now);
    pending = false;
  }

  void updateRail(uint32_t now, float v, bool valid) {
    if (!valid) {                              // no usable reading: timers only
      rail = RAIL_UNKNOWN; lowActive = critActive = okActive = false;
      return;
    }
    rail = v < cfg.criticalV ? RAIL_CRITICAL : (v < cfg.shedV ? RAIL_SAG : RAIL_OK);

    if (v < cfg.criticalV) {
      if (!critActive) { critActive = true; critSinceMs = now; }
      else if (since(now, critSinceMs) >= cfg.criticalHoldMs && mode != PM_EMERGENCY)
        enterEmergency("rail critical (< criticalV)", now);
    } else critActive = false;

    if (v < cfg.shedV) {
      okActive = false;
      if (!lowActive) { lowActive = true; lowSinceMs = now; }
      else if (since(now, lowSinceMs) >= (justShed ? cfg.shedSettleMs : cfg.shedDebounceMs)) {
        lowSinceMs = now;
        if (mode == PM_ACTIVE && shedOne(now)) {
          justShed = true;
          brownouts++;
          streak++;
          lockout = true; lockoutAtMs = now;
          if (streak >= cfg.maxBrownouts) enterEmergency("repeated brownouts", now);
        }
      }
    } else {
      lowActive = false;
      justShed = false;
      if (!okActive) { okActive = true; okSinceMs = now; }
      else if (since(now, okSinceMs) >= cfg.streakDecayMs) streak = 0;
    }
  }

  // switch off the running channel with the HIGHEST rank (least important)
  bool shedOne(uint32_t now) {
    int8_t victim = -1;
    for (uint8_t i = 0; i < PC_CHANNELS; i++)
      if (on[i] && (victim < 0 || cfg.rank[i] > cfg.rank[victim])) victim = i;
    if (victim < 0) return false;
    switchOff(victim, now);
    return true;
  }
};

node {
    PowerSequencer seq;
    bool firstRun = true;
    bool lastPin[PC_CHANNELS] = {false, false, false, false};
    uint8_t lastRail = 255, lastMode = 255, lastBrown = 255;
    bool lastPend = false;

    void evaluate(Context ctx) {
        uint32_t now = millis();

        seq.cfg.staggerMs = (uint32_t) getValue<input_STAG>(ctx);
        if (firstRun || isInputDirty<input_MODE>(ctx)) {
            uint8_t m = (uint8_t) getValue<input_MODE>(ctx);
            seq.setMode(m == 0 ? PM_ACTIVE : (m == 2 ? PM_EMERGENCY : PM_SLEEP), now);
        }
        if (isInputDirty<input_RST>(ctx)) seq.reset(now);

        seq.setWant(0, getValue<input_REQ1>(ctx));
        seq.setWant(1, getValue<input_REQ2>(ctx));
        seq.setWant(2, getValue<input_REQ3>(ctx));
        seq.setWant(3, getValue<input_REQ4>(ctx));

        float v = getValue<input_VBAT>(ctx);
        seq.tick(now, v, v > 1.0f);

        bool activeLow = getValue<input_ACTLOW>(ctx);
        if (firstRun || seq.pinLevel(0, activeLow) != lastPin[0]) { lastPin[0] = seq.pinLevel(0, activeLow); emitValue<output_PIN1>(ctx, lastPin[0]); }
        if (firstRun || seq.pinLevel(1, activeLow) != lastPin[1]) { lastPin[1] = seq.pinLevel(1, activeLow); emitValue<output_PIN2>(ctx, lastPin[1]); }
        if (firstRun || seq.pinLevel(2, activeLow) != lastPin[2]) { lastPin[2] = seq.pinLevel(2, activeLow); emitValue<output_PIN3>(ctx, lastPin[2]); }
        if (firstRun || seq.pinLevel(3, activeLow) != lastPin[3]) { lastPin[3] = seq.pinLevel(3, activeLow); emitValue<output_PIN4>(ctx, lastPin[3]); }
        if (firstRun || seq.rail != lastRail)       { lastRail = seq.rail;           emitValue<output_RAIL>(ctx, (Number) lastRail); }
        if (firstRun || seq.mode != lastMode)       { lastMode = seq.mode;           emitValue<output_MODEO>(ctx, (Number) lastMode); }
        if (firstRun || seq.pending != lastPend)    { lastPend = seq.pending;        emitValue<output_PEND>(ctx, lastPend); }
        if (firstRun || seq.brownouts != lastBrown) { lastBrown = (uint8_t) seq.brownouts; emitValue<output_BROWN>(ctx, (Number) seq.brownouts); }

        firstRun = false;
        setTimeout(ctx, 20);   // re-evaluate every 20 ms so the timers keep running
    }
}
