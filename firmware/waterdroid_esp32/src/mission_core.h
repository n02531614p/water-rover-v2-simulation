// ============================================================================
// mission_core.h - Issue #11 central state machine (Navigating -> Pumping ->
// Dosing -> Delivering) for Water Rover v2. Pure C++11, time passed in.
//
// The ESP32 has no localisation hardware yet (Issue #7 sensors are not in the
// parts list), so ARRIVAL is an input event: it comes from the navigation
// subsystem when fitted, from the ROS 2 simulation, or from the operator's
// "Arrived" buttons on the Node-RED dashboard. Everything after arrival -
// pumping, dosing, waiting for the filter, the arm and the delivery pump - is
// sequenced here, and the motors are inhibited whenever a pump may run so the
// power bank never sees drive + pump inrush at the same time (Issue #10).
//
// State ids match the Node-RED serial bridge (esp32_central_controller_serial_bridge.json):
//   0 IDLE  1 NAV_RIVER  2 INTAKE_DOSE  3 NAV_HOMESTEAD  4 DELIVER_WATER  5 EMERGENCY_STOP
// ============================================================================
#ifndef MISSION_CORE_H
#define MISSION_CORE_H

#include <stdint.h>

enum MissionState : uint8_t {
  MS_IDLE = 0, MS_EN_ROUTE_RIVER, MS_PUMPING, MS_FILTERING, MS_DOSING_CHLORINE,
  MS_EN_ROUTE_HOMESTEAD, MS_DISPENSING, MS_RETURNING, MS_COMPLETE, MS_PAUSED, MS_EMERGENCY_STOP
};

struct MissionInputs {
  bool  intakeBusy = false;     // IntakeController.want or PUMPING
  bool  alumBusy = false, chlorineBusy = false;
  float s1Pct = 0, s2Pct = 0;   // filtration levels
  bool  armDeployed = false, armStowed = true;
  bool  deliveryBusy = false;
  float cleanMl = 0;            // Stage 3 volume estimate
};

struct MissionActions {          // one-shot requests, cleared by the caller after use
  bool  startIntake = false;
  float doseAlumMl = 0;          // > 0 -> start an alum dose
  float doseChlorineMl = 0;      // > 0 -> start a chlorine dose
  bool  armExtend = false, armRetract = false;
  float deliverMl = 0;           // > 0 -> request a delivery
  bool  stopAll = false;
  void clear() { *this = MissionActions(); }
};

struct MissionConfig {
  uint8_t  homesteads = 3;
  float    batchMl = 380;         // Stage 1 / Stage 3 batch (400 mL tanks)
  float    alumMlPerL = 3.0f;     // 1 % alum stock
  float    chlorineMlPerL = 3.0f; // 0.07 % NaOCl stock (bleach 1:50) -> ~2 mg/L
  uint32_t pumpTimeoutMs = 120000;
  uint32_t filterTimeoutMs = 600000;
  uint32_t contactMs = 30UL * 60UL * 1000UL;   // chlorine contact time (WHO: >= 30 min)
  uint32_t actionTimeoutMs = 60000;
  float    filterDoneS1 = 5, filterDoneS2 = 10;
};

struct MissionCore {
  MissionConfig  cfg;
  MissionState   state = MS_IDLE;
  MissionActions act;
  bool           driveAllowed = true;   // false = motors inhibited
  uint8_t        stop = 0;              // homestead index being served
  uint8_t        sub = 0;
  uint32_t       t0 = 0;
  MissionState   resumeTo = MS_IDLE;
  bool           estop = false;

  // ---- operator / supervisor commands ----
  void start(uint32_t now)  { if (state == MS_IDLE || state == MS_COMPLETE) { stop = 0; go(MS_EN_ROUTE_RIVER, now); } }
  void pause(uint32_t now)  { if (state != MS_PAUSED && state != MS_EMERGENCY_STOP && state != MS_IDLE) { resumeTo = state; act.stopAll = true; go(MS_PAUSED, now); } }
  void resume(uint32_t now) { if (state == MS_PAUSED) go(resumeTo, now); }
  void abort(uint32_t now)  { act.stopAll = true; act.armRetract = true; go(MS_RETURNING, now); }
  void setEstop(bool on, uint32_t now) {
    if (on && !estop) { resumeTo = (state == MS_EMERGENCY_STOP) ? resumeTo : state; act.stopAll = true; go(MS_EMERGENCY_STOP, now); }
    if (!on && estop) go(MS_PAUSED, now);   // release -> paused; operator must resume on purpose
    estop = on;
  }

  // ---- arrival events (navigation / ROS / dashboard) ----
  void arrivedRiver(uint32_t now)     { if (state == MS_EN_ROUTE_RIVER) go(MS_PUMPING, now); }
  void arrivedHomestead(uint32_t now) { if (state == MS_EN_ROUTE_HOMESTEAD) go(MS_DISPENSING, now); }
  void arrivedDepot(uint32_t now)     { if (state == MS_RETURNING) go(MS_COMPLETE, now); }

  void tick(uint32_t now, const MissionInputs& in) {
    uint32_t el = now - t0;
    switch (state) {
      case MS_PUMPING:
        if (sub == 0) {
          act.startIntake = true;
          act.doseAlumMl = cfg.batchMl / 1000.0f * cfg.alumMlPerL;
          sub = 1; t0 = now;
        } else if (el > 1000 && ((!in.intakeBusy && !in.alumBusy) || el > cfg.pumpTimeoutMs)) {
          go(MS_FILTERING, now);
        }
        break;
      case MS_FILTERING:
        if ((in.s1Pct <= cfg.filterDoneS1 && in.s2Pct <= cfg.filterDoneS2) || el > cfg.filterTimeoutMs)
          go(MS_DOSING_CHLORINE, now);
        break;
      case MS_DOSING_CHLORINE:
        if (sub == 0) { act.doseChlorineMl = in.cleanMl / 1000.0f * cfg.chlorineMlPerL; sub = 1; t0 = now; }
        else if (sub == 1 && el > 500 && !in.chlorineBusy) { sub = 2; t0 = now; }
        else if (sub == 2 && el >= cfg.contactMs) { stop = 0; go(MS_EN_ROUTE_HOMESTEAD, now); }
        break;
      case MS_DISPENSING:
        if (sub == 0) { act.armExtend = true; sub = 1; t0 = now; }
        else if (sub == 1 && (in.armDeployed || el > cfg.actionTimeoutMs)) {
          uint8_t left = cfg.homesteads > stop ? cfg.homesteads - stop : 1;
          act.deliverMl = in.cleanMl / left;
          sub = 2; t0 = now;
        } else if (sub == 2 && el > 500 && (!in.deliveryBusy || el > cfg.actionTimeoutMs)) {
          act.armRetract = true; sub = 3; t0 = now;
        } else if (sub == 3 && (in.armStowed || el > cfg.actionTimeoutMs)) {
          stop++;
          go(stop < cfg.homesteads ? MS_EN_ROUTE_HOMESTEAD : MS_RETURNING, now);
        }
        break;
      default:
        break;
    }
    driveAllowed = allowedFor(state);
  }

  // Motors may run only while travelling, idle or paused (manual teleop) -
  // never while pumping / dosing / dispensing, never in EMERGENCY_STOP.
  static bool allowedFor(MissionState s) {
    return !(s == MS_PUMPING || s == MS_FILTERING || s == MS_DOSING_CHLORINE ||
             s == MS_DISPENSING || s == MS_EMERGENCY_STOP);
  }

  uint8_t stateId() const {
    switch (state) {
      case MS_EN_ROUTE_RIVER: return 1;
      case MS_PUMPING: case MS_FILTERING: case MS_DOSING_CHLORINE: return 2;
      case MS_EN_ROUTE_HOMESTEAD: case MS_RETURNING: return 3;
      case MS_DISPENSING: return 4;
      case MS_EMERGENCY_STOP: return 5;
      default: return 0;
    }
  }
  static const char* name(MissionState s) {
    static const char* N[] = {"STATE_IDLE", "STATE_EN_ROUTE_RIVER", "STATE_PUMPING", "STATE_FILTERING",
                              "STATE_DOSING_CHLORINE", "STATE_EN_ROUTE_HOMESTEAD", "STATE_DISPENSING",
                              "STATE_RETURNING", "STATE_COMPLETE", "STATE_PAUSED", "STATE_EMERGENCY_STOP"};
    return N[s];
  }

 private:
  void go(MissionState s, uint32_t now) { state = s; sub = 0; t0 = now; driveAllowed = allowedFor(s); }
};

#endif  // MISSION_CORE_H
