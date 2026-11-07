/*
 * Water Rover v2 / Waterdroid - integrated firmware for the ONE ESP32 "autonomous brain"
 *
 * Merges every team member's subsystem onto one board with one pin map (src/rover_pins.h):
 *   #1  Intake & raw water collection      src/pump_cores.h   IntakeController   relay CH1
 *   #3  Chlorine dosing                    src/pump_cores.h   DosingPump         relay CH3
 *   #5  Transfer pump column -> delivery   src/pump_cores.h   DosingPump         relay CH2 (alum dropped)
 *   #5  Filtration column monitoring       src/motion_cores.h FiltrationMonitor  S1/S2/S3 ADC
 *   #6  Servo-driven INTAKE boom (SG90)    src/motion_cores.h ServoArm           GPIO13 (delivery nozzle is fixed)
 *   #8  Clean water delivery pump          src/pump_cores.h   DeliveryController relay CH4
 *   #9  4WD propulsion (L298N)             src/motion_cores.h DriveController    ENA/ENB + IN1-4
 *   #10 Power distribution & sequencing    src/power_core.h   PowerSequencer     all relays + rail
 *   #11 Central state machine              src/mission_core.h MissionCore        + serial bridge
 *   #12 MQTT telemetry / dashboard / E-stop (this file)
 *   #7  Fixed route by dead reckoning     src/nav_cores.h    RouteNavigator     (no GPS / compass yet)
 *   #1/#5 Water-flow sensor + volume model src/nav_cores.h   FlowMeter, VolumeModel (replaces level sensors)
 * Each core is pure C++ shared with the XOD nodes (xod/waterdroid) and the host tests
 * (firmware/tests), so the logic is tested off-board.
 *
 * Board : ESP32 DevKit V1, Arduino-ESP32 core >= 2.0.x
 * Libs  : PubSubClient (Nick O'Leary), ESP32Servo (Kevin Harrington)
 * MQTT  : topic map in docs/TOPICS.md (all under rover/...)
 * Serial: 115200 baud, one JSON line per second {"state_id":..,"battery_v":..} for the
 *         Node-RED serial bridge; accepts {"cmd":"SET_STATE","state":1} / {"cmd":"ESTOP"}
 *         and plain lines: start | pause | resume | abort | estop | reset |
 *         arrived_river | arrived_home | arrived_depot | nav_auto | nav_manual | nav_reset |
 *         calib_forward | calib_turn
 *
 * The loop never blocks: relays, pumps and the e-stop keep working when Wi-Fi is down.
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

#include "src/rover_pins.h"
#include "src/power_core.h"
#include "src/pump_cores.h"
#include "src/motion_cores.h"
#include "src/mission_core.h"
#include "src/nav_cores.h"

// ============================== CONFIG ==============================
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";
const char* MQTT_HOST = "192.168.1.50";        // Node-RED / Mosquitto host
const uint16_t MQTT_PORT = 1883;
const char* MQTT_CLIENT = "waterdroid-esp32";

// ---- Sensors fitted (Nov 2026: ONE water-flow sensor on the intake line, no level sensors) ----
#define USE_LEVEL_SENSORS 0                    // 1 = analog level sensors on S2 (GPIO35) and S3 (GPIO36)
const float FLOW_PULSES_PER_L = 5880.0f;       // YF-S401 (0.3-6 L/min) ~5880 pulses/L. CALIBRATE: pump 400 mL into a jug
const float FILTER_ML_PER_S   = 3.0f;          // gravity Stage 1 -> column. CALIBRATE: time 380 mL through it
const float TRANSFER_ML_PER_S = 8.0f;          // transfer pump (column -> delivery tank). CALIBRATE: time 200 mL
const float BATCH_ML          = 380.0f;        // water per trip (400 mL tanks, leave headroom)

// ---- Fixed route, dead reckoning (no GPS / compass yet). Route points: src/nav_cores.h ----
const float NAV_SPEED_MPS = 0.30f;             // CALIBRATE: send "calib_forward" (5 s), measure metres / 5
const float NAV_TURN_DPS  = 90.0f;             // CALIBRATE: send "calib_turn" (5 s pivot), measure degrees / 5
bool autoNav = true;                           // false = drive by dashboard D-pad and press "Arrived"

// Level sensor calibration (only with USE_LEVEL_SENSORS): raw ADC dry = empty, wet = full
struct Cal { int dry, wet; };
Cal CAL_S2 = {200, 2200}, CAL_S3 = {200, 2200};
const float CLEAN_TANK_ML = 400.0f;            // Stage 3 capacity

// 5 V rail divider (rover_pins.h) and calibration
const float R_TOP = 10000.0f, R_BOTTOM = 10000.0f, VCAL = 1.000f;

const uint8_t RELAY_PIN[PC_CHANNELS] = {PIN_RELAY_CH1, PIN_RELAY_CH2, PIN_RELAY_CH3, PIN_RELAY_CH4};
const char*   RELAY_NAME[PC_CHANNELS] = {"intake_pump", "transfer_pump", "chlorine_pump", "delivery_pump"};
enum { CH_INTAKE = 0, CH_TRANSFER = 1, CH_CHLORINE = 2, CH_DELIVERY = 3 };

// ============================== STATE ==============================
PowerSequencer     seq;
IntakeController   intake;
DosingPump         transfer, chlorine;   // transfer = column -> delivery tank (relay CH2)
DeliveryController delivery;
FiltrationMonitor  filt;
ServoArm           arm;
PumpTest           ptest;              // bench test mode: rover/test/cmd "intake 10"
DriveController    drive;
MissionCore        mission;
FlowMeter          flow;
VolumeModel        vol;
RouteNavigator     nav;
volatile uint32_t  flowPulses = 0;
void IRAM_ATTR onFlowPulse() { flowPulses = flowPulses + 1; }   // (++ on volatile is deprecated in C++20)

Servo        servo;
WiFiClient   wifi;
PubSubClient mqtt(wifi);

float s1 = 0, s2 = 0, s3 = 0;      // smoothed levels %
float railV = 0; bool railValid = false;
bool  estop = false;
bool  atHomestead = false;         // true between arrived_home and leaving DISPENSING
float intakeTargetMl = BATCH_ML;   // this fill (batch minus water already on board)
float lastFlowMl = 0, lastTransferMl = 0, lastChlorMl = 0, lastDelivMl = 0;
uint32_t calibUntil = 0; const char* calibCmd = "stop";
uint32_t tSample = 0, tLogic = 0, tPub = 0, tNet = 0, tSerial = 0, tWifiKick = 0;
String serialLine;

// ============================== HELPERS ==============================
float readLevel(int pin, const Cal& c) { return levelPct(analogRead(pin), c.dry, c.wet); }

void sampleRail() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_RAIL_SENSE);
  float pinV = (mv / 8.0f) / 1000.0f;
  railValid = pinV > 0.2f;
  float v = pinV * (R_TOP + R_BOTTOM) / R_BOTTOM * VCAL;
  railV = railValid ? (railV == 0 ? v : railV * 0.7f + v * 0.3f) : 0.0f;
}

void setEstop(bool on, const char* src) {
  uint32_t now = millis();
  if (on == estop) return;
  estop = on;
  mission.setEstop(on, now);
  arm.freeze(on);
  if (on) {
    seq.setMode(PM_EMERGENCY, now);
    intake.stop(); transfer.abort(); chlorine.abort(); delivery.abort("e-stop"); ptest.stopAll();
    drive.stopNow();
  } else {
    seq.reset(now);                  // -> SLEEP; operator returns to ACTIVE on purpose
    seq.setMode(PM_ACTIVE, now);
  }
  Serial.printf("{\"event\":\"estop\",\"on\":%d,\"src\":\"%s\"}\n", on, src);
  if (mqtt.connected()) mqtt.publish("rover/estop", on ? "1" : "0", true);
}

void startIntake() {
  float onBoard = vol.rawMl + vol.columnMl + vol.cleanMl;
  intakeTargetMl = BATCH_ML - onBoard;
  if (intakeTargetMl < 0) intakeTargetMl = 0;
  flow.resetBatch();
  intake.start();
}

// ----- one command dispatcher for MQTT, serial and the mission -----
void missionCommand(const String& c) {
  uint32_t now = millis();
  if (c == "start") { if (mission.state == MS_IDLE || mission.state == MS_COMPLETE) nav.reset(); mission.start(now); }
  else if (c == "nav_auto") autoNav = true;
  else if (c == "nav_manual") { autoNav = false; nav.halt(); }
  else if (c == "nav_reset") nav.reset();                       // rover is at the depot, facing the river
  else if (c == "calib_forward" || c == "calib_turn") {         // 5 s timed move for calibration
    if (mission.state == MS_IDLE || mission.state == MS_PAUSED || mission.state == MS_COMPLETE) {
      calibCmd = (c == "calib_forward") ? "forward" : "left"; calibUntil = now + 5000;
    }
  }
  else if (c == "pause") mission.pause(now);
  else if (c == "resume") mission.resume(now);
  else if (c == "abort" || c == "home") mission.abort(now);
  // manual arrival = also re-sync the dead-reckoned pose to that route point
  else if (c == "arrived_river") { nav.syncTo(RA_RIVER); mission.arrivedRiver(now); }
  else if (c == "arrived_home") { nav.syncTo(RA_HOME); atHomestead = true; mission.arrivedHomestead(now); }
  else if (c == "arrived_depot") { nav.syncTo(RA_DEPOT); mission.arrivedDepot(now); }
}

float jsonNumber(const String& s, const char* key, float def) {
  int k = s.indexOf(String("\"") + key + "\"");
  if (k < 0) return def;
  int c = s.indexOf(':', k);
  return c < 0 ? def : s.substring(c + 1).toFloat();
}

// Bench test: "<pump> <seconds>" e.g. "intake 10", "delivery 5"; "stop" ends every test run.
// Ignores the flow sensor / tank estimates; the e-stop, the brownout guard and the
// intake-lowered interlock still apply (an intake test lowers the intake first).
void testCommand(const String& c) {
  uint32_t now = millis();
  if (c == "stop" || c == "off") { ptest.stopAll(); return; }
  if (estop) return;
  int sp = c.indexOf(' ');
  String name = sp < 0 ? c : c.substring(0, sp);
  float secs = sp < 0 ? 5.0f : c.substring(sp + 1).toFloat();
  int ch = name == "intake" ? CH_INTAKE : name == "transfer" ? CH_TRANSFER :
           (name == "chlorine" || name == "dosing") ? CH_CHLORINE : (name == "delivery" || name == "outlet") ? CH_DELIVERY : -1;
  if (ch < 0) return;
  if (ch == CH_INTAKE) arm.command("EXTEND");
  if (ptest.start((uint8_t)ch, secs, now))
    Serial.printf("{\"event\":\"test\",\"pump\":\"%s\",\"s\":%.1f}\n", RELAY_NAME[ch], secs);
}

void onMqtt(char* topic, byte* payload, unsigned int len) {
  String t(topic), m;
  for (unsigned i = 0; i < len; i++) m += (char)payload[i];
  m.trim();
  String lm = m; lm.toLowerCase();
  uint32_t now = millis();

  if (t == "rover/estop") setEstop(lm == "1" || lm == "true" || lm == "on", "mqtt");
  else if (t == "rover/mission/cmd") missionCommand(lm);
  else if (t == "rover/power/cmd") {
    if (lm == "active") seq.setMode(PM_ACTIVE, now);
    else if (lm == "sleep") seq.setMode(PM_SLEEP, now);
    else if (lm == "emergency" || lm == "estop") setEstop(true, "power");
    else if (lm == "reset") setEstop(false, "power");
  }
  else if (t == "rover/intake/cmd") {             // manual intake: lower the servo boom first, pump once it is down
    if (lm == "start" || lm == "on") { arm.command("EXTEND"); startIntake(); }
    else if (lm == "reset") intake.clearFault();
    else intake.stop();
  }
  else if (t == "rover/nav/cmd") missionCommand(lm.startsWith("nav_") || lm.startsWith("calib_") ? lm : "nav_" + lm);
  else if (t == "rover/filtration/cmd") filt.manualPause = (lm == "pause");
  else if (t == "rover/dosing/cmd") {
    if (lm == "stop") chlorine.abort();
    else if (lm.indexOf("refill") >= 0) chlorine.setStock(250);
    else if (lm.indexOf("chlor") >= 0) chlorine.start(jsonNumber(lm, "ml", 0));   // alum is no longer fitted
  }
  else if (t == "rover/transfer/cmd") {           // transfer pump: {"ml":N} | run (= what is in the column) | stop
    if (lm == "stop") transfer.abort();
    else transfer.start(lm.startsWith("{") ? jsonNumber(lm, "ml", 0) : vol.columnMl);
  }
  else if (t == "rover/arm/cmd") arm.command(m.c_str());
  else if (t == "rover/test/cmd") testCommand(lm);
  else if (t == "rover/delivery/cmd") {
    if (lm == "stop") delivery.abort("operator");
    else delivery.request(jsonNumber(lm, "ml", 250));
  }
  else if (t == "rover/drive/cmd") {
    if (lm.startsWith("{")) drive.set((int)jsonNumber(lm, "l", 0), (int)jsonNumber(lm, "r", 0), now);
    else drive.named(lm.c_str(), now);
  }
}

void onSerialLine(String l) {
  l.trim();
  String u = l; u.toUpperCase();
  if (u.indexOf("ESTOP") >= 0) { setEstop(true, "serial"); return; }
  if (u.indexOf("SET_STATE") >= 0) {
    int st = (int)jsonNumber(l, "state", -1);
    if (st == 1) missionCommand("start");
    else if (st == 0) { setEstop(false, "serial"); missionCommand("abort"); }
    else if (st == 5) setEstop(true, "serial");
    return;
  }
  l.toLowerCase();
  if (l.startsWith("test ")) { testCommand(l.substring(5)); return; }
  if (l == "estop") setEstop(true, "serial");
  else if (l == "reset") setEstop(false, "serial");
  else if (l.startsWith("extend") || l.startsWith("retract") || l.startsWith("angle:")) arm.command(l.c_str());
  else missionCommand(l);
}

// ============================== CONTROL STEP (20 ms) ==============================
void controlStep() {
  uint32_t now = millis();

  // ---- #11 mission ----
  MissionInputs in;
  in.intakeBusy = intake.requested;            // paused by the column-overflow guard still counts as busy
  in.transferBusy = transfer.busy();
  in.columnMl = vol.columnMl;
  in.chlorineBusy = chlorine.busy();
  in.s1Pct = s1; in.s2Pct = s2;           // estimated by the VolumeModel without level sensors
  in.armDeployed = arm.deployed(); in.armStowed = arm.stowed();
  in.deliveryBusy = delivery.state == DL_WAIT_ARM || delivery.state == DL_PUMPING;
  in.cleanMl = vol.cleanMl;
  mission.tick(now, in);

  MissionActions& a = mission.act;
  if (a.stopAll) { intake.stop(); transfer.abort(); chlorine.abort(); delivery.abort("mission"); }
  if (a.startIntake) startIntake();
  if (a.transferMl > 0) transfer.start(a.transferMl);
  if (a.doseChlorineMl > 0) chlorine.start(a.doseChlorineMl);
  if (a.armExtend) arm.command("EXTEND");
  if (a.armRetract) arm.command("RETRACT");
  if (a.deliverMl > 0) delivery.request(a.deliverMl);
  a.clear();
  if (mission.state == MS_EN_ROUTE_HOMESTEAD || mission.state == MS_RETURNING || mission.state == MS_EN_ROUTE_RIVER)
    atHomestead = false;           // moving again -> no longer parked at a container

  // ---- water book-keeping: flow sensor + doses + deliveries -> estimated tank volumes ----
  flow.tick(now, flowPulses);
  vol.addRaw(flow.totalMl - lastFlowMl);          lastFlowMl = flow.totalMl;
  vol.moveColumnToClean(transfer.totalMl - lastTransferMl); lastTransferMl = transfer.totalMl;
  vol.addClean(chlorine.totalMl - lastChlorMl);   lastChlorMl = chlorine.totalMl;
  vol.removeClean(delivery.totalMl - lastDelivMl); lastDelivMl = delivery.totalMl;
  vol.tick(now);
#if !USE_LEVEL_SENSORS
  s1 = vol.rawPct(); s2 = vol.columnPct(); s3 = vol.cleanPct();
#endif

  // ---- #5 filtration + #1 intake: fill = flow-sensor volume, dry run = no pulses ----
  filt.tick(now, s1, s2, s3, seq.on[CH_INTAKE]);
  float fillPct = intakeTargetMl > 0 ? 100.0f * flow.batchMl / intakeTargetMl : 100.0f;
  // the intake pump only runs with the servo boom lowered into the water
  intake.tick(now, seq.on[CH_INTAKE], fillPct, !filt.allowIntake || estop || !arm.deployed());

  // ---- #3 dosing, #8 delivery ----
  transfer.tick(now, seq.on[CH_TRANSFER]);
  chlorine.tick(now, seq.on[CH_CHLORINE]);
  bool manualDelivery = mission.state == MS_IDLE || mission.state == MS_PAUSED || mission.state == MS_COMPLETE;
  delivery.tick(now, seq.on[CH_DELIVERY], atHomestead || manualDelivery, true, s3);   // fixed nozzle: no arm interlock

  // ---- #10 power sequencing: pumps start one at a time, brownout guard ----
  seq.setWant(CH_INTAKE, intake.want);
  seq.setWant(CH_TRANSFER, transfer.want);
  seq.setWant(CH_CHLORINE, chlorine.want);
  seq.setWant(CH_DELIVERY, delivery.want);
  for (uint8_t i = 0; i < PC_CHANNELS; i++) {        // bench test mode overrides the want (not the safety)
    bool w = ptest.tick(i, seq.on[i], now);
    if (i == CH_INTAKE && !arm.deployed()) w = false; // intake only with the boom lowered
    if (w && !estop) seq.setWant(i, true);
  }
  seq.tick(now, railV, railValid);
  for (uint8_t i = 0; i < PC_CHANNELS; i++) digitalWrite(RELAY_PIN[i], seq.pinLevel(i, RELAY_ACTIVE_LOW));

  // ---- #7 fixed route: dead-reckoning navigator drives the travel legs ----
  bool travelling = mission.state == MS_EN_ROUTE_RIVER || mission.state == MS_EN_ROUTE_HOMESTEAD ||
                    mission.state == MS_RETURNING;
  if (mission.state == MS_RETURNING)                 // abort / last leg: head straight for the depot
    while (nav.idx < nav.n && nav.pts[nav.idx].action != RA_DEPOT) nav.idx++;
  int8_t lastCmd = nav.cmd;
  float duty = (abs(drive.left) + abs(drive.right)) / 2.0f;
  float exec = lastCmd == 0 ? 0 : duty / ((lastCmd == 1 || lastCmd == -1) ? drive.cruise : drive.turn);
  nav.tick(now, exec);
  if (autoNav && travelling) nav.go(); else nav.halt();
  if (nav.event != RA_NONE) {
    if (nav.event == RA_RIVER) mission.arrivedRiver(now);
    else if (nav.event == RA_HOME) { atHomestead = true; mission.arrivedHomestead(now); }
    else if (nav.event == RA_DEPOT) mission.arrivedDepot(now);
    nav.event = RA_NONE;
  }
  if (autoNav && travelling) {
    static const char* NAMES[] = {"right", "reverse", "stop", "forward", "left"};   // cmd -2..2
    drive.named(NAMES[nav.cmd + 2], now);
  } else if ((int32_t)(calibUntil - now) > 0) {
    drive.named(calibCmd, now);
  }

  // ---- #9 drive: never together with a pump, never in e-stop ----
  bool anyPump = seq.on[0] || seq.on[1] || seq.on[2] || seq.on[3];
  // never drive with the intake lowered (the hose would drag) or while a pump runs
  drive.inhibit = estop || !mission.driveAllowed || anyPump || !arm.stowed() || seq.mode == PM_EMERGENCY;
  drive.tick(now);
  digitalWrite(PIN_L298_IN1, drive.out.in1); digitalWrite(PIN_L298_IN2, drive.out.in2);
  digitalWrite(PIN_L298_IN3, drive.out.in3); digitalWrite(PIN_L298_IN4, drive.out.in4);
  analogWrite(PIN_L298_ENA, drive.out.ena);  analogWrite(PIN_L298_ENB, drive.out.enb);

  // ---- #6 arm ----
  if (arm.tick(now)) servo.write(arm.angle);

  digitalWrite(PIN_LED, estop ? (now / 150) % 2 : (mission.state != MS_IDLE));
}

// ============================== TELEMETRY (#12) ==============================
void publishAll() {
  char js[700];
  const char* st = MissionCore::name(mission.state);

  snprintf(js, sizeof(js), "{\"state\":\"%s\",\"state_id\":%u,\"battery_v\":%.2f,\"stop\":%u}",
           st, mission.stateId(), railV, mission.stop);
  mqtt.publish("rover/state", js);

  snprintf(js, sizeof(js), "{\"pump\":%s,\"state\":\"%s\",\"level_pct\":%.1f,\"run_s\":%.1f,\"flow_ml_s\":%.1f,"
           "\"batch_ml\":%.0f,\"target_ml\":%.0f,\"total_ml\":%.0f,\"fault\":\"%s\"}",
           seq.on[CH_INTAKE] ? "true" : "false", IntakeController::name(intake.state), s1, intake.runMs / 1000.0f,
           flow.rateMlPerS, flow.batchMl, intakeTargetMl, flow.totalMl,
           (intake.state == IN_FAULT_DRY || intake.state == IN_FAULT_TIMEOUT) ? IntakeController::name(intake.state) : "");
  mqtt.publish("rover/intake/status", js);

  snprintf(js, sizeof(js), "{\"chlorine_on\":%s,\"chlorine_total_ml\":%.2f,\"chlorine_stock_ml\":%.1f,\"chlorine_last_ml\":%.2f}",
           seq.on[CH_CHLORINE] ? "true" : "false", chlorine.totalMl, chlorine.stockMl, chlorine.lastDoseMl);
  mqtt.publish("rover/dosing/status", js);

  snprintf(js, sizeof(js), "{\"pump\":%s,\"total_ml\":%.0f,\"column_ml\":%.0f,\"clean_ml\":%.0f}",
           seq.on[CH_TRANSFER] ? "true" : "false", transfer.totalMl, vol.columnMl, vol.cleanMl);
  mqtt.publish("rover/transfer/status", js);

  snprintf(js, sizeof(js), "{\"s1\":%.1f,\"s2\":%.1f,\"s3\":%.1f,\"intake\":%s,\"flow\":%.3f,\"health\":\"%s\"}",
           s1, s2, s3, seq.on[CH_INTAKE] ? "true" : "false", filt.flowPctPerS, FiltrationMonitor::name(filt.health));
  mqtt.publish("rover/filtration/status", js);

  snprintf(js, sizeof(js), "{\"angle\":%d,\"target\":%d,\"moving\":%s,\"deployed\":%s,\"stowed\":%s}",
           arm.angle, arm.target, arm.moving() ? "true" : "false", arm.deployed() ? "true" : "false", arm.stowed() ? "true" : "false");
  mqtt.publish("rover/arm/status", js);

  snprintf(js, sizeof(js), "{\"pump\":%s,\"state\":\"%s\",\"delivered_ml\":%.0f,\"delivered_total_ml\":%.0f,"
           "\"deliveries\":%u,\"clean_ml\":%.0f,\"reason\":\"%s\"}",
           seq.on[CH_DELIVERY] ? "true" : "false", DeliveryController::name(delivery.state), delivery.deliveredMl,
           delivery.totalMl, delivery.count, s3 / 100.0f * CLEAN_TANK_ML, delivery.reason);
  mqtt.publish("rover/delivery/status", js);

  snprintf(js, sizeof(js), "{\"left\":%d,\"right\":%d,\"inhibit\":%s}", drive.left, drive.right, drive.inhibit ? "true" : "false");
  mqtt.publish("rover/drive/status", js);

  // dead-reckoned pose on the fixed route (estimate - re-synced at every manual "Arrived")
  if (nav.idx < nav.n && nav.pts[nav.idx].action != RA_REVERSE)
    snprintf(js, sizeof(js), "{\"x\":%.2f,\"y\":%.2f,\"yaw\":%.2f,\"target\":[%.2f,%.2f],\"next\":\"%s\","
             "\"phase\":\"%s\",\"leg_left\":%.2f,\"point\":%u,\"auto\":%s,\"source\":\"dead_reckoning\"}",
             nav.x, nav.y, nav.yaw, nav.pts[nav.idx].x, nav.pts[nav.idx].y, RouteNavigator::actionName(nav.pts[nav.idx].action),
             RouteNavigator::phaseName(nav.phase), nav.legLeft, nav.idx, autoNav ? "true" : "false");
  else
    snprintf(js, sizeof(js), "{\"x\":%.2f,\"y\":%.2f,\"yaw\":%.2f,\"target\":null,\"phase\":\"%s\",\"point\":%u,"
             "\"auto\":%s,\"source\":\"dead_reckoning\"}", nav.x, nav.y, nav.yaw, RouteNavigator::phaseName(nav.phase),
             nav.idx, autoNav ? "true" : "false");
  mqtt.publish("rover/nav/pose", js);

  // Same schema as power-management/power_management.ino so the Power tab works unchanged
  snprintf(js, sizeof(js),
    "{\"mode\":\"%s\",\"mode_id\":%d,\"rail_v\":%.2f,\"battery_v\":%.2f,\"rail\":\"%s\","
    "\"ch\":[%d,%d,%d,%d],\"want\":[%d,%d,%d,%d],\"pending\":%d,\"brownouts\":%u,\"fault\":\"%s\",\"uptime_s\":%lu}",
    PowerSequencer::modeName(seq.mode), (int)seq.mode, railV, railV, PowerSequencer::railName(seq.rail),
    seq.on[0], seq.on[1], seq.on[2], seq.on[3], seq.want[0], seq.want[1], seq.want[2], seq.want[3],
    seq.pending, seq.brownouts, seq.fault, (unsigned long)(millis() / 1000UL));
  mqtt.publish("rover/power/status", js);

  snprintf(js, sizeof(js),
    "{\"state\":\"%s\",\"state_id\":%u,\"battery_v\":%.2f,\"estop\":%s,\"s1\":%.1f,\"s2\":%.1f,\"s3\":%.1f,"
    "\"relays\":[%d,%d,%d,%d],\"arm\":%d,\"drive\":[%d,%d],\"transfer_ml\":%.0f,\"chlorine_ml\":%.2f,"
    "\"delivered_ml\":%.0f,\"health\":\"%s\",\"power\":\"%s\",\"rssi\":%d}",
    st, mission.stateId(), railV, estop ? "true" : "false", s1, s2, s3,
    seq.on[0], seq.on[1], seq.on[2], seq.on[3], arm.angle, drive.left, drive.right,
    transfer.totalMl, chlorine.totalMl, delivery.totalMl, FiltrationMonitor::name(filt.health),
    PowerSequencer::modeName(seq.mode), WiFi.RSSI());
  mqtt.publish("rover/telemetry", js);

  uint32_t now = millis();
  snprintf(js, sizeof(js), "{\"active\":[%d,%d,%d,%d],\"left_s\":[%.1f,%.1f,%.1f,%.1f]}",
    ptest.active[0], ptest.active[1], ptest.active[2], ptest.active[3],
    ptest.remainingS(0, now), ptest.remainingS(1, now), ptest.remainingS(2, now), ptest.remainingS(3, now));
  mqtt.publish("rover/test/status", js);
}

bool wifiConfigured() { return strcmp(WIFI_SSID, "YOUR_WIFI") != 0 && WIFI_SSID[0] != 0; }

void serviceNetwork(uint32_t now) {
  if (!wifiConfigured()) return;                  // placeholders still in: run offline (serial only)
  if (now - tNet < 5000 && (tNet != 0)) return;
  tNet = now;
  if (WiFi.status() != WL_CONNECTED) {             // auto-reconnect is on; only kick it if it stays down
    if (now - tWifiKick > 30000) { tWifiKick = now; WiFi.reconnect(); }
    return;
  }
  if (!mqtt.connected() && mqtt.connect(MQTT_CLIENT, nullptr, nullptr, "rover/online", 0, true, "0")) {
    const char* subs[] = {"rover/estop", "rover/mission/cmd", "rover/power/cmd", "rover/intake/cmd",
                          "rover/filtration/cmd", "rover/dosing/cmd", "rover/arm/cmd",
                          "rover/delivery/cmd", "rover/drive/cmd", "rover/nav/cmd", "rover/transfer/cmd",
                          "rover/test/cmd"};
    for (const char* s : subs) mqtt.subscribe(s);
    mqtt.publish("rover/online", "1", true);
  }
}

// ============================== ARDUINO ==============================
void setup() {
  // Relays OFF before anything else (active-LOW boards click on at boot otherwise)
  for (uint8_t i = 0; i < PC_CHANNELS; i++) { digitalWrite(RELAY_PIN[i], RELAY_ACTIVE_LOW ? HIGH : LOW); pinMode(RELAY_PIN[i], OUTPUT); }
  const uint8_t outs[] = {PIN_L298_IN1, PIN_L298_IN2, PIN_L298_IN3, PIN_L298_IN4, PIN_LED};
  for (uint8_t p : outs) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  analogWrite(PIN_L298_ENA, 0); analogWrite(PIN_L298_ENB, 0);

  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, 500, 2400);           // SG90 pulse range
  servo.write(ServoArm::STOW);

  // Pump calibration (measure and adjust)
  transfer.rateMlPerS = TRANSFER_ML_PER_S; transfer.maxDoseMl = 400.0f; transfer.stockMl = 1e9f;   // not a chemical: no stock limit
  chlorine.rateMlPerS = 1.0f; chlorine.maxDoseMl = 4.0f;
  delivery.flowMlPerS = 25.0f;

  // Flow sensor (intake line) + estimated volumes, route navigator calibration
  pinMode(PIN_FLOW_SENSOR, INPUT);               // GPIO34 has no pull-up: see docs/HARDWARE.md divider
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), onFlowPulse, FALLING);
  flow.pulsesPerLitre = FLOW_PULSES_PER_L;
  vol.filterMlPerS = FILTER_ML_PER_S;
  vol.drainMlPerS = 0;                           // the transfer pump moves column water, not gravity
  intake.cfg.fullPct = 100.0f;                   // "full" = the flow sensor counted the batch
  intake.cfg.maxRunMs = 90000;                   // motor-burnout cutoff (a 380 mL batch takes ~20-60 s)
  intake.cfg.dryCheckMs = 4000;                  // relay ON 4 s ...
  intake.cfg.minRisePct = 1.0f;                  // ... with < 1 % of the batch counted -> DRY_RUN
  nav.speedMps = NAV_SPEED_MPS; nav.turnDegPerS = NAV_TURN_DPS;
  mission.cfg.homesteads = nav.homes();
  mission.cfg.batchMl = BATCH_ML;
#if USE_LEVEL_SENSORS
  s2 = readLevel(PIN_LEVEL_S2, CAL_S2); s3 = readLevel(PIN_LEVEL_S3, CAL_S3);
#endif

  seq.setMode(PM_ACTIVE, millis());             // boot-safe: nothing is requested yet
  if (wifiConfigured()) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  } else {
    Serial.println("{\"event\":\"wifi_off\",\"why\":\"set WIFI_SSID / WIFI_PASS / MQTT_HOST at the top of the sketch\"}");
  }
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setBufferSize(1024);                     // telemetry frame is > 256 bytes
  Serial.println("{\"event\":\"boot\",\"fw\":\"waterdroid_esp32\"}");
}

void loop() {
  uint32_t now = millis();

  if (now - tSample >= 100) {                   // 10 Hz sensors, EMA smoothing
    tSample = now;
    const float k = 0.1f;
#if USE_LEVEL_SENSORS
    s2 += k * (readLevel(PIN_LEVEL_S2, CAL_S2) - s2);
    s3 += k * (readLevel(PIN_LEVEL_S3, CAL_S3) - s3);
#else
    (void)k;
#endif
    sampleRail();
  }
  if (now - tLogic >= 20) { tLogic = now; controlStep(); }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { onSerialLine(serialLine); serialLine = ""; }
    else if (c != '\r' && serialLine.length() < 96) serialLine += c;
  }
  if (now - tSerial >= 1000) {                  // Node-RED serial bridge frame (Issue #11)
    tSerial = now;
    Serial.printf("{\"state_id\":%u,\"state\":\"%s\",\"battery_v\":%.2f,\"estop\":%d,\"wifi\":%d,\"mqtt\":%d}\n",
                  mission.stateId(), MissionCore::name(mission.state), railV, estop,
                  WiFi.status() == WL_CONNECTED, mqtt.connected());
  }

  serviceNetwork(now);
  if (mqtt.connected()) {
    mqtt.loop();
    if (now - tPub >= 1000) { tPub = now; publishAll(); }
  }
}
