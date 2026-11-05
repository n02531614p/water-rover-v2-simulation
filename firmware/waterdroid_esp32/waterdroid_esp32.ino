/*
 * Water Rover v2 / Waterdroid - integrated firmware for the ONE ESP32 "autonomous brain"
 *
 * Merges every team member's subsystem onto one board with one pin map (src/rover_pins.h):
 *   #1  Intake & raw water collection      src/pump_cores.h   IntakeController   relay CH1
 *   #3  Chemical dosing (alum, chlorine)   src/pump_cores.h   DosingPump x2      relay CH2/CH3
 *   #5  Filtration column monitoring       src/motion_cores.h FiltrationMonitor  S1/S2/S3 ADC
 *   #6  Servo delivery arm (SG90)          src/motion_cores.h ServoArm           GPIO13
 *   #8  Clean water delivery pump          src/pump_cores.h   DeliveryController relay CH4
 *   #9  4WD propulsion (L298N)             src/motion_cores.h DriveController    ENA/ENB + IN1-4
 *   #10 Power distribution & sequencing    src/power_core.h   PowerSequencer     all relays + rail
 *   #11 Central state machine              src/mission_core.h MissionCore        + serial bridge
 *   #12 MQTT telemetry / dashboard / E-stop (this file)
 * Each core is pure C++ shared with the XOD nodes (xod/waterdroid) and the host tests
 * (firmware/tests), so the logic is tested off-board.
 *
 * Board : ESP32 DevKit V1, Arduino-ESP32 core >= 2.0.x
 * Libs  : PubSubClient (Nick O'Leary), ESP32Servo (Kevin Harrington)
 * MQTT  : topic map in docs/TOPICS.md (all under rover/...)
 * Serial: 115200 baud, one JSON line per second {"state_id":..,"battery_v":..} for the
 *         Node-RED serial bridge; accepts {"cmd":"SET_STATE","state":1} / {"cmd":"ESTOP"}
 *         and plain lines: start | pause | resume | abort | estop | reset |
 *         arrived_river | arrived_home | arrived_depot
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

// ============================== CONFIG ==============================
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";
const char* MQTT_HOST = "192.168.1.50";        // Node-RED / Mosquitto host
const uint16_t MQTT_PORT = 1883;
const char* MQTT_CLIENT = "waterdroid-esp32";

// Level sensor calibration (raw ADC 0-4095): dry = empty, wet = full. CALIBRATE!
struct Cal { int dry, wet; };
Cal CAL_S1 = {200, 2200}, CAL_S2 = {200, 2200}, CAL_S3 = {200, 2200};
const float CLEAN_TANK_ML = 1000.0f;           // Stage 3 capacity (for volume estimate)

// 5 V rail divider (rover_pins.h) and calibration
const float R_TOP = 10000.0f, R_BOTTOM = 10000.0f, VCAL = 1.000f;

const uint8_t RELAY_PIN[PC_CHANNELS] = {PIN_RELAY_CH1, PIN_RELAY_CH2, PIN_RELAY_CH3, PIN_RELAY_CH4};
const char*   RELAY_NAME[PC_CHANNELS] = {"intake_pump", "alum_pump", "chlorine_pump", "delivery_pump"};
enum { CH_INTAKE = 0, CH_ALUM = 1, CH_CHLORINE = 2, CH_DELIVERY = 3 };

// ============================== STATE ==============================
PowerSequencer     seq;
IntakeController   intake;
DosingPump         alum, chlorine;
DeliveryController delivery;
FiltrationMonitor  filt;
ServoArm           arm;
DriveController    drive;
MissionCore        mission;

Servo        servo;
WiFiClient   wifi;
PubSubClient mqtt(wifi);

float s1 = 0, s2 = 0, s3 = 0;      // smoothed levels %
float railV = 0; bool railValid = false;
bool  estop = false;
bool  atHomestead = false;         // true between arrived_home and leaving DISPENSING
uint32_t tSample = 0, tLogic = 0, tPub = 0, tNet = 0, tSerial = 0;
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
    intake.stop(); alum.abort(); chlorine.abort(); delivery.abort("e-stop");
    drive.stopNow();
  } else {
    seq.reset(now);                  // -> SLEEP; operator returns to ACTIVE on purpose
    seq.setMode(PM_ACTIVE, now);
  }
  Serial.printf("{\"event\":\"estop\",\"on\":%d,\"src\":\"%s\"}\n", on, src);
  if (mqtt.connected()) mqtt.publish("rover/estop", on ? "1" : "0", true);
}

// ----- one command dispatcher for MQTT, serial and the mission -----
void missionCommand(const String& c) {
  uint32_t now = millis();
  if (c == "start") mission.start(now);
  else if (c == "pause") mission.pause(now);
  else if (c == "resume") mission.resume(now);
  else if (c == "abort" || c == "home") mission.abort(now);
  else if (c == "arrived_river") mission.arrivedRiver(now);
  else if (c == "arrived_home") { atHomestead = true; mission.arrivedHomestead(now); }
  else if (c == "arrived_depot") mission.arrivedDepot(now);
}

float jsonNumber(const String& s, const char* key, float def) {
  int k = s.indexOf(String("\"") + key + "\"");
  if (k < 0) return def;
  int c = s.indexOf(':', k);
  return c < 0 ? def : s.substring(c + 1).toFloat();
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
  else if (t == "rover/intake/cmd") { if (lm == "start" || lm == "on") intake.start(); else if (lm == "reset") intake.clearFault(); else intake.stop(); }
  else if (t == "rover/filtration/cmd") filt.manualPause = (lm == "pause");
  else if (t == "rover/dosing/cmd") {
    if (lm == "stop") { alum.abort(); chlorine.abort(); }
    else if (lm.indexOf("refill") >= 0) { if (lm.indexOf("chlor") >= 0) chlorine.setStock(250); else alum.setStock(250); }
    else {
      float ml = jsonNumber(lm, "ml", 0);
      if (lm.indexOf("chlor") >= 0) chlorine.start(ml); else alum.start(ml);
    }
  }
  else if (t == "rover/arm/cmd") arm.command(m.c_str());
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
  in.intakeBusy = intake.want;
  in.alumBusy = alum.busy();
  in.chlorineBusy = chlorine.busy();
  in.s1Pct = s1; in.s2Pct = s2;
  in.armDeployed = arm.deployed(); in.armStowed = arm.stowed();
  in.deliveryBusy = delivery.state == DL_WAIT_ARM || delivery.state == DL_PUMPING;
  in.cleanMl = s3 / 100.0f * CLEAN_TANK_ML;
  mission.tick(now, in);

  MissionActions& a = mission.act;
  if (a.stopAll) { intake.stop(); alum.abort(); chlorine.abort(); delivery.abort("mission"); }
  if (a.startIntake) intake.start();
  if (a.doseAlumMl > 0) alum.start(a.doseAlumMl);
  if (a.doseChlorineMl > 0) chlorine.start(a.doseChlorineMl);
  if (a.armExtend) arm.command("EXTEND");
  if (a.armRetract) arm.command("RETRACT");
  if (a.deliverMl > 0) delivery.request(a.deliverMl);
  a.clear();
  if (mission.state == MS_EN_ROUTE_HOMESTEAD || mission.state == MS_RETURNING || mission.state == MS_EN_ROUTE_RIVER)
    atHomestead = false;           // moving again -> no longer parked at a container

  // ---- #5 filtration + #1 intake (batch = what Stage 3 can hold) ----
  filt.tick(now, s1, s2, s3, seq.on[CH_INTAKE]);
  bool batchFull = (s1 + s2 * 0.25f + s3) >= filt.s3FullPct;
  if (batchFull && intake.requested) intake.stop();
  intake.tick(now, seq.on[CH_INTAKE], s1, !filt.allowIntake || estop);

  // ---- #3 dosing, #8 delivery ----
  alum.tick(now, seq.on[CH_ALUM]);
  chlorine.tick(now, seq.on[CH_CHLORINE]);
  bool manualDelivery = mission.state == MS_IDLE || mission.state == MS_PAUSED || mission.state == MS_COMPLETE;
  delivery.tick(now, seq.on[CH_DELIVERY], atHomestead || manualDelivery, arm.deployed(), s3);

  // ---- #10 power sequencing: pumps start one at a time, brownout guard ----
  seq.setWant(CH_INTAKE, intake.want);
  seq.setWant(CH_ALUM, alum.want);
  seq.setWant(CH_CHLORINE, chlorine.want);
  seq.setWant(CH_DELIVERY, delivery.want);
  seq.tick(now, railV, railValid);
  for (uint8_t i = 0; i < PC_CHANNELS; i++) digitalWrite(RELAY_PIN[i], seq.pinLevel(i, RELAY_ACTIVE_LOW));

  // ---- #9 drive: never together with a pump, never in e-stop ----
  bool anyPump = seq.on[0] || seq.on[1] || seq.on[2] || seq.on[3];
  drive.inhibit = estop || !mission.driveAllowed || anyPump || seq.mode == PM_EMERGENCY;
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

  snprintf(js, sizeof(js), "{\"pump\":%s,\"state\":\"%s\",\"level_pct\":%.1f,\"run_s\":%.1f,\"fault\":\"%s\"}",
           seq.on[CH_INTAKE] ? "true" : "false", IntakeController::name(intake.state), s1, intake.runMs / 1000.0f,
           (intake.state == IN_FAULT_DRY || intake.state == IN_FAULT_TIMEOUT) ? IntakeController::name(intake.state) : "");
  mqtt.publish("rover/intake/status", js);

  snprintf(js, sizeof(js), "{\"alum_on\":%s,\"chlorine_on\":%s,\"alum_total_ml\":%.2f,\"chlorine_total_ml\":%.2f,"
           "\"alum_stock_ml\":%.1f,\"chlorine_stock_ml\":%.1f,\"alum_last_ml\":%.2f,\"chlorine_last_ml\":%.2f}",
           seq.on[CH_ALUM] ? "true" : "false", seq.on[CH_CHLORINE] ? "true" : "false",
           alum.totalMl, chlorine.totalMl, alum.stockMl, chlorine.stockMl, alum.lastDoseMl, chlorine.lastDoseMl);
  mqtt.publish("rover/dosing/status", js);

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
    "\"relays\":[%d,%d,%d,%d],\"arm\":%d,\"drive\":[%d,%d],\"alum_ml\":%.2f,\"chlorine_ml\":%.2f,"
    "\"delivered_ml\":%.0f,\"health\":\"%s\",\"power\":\"%s\",\"rssi\":%d}",
    st, mission.stateId(), railV, estop ? "true" : "false", s1, s2, s3,
    seq.on[0], seq.on[1], seq.on[2], seq.on[3], arm.angle, drive.left, drive.right,
    alum.totalMl, chlorine.totalMl, delivery.totalMl, FiltrationMonitor::name(filt.health),
    PowerSequencer::modeName(seq.mode), WiFi.RSSI());
  mqtt.publish("rover/telemetry", js);
}

void serviceNetwork(uint32_t now) {
  if (now - tNet < 5000 && (tNet != 0)) return;
  tNet = now;
  if (WiFi.status() != WL_CONNECTED) { WiFi.begin(WIFI_SSID, WIFI_PASS); return; }   // non-blocking retry
  if (!mqtt.connected() && mqtt.connect(MQTT_CLIENT, nullptr, nullptr, "rover/online", 0, true, "0")) {
    const char* subs[] = {"rover/estop", "rover/mission/cmd", "rover/power/cmd", "rover/intake/cmd",
                          "rover/filtration/cmd", "rover/dosing/cmd", "rover/arm/cmd",
                          "rover/delivery/cmd", "rover/drive/cmd"};
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
  alum.rateMlPerS = 1.0f;     alum.maxDoseMl = 10.0f;
  chlorine.rateMlPerS = 1.0f; chlorine.maxDoseMl = 5.0f;
  delivery.flowMlPerS = 25.0f;

  s1 = readLevel(PIN_LEVEL_S1, CAL_S1); s2 = readLevel(PIN_LEVEL_S2, CAL_S2); s3 = readLevel(PIN_LEVEL_S3, CAL_S3);

  seq.setMode(PM_ACTIVE, millis());             // boot-safe: nothing is requested yet
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
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
    s1 += k * (readLevel(PIN_LEVEL_S1, CAL_S1) - s1);
    s2 += k * (readLevel(PIN_LEVEL_S2, CAL_S2) - s2);
    s3 += k * (readLevel(PIN_LEVEL_S3, CAL_S3) - s3);
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
    Serial.printf("{\"state_id\":%u,\"state\":\"%s\",\"battery_v\":%.2f,\"estop\":%d}\n",
                  mission.stateId(), MissionCore::name(mission.state), railV, estop);
  }

  serviceNetwork(now);
  if (mqtt.connected()) {
    mqtt.loop();
    if (now - tPub >= 1000) { tPub = now; publishAll(); }
  }
}
