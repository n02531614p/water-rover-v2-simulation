/*
 * Water Rover v2 - Power Distribution & Load Management (Issue #10)
 * Board : ESP32 DevKit (any 30/38-pin)        Libs : PubSubClient (Nick O'Leary)
 * Role  : Mirrors the XOD node `relay-sequencer` (power_core.h is the same logic).
 *         - 4-channel relay sequencing: loads start ONE AT A TIME, never together
 *         - 5 V rail monitor (ADC divider) with brownout load-shedding
 *         - power modes ACTIVE / SLEEP / EMERGENCY, controlled from Node-RED over MQTT
 *
 * MQTT  rover/power/status     (out)  JSON, 1 Hz ACTIVE / every 10 s SLEEP / on change
 *       rover/power/cmd        (in)   active | sleep | emergency | estop | reset
 *       rover/power/req/1..4   (in)   1|0|on|off   (mission controller asks for a load)
 *       rover/power/online     (out)  1 / 0 (last-will)
 *
 * Relay channels (CH1 matches filtration_monitor-1.ino): see RELAY_PIN below.
 * NOTE: this sketch never blocks, so the relay logic keeps running when Wi-Fi/MQTT is down.
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include "power_core.h"

// ---------- CONFIG ----------
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";
const char* MQTT_HOST = "192.168.1.50";          // Node-RED / broker IP
const uint16_t MQTT_PORT = 1883;
const char* TOPIC_STATUS = "rover/power/status";
const char* TOPIC_CMD    = "rover/power/cmd";
const char* TOPIC_REQ    = "rover/power/req/";   // + 1..4
const char* TOPIC_ONLINE = "rover/power/online";

// Relay module inputs (avoid GPIO 0/2/12/15 strapping pins and 34-39 input-only pins)
const uint8_t RELAY_PIN[PC_CHANNELS] = {26, 27, 25, 33};   // CH1 intake pump, CH2 left motor, CH3 right motor, CH4 aux pump
const char*   RELAY_NAME[PC_CHANNELS] = {"intake_pump", "left_motor", "right_motor", "aux_pump"};
const bool RELAY_ACTIVE_LOW = true;              // typical SRD-05VDC 4-ch boards: IN LOW = relay ON

// 5 V rail sense: rail -> R_TOP -> GPIO39 -> R_BOTTOM -> GND   (+100 nF from GPIO39 to GND)
const uint8_t PIN_RAIL_SENSE = 39;               // ADC1_CH3 (works with Wi-Fi on)
const float R_TOP = 10000.0f, R_BOTTOM = 10000.0f;
const float VCAL  = 1.000f;                      // multimeter reading / sketch reading (calibrate!)

const uint32_t COMMS_LOSS_SLEEP_MS = 15000;      // MQTT down this long -> SLEEP (0 = disabled)
const uint32_t PUBLISH_ACTIVE_MS = 1000, PUBLISH_SLEEP_MS = 10000;

// ---------- STATE ----------
PowerSequencer seq;
WiFiClient wifi;
PubSubClient mqtt(wifi);

float railV = 0.0f;  bool railValid = false;
uint32_t tSample = 0, tLogic = 0, tPublish = 0, tNet = 0, lastMqttOkMs = 0;
uint32_t lastSig = 0xFFFFFFFF;
PowerMode lastMode = PM_SLEEP;

// ---------- RAIL MEASUREMENT ----------
void sampleRail() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_RAIL_SENSE);
  float pinV = (mv / 8.0f) / 1000.0f;
  float v = pinV * (R_TOP + R_BOTTOM) / R_BOTTOM * VCAL;
  railValid = (pinV > 0.2f);                     // < 0.2 V at the pin = divider not connected
  railV = railValid ? (railV == 0.0f ? v : railV * 0.7f + v * 0.3f) : 0.0f;   // light smoothing
}

// ---------- OUTPUTS ----------
void applyRelays() {
  for (uint8_t i = 0; i < PC_CHANNELS; i++) digitalWrite(RELAY_PIN[i], seq.pinLevel(i, RELAY_ACTIVE_LOW));
}

// ---------- MQTT ----------
void onMessage(char* topic, byte* payload, unsigned int len) {
  char buf[24]; unsigned n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
  for (unsigned i = 0; i < n; i++) buf[i] = tolower(payload[i]);
  buf[n] = 0;
  String t(topic), m(buf); m.trim();
  uint32_t now = millis();
  if (t == TOPIC_CMD) {
    if (m == "active") seq.setMode(PM_ACTIVE, now);
    else if (m == "sleep") seq.setMode(PM_SLEEP, now);
    else if (m == "emergency" || m == "estop") seq.setMode(PM_EMERGENCY, now);
    else if (m == "reset") seq.reset(now);
  } else if (t.startsWith(TOPIC_REQ)) {
    int ch = t.substring(strlen(TOPIC_REQ)).toInt();          // 1..4
    if (ch >= 1 && ch <= PC_CHANNELS) seq.setWant(ch - 1, m == "1" || m == "on" || m == "true");
  }
}

void publishStatus() {
  char js[360];
  snprintf(js, sizeof(js),
    "{\"mode\":\"%s\",\"mode_id\":%d,\"rail_v\":%.2f,\"battery_v\":%.2f,\"rail\":\"%s\","
    "\"ch\":[%d,%d,%d,%d],\"want\":[%d,%d,%d,%d],\"pending\":%d,\"brownouts\":%u,"
    "\"fault\":\"%s\",\"uptime_s\":%lu}",
    PowerSequencer::modeName(seq.mode), (int)seq.mode, railV, railV, PowerSequencer::railName(seq.rail),
    seq.on[0], seq.on[1], seq.on[2], seq.on[3], seq.want[0], seq.want[1], seq.want[2], seq.want[3],
    seq.pending, seq.brownouts, seq.fault, millis() / 1000UL);
  mqtt.publish(TOPIC_STATUS, js);
  tPublish = millis();
}

// Non-blocking network service: at most one short attempt every 5 s.
void serviceNetwork(uint32_t now) {
  if (WiFi.status() != WL_CONNECTED) {
    if (now - tNet > 5000) { tNet = now; WiFi.disconnect(); WiFi.begin(WIFI_SSID, WIFI_PASS); }
    return;
  }
  if (!mqtt.connected()) {
    if (now - tNet > 5000) {
      tNet = now;
      if (mqtt.connect("water-rover-power", nullptr, nullptr, TOPIC_ONLINE, 0, true, "0")) {
        mqtt.subscribe(TOPIC_CMD);
        char sub[32]; snprintf(sub, sizeof(sub), "%s#", TOPIC_REQ); mqtt.subscribe(sub);
        mqtt.publish(TOPIC_ONLINE, "1", true);
      }
    }
    return;
  }
  mqtt.loop();
  lastMqttOkMs = now;
}

// ---------- ARDUINO ----------
void setup() {
  for (uint8_t i = 0; i < PC_CHANNELS; i++) {            // relays OFF before anything else
    digitalWrite(RELAY_PIN[i], seq.pinLevel(i, RELAY_ACTIVE_LOW));
    pinMode(RELAY_PIN[i], OUTPUT);
  }
  Serial.begin(115200);
  analogSetPinAttenuation(PIN_RAIL_SENSE, ADC_11db);
  wifi.setTimeout(2);                                    // seconds: keep connect attempts short
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setSocketTimeout(2);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastMqttOkMs = millis();
  Serial.println("Power manager ready: boot mode SLEEP (send 'active' on rover/power/cmd)");
}

void loop() {
  uint32_t now = millis();
  serviceNetwork(now);

  if (now - tSample >= 20) { tSample = now; sampleRail(); }

  if (now - tLogic >= 20) {
    tLogic = now;
    if (COMMS_LOSS_SLEEP_MS && seq.mode == PM_ACTIVE && now - lastMqttOkMs > COMMS_LOSS_SLEEP_MS) {
      seq.setMode(PM_SLEEP, now);                        // fail-safe: lost the operator
      Serial.println("MQTT lost -> SLEEP");
    }
    seq.tick(now, railV, railValid);
    applyRelays();
    if (seq.mode != lastMode) {                          // saves power while sleeping
      setCpuFrequencyMhz(seq.mode == PM_ACTIVE ? 240 : 80);
      lastMode = seq.mode;
    }
  }

  uint32_t sig = (seq.on[0] | seq.on[1] << 1 | seq.on[2] << 2 | seq.on[3] << 3) | (uint32_t)seq.mode << 4 |
                 (uint32_t)seq.rail << 6 | (uint32_t)seq.pending << 8 | (uint32_t)seq.brownouts << 9;
  uint32_t every = seq.mode == PM_ACTIVE ? PUBLISH_ACTIVE_MS : PUBLISH_SLEEP_MS;
  if (mqtt.connected() && (sig != lastSig || now - tPublish >= every)) { lastSig = sig; publishStatus(); }
}
