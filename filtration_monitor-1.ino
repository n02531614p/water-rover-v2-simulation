/*
 * Water Rover v2 - Stage 2 Filtration Column & Flow Monitoring
 * Board : ESP32 (analog level sensors on ADC1 pins, so Wi-Fi can stay on)
 * Role  : Mirrors the XOD node logic (analog read -> threshold -> pause intake)
 *         and publishes tank levels + health to Node-RED over MQTT.
 * Libs  : PubSubClient (Nick O'Leary)
 */
#include <WiFi.h>
#include <PubSubClient.h>

// ---------- CONFIG ----------
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";
const char* MQTT_HOST = "192.168.1.50";   // Node-RED / broker IP
const uint16_t MQTT_PORT = 1883;
const char* TOPIC_STATUS = "rover/filtration/status";
const char* TOPIC_CMD    = "rover/filtration/cmd";   // pause | resume | auto

// Pins (ADC1 only: 32-39)
const int PIN_S1 = 34;   // Stage 1 raw tank level sensor
const int PIN_S2 = 35;   // Stage 2 filtration column level sensor
const int PIN_S3 = 32;   // Stage 3 clean tank level sensor
const int PIN_RELAY_INTAKE = 26;  // Relay CH1 -> intake pump
const bool RELAY_ACTIVE_LOW = true;

// Per-sensor calibration (raw ADC 0-4095): dry = empty, wet = full. CALIBRATE THESE.
struct Cal { int dry; int wet; };
Cal CAL_S1 = {200, 2200};
Cal CAL_S2 = {200, 2200};
Cal CAL_S3 = {200, 2200};

// Thresholds (% of sensor range)
const float S1_LOW_PCT      = 10;  // source nearly empty -> pause (dry-run protection)
const float S2_HIGH_PCT     = 90;  // column overflow     -> pause intake
const float S2_RESUME_PCT   = 50;  // hysteresis: resume below this
const float S3_FULL_PCT     = 95;  // clean tank full     -> pause intake
const float S2_CLOG_PCT     = 70;  // column this full while S3 barely rising => clog
const float MIN_S3_RISE     = 0.5; // % gained in the window if flowing normally
const unsigned long CLOG_WINDOW_MS = 30000;

// ---------- STATE ----------
enum Health { HEALTHY, CLOGGING, OVERFLOW, SOURCE_LOW, TANK_FULL, MANUAL_PAUSE };
const char* HEALTH_TXT[] = {"HEALTHY","CLOGGING","OVERFLOW","SOURCE_LOW","TANK_FULL","MANUAL_PAUSE"};

WiFiClient wifi;
PubSubClient mqtt(wifi);

float s1 = 0, s2 = 0, s3 = 0;   // smoothed levels (%)
bool intakeOn = false;
bool overflowLatch = false;
bool manualPause = false;
bool cloggedFlag = false;
Health health = HEALTHY;

float s3Ref = 0;
unsigned long windowStart = 0;
float flowRate = 0;             // % per second rise of Stage 3 (proxy for throughput)

unsigned long tSample = 0, tLogic = 0, tPub = 0;

// ---------- HELPERS ----------
float readPct(int pin, const Cal& c) {
  long raw = analogRead(pin);
  float p = (float)(raw - c.dry) * 100.0f / (float)(c.wet - c.dry);
  return constrain(p, 0.0f, 100.0f);
}

void setIntake(bool on) {
  intakeOn = on;
  digitalWrite(PIN_RELAY_INTAKE, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void onMessage(char* topic, byte* payload, unsigned int len) {
  String m; for (unsigned i = 0; i < len; i++) m += (char)payload[i];
  m.trim(); m.toLowerCase();
  if (m == "pause") manualPause = true;
  else if (m == "resume" || m == "auto") manualPause = false;
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) delay(250);
}

void connectMqtt() {
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connect("water-rover-esp32")) mqtt.subscribe(TOPIC_CMD);
}

// ---------- LOGIC (equivalent of the XOD patch) ----------
void evaluate() {
  // Stage 2 overflow with hysteresis latch
  if (s2 >= S2_HIGH_PCT) overflowLatch = true;
  else if (s2 <= S2_RESUME_PCT) overflowLatch = false;

  bool sourceLow = s1 <= S1_LOW_PCT;
  bool tankFull  = s3 >= S3_FULL_PCT;

  // Throughput / clog detection over a sliding window
  unsigned long now = millis();
  if (now - windowStart >= CLOG_WINDOW_MS) {
    float rise = s3 - s3Ref;
    flowRate = rise / ((now - windowStart) / 1000.0f);
    cloggedFlag = intakeOn && s2 >= S2_CLOG_PCT && rise < MIN_S3_RISE && !tankFull;
    s3Ref = s3;
    windowStart = now;
  }

  bool allowIntake = !(overflowLatch || sourceLow || tankFull || manualPause);
  setIntake(allowIntake);

  // Priority-ordered health badge
  if (manualPause)        health = MANUAL_PAUSE;
  else if (overflowLatch) health = OVERFLOW;
  else if (sourceLow)     health = SOURCE_LOW;
  else if (tankFull)      health = TANK_FULL;
  else if (cloggedFlag)   health = CLOGGING;
  else                    health = HEALTHY;
}

void publishStatus() {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"s1\":%.1f,\"s2\":%.1f,\"s3\":%.1f,\"intake\":%s,\"flow\":%.3f,\"health\":\"%s\"}",
    s1, s2, s3, intakeOn ? "true" : "false", flowRate, HEALTH_TXT[health]);
  mqtt.publish(TOPIC_STATUS, buf);
  Serial.println(buf);
}

// ---------- ARDUINO ----------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(PIN_RELAY_INTAKE, OUTPUT);
  setIntake(false);

  connectWifi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);

  s1 = readPct(PIN_S1, CAL_S1);
  s2 = readPct(PIN_S2, CAL_S2);
  s3 = readPct(PIN_S3, CAL_S3);
  s3Ref = s3;
  windowStart = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - tSample >= 100) {           // 10 Hz sampling, EMA smoothing
    tSample = now;
    const float a = 0.1f;
    s1 += a * (readPct(PIN_S1, CAL_S1) - s1);
    s2 += a * (readPct(PIN_S2, CAL_S2) - s2);
    s3 += a * (readPct(PIN_S3, CAL_S3) - s3);
  }
  if (now - tLogic >= 500) { tLogic = now; evaluate(); }

  if (now - tPub >= 1000) {
    tPub = now;
    connectWifi();
    connectMqtt();
    if (mqtt.connected()) publishStatus();
  }
  mqtt.loop();
}
