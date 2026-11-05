#pragma once
#define WL_CONNECTED 3
#define WIFI_STA 1
struct WiFiStub { void disconnect() {} int status() { return 0; } void begin(const char*, const char*) {} void mode(int) {} int RSSI() { return 0; } };
static WiFiStub WiFi;
class WiFiClient { public: void setTimeout(int) {} };
