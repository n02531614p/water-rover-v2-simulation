#pragma once
#include <stdint.h>
class WiFiClient;
class PubSubClient {
 public:
  explicit PubSubClient(WiFiClient&) {}
  PubSubClient& setServer(const char*, uint16_t) { return *this; }
  PubSubClient& setCallback(void (*)(char*, uint8_t*, unsigned int)) { return *this; }
  bool setBufferSize(uint16_t) { return true; }
  bool connect(const char*) { return false; }
  bool connect(const char*, const char*, const char*, const char*, uint8_t, bool, const char*) { return false; }
  bool connected() { return false; }
  bool subscribe(const char*) { return true; }
  bool publish(const char*, const char*) { return true; }
  bool publish(const char*, const char*, bool) { return true; }
  bool loop() { return true; }
  PubSubClient& setSocketTimeout(uint16_t) { return *this; }
};
