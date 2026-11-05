// Minimal Arduino-ESP32 API stubs: lets g++ syntax-check the .ino on a PC.
// NOT a simulator - only signatures. Used by firmware/tests/run_tests.sh.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <algorithm>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
enum adc_attenuation_t { ADC_0db, ADC_11db };
inline uint32_t millis() { return 0; }
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline void analogWrite(uint8_t, int) {}
inline int analogRead(uint8_t) { return 0; }
inline uint32_t analogReadMilliVolts(uint8_t) { return 0; }
inline void analogReadResolution(uint8_t) {}
inline void analogSetAttenuation(adc_attenuation_t) {}
inline void delay(uint32_t) {}
typedef uint8_t byte;
class String {
 public:
  std::string s;
  String() {}
  String(const char* c) : s(c ? c : "") {}
  String(const std::string& x) : s(x) {}
  String(char c) : s(1, c) {}
  void trim() { size_t a = s.find_first_not_of(" \r\n\t"); size_t b = s.find_last_not_of(" \r\n\t"); s = a == std::string::npos ? "" : s.substr(a, b - a + 1); }
  void toLowerCase() { std::transform(s.begin(), s.end(), s.begin(), ::tolower); }
  void toUpperCase() { std::transform(s.begin(), s.end(), s.begin(), ::toupper); }
  int indexOf(const String& x) const { size_t p = s.find(x.s); return p == std::string::npos ? -1 : (int)p; }
  int indexOf(char c, unsigned from = 0) const { size_t p = s.find(c, from); return p == std::string::npos ? -1 : (int)p; }
  String substring(unsigned a) const { return String(s.substr(a)); }
  String substring(unsigned a, unsigned b) const { return String(s.substr(a, b - a)); }
  float toFloat() const { return (float)atof(s.c_str()); }
  long toInt() const { return atol(s.c_str()); }
  bool startsWith(const char* p) const { return s.rfind(p, 0) == 0; }
  bool startsWith(const String& p) const { return s.rfind(p.s, 0) == 0; }
  unsigned length() const { return (unsigned)s.size(); }
  const char* c_str() const { return s.c_str(); }
  char charAt(unsigned i) const { return s[i]; }
  String& operator+=(char c) { s += c; return *this; }
  String& operator+=(const String& o) { s += o.s; return *this; }
  bool operator==(const char* c) const { return s == c; }
  bool operator==(const String& o) const { return s == o.s; }
  bool operator!=(const char* c) const { return s != c; }
};
inline String operator+(const String& a, const String& b) { return String(a.s + b.s); }
inline String operator+(const String& a, const char* b) { return String(a.s + b); }
inline String operator+(const char* a, const String& b) { return String(std::string(a) + b.s); }
struct SerialStub {
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }
  void println(const char*) {}
  void println(const String&) {}
  void print(const char*) {}
  int printf(const char* f, ...) __attribute__((format(printf, 2, 3))) { va_list a; va_start(a, f); int n = vsnprintf(nullptr, 0, f, a); va_end(a); return n; }
};
static SerialStub Serial;
inline bool isDigit(char c) { return c >= '0' && c <= '9'; }
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
inline void analogSetPinAttenuation(uint8_t, adc_attenuation_t) {}
inline void setCpuFrequencyMhz(uint32_t) {}
