// Arduino-ESP32 stand-ins that REPLAY a CoolTerm log through the real sketch (host simulation).
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <deque>
#define PI 3.14159265358979f
#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define OUTPUT 1
#define SERIAL_8N1 0
extern uint32_t g_ms; extern uint64_t g_us;
inline uint32_t millis() { return g_ms; }
inline uint32_t micros() { return (uint32_t)g_us; }
inline void delay(uint32_t) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return 1; }
inline void pinMode(int, int) {}
struct Stream_ {
  std::deque<char> in;
  void begin(long, int = 0, int = 0, int = 0) {}
  void setTxBufferSize(size_t) {}
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; char c = in.front(); in.pop_front(); return c; }
  void println(const char* s) { if (s[0] == 'E' || s[0] == 'Q' || s[0] == 'C' || s[0] == 'K') printf("%s\n", s); }
  void println() {}
  size_t write(const uint8_t*, size_t n) { return n; }
};
struct HardwareSerial : Stream_ { HardwareSerial(int) {} };
extern Stream_ Serial;
#include "Wire.h"
