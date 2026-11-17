// Minimal stand-ins for the Arduino-ESP32 API, ONLY to syntax/type-check the sketch on a PC.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cmath>
#define PI 3.14159265358979f
#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define OUTPUT 1
#define SERIAL_8N1 0
typedef bool boolean;
inline uint32_t millis() { return 0; }
inline uint32_t micros() { return 0; }
inline void delay(uint32_t) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return 1; }
inline void pinMode(int, int) {}
struct Stream_ {
  void begin(long, int = 0, int = 0, int = 0) {}
  void setTxBufferSize(size_t) {}
  int available() { return 0; }
  int read() { return -1; }
  void println(const char*) {}
  void println() {}
  size_t write(const uint8_t*, size_t n) { return n; }
};
typedef Stream_ HardwareSerial_;
struct HardwareSerial : Stream_ { HardwareSerial(int) {} };
extern Stream_ Serial;
#include "Wire.h"
