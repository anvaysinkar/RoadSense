#pragma once
#include <cstdint>
struct TwoWire { void begin(int = 0, int = 0) {} void end() {} void setClock(uint32_t) {}
  void beginTransmission(uint8_t) {} void write(uint8_t) {} uint8_t endTransmission(bool = true) { return 0; }
  uint8_t requestFrom(uint8_t, uint8_t, uint8_t) { return 14; } int available() { return 0; } int read() { return 0; } };
extern TwoWire Wire;
