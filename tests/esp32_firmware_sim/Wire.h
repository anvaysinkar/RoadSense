#pragma once
#include <cstdint>
#include <deque>
extern std::deque<uint8_t> g_i2c;
struct TwoWire { void begin(int = 0, int = 0) {} void end() {} void setClock(uint32_t) {}
  void beginTransmission(uint8_t) {} void write(uint8_t) {} uint8_t endTransmission(bool = true) { return 0; }
  uint8_t requestFrom(uint8_t, uint8_t n, uint8_t) { return n; }
  int available() { return (int)g_i2c.size(); }
  int read() { if (g_i2c.empty()) return 0; uint8_t b = g_i2c.front(); g_i2c.pop_front(); return b; } };
extern TwoWire Wire;
