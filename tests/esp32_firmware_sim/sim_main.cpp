// sim_main.cpp -- run RoadSense_v4_ML.ino on the PC, fed sample-by-sample from a log.
// build: see tests/Makefile (target sim)
// usage: ./sim ../../edge_ml/logs/hatchback.txt > ../../edge_ml/results/fw_hatchback.txt
#include "mock_arduino.h"
#include "RoadSense_v4_ML.ino"   // found via -I ../../firmware/esp32/RoadSense_v4_ML
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
uint32_t g_ms = 0; uint64_t g_us = 0;
std::deque<uint8_t> g_i2c;
Stream_ Serial; TwoWire Wire; LittleFSFS LittleFS;
struct Raw { uint32_t ms; int16_t v[6]; };
int main(int argc, char** argv) {
  std::ifstream f(argv[1]); std::string l;
  std::vector<Raw> raw; std::vector<uint32_t> ks;
  // use the LONGEST #BOOT session, like the Python tools
  std::vector<std::vector<Raw>> segR(1); std::vector<std::vector<uint32_t>> segK(1);
  while (std::getline(f, l)) {
    if (l.find("#BOOT") != std::string::npos) { segR.emplace_back(); segK.emplace_back(); continue; }
    if (l.rfind("R,", 0) == 0) {
      Raw r; long ms; int a[6];
      if (sscanf(l.c_str(), "R,%ld,%d,%d,%d,%d,%d,%d", &ms, &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) == 7) {
        r.ms = ms; for (int i = 0; i < 6; i++) r.v[i] = a[i];
        if (segR.back().empty() || r.ms > segR.back().back().ms) segR.back().push_back(r);
      }
    } else if (l.rfind("K,", 0) == 0) { long k; if (sscanf(l.c_str(), "K,%ld", &k) == 1) segK.back().push_back(k); }
  }
  size_t best = 0; for (size_t i = 0; i < segR.size(); i++) if (segR[i].size() > segR[best].size()) best = i;
  raw = segR[best]; ks = segK[best];
  if (raw.empty()) return 1;
  // first calibration = setup(); later ones = 'c' key at the logged K times
  g_ms = ks.empty() ? raw[0].ms : ks[0];
  setup();
  size_t ki = 1;
  for (const Raw& r : raw) {
    g_ms = r.ms; g_us = (uint64_t)r.ms * 1000;
    while (ki < ks.size() && r.ms >= ks[ki]) { Serial.in.push_back('c'); ki++; }
    nextSampleUs = (uint32_t)g_us;                  // make this loop() take exactly this sample
    g_i2c.clear();
    int16_t b[7] = { r.v[0], r.v[1], r.v[2], 0, r.v[3], r.v[4], r.v[5] };
    for (int i = 0; i < 7; i++) { g_i2c.push_back((uint8_t)((uint16_t)b[i] >> 8)); g_i2c.push_back((uint8_t)(b[i] & 0xff)); }
    loop();
  }
  return 0;
}
