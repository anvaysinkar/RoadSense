#pragma once
#include "FS.h"
struct LittleFSFS { bool begin(bool) { return true; } File open(const char*, const char*) { return File(); }
  bool remove(const char*) { return true; } size_t totalBytes() { return 1; } size_t usedBytes() { return 0; } };
extern LittleFSFS LittleFS;
