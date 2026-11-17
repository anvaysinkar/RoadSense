#pragma once
#include <cstddef>
#include <cstdint>
struct File { explicit operator bool() const { return true; } size_t write(const uint8_t*, size_t n) { return n; }
  void flush() {} void close() {} void println(const char*) {} int available() { return 0; } int read(uint8_t*, size_t) { return 0; } };
