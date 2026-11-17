#pragma once
#include <cstdint>
struct TinyGPSLocation { bool isValid() { return false; } uint32_t age() { return 0; } double lat() { return 0; } double lng() { return 0; } };
struct TinyGPSInteger { uint32_t value() { return 0; } };
struct TinyGPSPlus { TinyGPSLocation location; TinyGPSInteger satellites; bool encode(char) { return false; } };
