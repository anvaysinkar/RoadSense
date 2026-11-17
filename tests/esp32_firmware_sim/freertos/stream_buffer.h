#pragma once
#include <cstddef>
#include <cstdint>
typedef void* StreamBufferHandle_t; typedef void* SemaphoreHandle_t; typedef uint32_t TickType_t;
#define portMAX_DELAY 0xffffffff
#define pdMS_TO_TICKS(x) (x)
inline size_t xStreamBufferSpacesAvailable(StreamBufferHandle_t) { return 0; }
inline size_t xStreamBufferSend(StreamBufferHandle_t, const void*, size_t n, TickType_t) { return n; }
inline size_t xStreamBufferReceive(StreamBufferHandle_t, void*, size_t, TickType_t) { return 0; }
inline StreamBufferHandle_t xStreamBufferCreate(size_t, size_t) { return nullptr; }
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return nullptr; }
inline int xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return 1; }
inline int xSemaphoreGive(SemaphoreHandle_t) { return 1; }
inline void vTaskDelay(TickType_t) {}
inline int xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t, void*, int, void*, int) { return 1; }
