/*
 * gps_nmea.c -- NEO-6M on USART1 (PA10 = RX <- GPS TX), 9600 baud, circular DMA.
 * Small NMEA parser: $--RMC for position and fix status, $--GGA for satellite count.
 * Checksums are verified; bad sentences are ignored.
 */
#include "gps_nmea.h"
#include <stdlib.h>
#include <string.h>

#ifndef GPS_PC_TEST
#include "usart.h"
#define GPS_DMA_N 512
static uint8_t dmaBuf[GPS_DMA_N];
static uint32_t rdPos = 0;
#endif

static char sent[100]; static int sLen = 0;
static double fixLat = 0, fixLon = 0;
static int fixValid = 0; static uint32_t fixMs = 0, nowCache = 0;
static uint32_t sats = 0, goodSentences = 0;

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

/* ddmm.mmmm -> degrees */
static double nmea_deg(const char* s, char hemi) {
  if (!*s) return 0;
  double v = atof(s);
  int d = (int)(v / 100);
  double deg = d + (v - d * 100) / 60.0;
  return (hemi == 'S' || hemi == 'W') ? -deg : deg;
}

static void parse(char* s) {
  /* checksum: XOR of chars between '$' and '*' */
  char* star = strchr(s, '*');
  if (s[0] != '$' || !star || !star[1] || !star[2]) return;
  uint8_t cs = 0; for (char* p = s + 1; p < star; p++) cs ^= (uint8_t)*p;
  int h = hexval(star[1]), l = hexval(star[2]);
  if (h < 0 || l < 0 || cs != (uint8_t)(h * 16 + l)) return;
  *star = 0;
  char* f[20]; int nf = 0;
  for (char* p = s; nf < 20; ) { f[nf++] = p; char* c = strchr(p, ','); if (!c) break; *c = 0; p = c + 1; }
  goodSentences++;
  if (nf >= 7 && strlen(f[0]) == 6 && !strcmp(f[0] + 3, "RMC")) {
    if (f[2][0] == 'A' && f[3][0] && f[5][0]) {
      fixLat = nmea_deg(f[3], f[4][0]);
      fixLon = nmea_deg(f[5], f[6][0]);
      fixValid = 1; fixMs = nowCache;
    } else {
      fixValid = 0;
    }
  } else if (nf >= 8 && strlen(f[0]) == 6 && !strcmp(f[0] + 3, "GGA")) {
    sats = (uint32_t)atoi(f[7]);
  }
}

void gps_feed_byte(char c, uint32_t nowMs) {
  nowCache = nowMs;
  if (c == '$') sLen = 0;
  if (c == '\r' || c == '\n') {
    if (sLen > 0) { sent[sLen] = 0; parse(sent); }
    sLen = 0;
    return;
  }
  if (sLen < (int)sizeof sent - 1) sent[sLen++] = c; else sLen = 0;
}

int gps_get(double* lat, double* lon) {
  if (fixValid && nowCache - fixMs < 3000) { *lat = fixLat; *lon = fixLon; return 1; }
  return 0;
}

uint32_t gps_sats(void) { return sats; }
uint32_t gps_sentences(void) { return goodSentences; }

#ifndef GPS_PC_TEST
void gps_start(void) {
  rdPos = 0;
  HAL_UART_Receive_DMA(&huart1, dmaBuf, GPS_DMA_N);
}

void gps_poll(void) {
  uint32_t now = HAL_GetTick();
  if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {   /* DMA stopped after a UART error: restart */
    HAL_UART_AbortReceive(&huart1);
    gps_start();
    return;
  }
  uint32_t wr = GPS_DMA_N - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
  while (rdPos != wr) {
    gps_feed_byte((char)dmaBuf[rdPos], now);
    rdPos = (rdPos + 1) % GPS_DMA_N;
  }
  nowCache = now;
}
#endif
