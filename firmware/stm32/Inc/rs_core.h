/*
 * rs_core.h -- RoadSense signal chain, ABC calibration, event detection and
 * on-device classification. Pure C, no HAL calls: the same file runs on the
 * STM32 and in the PC test harness (pctest/), which is how the port is checked
 * against the logged drives.
 *
 * Output lines (identical to the ESP32 v4 firmware, so the Python tools still work):
 *   R,ms,ax,ay,az,gx,gy,gz                     raw 100 Hz counts
 *   ms,lat,lon,effectiveDelta,LABEL            10 Hz summary
 *   E,ms,lat,lon,LABEL,jerkPk_mg,jerkZ,heavePP_mg,heaveZ,heaveDurMs,nHits,hitSpanMs,firstSign
 *   Q,ms,pBreaker,pNone,pSurface,us            forest output (RS_USE_ML 1)
 *   M,ms,lat,lon,LABEL,src                     ground-truth tag
 *   K,ms  / C,ms,mu_j,sig_j,sig_h,mu_r,sig_r,gx,gy,gz   calibration start / result
 */
#ifndef RS_CORE_H
#define RS_CORE_H

#include <stdint.h>

/* ---- provided by the platform (app.c on the STM32, pctest on the PC) ---- */
void     rs_port_write(const char* line);            /* send one line (newline added by the port) */
int      rs_port_gps(double* lat, double* lon);       /* 1 = recent valid fix */
uint32_t rs_port_gps_sats(void);
uint32_t rs_port_us(void);                            /* free-running microsecond counter */

/* ---- core API ---- */
void rs_core_init(void);
void rs_core_start_calib(uint32_t ms);
void rs_core_sample(uint32_t ms, const int16_t v[6]);  /* one 100 Hz sample: ax ay az gx gy gz */
void rs_core_mark(uint32_t ms, const char* label, const char* src);
void rs_core_status(void);
void rs_core_set_raw(int on);
int  rs_core_get_raw(void);

#endif
