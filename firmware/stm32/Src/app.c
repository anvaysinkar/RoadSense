/*
 * app.c -- RoadSense on STM32F411 Black Pill.
 *   TIM2 interrupt at 100 Hz sets a flag; the main loop then reads the MPU6050 and runs
 *   the RoadSense core (signal chain, ABC calibration, trigger, 21 features, forest).
 *   GPS bytes arrive by circular DMA and are parsed in the loop. Everything goes out
 *   over USB CDC to CoolTerm (any baud setting works).
 *
 * KEYS (type in CoolTerm)
 *   p pothole tag   b breaker tag   r rough tag   o other tag
 *   c recalibrate (car still or on a smooth stretch)   s status   x raw stream on/off   ? help
 */
#include "app.h"
#include "main.h"
#include "tim.h"
#include "rs_core.h"
#include "rs_config.h"
#include "mpu6050.h"
#include "gps_nmea.h"
#include "usb_io.h"
#include <stdio.h>

static volatile uint32_t ticks = 0;
static uint32_t ticksDone = 0, missed = 0;

/* ---- ports used by rs_core ---- */
void rs_port_write(const char* line) { usb_io_write_line(line); }
int rs_port_gps(double* lat, double* lon) { return gps_get(lat, lon); }
uint32_t rs_port_gps_sats(void) { return gps_sats(); }
uint32_t rs_port_us(void) { return DWT->CYCCNT / (SystemCoreClock / 1000000u); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM2) ticks++;
}

static void help(void) {
  rs_port_write("#KEYS p=pothole b=breaker r=rough o=other | c=recalibrate s=status x=raw on/off ?=help");
}

static void status(void) {
  char l[128];
  rs_core_status();
  snprintf(l, sizeof l, "#IO gps_sentences=%lu i2c_errors=%lu usb_dropped_lines=%lu missed_samples=%lu",
           (unsigned long)gps_sentences(), (unsigned long)mpu_err_count(),
           (unsigned long)usb_io_dropped(), (unsigned long)missed);
  rs_port_write(l);
}

static void keys(void) {
  int c;
  while ((c = usb_io_getc()) >= 0) {
    uint32_t ms = HAL_GetTick();
    switch (c) {
      case 'p': rs_core_mark(ms, "POTHOLE", "key"); break;
      case 'b': rs_core_mark(ms, "BREAKER", "key"); break;
      case 'r': rs_core_mark(ms, "ROUGH", "key"); break;
      case 'o': rs_core_mark(ms, "OTHER", "key"); break;
      case 'c': rs_core_start_calib(ms); break;
      case 's': status(); break;
      case 'x': rs_core_set_raw(!rs_core_get_raw()); rs_port_write(rs_core_get_raw() ? "#RAW on" : "#RAW off"); break;
      case '?': help(); break;
      default: break;
    }
  }
}

void app_init(void) {
  char l[128];
  /* microsecond counter for timing the forest */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  HAL_Delay(100);
  rs_port_write(RS_USE_ML ? "#BOOT RoadSense STM32F411 (ML filter ON)" : "#BOOT RoadSense STM32F411 (rules only)");
  int e = mpu_init();
  int who = mpu_whoami();
  snprintf(l, sizeof l, "#MPU init err=%d WHO_AM_I=0x%02X (0x68 expected; clones may differ)", e, who & 0xFF);
  rs_port_write(l);
  gps_start();
  help();
  rs_core_init();
  rs_core_start_calib(HAL_GetTick());
  ticksDone = ticks;
  HAL_TIM_Base_Start_IT(&htim2);
}

void app_loop(void) {
  usb_io_pump();
  gps_poll();
  keys();
  uint32_t t = ticks;
  if (t == ticksDone) return;
  if (t - ticksDone > 1) missed += t - ticksDone - 1;   /* loop fell behind: count, don't burst */
  ticksDone = t;
  int16_t v[6];
  if (!mpu_read(v)) return;
  rs_core_sample(HAL_GetTick(), v);
}
