#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include <stdint.h>

void gps_start(void);              /* start circular DMA reception on USART1 */
void gps_poll(void);               /* call often from the main loop */
int  gps_get(double* lat, double* lon);   /* 1 = valid fix less than 3 s old */
uint32_t gps_sats(void);
uint32_t gps_sentences(void);      /* count of good sentences (bring-up check) */

/* parser, usable on its own (PC test) */
void gps_feed_byte(char c, uint32_t nowMs);

#endif
