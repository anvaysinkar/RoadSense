#include "gps_nmea.h"
#include <stdio.h>
int main(void){
  const char* s="$GPRMC,083559.00,A,1830.6643,N,07349.9047,E,0.004,77.52,091202,,,A*59\r\n$GPGGA,083559.00,1830.6643,N,07349.9047,E,1,08,1.01,499.6,M,48.0,M,,*56\r\n";
  for(const char*p=s;*p;p++) gps_feed_byte(*p,1000);
  double a,b; int ok=gps_get(&a,&b); printf("ok=%d lat=%.6f lon=%.6f sats=%u sentences=%u\n",ok,a,b,gps_sats(),gps_sentences());
}
