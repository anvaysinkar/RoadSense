/* pc_main.c -- run rs_core.c on the PC, fed sample by sample from a CoolTerm log.
 * usage: ./pctest log.txt > events.txt   (prints K, C, E, Q lines only) */
#include "rs_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
void rs_port_write(const char* l) { if (l[0]=='E'||l[0]=='Q'||l[0]=='K'||l[0]=='C') puts(l); }
int rs_port_gps(double* a, double* b) { (void)a; (void)b; return 0; }
uint32_t rs_port_gps_sats(void) { return 0; }
uint32_t rs_port_us(void) { return 0; }
typedef struct { uint32_t ms; int16_t v[6]; } raw_t;
#define MAXS 64
static raw_t* seg[MAXS]; static size_t segN[MAXS], segCap[MAXS]; static uint32_t segK[MAXS][64]; static int segKn[MAXS];
int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s log.txt\n", argv[0]); return 2; }
  FILE* f = fopen(argv[1], "r"); if (!f) return 1;
  int s = 0; char l[512];
  while (fgets(l, sizeof l, f)) {
    if (strstr(l, "#BOOT")) { if (s < MAXS - 1) s++; continue; }
    if (!strncmp(l, "R,", 2)) {
      long ms; int a[6];
      if (sscanf(l, "R,%ld,%d,%d,%d,%d,%d,%d", &ms, &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) == 7) {
        if (segN[s] && (uint32_t)ms <= seg[s][segN[s]-1].ms) continue;
        if (segN[s] == segCap[s]) { segCap[s] = segCap[s] ? 2*segCap[s] : 4096; seg[s] = realloc(seg[s], segCap[s]*sizeof(raw_t)); }
        raw_t* r = &seg[s][segN[s]++]; r->ms = ms; for (int i = 0; i < 6; i++) r->v[i] = a[i];
      }
    } else if (!strncmp(l, "K,", 2)) { long k; if (sscanf(l, "K,%ld", &k) == 1 && segKn[s] < 64) segK[s][segKn[s]++] = k; }
  }
  int best = 0; for (int i = 0; i <= s; i++) if (segN[i] > segN[best]) best = i;
  raw_t* raw = seg[best]; size_t n = segN[best];
  rs_core_init();
  rs_core_start_calib(segKn[best] ? segK[best][0] : raw[0].ms);
  int ki = 1;
  for (size_t i = 0; i < n; i++) {
    while (ki < segKn[best] && raw[i].ms >= segK[best][ki]) { rs_core_start_calib(raw[i].ms); ki++; }
    rs_core_sample(raw[i].ms, raw[i].v);
  }
  return 0;
}
