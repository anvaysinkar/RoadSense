/*
 * rs_baseline.h -- ABC (Adaptive Baseline Calibration), C version of AdaptiveBaseline.h.
 * Tracks mu and sigma of one signal on normal road; threshold = mu + k * sigma.
 */
#ifndef RS_BASELINE_H
#define RS_BASELINE_H

#include <stdint.h>

typedef struct {
  double sum, sumSq, var_;
  uint32_t n;
  float mu, sigma, floorAbs, floorRel;
  int ready;
} rs_baseline_t;

void  rs_bl_begin(rs_baseline_t* b);
void  rs_bl_add(rs_baseline_t* b, float x);
int   rs_bl_finish(rs_baseline_t* b, float floorAbs, float floorRel);  /* 0 if no samples */
void  rs_bl_adapt(rs_baseline_t* b, float x, float alpha);              /* normal-road samples only */

static inline float rs_bl_thr(const rs_baseline_t* b, float k) { return b->mu + k * b->sigma; }
static inline float rs_bl_z(const rs_baseline_t* b, float x) { return b->sigma > 0 ? (x - b->mu) / b->sigma : 0.0f; }

#endif
