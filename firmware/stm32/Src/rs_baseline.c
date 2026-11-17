#include "rs_baseline.h"
#include <math.h>

static void apply_floor(rs_baseline_t* b) {
  float f = b->floorAbs;
  float r = b->floorRel * fabsf(b->mu);
  if (r > f) f = r;
  if (b->sigma < f) b->sigma = f;
}

void rs_bl_begin(rs_baseline_t* b) { b->sum = 0; b->sumSq = 0; b->n = 0; b->ready = 0; }

void rs_bl_add(rs_baseline_t* b, float x) { b->sum += x; b->sumSq += (double)x * x; b->n++; }

int rs_bl_finish(rs_baseline_t* b, float floorAbs, float floorRel) {
  if (b->n < 2) return 0;
  b->mu = (float)(b->sum / b->n);
  double var = b->sumSq / b->n - (double)b->mu * b->mu;
  b->sigma = (float)sqrt(var > 0 ? var : 0);
  b->floorAbs = floorAbs; b->floorRel = floorRel;
  apply_floor(b);
  b->var_ = (double)b->sigma * b->sigma;
  b->ready = 1;
  return 1;
}

void rs_bl_adapt(rs_baseline_t* b, float x, float alpha) {
  if (!b->ready) return;
  float d = x - b->mu;
  b->mu += alpha * d;
  b->var_ += alpha * ((double)d * d - b->var_);
  if (b->var_ < 0) b->var_ = 0;
  b->sigma = (float)sqrt(b->var_);
  apply_floor(b);
}
