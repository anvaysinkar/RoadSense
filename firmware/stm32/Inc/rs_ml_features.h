/*
 * rs_ml_features.h -- the 21 ML features of one event window (C version).
 * Same maths and order as the ESP32 rs_ml_features.h / Python rs_features.py
 * ("norm" set). Window = 0.2 s before the trigger + 1.2 s after (about 141 samples).
 */
#ifndef RS_ML_FEATURES_H
#define RS_ML_FEATURES_H

#include <stdint.h>

#define RSML_MAXW 160
#define RSML_FS   100.0f

typedef struct {
  float mu_j, sig_j;   /* jerk baseline (g)          */
  float mu_h, sig_h;   /* heave baseline (g)         */
  float mu_r, sig_r;   /* 1 s mean jerk baseline (g) */
  float m1s;           /* 1 s mean jerk at the trigger (g) */
} rsml_cal_t;

typedef struct {
  int n; uint32_t t0;
  uint32_t ms[RSML_MAXW];
  float j[RSML_MAXW], h[RSML_MAXW], av[RSML_MAXW], ah[RSML_MAXW];
} rsml_win_t;

void rsml_reset(rsml_win_t* w, uint32_t t0);
void rsml_push(rsml_win_t* w, uint32_t ms, float j, float h, float av, float ah);

/* durMs, nHits, spanMs, firstSign: the rule classifier's own window measurements */
void rsml_features(const rsml_win_t* w, const rsml_cal_t* c,
                   float durMs, int nHits, float spanMs, int firstSign, float* f);

#endif
