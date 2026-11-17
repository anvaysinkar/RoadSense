#include "rs_ml_features.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void rsml_reset(rsml_win_t* w, uint32_t t0) { w->n = 0; w->t0 = t0; }

void rsml_push(rsml_win_t* w, uint32_t ms, float j, float h, float av, float ah) {
  if (w->n >= RSML_MAXW) return;
  w->ms[w->n] = ms; w->j[w->n] = j; w->h[w->n] = h; w->av[w->n] = av; w->ah[w->n] = ah; w->n++;
}

/* population mean / std, two-pass (matches numpy .mean() / .std()) */
static void meanstd(const float* x, int n, float* mean, float* sd) {
  float s = 0; for (int i = 0; i < n; i++) s += x[i];
  float m = s / n, v = 0;
  for (int i = 0; i < n; i++) { float d = x[i] - m; v += d * d; }
  *mean = m; *sd = sqrtf(v / n);
}

/* power fractions of mean-removed vertical accel in [0.5,4) [4,10) [10,20) [20,50] Hz
 * (DFT bins k = 1 .. n/2, one Goertzel pass per bin) */
static void bands(const float* av, int n, float mean, float* bf) {
  static const float lo[4] = {0.5f, 4.0f, 10.0f, 20.0f};
  static const float hi[4] = {4.0f, 10.0f, 20.0f, 50.01f};
  float band[4] = {0, 0, 0, 0}; float tot = 0;
  for (int k = 1; k <= n / 2; k++) {
    float w = 2.0f * (float)M_PI * k / n, coeff = 2.0f * cosf(w);
    float s1 = 0, s2 = 0;
    for (int i = 0; i < n; i++) { float s0 = (av[i] - mean) + coeff * s1 - s2; s2 = s1; s1 = s0; }
    float p = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    if (p < 0) p = 0;
    tot += p;
    float f = k * RSML_FS / n;
    for (int b = 0; b < 4; b++) if (f >= lo[b] && f < hi[b]) band[b] += p;
  }
  for (int b = 0; b < 4; b++) bf[b] = tot > 0 ? band[b] / tot : 0.0f;
}

void rsml_features(const rsml_win_t* w, const rsml_cal_t* c,
                   float durMs, int nHits, float spanMs, int firstSign, float* f) {
  const int n = w->n;
  float jpk = w->j[0]; int ipk = 0;
  for (int i = 1; i < n; i++) if (w->j[i] > jpk) { jpk = w->j[i]; ipk = i; }
  float jmean, jstd, hmean, hstd, avmean, avstd, ahmean, ahstd;
  meanstd(w->j, n, &jmean, &jstd);
  meanstd(w->h, n, &hmean, &hstd);
  meanstd(w->av, n, &avmean, &avstd);
  meanstd(w->ah, n, &ahmean, &ahstd);
  (void)hmean; (void)ahmean;
  float hmax = w->h[0], hmin = w->h[0], avpk = 0;
  for (int i = 0; i < n; i++) {
    if (w->h[i] > hmax) hmax = w->h[i];
    if (w->h[i] < hmin) hmin = w->h[i];
    float d = fabsf(w->av[i] - avmean); if (d > avpk) avpk = d;
  }
  f[0]  = (jpk - c->mu_j) / c->sig_j;     /* jpk_z   */
  f[1]  = (jmean - c->mu_j) / c->sig_j;   /* jmean_z */
  f[2]  = jstd / c->sig_j;                /* jstd_n  */
  f[3]  = (c->m1s - c->mu_r) / c->sig_r;  /* m1s_z   */
  f[4]  = (hmax - c->mu_h) / c->sig_h;    /* hmax_n  */
  f[5]  = (hmin - c->mu_h) / c->sig_h;    /* hmin_n  */
  f[6]  = (hmax - hmin) / c->sig_h;       /* hpp_n   */
  f[7]  = hstd / c->sig_h;                /* hstd_n  */
  f[8]  = avstd / c->sig_j;               /* avstd_n */
  f[9]  = avpk / c->sig_j;                /* avpk_n  */
  f[10] = ahstd / c->sig_j;               /* ahstd_n */
  f[11] = durMs;                          /* dur     */
  f[12] = (float)nHits;                   /* nhit    */
  f[13] = spanMs;                         /* span    */
  f[14] = jpk / (c->m1s > 1e-4f ? c->m1s : 1e-4f);   /* ratio_local */
  f[15] = (float)(int32_t)(w->ms[ipk] - w->t0);      /* t_peak_ms   */
  bands(w->av, n, avmean, &f[16]);                   /* bf_0_4 .. bf_20_50 */
  f[20] = (float)firstSign;                          /* first_sign  */
}
