// rs_ml_features.h -- ML feature extraction for RoadSense (ESP32 / any C++).
// Mirrors rs_features.window_features() for the ABC-normalised feature set
// ("norm" in ml_experiment.py). Feature ORDER must match RSML_FEATURE_NAMES
// in the generated rs_ml_model.h.
//
// The window is the same one the rule classifier already uses:
// 0.2 s before the trigger (pre ring) + 1.2 s after it, ~141 samples at 100 Hz.
// Cost per event on an ESP32 @240 MHz: well under 1 ms (one pass for the
// statistics + Goertzel over <=80 DFT bins); runs once per event, not per sample.
#pragma once
#include <math.h>
#include <stdint.h>

#define RSML_MAXW   160          // samples kept per event window (141 used)
#define RSML_FS     100.0f

struct RsmlCal {                 // calibration snapshot taken at the trigger sample
  float mu_j, sig_j;             // jerk baseline (g)
  float mu_h, sig_h;             // heave baseline (g)
  float mu_r, sig_r;             // 1 s mean jerk baseline (g)
  float m1s;                     // 1 s mean jerk at the trigger (g)
};

struct RsmlWin {
  int n; uint32_t t0;
  uint32_t ms[RSML_MAXW];
  float j[RSML_MAXW], h[RSML_MAXW], av[RSML_MAXW], ah[RSML_MAXW];
};

static inline void rsml_reset(RsmlWin& w, uint32_t t0) { w.n = 0; w.t0 = t0; }

static inline void rsml_push(RsmlWin& w, uint32_t ms, float j, float h, float av, float ah) {
  if (w.n >= RSML_MAXW) return;
  w.ms[w.n] = ms; w.j[w.n] = j; w.h[w.n] = h; w.av[w.n] = av; w.ah[w.n] = ah; w.n++;
}

// population mean / std (two-pass, like numpy .mean() / .std()).
// float, not double: the ESP32 FPU is single precision (double is emulated in software).
static inline void rsml_meanstd(const float* x, int n, float& mean, float& sd) {
  float s = 0; for (int i = 0; i < n; i++) s += x[i];
  float m = s / n, v = 0;
  for (int i = 0; i < n; i++) { float d = x[i] - m; v += d * d; }
  mean = m; sd = sqrtf(v / n);
}

// Power fractions of the mean-removed vertical acceleration in
// [0.5,4) [4,10) [10,20) [20,50] Hz, DFT bins k = 1 .. n/2 (Goertzel per bin).
static inline void rsml_bands(const float* av, int n, float mean, float* bf) {
  static const float lo[4] = {0.5f, 4.0f, 10.0f, 20.0f};
  static const float hi[4] = {4.0f, 10.0f, 20.0f, 50.01f};
  float band[4] = {0, 0, 0, 0}; float tot = 0;
  for (int k = 1; k <= n / 2; k++) {
    float w = 2.0f * (float)M_PI * k / n, cw = cosf(w), coeff = 2.0f * cw;
    float s1 = 0, s2 = 0;
    for (int i = 0; i < n; i++) { float s0 = (av[i] - mean) + coeff * s1 - s2; s2 = s1; s1 = s0; }
    float p = s1 * s1 + s2 * s2 - coeff * s1 * s2;      // |X_k|^2
    if (p < 0) p = 0;
    tot += p;
    float f = k * RSML_FS / n;
    for (int b = 0; b < 4; b++) if (f >= lo[b] && f < hi[b]) band[b] += p;
  }
  for (int b = 0; b < 4; b++) bf[b] = tot > 0 ? band[b] / tot : 0.0f;
}

// Fills f[21] in the order of RSML_FEATURE_NAMES.
// durMs, nHits, spanMs, firstSign: the rule classifier's own window measurements.
static inline void rsml_features(const RsmlWin& w, const RsmlCal& c,
                                 float durMs, int nHits, float spanMs, int firstSign, float* f) {
  const int n = w.n;
  float jpk = w.j[0]; int ipk = 0;
  for (int i = 1; i < n; i++) if (w.j[i] > jpk) { jpk = w.j[i]; ipk = i; }
  float jmean, jstd, hmean, hstd, avmean, avstd, ahmean, ahstd;
  rsml_meanstd(w.j, n, jmean, jstd);
  rsml_meanstd(w.h, n, hmean, hstd);
  rsml_meanstd(w.av, n, avmean, avstd);
  rsml_meanstd(w.ah, n, ahmean, ahstd);
  float hmax = w.h[0], hmin = w.h[0], avpk = 0;
  for (int i = 0; i < n; i++) {
    if (w.h[i] > hmax) hmax = w.h[i];
    if (w.h[i] < hmin) hmin = w.h[i];
    float d = fabsf(w.av[i] - avmean); if (d > avpk) avpk = d;
  }
  // ABC-normalised amplitudes
  f[0]  = (jpk - c.mu_j) / c.sig_j;          // jpk_z
  f[1]  = (jmean - c.mu_j) / c.sig_j;        // jmean_z
  f[2]  = jstd / c.sig_j;                    // jstd_n
  f[3]  = (c.m1s - c.mu_r) / c.sig_r;        // m1s_z
  f[4]  = (hmax - c.mu_h) / c.sig_h;         // hmax_n
  f[5]  = (hmin - c.mu_h) / c.sig_h;         // hmin_n
  f[6]  = (hmax - hmin) / c.sig_h;           // hpp_n
  f[7]  = hstd / c.sig_h;                    // hstd_n
  f[8]  = avstd / c.sig_j;                   // avstd_n
  f[9]  = avpk / c.sig_j;                    // avpk_n
  f[10] = ahstd / c.sig_j;                   // ahstd_n
  f[11] = durMs;                             // dur  (per-vehicle heave threshold)
  f[12] = (float)nHits;                      // nhit (per-vehicle jerk threshold)
  f[13] = spanMs;                            // span
  // dimensionless, shared
  f[14] = jpk / (c.m1s > 1e-4f ? c.m1s : 1e-4f);   // ratio_local
  f[15] = (float)(int32_t)(w.ms[ipk] - w.t0);      // t_peak_ms
  rsml_bands(w.av, n, avmean, &f[16]);             // bf_0_4 .. bf_20_50
  f[20] = (float)firstSign;                        // first_sign
}
