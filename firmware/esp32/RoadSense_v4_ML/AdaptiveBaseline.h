/*
 * AdaptiveBaseline.h  --  ABC v3 (Adaptive Baseline Calibration)
 *
 * One AdaptiveBaseline object tracks the "normal" statistics of ONE signal:
 *     mu    = mean of the signal on normal road
 *     sigma = standard deviation of the signal on normal road
 * and gives a threshold   T = mu + k * sigma   for any dimensionless k.
 *
 * Changes vs v2:
 *  - After calibration, mu/sigma adapt with a SLOW exponential moving
 *    average (time constant ~30 s) instead of a 30%-per-second blend of a
 *    0.5 s buffer. v2 drifted noticeably within seconds ("spontaneous"
 *    thresholds); v3 is calm but still follows real changes over a drive.
 *  - The caller decides which samples are "normal" (only those are fed to
 *    adapt()), so potholes/breakers/rough patches never leak into the
 *    baseline.
 *  - sigma has a floor, so a calibration done while nearly stationary
 *    cannot produce a near-zero sigma (which would flag everything).
 *
 * No GPS speed or OBD-II term is used. A speed/RPM scaling term can be
 * added inside threshold() in phase 2 without changing any caller.
 */
#pragma once
#include <math.h>

class AdaptiveBaseline {
public:
  // ---- calibration ----
  void beginCalib() { sum = 0; sumSq = 0; n = 0; ready = false; }

  void addCalib(float x) { sum += x; sumSq += (double)x * x; n++; }

  // floorAbs: minimum sigma in signal units; floorRel: minimum sigma as a
  // fraction of mu. Returns false if no samples were collected.
  bool finishCalib(float floorAbs, float floorRel) {
    if (n < 2) return false;
    mu = sum / n;
    double var = sumSq / n - (double)mu * mu;
    sigma = sqrt(var > 0 ? var : 0);
    sigFloorAbs = floorAbs; sigFloorRel = floorRel;
    applyFloor();
    var_ = (double)sigma * sigma;
    ready = true;
    return true;
  }

  // ---- slow adaptation (call only with "normal" samples) ----
  // alpha = sampleInterval / timeConstant, e.g. 0.01 s / 30 s = 0.00033
  void adapt(float x, float alpha) {
    if (!ready) return;
    float d = x - mu;
    mu += alpha * d;
    var_ += alpha * ((double)d * d - var_);
    if (var_ < 0) var_ = 0;
    sigma = sqrt(var_);
    applyFloor();
  }

  float threshold(float k) const { return mu + k * sigma; }
  float z(float x) const { return sigma > 0 ? (x - mu) / sigma : 0; }
  float getMu() const { return mu; }
  float getSigma() const { return sigma; }
  bool  isReady() const { return ready; }

private:
  void applyFloor() {
    float f = sigFloorAbs;
    float r = sigFloorRel * fabsf(mu);
    if (r > f) f = r;
    if (sigma < f) sigma = f;
  }
  double sum = 0, sumSq = 0, var_ = 0;
  unsigned long n = 0;
  float mu = 0, sigma = 0, sigFloorAbs = 0, sigFloorRel = 0;
  bool ready = false;
};
