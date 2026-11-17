/*
 * rs_core.c -- RoadSense processing, ported line by line from RoadSense_v4_ML.ino (ESP32).
 * Every 100 Hz sample:
 *   jerk  = |a(t) - a(t-1)|                    sharp hits
 *   heave = 3 Hz low-pass of vertical accel    slow body motion ("vertical" learned from gravity)
 *   m1s   = mean jerk over the last 1 s        roughness
 * An event starts when jerk > mu_j + K_EVENT*sig_j and jerk > K_LOCAL * m1s, or when the heave
 * alone is beyond K_HEAVE_TRIG*sig_h. The window 0.2 s before .. 1.2 s after is then measured
 * and classified (forest if RS_USE_ML, otherwise heave-duration rule).
 */
#include "rs_core.h"
#include "rs_config.h"
#include "rs_baseline.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#if RS_USE_ML
#include "rs_ml_features.h"
#include "rs_ml_model.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum { CAL_GRAV, CAL_STATS, RUN } cal_state_t;
typedef struct { uint32_t ms; float jerk, heave, av, ah; } pre_s;

static rs_baseline_t jb, hb, rb;          /* jerk, heave, 1 s mean jerk */
static cal_state_t calState = CAL_GRAV;
static uint32_t calStartMs = 0;
static double gSum[3]; static uint32_t gN = 0;

static float gVec[3] = {0, 0, 1};
static float prevA[3]; static int havePrev = 0;
static float heaveLP = 0, heaveAlpha = 0;

static float rRing[RS_RWIN]; static int rIdx = 0, rCount = 0; static double rSum = 0;
static pre_s preRing[RS_PRE_N]; static int preIdx = 0, preCount = 0;

/* event state */
static int evActive = 0;
static uint32_t evT0 = 0;
static double evLat = 0, evLon = 0;
static float evTj = 0, evThH = 0, evJerkPk = 0, evHMax = -1e9f, evHMin = 1e9f;
static uint32_t evHeaveSamples = 0;
static int evHits = 0, evAbove = 0, evFirstSign = 0;
static uint32_t evLastAboveMs = 0, evFirstHitMs = 0, evLastHitMs = 0;
static const char* pendingEventLabel = 0;
#if RS_USE_ML
static rsml_win_t evWin;
static rsml_cal_t evCal;
static uint32_t mlDropped = 0, mlLastUs = 0, mlMaxUs = 0;
#endif

static uint32_t lastSummaryMs = 0;
static const char* currentTier = "GREEN";
static int rawStream = 1;
static uint32_t potholes = 0, breakers = 0, roughBursts = 0, marks = 0;
static char line[200];

static void cur_latlon(double* lat, double* lon) { if (!rs_port_gps(lat, lon)) { *lat = 0; *lon = 0; } }

static void pre_push(uint32_t ms, float jerk, float heave, float av, float ah) {
  pre_s p = { ms, jerk, heave, av, ah };
  preRing[preIdx] = p; preIdx = (preIdx + 1) % RS_PRE_N; if (preCount < (int)RS_PRE_N) preCount++;
}

void rs_core_set_raw(int on) { rawStream = on; }
int  rs_core_get_raw(void) { return rawStream; }

void rs_core_init(void) {
  heaveAlpha = RS_DT / (1.0f / (2.0f * (float)M_PI * RS_HEAVE_LP_HZ) + RS_DT);
}

void rs_core_start_calib(uint32_t ms) {
  calState = CAL_GRAV; calStartMs = ms;
  gSum[0] = gSum[1] = gSum[2] = 0; gN = 0;
  rs_bl_begin(&jb); rs_bl_begin(&hb); rs_bl_begin(&rb);
  evActive = 0;
  snprintf(line, sizeof line, "K,%lu", (unsigned long)ms);
  rs_port_write(line);
  rs_port_write("#CALIBRATING 8 s - drive normally on a SMOOTH stretch (or stay still)");
}

void rs_core_mark(uint32_t ms, const char* label, const char* src) {
  double lat, lon; cur_latlon(&lat, &lon);
  snprintf(line, sizeof line, "M,%lu,%.6f,%.6f,%s,%s", (unsigned long)ms, lat, lon, label, src);
  rs_port_write(line); marks++;
}

void rs_core_status(void) {
  double lat, lon; int fix = rs_port_gps(&lat, &lon);
  snprintf(line, sizeof line, "#STATUS cal=%d mu_j=%.4f sig_j=%.4f sig_h=%.4f mu_r=%.4f sig_r=%.4f sats=%lu fix=%d P=%lu B=%lu RB=%lu marks=%lu",
           (int)calState, jb.mu, jb.sigma, hb.sigma, rb.mu, rb.sigma, (unsigned long)rs_port_gps_sats(), fix,
           (unsigned long)potholes, (unsigned long)breakers, (unsigned long)roughBursts, (unsigned long)marks);
  rs_port_write(line);
#if RS_USE_ML
  snprintf(line, sizeof line, "#ML forest %d trees, dropped=%lu, last=%lu us, max=%lu us",
           RSML_NTREES, (unsigned long)mlDropped, (unsigned long)mlLastUs, (unsigned long)mlMaxUs);
  rs_port_write(line);
#endif
}

/* ---- event window measurements ---- */
static void ev_feed(uint32_t ms, float jerk, float heave) {
  if (jerk > evJerkPk) evJerkPk = jerk;
  if (heave > evHMax) evHMax = heave;
  if (heave < evHMin) evHMin = heave;
  float hd = heave - hb.mu;
  if (fabsf(hd) > evThH) {
    evHeaveSamples++;
    if (evFirstSign == 0) evFirstSign = hd > 0 ? 1 : -1;
  }
  int above = jerk > evTj;
  if (above) {
    if (!evAbove && (evHits == 0 || ms - evLastAboveMs > RS_HIT_GAP_MS)) {
      evHits++;
      if (evHits == 1) evFirstHitMs = ms;
      evLastHitMs = ms;
    }
    evLastAboveMs = ms;
  }
  evAbove = above;
}

static void ev_start(uint32_t ms, float m1s) {
  evActive = 1; evT0 = ms;
#if RS_USE_ML
  evCal.mu_j = jb.mu; evCal.sig_j = jb.sigma; evCal.mu_h = hb.mu; evCal.sig_h = hb.sigma;
  evCal.mu_r = rb.mu; evCal.sig_r = rb.sigma; evCal.m1s = m1s;
  rsml_reset(&evWin, ms);
#else
  (void)m1s;
#endif
  cur_latlon(&evLat, &evLon);
  evTj = rs_bl_thr(&jb, RS_K_EVENT);
  evThH = RS_K_HEAVE * hb.sigma;
  evJerkPk = 0; evHMax = -1e9f; evHMin = 1e9f; evHeaveSamples = 0;
  evHits = 0; evAbove = 0; evLastAboveMs = 0; evFirstSign = 0;
  int start = (preIdx - preCount + (int)RS_PRE_N) % (int)RS_PRE_N;
  for (int i = 0; i < preCount; i++) {
    const pre_s* p = &preRing[(start + i) % RS_PRE_N];
    ev_feed(p->ms, p->jerk, p->heave);
#if RS_USE_ML
    rsml_push(&evWin, p->ms, p->jerk, p->heave, p->av, p->ah);
#endif
  }
}

static void ev_finish(void) {
  evActive = 0;
  uint32_t heaveDur = evHeaveSamples * 10;
  const char* label;
#if RS_USE_ML
  float feat[RSML_NFEAT], pr[RSML_NCLASS];
  uint32_t t0us = rs_port_us();
  rsml_features(&evWin, &evCal, (float)heaveDur, evHits, (float)(evLastHitMs - evFirstHitMs), evFirstSign, feat);
  rsml_predict(feat, pr);
  mlLastUs = rs_port_us() - t0us; if (mlLastUs > mlMaxUs) mlMaxUs = mlLastUs;
  if (pr[RSML_CLASS_NONE] >= RS_ML_KEEP_PNONE)                { label = "NONE"; mlDropped++; }
  else if (pr[RSML_CLASS_BREAKER] >= pr[RSML_CLASS_SURFACE]) { label = "BREAKER"; breakers++; }
  else if (evHits >= RS_ROUGH_BURST_HITS)                    { label = "ROUGHBURST"; roughBursts++; }
  else                                                       { label = "POTHOLE"; potholes++; }
#else
  if (evHits >= RS_ROUGH_BURST_HITS)          { label = "ROUGHBURST"; roughBursts++; }
  else if (heaveDur >= RS_HEAVE_DUR_BRK_MS)   { label = "BREAKER"; breakers++; }
  else                                        { label = "POTHOLE"; potholes++; }
#endif
  float heavePP = evHMax - evHMin;
  snprintf(line, sizeof line, "E,%lu,%.6f,%.6f,%s,%.1f,%.2f,%.1f,%.2f,%lu,%d,%lu,%d",
           (unsigned long)evT0, evLat, evLon, label,
           evJerkPk * 1000.0f, rs_bl_z(&jb, evJerkPk),
           heavePP * 1000.0f, hb.sigma > 0 ? heavePP / hb.sigma : 0.0f,
           (unsigned long)heaveDur, evHits, (unsigned long)(evLastHitMs - evFirstHitMs), evFirstSign);
  rs_port_write(line);
#if RS_USE_ML
  snprintf(line, sizeof line, "Q,%lu,%.3f,%.3f,%.3f,%lu", (unsigned long)evT0,
           pr[RSML_CLASS_BREAKER], pr[RSML_CLASS_NONE], pr[RSML_CLASS_SURFACE], (unsigned long)mlLastUs);
  rs_port_write(line);
  if (strcmp(label, "NONE") == 0) return;   /* dropped by the forest: not on the map */
#endif
  if (strcmp(label, "ROUGHBURST") != 0) pendingEventLabel = label;
}

/* ---- one 100 Hz sample ---- */
void rs_core_sample(uint32_t ms, const int16_t v[6]) {
  if (rawStream) {
    snprintf(line, sizeof line, "R,%lu,%d,%d,%d,%d,%d,%d", (unsigned long)ms, v[0], v[1], v[2], v[3], v[4], v[5]);
    rs_port_write(line);
  }
  float a[3] = { v[0] / RS_ACC_LSB_PER_G, v[1] / RS_ACC_LSB_PER_G, v[2] / RS_ACC_LSB_PER_G };

  float jerk = 0;
  if (havePrev) {
    float dx = a[0] - prevA[0], dy = a[1] - prevA[1], dz = a[2] - prevA[2];
    jerk = sqrtf(dx * dx + dy * dy + dz * dz);
  }
  prevA[0] = a[0]; prevA[1] = a[1]; prevA[2] = a[2]; havePrev = 1;

  /* phase A: gravity direction */
  if (calState == CAL_GRAV) {
    gSum[0] += a[0]; gSum[1] += a[1]; gSum[2] += a[2]; gN++;
    if (ms - calStartMs >= RS_GRAV_CALIB_MS && gN > 10) {
      for (int i = 0; i < 3; i++) gVec[i] = (float)(gSum[i] / gN);
      heaveLP = 0; rIdx = 0; rCount = 0; rSum = 0; preCount = 0;
      calState = CAL_STATS;
    }
    return;
  }

  /* heave: low-passed vertical acceleration */
  float gm = sqrtf(gVec[0] * gVec[0] + gVec[1] * gVec[1] + gVec[2] * gVec[2]);
  float proj = (a[0] * gVec[0] + a[1] * gVec[1] + a[2] * gVec[2]) / gm;
  float av = proj - gm;
  float ah2 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2] - proj * proj;
  float ah = ah2 > 0 ? sqrtf(ah2) : 0;
  heaveLP += heaveAlpha * (av - heaveLP);
  float heave = heaveLP;

  /* 1 s roughness window */
  if (rCount == RS_RWIN) rSum -= rRing[rIdx]; else rCount++;
  rRing[rIdx] = jerk; rSum += jerk; rIdx = (rIdx + 1) % RS_RWIN;
  float m1s = (float)(rSum / rCount);

  /* phase B: noise statistics */
  if (calState == CAL_STATS) {
    rs_bl_add(&jb, jerk); rs_bl_add(&hb, heave);
    if (rCount == RS_RWIN && (ms % 100) < 10) rs_bl_add(&rb, m1s);
    if (ms - calStartMs >= RS_GRAV_CALIB_MS + RS_STAT_CALIB_MS) {
      rs_bl_finish(&jb, 0.002f, 0.15f);
      rs_bl_finish(&hb, 0.003f, 0.0f);
      if (!rs_bl_finish(&rb, 0.001f, 0.10f)) { rs_bl_add(&rb, m1s); rs_bl_add(&rb, m1s); rs_bl_finish(&rb, 0.001f, 0.10f); }
      calState = RUN;
      snprintf(line, sizeof line, "C,%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f", (unsigned long)ms,
               jb.mu, jb.sigma, hb.sigma, rb.mu, rb.sigma, gVec[0], gVec[1], gVec[2]);
      rs_port_write(line);
      rs_port_write("#CALIBRATION COMPLETE - detecting");
    }
    pre_push(ms, jerk, heave, av, ah);
    return;
  }

  /* RUN: discrete events */
  if (!evActive) {
    int jerkTrig  = jerk > rs_bl_thr(&jb, RS_K_EVENT) && jerk > RS_K_LOCAL * m1s;
    int heaveTrig = fabsf(heave - hb.mu) > RS_K_HEAVE_TRIG * hb.sigma;
    if (jerkTrig || heaveTrig) ev_start(ms, m1s);
  }
  if (evActive) {
    ev_feed(ms, jerk, heave);
#if RS_USE_ML
    rsml_push(&evWin, ms, jerk, heave, av, ah);
#endif
    if (ms - evT0 >= RS_POST_MS) ev_finish();
  }
  pre_push(ms, jerk, heave, av, ah);

  /* roughness tier */
  if (m1s < rs_bl_thr(&rb, RS_K_ROUGH_YELLOW))   currentTier = "GREEN";
  else if (m1s < rs_bl_thr(&rb, RS_K_ROUGH_RED)) currentTier = "YELLOW";
  else                                           currentTier = "RED";

  /* slow adaptation on normal road only */
  int normal = !evActive && currentTier[0] == 'G';
  if (normal) {
    const float aS = RS_DT / RS_TC_BASELINE_S;
    rs_bl_adapt(&jb, jerk, aS); rs_bl_adapt(&hb, heave, aS);
  }
  if (!evActive) {
    const float aG = RS_DT / RS_TC_GRAVITY_S;
    for (int i = 0; i < 3; i++) gVec[i] += aG * (a[i] - gVec[i]);
  }

  /* 10 Hz summary row */
  if (ms - lastSummaryMs >= 100) {
    lastSummaryMs = ms;
    if (normal) rs_bl_adapt(&rb, m1s, 0.1f / RS_TC_BASELINE_S);
    double lat, lon; cur_latlon(&lat, &lon);
    float eff = (m1s - rb.mu) * 1000.0f; if (eff < 0) eff = 0;
    const char* label = pendingEventLabel ? pendingEventLabel : currentTier;
    pendingEventLabel = 0;
    snprintf(line, sizeof line, "%lu,%.6f,%.6f,%.2f,%s", (unsigned long)ms, lat, lon, eff, label);
    rs_port_write(line);
  }
}
