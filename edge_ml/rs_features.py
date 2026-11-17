#!/usr/bin/env python3
"""
rs_features.py -- event candidates + ML features for RoadSense.

replay_ml() is the same pipeline as roadsense_analyze.replay() (identical
trigger, window and rule label), extended so that every event window also
yields the ML feature vector. Everything here is computed from the same
141-sample window the firmware already analyses (0.2 s before .. 1.2 s after
the trigger), so the ESP32 can compute it too (see firmware/esp32/RoadSense_v4_ML/rs_ml_features.h).

Two feature sets (same events, same model type; only the units differ):
  RAW  : amplitudes in g, durations/hit counts against FIXED absolute thresholds
         (the hatchback's calibrated values, i.e. a static threshold carried over)
  NORM : amplitudes divided by the vehicle's own calibrated noise (ABC), and
         durations/hit counts against the per-vehicle thresholds
Shared dimensionless features (in both sets): local contrast ratio, jerk-peak
timing, spectral band fractions of vertical acceleration, first heave sign.
"""
import math
import numpy as np
import roadsense_analyze as ra

FS = 100.0
BANDS = [(0.5, 4.0), (4.0, 10.0), (10.0, 20.0), (20.0, 50.01)]

# fixed absolute thresholds for the RAW set = hatchback calibration (C line of the tuning drive)
HATCH_MU_J, HATCH_SIG_J, HATCH_SIG_H = 0.09480, 0.06020, 0.01711
ABS_TJ = HATCH_MU_J + 5.0 * HATCH_SIG_J          # = K_EVENT * sigma, absolute
ABS_TH = 2.0 * HATCH_SIG_H                        # = K_HEAVE * sigma_h, absolute

SHARED = ["ratio_local", "t_peak_ms", "bf_0_4", "bf_4_10", "bf_10_20", "bf_20_50", "first_sign"]
RAW = ["jpk", "jmean", "jstd", "m1s", "hmax", "hmin", "hpp", "hstd", "avstd", "avpk", "ahstd",
       "dur_abs", "nhit_abs", "span_abs"]
NORM = ["jpk_z", "jmean_z", "jstd_n", "m1s_z", "hmax_n", "hmin_n", "hpp_n", "hstd_n", "avstd_n", "avpk_n",
        "ahstd_n", "dur", "nhit", "span"]
FEATURES = {"raw": RAW + SHARED, "norm": NORM + SHARED}


def band_fractions(av):
    """Power fractions of the (mean-removed) vertical acceleration in BANDS.
    Uses DFT bins k=1..n/2 (same as Goertzel per bin on the ESP32)."""
    x = np.asarray(av, float); x = x - x.mean(); n = len(x)
    P = np.abs(np.fft.rfft(x)) ** 2
    f = np.arange(len(P)) * FS / n
    tot = P[1:].sum()
    if tot <= 0:
        return [0.0] * len(BANDS)
    return [float(P[(f >= lo) & (f < hi) & (np.arange(len(P)) >= 1)].sum() / tot) for lo, hi in BANDS]


def hits(ms, j, thr, gap=60):
    """Separate excursions of jerk above thr (same rule as the firmware)."""
    n, above, last_above, first, last = 0, False, 0, 0, 0
    for t, x in zip(ms, j):
        a = x > thr
        if a:
            if not above and (n == 0 or t - last_above > gap):
                n += 1
                if n == 1: first = t
                last = t
            last_above = t
        above = a
    return n, (last - first)


def window_features(w, cal, rule):
    """w: dict of lists ms, j, h, av, ah over the event window. cal: calibration
    snapshot at trigger. rule: the rule-based event dict (heaveDurMs, nHits ...)."""
    ms = np.array(w["ms"]); j = np.array(w["j"]); h = np.array(w["h"])
    av = np.array(w["av"]); ah = np.array(w["ah"])
    t0 = w["t0"]
    jpk = float(j.max()); ipk = int(j.argmax())
    m1s = cal["m1s"]
    f = {}
    # shared, dimensionless
    f["ratio_local"] = jpk / max(m1s, 1e-4)
    f["t_peak_ms"] = float(ms[ipk] - t0)
    f["bf_0_4"], f["bf_4_10"], f["bf_10_20"], f["bf_20_50"] = band_fractions(av)
    f["first_sign"] = float(rule["firstSign"])
    # raw units
    f["jpk"] = jpk; f["jmean"] = float(j.mean()); f["jstd"] = float(j.std())
    f["m1s"] = m1s
    f["hmax"] = float(h.max()); f["hmin"] = float(h.min()); f["hpp"] = float(h.max() - h.min())
    f["hstd"] = float(h.std()); f["avstd"] = float(av.std()); f["avpk"] = float(np.abs(av - av.mean()).max())
    f["ahstd"] = float(ah.std())
    f["dur_abs"] = float(np.sum(np.abs(h) > ABS_TH) * 10)
    n, span = hits(ms, j, ABS_TJ)
    f["nhit_abs"] = float(n); f["span_abs"] = float(span)
    # normalised by this vehicle's calibrated noise (ABC)
    sj, mj, sh, mh = cal["sig_j"], cal["mu_j"], cal["sig_h"], cal["mu_h"]
    sr, mr = cal["sig_r"], cal["mu_r"]
    f["jpk_z"] = (jpk - mj) / sj; f["jmean_z"] = (f["jmean"] - mj) / sj; f["jstd_n"] = f["jstd"] / sj
    f["m1s_z"] = (m1s - mr) / sr
    f["hmax_n"] = (f["hmax"] - mh) / sh; f["hmin_n"] = (f["hmin"] - mh) / sh
    f["hpp_n"] = f["hpp"] / sh; f["hstd_n"] = f["hstd"] / sh
    f["avstd_n"] = f["avstd"] / sj; f["avpk_n"] = f["avpk"] / sj; f["ahstd_n"] = f["ahstd"] / sj
    f["dur"] = float(rule["heaveDurMs"]); f["nhit"] = float(rule["nHits"]); f["span"] = float(rule["hitSpanMs"])
    return f


def replay_ml(raw, kcal, P, gps_lookup=None):
    """Mirror of roadsense_analyze.replay() that also returns ML features per event."""
    LSB = P["ACC_LSB_PER_G"]; DT = 0.01
    heave_alpha = DT / (1.0 / (2 * math.pi * P["HEAVE_LP_HZ"]) + DT)
    PRE_N = int(P["PRE_MS"] // 10); RWIN = 100
    jb, hb, rb = ra.Baseline(), ra.Baseline(), ra.Baseline()
    cal_starts = sorted(kcal) if kcal else []
    if not cal_starts or (raw and raw[0][0] < cal_starts[0] - 20):
        if raw:
            cal_starts = [raw[0][0]] + [k for k in cal_starts if k > raw[0][0]]
    ci = 0
    state = None; cal_start = 0; gsum = [0.0, 0.0, 0.0]; gn = 0
    gvec = [0.0, 0.0, 1.0]
    prev = None; heave_lp = 0.0
    rring = [0.0] * RWIN; ridx = 0; rcount = 0; rsum = 0.0
    pre = []   # (ms, jerk, heave, av, ah)
    ev = None; events = []; last_sum = 0

    def ev_feed(ev, ms, jerk, heave):
        if jerk > ev["_pk"]: ev["_pk"] = jerk
        if heave > ev["_hmax"]: ev["_hmax"] = heave
        if heave < ev["_hmin"]: ev["_hmin"] = heave
        hd = heave - hb.mu
        if abs(hd) > ev["_thh"]:
            ev["_hs"] += 1
            if ev["firstSign"] == 0:
                ev["firstSign"] = 1 if hd > 0 else -1
        above = jerk > ev["_tj"]
        if above:
            if not ev["_above"] and (ev["nHits"] == 0 or ms - ev["_lastAbove"] > P["HIT_GAP_MS"]):
                ev["nHits"] += 1
                if ev["nHits"] == 1: ev["_first"] = ms
                ev["_last"] = ms
            ev["_lastAbove"] = ms
        ev["_above"] = above

    def push(ev, s):
        for k, v in zip(("ms", "j", "h", "av", "ah"), s):
            ev["_w"][k].append(v)

    for (ms, ax, ay, az, gx, gy, gz) in raw:
        while ci < len(cal_starts) and ms >= cal_starts[ci]:
            state = "GRAV"; cal_start = cal_starts[ci]; ci += 1
            gsum = [0.0, 0.0, 0.0]; gn = 0
            jb.begin(); hb.begin(); rb.begin(); ev = None
        if state is None:
            continue
        a = (ax / LSB, ay / LSB, az / LSB)
        jerk = 0.0
        if prev is not None:
            jerk = math.sqrt(sum((a[i] - prev[i]) ** 2 for i in range(3)))
        prev = a
        if state == "GRAV":
            for i in range(3): gsum[i] += a[i]
            gn += 1
            if ms - cal_start >= P["GRAV_CALIB_MS"] and gn > 10:
                gvec = [gsum[i] / gn for i in range(3)]
                heave_lp = 0.0; ridx = 0; rcount = 0; rsum = 0.0; pre = []
                state = "STATS"
            continue
        gm = math.sqrt(sum(g * g for g in gvec))
        proj = (a[0] * gvec[0] + a[1] * gvec[1] + a[2] * gvec[2]) / gm
        av = proj - gm
        amag2 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2]
        ah = math.sqrt(max(0.0, amag2 - proj * proj))
        heave_lp += heave_alpha * (av - heave_lp)
        heave = heave_lp
        if rcount == RWIN: rsum -= rring[ridx]
        else: rcount += 1
        rring[ridx] = jerk; rsum += jerk; ridx = (ridx + 1) % RWIN
        m1s = rsum / rcount
        samp = (ms, jerk, heave, av, ah)
        if state == "STATS":
            jb.add(jerk); hb.add(heave)
            if rcount == RWIN and (ms % 100) < 10: rb.add(m1s)
            if ms - cal_start >= P["GRAV_CALIB_MS"] + P["STAT_CALIB_MS"]:
                jb.finish(0.002, 0.15); hb.finish(0.003, 0.0)
                if not rb.finish(0.001, 0.10):
                    rb.add(m1s); rb.add(m1s); rb.finish(0.001, 0.10)
                state = "RUN"
            pre.append(samp); pre = pre[-PRE_N:]
            continue
        trig = (jerk > jb.thr(P["K_EVENT"]) and jerk > P["K_LOCAL"] * m1s) or \
               abs(heave - hb.mu) > P["K_HEAVE_TRIG"] * hb.sigma
        if ev is None and trig:
            lat, lon = gps_lookup(ms) if gps_lookup else (0.0, 0.0)
            ev = dict(t=ms, lat=lat, lon=lon, nHits=0, firstSign=0, _pk=0.0, _hmax=-1e9, _hmin=1e9,
                      _hs=0, _above=False, _lastAbove=0, _first=0, _last=0,
                      _tj=jb.thr(P["K_EVENT"]), _thh=P["K_HEAVE"] * hb.sigma,
                      _cal=dict(mu_j=jb.mu, sig_j=jb.sigma, mu_h=hb.mu, sig_h=hb.sigma,
                                mu_r=rb.mu, sig_r=rb.sigma, m1s=m1s),
                      _w=dict(ms=[], j=[], h=[], av=[], ah=[], t0=ms))
            for s in pre:
                ev_feed(ev, s[0], s[1], s[2]); push(ev, s)
        if ev is not None:
            ev_feed(ev, ms, jerk, heave); push(ev, samp)
            if ms - ev["t"] >= P["POST_MS"]:
                pp = ev["_hmax"] - ev["_hmin"]
                out = dict(t=ev["t"], lat=ev["lat"], lon=ev["lon"],
                           jerkPk_mg=ev["_pk"] * 1000, jerkZ=jb.z(ev["_pk"]),
                           heavePP_mg=pp * 1000, heaveZ=(pp / hb.sigma if hb.sigma > 0 else 0),
                           heaveDurMs=ev["_hs"] * 10, nHits=ev["nHits"],
                           hitSpanMs=ev["_last"] - ev["_first"], firstSign=ev["firstSign"])
                out["label"] = ra.classify(out, P)
                out["feat"] = window_features(ev["_w"], ev["_cal"], out)
                out["cal"] = ev["_cal"]; out["n_win"] = len(ev["_w"]["ms"]); out["win"] = ev["_w"]
                events.append(out)
                ev = None
        pre.append(samp); pre = pre[-PRE_N:]
        if m1s < rb.thr(P["K_ROUGH_YELLOW"]): tier = "GREEN"
        elif m1s < rb.thr(P["K_ROUGH_RED"]): tier = "YELLOW"
        else: tier = "RED"
        normal = ev is None and tier == "GREEN"
        if normal:
            aS = DT / P["TC_BASELINE_S"]
            jb.adapt(jerk, aS); hb.adapt(heave, aS)
        if ev is None:
            aG = DT / P["TC_GRAVITY_S"]
            for i in range(3): gvec[i] += aG * (a[i] - gvec[i])
        if ms - last_sum >= 100:          # 10 Hz roughness-baseline adaptation, as in replay()
            last_sum = ms
            if normal: rb.adapt(m1s, 0.1 / P["TC_BASELINE_S"])
    return events


def run(seg, P):
    return replay_ml(seg["raw"], seg["kcal"], P, ra.make_gps_lookup(seg))
