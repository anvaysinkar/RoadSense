#!/usr/bin/env python3
"""
wd_dataset.py -- Step 1 of the whole-drive plan: sliding-window dataset.

For each drive (analysed part = after the final calibration):
  * 1.4 s windows (140 samples) every 0.2 s over the whole drive
  * features: the same families as before, in RAW units and ABC-NORMALISED
    (divided by the car's own running baseline at the window start)
  * labels from the passenger tags, allowing for tag lag:
      each merged tag (BREAKER / SURFACE) owns a span of windows whose anchor
      (window start + 0.2 s, i.e. where a trigger would sit) lies in
      [tag - 5 s, tag + 1.5 s]  (the paper's matching rule).
      In that span the window with the strongest response (heave for a breaker,
      jerk for a surface defect) and its two neighbours are the POSITIVES;
      the rest of the span is IGNORE (not used for training).
      Every window outside all spans is NONE (no discrete anomaly).
No trigger / detector is involved: the model sees the whole drive.
"""
import json, os
import numpy as np
import roadsense_analyze as ra
import eval_3vehicles as ev3
import full_dataset as fd

VEH = ["hatchback", "sedan", "suv"]
P = dict(ra.DEFAULTS); P.update(json.load(open("best_3veh.json")))
WIN, HOP, PRE = 140, 20, 20          # samples: 1.4 s window, 0.2 s hop, anchor 0.2 s after start
SPAN_BEFORE, SPAN_AFTER = 5000, 1500  # ms, paper matching rule
OUT = "results/wholedrive"
FS = 100.0
BANDS = [(0.5, 4.0), (4.0, 10.0), (10.0, 20.0), (20.0, 50.01)]

SHARED = ["ratio_local", "t_peak_s", "bf_0_4", "bf_4_10", "bf_10_20", "bf_20_50", "first_sign"]
RAW = ["jpk", "jmean", "jstd", "m1s", "hmax", "hmin", "hpp", "hstd", "avstd", "avpk", "ahstd", "dur_abs", "nhit_abs", "span_abs"]
NORM = ["jpk_z", "jmean_z", "jstd_n", "m1s_z", "hmax_n", "hmin_n", "hpp_n", "hstd_n", "avstd_n", "avpk_n", "ahstd_n", "dur", "nhit", "span"]
FEATURES = {"raw": RAW + SHARED, "norm": NORM + SHARED}

# fixed absolute thresholds for the RAW set = hatchback calibration (static values carried over)
H_MU_J, H_SIG_J, H_SIG_H = 0.09480, 0.06020, 0.01711
ABS_TJ, ABS_TH = H_MU_J + 5 * H_SIG_J, 2 * H_SIG_H


def bands(av):
    x = av - av.mean(); Pw = np.abs(np.fft.rfft(x)) ** 2; f = np.arange(len(Pw)) * FS / len(x)
    tot = Pw[1:].sum()
    if tot <= 0: return [0.0] * 4
    return [float(Pw[(f >= lo) & (f < hi) & (np.arange(len(Pw)) >= 1)].sum() / tot) for lo, hi in BANDS]


def hits(ms, j, thr, gap=60):
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
    return n, last - first


def window_rows(rows):
    """rows: analysed RUN samples (dicts from fd.trace). Returns list of window dicts."""
    ms = np.array([r["t_ms"] for r in rows], float)
    j = np.array([r["jerk_g"] for r in rows]); h = np.array([r["heave_g"] for r in rows])
    av = np.array([r["vert_acc_g"] for r in rows]); ah = np.array([r["horiz_acc_g"] for r in rows])
    m1 = np.array([r["jerk_1s_mean_g"] for r in rows])
    mj = np.array([r["mu_jerk"] for r in rows]); sj = np.array([r["sigma_jerk"] for r in rows])
    mh = np.array([r["mu_heave"] for r in rows]); sh = np.array([r["sigma_heave"] for r in rows])
    mr = np.array([r["mu_r"] for r in rows]); sr = np.array([r["sigma_r"] for r in rows])
    lat = [r["lat"] for r in rows]; lon = [r["lon"] for r in rows]
    out = []
    for s in range(0, len(rows) - WIN + 1, HOP):
        e = s + WIN
        if ms[e - 1] - ms[s] > (WIN + 10) * 10:   # skip windows spanning a data gap
            continue
        a = s + PRE                               # anchor sample (where a trigger would sit)
        jw, hw, avw, ahw, tw = j[s:e], h[s:e], av[s:e], ah[s:e], ms[s:e]
        # baseline as known BEFORE the window (no peeking at the window itself)
        b_mj, b_sj, b_mh, b_sh, b_mr, b_sr, b_m1 = mj[s], sj[s], mh[s], sh[s], mr[s], sr[s], m1[s]
        ipk = int(jw.argmax()); jpk = float(jw[ipk])
        hd = hw - b_mh
        act = np.abs(hd) > 2.0 * b_sh
        fs_ = float(np.sign(hd[act][0])) if act.any() else 0.0
        n_n, sp_n = hits(tw, jw, b_mj + 5 * b_sj)
        n_a, sp_a = hits(tw, jw, ABS_TJ)
        bf = bands(avw)
        f = dict(t_start=ms[s], t_anchor=ms[a], lat=lat[a], lon=lon[a],
                 ratio_local=jpk / max(b_m1, 1e-4), t_peak_s=(tw[ipk] - ms[s]) / 1000.0,
                 bf_0_4=bf[0], bf_4_10=bf[1], bf_10_20=bf[2], bf_20_50=bf[3], first_sign=fs_,
                 jpk=jpk, jmean=float(jw.mean()), jstd=float(jw.std()), m1s=float(b_m1),
                 hmax=float(hw.max()), hmin=float(hw.min()), hpp=float(hw.max() - hw.min()), hstd=float(hw.std()),
                 avstd=float(avw.std()), avpk=float(np.abs(avw - avw.mean()).max()), ahstd=float(ahw.std()),
                 dur_abs=float((np.abs(hw) > ABS_TH).sum() * 10), nhit_abs=float(n_a), span_abs=float(sp_a),
                 jpk_z=(jpk - b_mj) / b_sj, jmean_z=(float(jw.mean()) - b_mj) / b_sj, jstd_n=float(jw.std()) / b_sj,
                 m1s_z=(b_m1 - b_mr) / b_sr, hmax_n=(float(hw.max()) - b_mh) / b_sh, hmin_n=(float(hw.min()) - b_mh) / b_sh,
                 hpp_n=float(hw.max() - hw.min()) / b_sh, hstd_n=float(hw.std()) / b_sh,
                 avstd_n=float(avw.std()) / b_sj, avpk_n=float(np.abs(avw - avw.mean()).max()) / b_sj,
                 ahstd_n=float(ahw.std()) / b_sj, dur=float(act.sum() * 10), nhit=float(n_n), span=float(sp_n),
                 tier_frac_green=float(np.mean([r.get("tier") == "GREEN" for r in rows[s:e]])))
        out.append(f)
    return out


def label(wins, gt):
    """Assign NONE / BREAKER / SURFACE / IGNORE plus the tag each positive belongs to."""
    n = len(wins)
    lab = ["NONE"] * n; tag_id = [-1] * n
    anchors = np.array([w["t_anchor"] for w in wins])
    # breakers first so they win if spans overlap
    order = sorted(range(len(gt)), key=lambda i: 0 if gt[i]["group"] == "BREAKER" else 1)
    for gi in order:
        g = gt[gi]
        idx = np.where((anchors >= g["t"] - SPAN_BEFORE) & (anchors <= g["t"] + SPAN_AFTER))[0]
        if len(idx) == 0: continue
        for i in idx:
            if lab[i] == "NONE": lab[i] = "IGNORE"
        score = np.array([wins[i]["hpp_n"] if g["group"] == "BREAKER" else wins[i]["jpk_z"] for i in idx])
        best = idx[int(score.argmax())]
        for i in (best - 1, best, best + 1):
            if 0 <= i < n and (lab[i] in ("NONE", "IGNORE") or (g["group"] == "BREAKER")):
                lab[i] = g["group"]; tag_id[i] = gi
    return lab, tag_id


def build():
    data = {}
    for v in VEH:
        seg = max(ra.parse_log(f"../datasets/raw_drives/{v}.txt"), key=lambda s: len(s["raw"]))
        rows, events = fd.trace(seg, P)
        look = ra.make_gps_lookup(seg)
        t_an = ev3.calib_end(seg, P)
        for r in rows:
            r["lat"], r["lon"] = look(r["t_ms"]) if look else (0.0, 0.0)
        run = [r for r in rows if r["t_ms"] >= t_an and r.get("phase") == "RUN"]
        wins = window_rows(run)
        gt = ev3.gt_from_tags(seg["marks"], t_an)
        lab, tid = label(wins, gt)
        data[v] = dict(seg=seg, wins=wins, y=np.array(lab), tag_id=np.array(tid), gt=gt, t_an=t_an,
                       rule_events=[e for e in events if e["t"] >= t_an])
    return data


def X(wins, fs):
    return np.array([[w[k] for k in FEATURES[fs]] for w in wins], float)


if __name__ == "__main__":
    import csv
    os.makedirs(OUT, exist_ok=True)
    data = build()
    for v in VEH:
        d = data[v]; y = d["y"]
        print(v, len(d["wins"]), "windows:", {k: int((y == k).sum()) for k in ("NONE", "SURFACE", "BREAKER", "IGNORE")},
              "tags", len(d["gt"]))
        cols = ["t_start", "t_anchor", "lat", "lon"] + RAW + [c for c in NORM if c not in RAW] + SHARED + ["tier_frac_green"]
        with open(f"{OUT}/{v}_windows_1p4s_hop0p2s.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(cols + ["label", "tag_index"])
            for win, yy, ti in zip(d["wins"], y, d["tag_id"]):
                w.writerow([f"{win[c]:.6f}" if c in ("lat", "lon") else f"{win[c]:.6g}" for c in cols] + [yy, int(ti)])
