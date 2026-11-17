#!/usr/bin/env python3
"""
full_dataset.py -- complete start-to-end datasets for each drive (smooth road included).

For every vehicle it writes two CSV files:

  <vehicle>_samples_100Hz.csv   one row per IMU sample (every 10 ms) for the whole drive:
                                raw sensor counts, acceleration in g, the firmware's own signals
                                (jerk, heave, 1 s mean jerk), its running baseline (mu, sigma),
                                the roughness tier, event state, GPS position and the tags.
  <vehicle>_windows_1s.csv      one row per 1-second window (non-overlapping) for the whole drive:
                                summary features of the window + labels. This is the table to
                                train on: GREEN / YELLOW / RED road, events and tags all in one.

The signal columns come from an exact mirror of RoadSense_v3.ino (same as roadsense_analyze.replay),
run with the frozen parameters in best_3veh.json. Verified: the events it finds are identical to
replay()'s.
"""
import csv, json, math, os, sys, bisect
import numpy as np
import roadsense_analyze as ra
import eval_3vehicles as ev3

VEH = ["hatchback", "sedan", "suv"]
OUT = "results/full_dataset"
P = dict(ra.DEFAULTS); P.update(json.load(open("best_3veh.json")))
GROUP = {"BREAKER": "BREAKER", "POTHOLE": "SURFACE", "ROUGH": "SURFACE", "OTHER": "OTHER"}


def trace(seg, P):
    """Replay the firmware on one session and record every sample."""
    raw, kcal = seg["raw"], seg["kcal"]
    look = ra.make_gps_lookup(seg)
    LSB = P["ACC_LSB_PER_G"]; DT = 0.01
    heave_alpha = DT / (1.0 / (2 * math.pi * P["HEAVE_LP_HZ"]) + DT)
    PRE_N = int(P["PRE_MS"] // 10); RWIN = 100
    jb, hb, rb = ra.Baseline(), ra.Baseline(), ra.Baseline()
    cal_starts = sorted(kcal) if kcal else []
    if not cal_starts or (raw and raw[0][0] < cal_starts[0] - 20):
        cal_starts = [raw[0][0]] + [k for k in cal_starts if k > raw[0][0]]
    ci = 0; state = None; cal_start = 0; gsum = [0.0] * 3; gn = 0; gvec = [0.0, 0.0, 1.0]
    prev = None; heave_lp = 0.0; rring = [0.0] * RWIN; ridx = 0; rcount = 0; rsum = 0.0
    pre = []; ev = None; events = []; rows = []; last_sum = 0; tier = ""
    ev_id = 0

    def ev_feed(ev, ms, jerk, heave):
        if jerk > ev["_pk"]: ev["_pk"] = jerk
        if heave > ev["_hmax"]: ev["_hmax"] = heave
        if heave < ev["_hmin"]: ev["_hmin"] = heave
        hd = heave - hb.mu
        if abs(hd) > ev["_thh"]:
            ev["_hs"] += 1
            if ev["firstSign"] == 0: ev["firstSign"] = 1 if hd > 0 else -1
        above = jerk > ev["_tj"]
        if above:
            if not ev["_above"] and (ev["nHits"] == 0 or ms - ev["_lastAbove"] > P["HIT_GAP_MS"]):
                ev["nHits"] += 1
                if ev["nHits"] == 1: ev["_first"] = ms
                ev["_last"] = ms
            ev["_lastAbove"] = ms
        ev["_above"] = above

    for (ms, ax, ay, az, gx, gy, gz) in raw:
        while ci < len(cal_starts) and ms >= cal_starts[ci]:
            state = "GRAV"; cal_start = cal_starts[ci]; ci += 1
            gsum = [0.0] * 3; gn = 0; jb.begin(); hb.begin(); rb.begin(); ev = None
        a = (ax / LSB, ay / LSB, az / LSB)
        row = dict(t_ms=ms, ax_raw=ax, ay_raw=ay, az_raw=az, gx_raw=gx, gy_raw=gy, gz_raw=gz,
                   ax_g=a[0], ay_g=a[1], az_g=a[2], phase=state or "PRE")
        if state is None:
            rows.append(row); continue
        jerk = 0.0
        if prev is not None:
            jerk = math.sqrt(sum((a[i] - prev[i]) ** 2 for i in range(3)))
        prev = a
        row["jerk_g"] = jerk
        if state == "GRAV":
            for i in range(3): gsum[i] += a[i]
            gn += 1
            if ms - cal_start >= P["GRAV_CALIB_MS"] and gn > 10:
                gvec = [gsum[i] / gn for i in range(3)]
                heave_lp = 0.0; ridx = 0; rcount = 0; rsum = 0.0; pre = []
                state = "STATS"
            rows.append(row); continue
        gm = math.sqrt(sum(g * g for g in gvec))
        proj = (a[0] * gvec[0] + a[1] * gvec[1] + a[2] * gvec[2]) / gm
        av = proj - gm
        ah = math.sqrt(max(0.0, a[0] ** 2 + a[1] ** 2 + a[2] ** 2 - proj * proj))
        heave_lp += heave_alpha * (av - heave_lp); heave = heave_lp
        if rcount == RWIN: rsum -= rring[ridx]
        else: rcount += 1
        rring[ridx] = jerk; rsum += jerk; ridx = (ridx + 1) % RWIN
        m1s = rsum / rcount
        row.update(vert_acc_g=av, horiz_acc_g=ah, heave_g=heave, jerk_1s_mean_g=m1s)
        if state == "STATS":
            jb.add(jerk); hb.add(heave)
            if rcount == RWIN and (ms % 100) < 10: rb.add(m1s)
            if ms - cal_start >= P["GRAV_CALIB_MS"] + P["STAT_CALIB_MS"]:
                jb.finish(0.002, 0.15); hb.finish(0.003, 0.0)
                if not rb.finish(0.001, 0.10):
                    rb.add(m1s); rb.add(m1s); rb.finish(0.001, 0.10)
                state = "RUN"
            pre.append((ms, jerk, heave)); pre = pre[-PRE_N:]
            row["phase"] = "STATS"
            rows.append(row); continue
        row["phase"] = "RUN"
        trig = (jerk > jb.thr(P["K_EVENT"]) and jerk > P["K_LOCAL"] * m1s) or \
               abs(heave - hb.mu) > P["K_HEAVE_TRIG"] * hb.sigma
        row.update(mu_jerk=jb.mu, sigma_jerk=jb.sigma, mu_heave=hb.mu, sigma_heave=hb.sigma,
                   mu_r=rb.mu, sigma_r=rb.sigma, jerk_z=jb.z(jerk), heave_z=(heave - hb.mu) / hb.sigma if hb.sigma else 0.0,
                   r_z=rb.z(m1s), trigger=int(trig))
        if ev is None and trig:
            ev_id += 1
            lat, lon = look(ms) if look else (0.0, 0.0)
            ev = dict(id=ev_id, t=ms, lat=lat, lon=lon, nHits=0, firstSign=0, _pk=0.0, _hmax=-1e9, _hmin=1e9, _hs=0,
                      _above=False, _lastAbove=0, _first=0, _last=0, _tj=jb.thr(P["K_EVENT"]), _thh=P["K_HEAVE"] * hb.sigma)
            for (pms, pj, ph) in pre: ev_feed(ev, pms, pj, ph)
        if ev is not None:
            ev_feed(ev, ms, jerk, heave)
            row["event_id"] = ev["id"]
            if ms - ev["t"] >= P["POST_MS"]:
                pp = ev["_hmax"] - ev["_hmin"]
                out = dict(id=ev["id"], t=ev["t"], t_end=ms, lat=ev["lat"], lon=ev["lon"], jerkPk_mg=ev["_pk"] * 1000,
                           heavePP_mg=pp * 1000, heaveDurMs=ev["_hs"] * 10, nHits=ev["nHits"],
                           hitSpanMs=ev["_last"] - ev["_first"], firstSign=ev["firstSign"])
                out["label"] = ra.classify(out, P)
                events.append(out); ev = None
        pre.append((ms, jerk, heave)); pre = pre[-PRE_N:]
        if m1s < rb.thr(P["K_ROUGH_YELLOW"]): tier = "GREEN"
        elif m1s < rb.thr(P["K_ROUGH_RED"]): tier = "YELLOW"
        else: tier = "RED"
        row["tier"] = tier
        normal = ev is None and tier == "GREEN"
        if normal:
            aS = DT / P["TC_BASELINE_S"]; jb.adapt(jerk, aS); hb.adapt(heave, aS)
        if ev is None:
            aG = DT / P["TC_GRAVITY_S"]
            for i in range(3): gvec[i] += aG * (a[i] - gvec[i])
        if ms - last_sum >= 100:
            last_sum = ms
            if normal: rb.adapt(m1s, 0.1 / P["TC_BASELINE_S"])
        rows.append(row)
    return rows, events


SAMPLE_COLS = ["t_ms", "t_s", "lat", "lon", "phase", "analysed",
               "ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw", "ax_g", "ay_g", "az_g",
               "jerk_g", "vert_acc_g", "horiz_acc_g", "heave_g", "jerk_1s_mean_g",
               "mu_jerk", "sigma_jerk", "mu_heave", "sigma_heave", "mu_r", "sigma_r",
               "jerk_z", "heave_z", "r_z", "trigger", "tier", "event_id", "event_label", "tag_pressed"]


def band(av, lo, hi, fs=100.0):
    x = np.asarray(av) - np.mean(av); Pw = np.abs(np.fft.rfft(x)) ** 2; f = np.arange(len(Pw)) * fs / len(x)
    tot = Pw[1:].sum()
    return float(Pw[(f >= lo) & (f < hi)][f[(f >= lo) & (f < hi)] > 0].sum() / tot) if tot > 0 else 0.0


def main():
    os.makedirs(OUT, exist_ok=True)
    summary = []
    for v in VEH:
        seg = max(ra.parse_log(f"../datasets/raw_drives/{v}.txt"), key=lambda s: len(s["raw"]))
        rows, events = trace(seg, P)
        # sanity: identical events to the reference replay
        ref, _ = ra.replay(seg["raw"], seg["kcal"], P, ra.make_gps_lookup(seg))
        assert [(e["t"], e["label"]) for e in ref] == [(e["t"], e["label"]) for e in events], v
        t_an = ev3.calib_end(seg, P)
        look = ra.make_gps_lookup(seg)
        t0 = rows[0]["t_ms"]
        ev_label = {e["id"]: e["label"] for e in events}
        # tags (raw key presses, after the repeated-press merge of the parser)
        marks = sorted(seg["marks"], key=lambda m: m["t"])
        mts = [m["t"] for m in marks]
        # ground truth groups (merged as in the evaluation) + matching to rule events
        gt = ev3.gt_from_tags(seg["marks"], t_an)
        an_events = [e for e in events if e["t"] >= t_an]
        pairs, _ = ev3.match(gt, an_events)
        ev_match = {an_events[i]["id"]: g["group"] for g, i in pairs if i is not None}
        mi = 0
        for r in rows:
            r["t_s"] = round((r["t_ms"] - t0) / 1000.0, 2)
            lat, lon = look(r["t_ms"]) if look else (0.0, 0.0)
            r["lat"], r["lon"] = round(lat, 6), round(lon, 6)
            r["analysed"] = int(r["t_ms"] >= t_an)
            if r.get("event_id"): r["event_label"] = ev_label.get(r["event_id"], "")
            # attach each tag to the first sample at or after it
            while mi < len(marks) and marks[mi]["t"] <= r["t_ms"]:
                r["tag_pressed"] = (r.get("tag_pressed", "") + " " + marks[mi]["label"]).strip(); mi += 1
        fn = f"{OUT}/{v}_samples_100Hz.csv"
        with open(fn, "w", newline="") as f:
            w = csv.writer(f); w.writerow(SAMPLE_COLS)
            for r in rows:
                w.writerow([("%.6f" % r[c]) if c in ("lat", "lon") else ("%.6g" % r[c]) if isinstance(r.get(c), float) else r.get(c, "") for c in SAMPLE_COLS])
        # ---------------- 1 s windows over the analysed part ----------------
        run = [r for r in rows if r["analysed"] and r.get("phase") == "RUN"]
        wrows = []
        start = run[0]["t_ms"]; end = run[-1]["t_ms"]
        idx = 0; wi = 0
        tag_list = [(m["t"], m["label"]) for m in marks if m["t"] >= t_an]
        while start + 1000 <= end + 10:
            seg_rows = []
            while idx < len(run) and run[idx]["t_ms"] < start + 1000:
                if run[idx]["t_ms"] >= start: seg_rows.append(run[idx])
                idx += 1
            if len(seg_rows) >= 50:
                j = np.array([r["jerk_g"] for r in seg_rows]); h = np.array([r["heave_g"] for r in seg_rows])
                av = np.array([r["vert_acc_g"] for r in seg_rows]); ah = np.array([r["horiz_acc_g"] for r in seg_rows])
                last = seg_rows[-1]; mid = seg_rows[len(seg_rows) // 2]
                tiers = [r["tier"] for r in seg_rows]
                tier = max(("GREEN", "YELLOW", "RED"), key=lambda t: (tiers.count(t), ("GREEN", "YELLOW", "RED").index(t)))
                eids = sorted({r["event_id"] for r in seg_rows if r.get("event_id")})
                trig_ids = [e for e in events if start <= e["t"] < start + 1000]
                tags_in = [lab for (tt, lab) in tag_list if start <= tt < start + 1000]
                # tag that this window most likely belongs to (tags lag the crossing: window start in [tag-5s, tag+1.5s])
                near = [lab for (tt, lab) in tag_list if tt - 5000 <= start + 500 <= tt + 1500]
                sj, mj = last["sigma_jerk"], last["mu_jerk"]; sh, mh = last["sigma_heave"], last["mu_heave"]
                wrows.append(dict(
                    window=wi, t_start_s=round((start - t0) / 1000, 2), t_start_ms=start, lat=mid["lat"], lon=mid["lon"], n_samples=len(seg_rows),
                    jerk_mean_g=j.mean(), jerk_std_g=j.std(), jerk_max_g=j.max(), jerk_p95_g=np.percentile(j, 95),
                    heave_min_g=h.min(), heave_max_g=h.max(), heave_pp_g=h.max() - h.min(), heave_std_g=h.std(),
                    vert_acc_std_g=av.std(), vert_acc_absmax_g=np.abs(av - av.mean()).max(), horiz_acc_std_g=ah.std(),
                    bf_0_4=band(av, 0.5, 4), bf_4_10=band(av, 4, 10), bf_10_20=band(av, 10, 20), bf_20_50=band(av, 20, 50.01),
                    jerk_max_z=(j.max() - mj) / sj, jerk_mean_z=(j.mean() - mj) / sj, heave_pp_n=(h.max() - h.min()) / sh,
                    heave_absmax_z=np.abs(h - mh).max() / sh, roughness_r_z=np.mean([r["r_z"] for r in seg_rows]),
                    frac_heave_beyond_2sigma=float(np.mean(np.abs(h - mh) > 2 * sh)),
                    mu_jerk=mj, sigma_jerk=sj, sigma_heave=sh,
                    tier=tier, frac_green=tiers.count("GREEN") / len(tiers), frac_yellow=tiers.count("YELLOW") / len(tiers), frac_red=tiers.count("RED") / len(tiers),
                    in_event=int(bool(eids)), event_trigger_label=" ".join(e["label"] for e in trig_ids),
                    event_matched_tag_group=" ".join(sorted({ev_match[e["id"]] for e in trig_ids if e["id"] in ev_match})),
                    tag_pressed_in_window=" ".join(tags_in), tag_nearby=" ".join(sorted(set(near))),
                ))
            start += 1000; wi += 1
        # one label column for training convenience
        for r in wrows:
            m = set(r["event_matched_tag_group"].split())
            if "BREAKER" in m: r["label_suggested"] = "BREAKER"
            elif "SURFACE" in m: r["label_suggested"] = "SURFACE"
            elif r["tier"] == "GREEN" and not r["in_event"] and not r["tag_nearby"]: r["label_suggested"] = "SMOOTH"
            else: r["label_suggested"] = "UNCERTAIN"
        wcols = list(wrows[0].keys())
        with open(f"{OUT}/{v}_windows_1s.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(wcols)
            for r in wrows:
                w.writerow([("%.6f" % r[c]) if c in ("lat", "lon") else ("%.6g" % r[c]) if isinstance(r[c], (float, np.floating)) else r[c] for c in wcols])
        n_run = len(run)
        tiers = [r["tier"] for r in run]
        lab = [r["label_suggested"] for r in wrows]
        summary.append((v, len(rows), round((rows[-1]["t_ms"] - t0) / 1000), n_run, round(n_run / 100), len(wrows),
                        tiers.count("GREEN") / n_run, tiers.count("YELLOW") / n_run, tiers.count("RED") / n_run,
                        len(an_events), {k: lab.count(k) for k in ("SMOOTH", "SURFACE", "BREAKER", "UNCERTAIN")}))
        print(v, "done")
    with open(f"{OUT}/summary.txt", "w") as f:
        for s in summary:
            line = (f"{s[0]}: {s[1]} samples ({s[2]} s total), analysed {s[3]} samples ({s[4]} s); {s[5]} one-second windows; "
                    f"tier share GREEN {s[6]:.0%} YELLOW {s[7]:.0%} RED {s[8]:.0%}; {s[9]} events; window labels {s[10]}")
            print(line); f.write(line + "\n")


if __name__ == "__main__":
    main()
