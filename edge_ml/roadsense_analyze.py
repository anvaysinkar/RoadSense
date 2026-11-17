#!/usr/bin/env python3
"""
roadsense_analyze.py -- offline replay, scoring and tuning for RoadSense v3 logs.

Works on:
  * a CoolTerm capture of RoadSense_v3.ino (has R lines = raw 100 Hz data), or
  * a flash dump (D key) -- no raw data, so only the classification step
    (HEAVE_DUR_BREAKER_MS, ROUGH_BURST_HITS) can be re-tuned.

What it does
  1. Parses the log (splits on #BOOT, so several power-ups in one file are fine).
  2. If raw data is present, replays the EXACT firmware pipeline in Python
     with any parameters you choose.
  3. Matches detected events to your ground-truth tags (p/b keys or button).
  4. Prints a confusion matrix + precision/recall/F1 per class.
  5. --tune: grid-searches the k constants to maximise macro-F1 and prints
     the values to paste into RoadSense_v3.ino.
  6. Exports
       <out>_events.csv  every event with all features and its matched tag
       <out>_map.csv     timestamp,lat,lon,effectiveDelta,LABEL  (old format,
                         for map_anomalies.py: events + YELLOW/RED rows)

Examples
  python3 roadsense_analyze.py hatchback_run1.txt
  python3 roadsense_analyze.py hatchback_run1.txt --tune --save-params best.json
  python3 roadsense_analyze.py sedan_run1.txt --params best.json      # test on another car
"""
import argparse, csv, json, math, sys, itertools
from collections import defaultdict

# ---------------- default parameters (must match RoadSense_v3.ino) ----------------
DEFAULTS = dict(
    K_EVENT=5.0, K_LOCAL=6.0, K_HEAVE_TRIG=6.0, K_HEAVE=2.0, HEAVE_DUR_BREAKER_MS=450, ROUGH_BURST_HITS=3,
    K_ROUGH_YELLOW=2.5, K_ROUGH_RED=4.0,
    PRE_MS=200, POST_MS=1200, HIT_GAP_MS=60,
    HEAVE_LP_HZ=3.0, TC_BASELINE_S=30.0, TC_GRAVITY_S=5.0,
    GRAV_CALIB_MS=1500, STAT_CALIB_MS=6500, ACC_LSB_PER_G=8192.0,
)
TAG_DEDUP_MS = 1000
EVENT_FIELDS = ["jerkPk_mg", "jerkZ", "heavePP_mg", "heaveZ", "heaveDurMs", "nHits", "hitSpanMs", "firstSign"]


# ---------------- parsing ----------------
def parse_log(path):
    segs = []
    cur = None

    def new_seg():
        s = dict(raw=[], events=[], marks=[], kcal=[], ccal=[], summary=[])
        segs.append(s)
        return s

    with open(path, "r", errors="replace") as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            if "#BOOT" in ln:   # boot ROM noise can precede it on the same line
                cur = new_seg()
                continue
            if cur is None:
                cur = new_seg()
            if ln.startswith("#"):
                continue
            p = ln.split(",")
            try:
                t = p[0]
                if t == "R" and len(p) == 8:
                    ms = int(p[1])
                    if cur["raw"] and ms <= cur["raw"][-1][0]:
                        cur.setdefault("bad_raw", 0)
                        cur["bad_raw"] = cur.get("bad_raw", 0) + 1   # duplicated / out-of-order line
                        continue
                    cur["raw"].append((ms,) + tuple(int(x) for x in p[2:8]))
                elif t == "E" and len(p) == 13:
                    ev = dict(t=int(p[1]), lat=float(p[2]), lon=float(p[3]), label=p[4])
                    vals = [float(x) for x in p[5:13]]
                    ev.update(dict(zip(EVENT_FIELDS, vals)))
                    cur["events"].append(ev)
                elif t == "M" and len(p) >= 5:
                    cur["marks"].append(dict(t=int(p[1]), lat=float(p[2]), lon=float(p[3]), label=p[4]))
                elif t == "K" and len(p) == 2:
                    cur["kcal"].append(int(p[1]))
                elif t == "C":
                    cur["ccal"].append(p[1:])
                elif len(p) == 5 and p[0].isdigit():
                    cur["summary"].append(dict(t=int(p[0]), lat=float(p[1]), lon=float(p[2]),
                                               eff=float(p[3]), label=p[4]))
            except (ValueError, IndexError):
                continue   # partial / corrupted line
    # merge repeated presses of the same tag key (e.g. key mashed because
    # nothing appeared on screen): same label within TAG_DEDUP_MS = one tag
    for s in segs:
        merged = []
        for m in sorted(s["marks"], key=lambda m: m["t"]):
            if merged and merged[-1]["label"] == m["label"] and m["t"] - merged[-1]["_last"] < TAG_DEDUP_MS:
                merged[-1]["_last"] = m["t"]; merged[-1]["_n"] += 1
                continue
            m = dict(m, _last=m["t"], _n=1); merged.append(m)
        s["dup_tags"] = len(s["marks"]) - len(merged)
        s["marks"] = merged
    return [s for s in segs if any(s[k] for k in ("raw", "events", "marks", "summary"))]


# ---------------- baseline (mirror of AdaptiveBaseline.h) ----------------
class Baseline:
    def __init__(self):
        self.begin()

    def begin(self):
        self.s = 0.0; self.ss = 0.0; self.n = 0; self.ready = False
        self.mu = 0.0; self.sigma = 0.0; self.var = 0.0; self.fa = 0.0; self.fr = 0.0

    def add(self, x):
        self.s += x; self.ss += x * x; self.n += 1

    def finish(self, fa, fr):
        if self.n < 2:
            return False
        self.mu = self.s / self.n
        v = self.ss / self.n - self.mu * self.mu
        self.sigma = math.sqrt(v if v > 0 else 0)
        self.fa, self.fr = fa, fr
        self._floor()
        self.var = self.sigma * self.sigma
        self.ready = True
        return True

    def _floor(self):
        f = max(self.fa, self.fr * abs(self.mu))
        if self.sigma < f:
            self.sigma = f

    def adapt(self, x, a):
        if not self.ready:
            return
        d = x - self.mu
        self.mu += a * d
        self.var += a * (d * d - self.var)
        if self.var < 0:
            self.var = 0
        self.sigma = math.sqrt(self.var)
        self._floor()

    def thr(self, k):
        return self.mu + k * self.sigma

    def z(self, x):
        return (x - self.mu) / self.sigma if self.sigma > 0 else 0.0


# ---------------- replay (mirror of RoadSense_v3.ino loop) ----------------
def classify(ev, P):
    if ev["nHits"] >= P["ROUGH_BURST_HITS"]:
        return "ROUGHBURST"
    if ev["heaveDurMs"] >= P["HEAVE_DUR_BREAKER_MS"]:
        return "BREAKER"
    return "POTHOLE"


def replay(raw, kcal, P, gps_lookup=None):
    """Returns (events, summary_rows). raw: list of (ms, ax, ay, az, gx, gy, gz)."""
    LSB = P["ACC_LSB_PER_G"]; DT = 0.01
    heave_alpha = DT / (1.0 / (2 * math.pi * P["HEAVE_LP_HZ"]) + DT)
    PRE_N = int(P["PRE_MS"] // 10); RWIN = 100
    jb, hb, rb = Baseline(), Baseline(), Baseline()

    cal_starts = sorted(kcal) if kcal else []
    if not cal_starts or (raw and raw[0][0] < cal_starts[0] - 20):
        if raw:
            cal_starts = [raw[0][0]] + [k for k in cal_starts if k > raw[0][0]]
    ci = 0

    state = None; cal_start = 0; gsum = [0.0, 0.0, 0.0]; gn = 0
    gvec = [0.0, 0.0, 1.0]
    prev = None; heave_lp = 0.0
    rring = [0.0] * RWIN; ridx = 0; rcount = 0; rsum = 0.0
    pre = []  # list of (ms, jerk, heave), max PRE_N
    ev = None; events = []; summary = []
    last_sum = 0; pending = None

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
        av = (a[0] * gvec[0] + a[1] * gvec[1] + a[2] * gvec[2]) / gm - gm
        heave_lp += heave_alpha * (av - heave_lp)
        heave = heave_lp

        if rcount == RWIN: rsum -= rring[ridx]
        else: rcount += 1
        rring[ridx] = jerk; rsum += jerk; ridx = (ridx + 1) % RWIN
        m1s = rsum / rcount

        if state == "STATS":
            jb.add(jerk); hb.add(heave)
            if rcount == RWIN and (ms % 100) < 10: rb.add(m1s)
            if ms - cal_start >= P["GRAV_CALIB_MS"] + P["STAT_CALIB_MS"]:
                jb.finish(0.002, 0.15); hb.finish(0.003, 0.0)
                if not rb.finish(0.001, 0.10):
                    rb.add(m1s); rb.add(m1s); rb.finish(0.001, 0.10)
                state = "RUN"
            pre.append((ms, jerk, heave)); pre = pre[-PRE_N:]
            continue

        # RUN
        trig = (jerk > jb.thr(P["K_EVENT"]) and jerk > P["K_LOCAL"] * m1s) or \
               abs(heave - hb.mu) > P["K_HEAVE_TRIG"] * hb.sigma
        if ev is None and trig:
            lat, lon = gps_lookup(ms) if gps_lookup else (0.0, 0.0)
            ev = dict(t=ms, lat=lat, lon=lon, nHits=0, firstSign=0, _pk=0.0, _hmax=-1e9, _hmin=1e9,
                      _hs=0, _above=False, _lastAbove=0, _first=0, _last=0,
                      _tj=jb.thr(P["K_EVENT"]), _thh=P["K_HEAVE"] * hb.sigma,
                      _jmu=jb.mu, _jsig=jb.sigma, _hsig=hb.sigma)
            for (pms, pj, ph) in pre:
                ev_feed(ev, pms, pj, ph)
        if ev is not None:
            ev_feed(ev, ms, jerk, heave)
            if ms - ev["t"] >= P["POST_MS"]:
                pp = ev["_hmax"] - ev["_hmin"]
                out = dict(t=ev["t"], lat=ev["lat"], lon=ev["lon"],
                           jerkPk_mg=ev["_pk"] * 1000, jerkZ=jb.z(ev["_pk"]),
                           heavePP_mg=pp * 1000, heaveZ=(pp / hb.sigma if hb.sigma > 0 else 0),
                           heaveDurMs=ev["_hs"] * 10, nHits=ev["nHits"],
                           hitSpanMs=ev["_last"] - ev["_first"], firstSign=ev["firstSign"])
                out["label"] = classify(out, P)
                events.append(out)
                if out["label"] != "ROUGHBURST":
                    pending = out["label"]
                ev = None
        pre.append((ms, jerk, heave)); pre = pre[-PRE_N:]

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
        if ms - last_sum >= 100:
            last_sum = ms
            if normal: rb.adapt(m1s, 0.1 / P["TC_BASELINE_S"])
            lat, lon = gps_lookup(ms) if gps_lookup else (0.0, 0.0)
            eff = max(0.0, (m1s - rb.mu) * 1000)
            summary.append(dict(t=ms, lat=lat, lon=lon, eff=eff, label=pending or tier))
            pending = None
    return events, summary


def make_gps_lookup(seg):
    pts = [(r["t"], r["lat"], r["lon"]) for r in seg["summary"] if r["lat"] != 0 or r["lon"] != 0]
    pts += [(m["t"], m["lat"], m["lon"]) for m in seg["marks"] if m["lat"] != 0 or m["lon"] != 0]
    pts.sort()
    if not pts:
        return None
    import bisect
    ts = [p[0] for p in pts]

    def look(ms):
        i = bisect.bisect_left(ts, ms)
        cands = [j for j in (i - 1, i) if 0 <= j < len(pts)]
        j = min(cands, key=lambda j: abs(ts[j] - ms))
        return (pts[j][1], pts[j][2]) if abs(ts[j] - ms) < 3000 else (0.0, 0.0)
    return look


# ---------------- scoring ----------------
def match(events, marks, before_ms, after_ms):
    """Each POTHOLE/BREAKER tag is matched to the nearest unmatched discrete event
    whose start lies in [tag - before_ms, tag + after_ms]."""
    gt = [m for m in marks if m["label"] in ("POTHOLE", "BREAKER")]
    used = set(); pairs = []
    for m in sorted(gt, key=lambda m: m["t"]):
        best = None
        for i, e in enumerate(events):
            if i in used or e["label"] == "ROUGHBURST":
                continue
            if m["t"] - before_ms <= e["t"] <= m["t"] + after_ms:
                if best is None or abs(e["t"] - m["t"]) < abs(events[best]["t"] - m["t"]):
                    best = i
        if best is not None:
            used.add(best)
        pairs.append((m, best))
    return pairs, used


def score(events, marks, before_ms=4000, after_ms=1000):
    pairs, used = match(events, marks, before_ms, after_ms)
    cm = defaultdict(int)
    for m, i in pairs:
        cm[(m["label"], events[i]["label"] if i is not None else "MISSED")] += 1
    fp = defaultdict(int)
    for i, e in enumerate(events):
        if i not in used and e["label"] in ("POTHOLE", "BREAKER"):
            fp[e["label"]] += 1
    res = {}
    for c in ("POTHOLE", "BREAKER"):
        tp = cm[(c, c)]
        pred = sum(v for (g, p), v in cm.items() if p == c) + fp[c]
        gtn = sum(v for (g, p), v in cm.items() if g == c)
        prec = tp / pred if pred else 0.0
        rec = tp / gtn if gtn else 0.0
        f1 = 2 * prec * rec / (prec + rec) if prec + rec else 0.0
        res[c] = dict(tp=tp, pred=pred, gt=gtn, precision=prec, recall=rec, f1=f1)
    classes = [c for c in res if res[c]["gt"] > 0]
    macro = sum(res[c]["f1"] for c in classes) / len(classes) if classes else 0.0
    return dict(cm=dict(cm), fp=dict(fp), per_class=res, macro_f1=macro, pairs=pairs)


def print_report(sc, title):
    print(f"\n=== {title} ===")
    cols = ["POTHOLE", "BREAKER", "ROUGHBURST", "MISSED"]
    hdr = "tag / detected"
    print(f"{hdr:<16}" + "".join(f"{c:>12}" for c in cols))
    for g in ("POTHOLE", "BREAKER"):
        print(f"{g:<16}" + "".join(f"{sc['cm'].get((g, c), 0):>12}" for c in cols))
    print(f"{'(untagged)':<16}" + "".join(f"{sc['fp'].get(c, 0):>12}" for c in cols[:2]) + "   <- false alarms")
    for c, r in sc["per_class"].items():
        if r["gt"] or r["pred"]:
            print(f"  {c:<8} precision {r['precision']:.2f}  recall {r['recall']:.2f}  F1 {r['f1']:.2f}"
                  f"  (tags {r['gt']}, detections {r['pred']})")
    print(f"  macro-F1 {sc['macro_f1']:.3f}")


def feature_table(events, pairs):
    rows = []
    for m, i in pairs:
        if i is None:
            rows.append((m["label"], "MISSED", None)); continue
        rows.append((m["label"], events[i]["label"], events[i]))
    if not rows:
        return
    print("\n--- features of tagged events (use this to see if POTHOLE vs BREAKER separate) ---")
    print(f"{'tag':<8}{'detected':<11}{'heaveDur':>9}{'heaveZ':>8}{'jerkZ':>8}{'nHits':>6}{'span':>6}{'sign':>5}")
    for g, d, e in rows:
        if e is None:
            print(f"{g:<8}{d:<11}"); continue
        print(f"{g:<8}{d:<11}{e['heaveDurMs']:>9.0f}{e['heaveZ']:>8.1f}{e['jerkZ']:>8.1f}"
              f"{e['nHits']:>6.0f}{e['hitSpanMs']:>6.0f}{e['firstSign']:>5.0f}")
    for c in ("POTHOLE", "BREAKER"):
        hd = sorted(e["heaveDurMs"] for g, d, e in rows if g == c and e)
        if hd:
            print(f"  {c}: heaveDur median {hd[len(hd)//2]:.0f} ms (min {hd[0]:.0f}, max {hd[-1]:.0f}, n={len(hd)})")


# ---------------- main ----------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--params", help="JSON file of parameters to use (from --save-params)")
    ap.add_argument("--tune", action="store_true", help="grid-search k constants against your tags")
    ap.add_argument("--save-params", help="write best/used parameters to this JSON file")
    ap.add_argument("--out", help="output prefix (default: log name without extension)")
    ap.add_argument("--tag-before", type=int, default=4000, help="ms an event may precede its tag")
    ap.add_argument("--tag-after", type=int, default=1000, help="ms an event may follow its tag")
    args = ap.parse_args()

    P = dict(DEFAULTS)
    if args.params:
        P.update(json.load(open(args.params)))
    segs = parse_log(args.log)
    if not segs:
        sys.exit("No RoadSense data found in " + args.log)
    has_raw = any(s["raw"] for s in segs)
    print(f"{args.log}: {len(segs)} session(s); raw 100 Hz data: {'YES' if has_raw else 'NO (flash dump / old log)'}")
    for i, s in enumerate(segs):
        dur = (s["raw"][-1][0] - s["raw"][0][0]) / 1000 if s["raw"] else 0
        print(f"  session {i+1}: {len(s['raw'])} raw samples ({dur:.0f} s), {len(s['events'])} device events, "
              f"{len(s['marks'])} tags, {len(s['summary'])} summary rows")
        if s.get("bad_raw"):
            print(f"    note: {s['bad_raw']} duplicated/out-of-order raw lines ignored")
        if s.get("dup_tags"):
            print(f"    note: {s['dup_tags']} repeated key presses merged (same tag within {TAG_DEDUP_MS} ms)")
        gaps = sum(1 for a, b in zip(s["raw"], s["raw"][1:]) if b[0] - a[0] > 30)
        if gaps:
            print(f"    note: {gaps} gaps >30 ms in raw data (dropped samples)")

    def run_all(P):
        evs, sums, marks = [], [], []
        for s in segs:
            if s["raw"]:
                e, sm = replay(s["raw"], s["kcal"], P, make_gps_lookup(s))
                if s["kcal"]:   # keep only data after the FINAL calibration (earlier ones were discarded by recalibrating)
                    t_start = max(s["kcal"]) + P["GRAV_CALIB_MS"] + P["STAT_CALIB_MS"]
                    e = [x for x in e if x["t"] >= t_start]; sm = [x for x in sm if x["t"] >= t_start]
                    s["marks"] = [m for m in s["marks"] if m["t"] >= t_start]
            else:
                e = [dict(x) for x in s["events"]]
                for x in e:
                    x["label"] = classify(x, P)
                sm = s["summary"]
            # sessions are kept separate (millis restarts on each boot)
            evs.append(e); sums.append(sm); marks.append(s["marks"])
        return evs, sums, marks

    def total_score(evs, marks):
        allpairs = []
        cm = defaultdict(int); fp = defaultdict(int)
        for e, m in zip(evs, marks):
            sc = score(e, m, args.tag_before, args.tag_after)
            for k, v in sc["cm"].items(): cm[k] += v
            for k, v in sc["fp"].items(): fp[k] += v
            allpairs.append(sc["pairs"])
        # recompute metrics on merged counts
        res = {}
        for c in ("POTHOLE", "BREAKER"):
            tp = cm[(c, c)]
            pred = sum(v for (g, p), v in cm.items() if p == c) + fp[c]
            gtn = sum(v for (g, p), v in cm.items() if g == c)
            pr = tp / pred if pred else 0.0; rc = tp / gtn if gtn else 0.0
            res[c] = dict(tp=tp, pred=pred, gt=gtn, precision=pr, recall=rc,
                          f1=2 * pr * rc / (pr + rc) if pr + rc else 0.0)
        classes = [c for c in res if res[c]["gt"] > 0]
        macro = sum(res[c]["f1"] for c in classes) / len(classes) if classes else 0.0
        return dict(cm=dict(cm), fp=dict(fp), per_class=res, macro_f1=macro, pairs=allpairs)

    evs, sums, marks = run_all(P)
    n_tags = sum(1 for ms in marks for m in ms if m["label"] in ("POTHOLE", "BREAKER"))
    if n_tags:
        sc = total_score(evs, marks)
        print_report(sc, "current parameters")
        for e, pairs in zip(evs, sc["pairs"]):
            feature_table(e, pairs)
    else:
        print("\nNo POTHOLE/BREAKER tags in this log -> cannot score accuracy (only exporting).")
        print("Event counts:", {l: sum(1 for es in evs for x in es if x["label"] == l)
                                for l in ("POTHOLE", "BREAKER", "ROUGHBURST")})

    if args.tune:
        if not n_tags:
            sys.exit("--tune needs tags (p/b keys or button) in the log.")
        if has_raw:
            grid_det = list(itertools.product([4, 5, 6, 7], [0, 3, 4, 6], [6, 8, 12, 999], [3.0, 4.0]))
        else:
            grid_det = [(P["K_EVENT"], P["K_LOCAL"], P["K_HEAVE_TRIG"], P["K_HEAVE"])]
        print(f"\nTuning: {len(grid_det)} detector settings x classification thresholds ...")
        best = None
        for gi, (ke, kl, kt, kh) in enumerate(grid_det):
            if gi and gi % 16 == 0:
                print(f"  {gi}/{len(grid_det)}", flush=True)
            Q = dict(P, K_EVENT=ke, K_LOCAL=kl, K_HEAVE_TRIG=kt, K_HEAVE=kh)
            evs_q, _, _ = run_all(Q)
            for hd in range(0, 801, 25):
                for rh in (4, 6, 8, 10, 99):
                    R = dict(Q, HEAVE_DUR_BREAKER_MS=hd, ROUGH_BURST_HITS=rh)
                    for es in evs_q:
                        for x in es:
                            x["label"] = classify(x, R)
                    s = total_score(evs_q, marks)
                    # tie-break: fewer false alarms, then values closest to defaults
                    key = (round(s["macro_f1"], 4), -sum(s["fp"].values()),
                           -abs(ke - DEFAULTS["K_EVENT"]) - abs(hd - DEFAULTS["HEAVE_DUR_BREAKER_MS"]) / 100)
                    if best is None or key > best[0]:
                        best = (key, dict(R), s)
        P = best[1]
        print_report(best[2], "TUNED parameters")
        print("\nPaste into RoadSense_v3.ino:")
        print(f"  const float    K_EVENT              = {P['K_EVENT']:.1f}f;")
        print(f"  const float    K_LOCAL              = {P['K_LOCAL']:.1f}f;")
        print(f"  const float    K_HEAVE_TRIG         = {P['K_HEAVE_TRIG']:.1f}f;")
        print(f"  const float    K_HEAVE              = {P['K_HEAVE']:.1f}f;")
        print(f"  const uint32_t HEAVE_DUR_BREAKER_MS = {int(P['HEAVE_DUR_BREAKER_MS'])};")
        print(f"  const int      ROUGH_BURST_HITS     = {int(P['ROUGH_BURST_HITS'])};")
        print("Caution: tuned on this drive only. Check it on a different vehicle's log with --params"
              " before trusting the numbers (that is also your cross-vehicle result).")
        evs, sums, marks = run_all(P)

    if args.save_params:
        json.dump({k: P[k] for k in ("K_EVENT", "K_LOCAL", "K_HEAVE_TRIG", "K_HEAVE",
                                     "HEAVE_DUR_BREAKER_MS", "ROUGH_BURST_HITS")},
                  open(args.save_params, "w"), indent=2)
        print(f"\nSaved parameters -> {args.save_params}")

    # ---- exports ----
    prefix = args.out or args.log.rsplit(".", 1)[0]
    with open(prefix + "_events.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["session", "t_ms", "lat", "lon", "label"] + EVENT_FIELDS + ["tag"])
        for si, (es, ms) in enumerate(zip(evs, marks)):
            pairs, _ = match(es, ms, args.tag_before, args.tag_after)
            tag_of = {i: m["label"] for m, i in pairs if i is not None}
            for i, e in enumerate(es):
                w.writerow([si + 1, e["t"], f"{e['lat']:.6f}", f"{e['lon']:.6f}", e["label"]] +
                           [round(e[k], 2) for k in EVENT_FIELDS] + [tag_of.get(i, "")])
    with open(prefix + "_map.csv", "w", newline="") as f:
        w = csv.writer(f)
        for es, sm in zip(evs, sums):
            rows = [(e["t"], e["lat"], e["lon"], e["jerkPk_mg"], e["label"]) for e in es
                    if e["label"] in ("POTHOLE", "BREAKER")]
            rows += [(r["t"], r["lat"], r["lon"], r["eff"], r["label"]) for r in sm
                     if r["label"] in ("YELLOW", "RED")]
            for r in sorted(rows):
                w.writerow([r[0], f"{r[1]:.6f}", f"{r[2]:.6f}", f"{r[3]:.2f}", r[4]])
    write_review(prefix, segs, evs, marks, args)
    print(f"\nWrote {prefix}_events.csv, {prefix}_map.csv, {prefix}_review.csv and {prefix}_ride_map.html")


MAP_HTML = r"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
:root{--bg:#fff;--fg:#1a1a1a;--muted:#666;--line:#ddd}
@media (prefers-color-scheme:dark){:root{--bg:#161616;--fg:#eee;--muted:#aaa;--line:#333}}
body{margin:0;font:14px/1.4 system-ui,sans-serif;background:var(--bg);color:var(--fg)}
header{padding:12px 16px}h1{font-size:17px;margin:0 0 4px}p{margin:0;color:var(--muted)}
#map{height:70vh;margin:0 16px;border-radius:8px;border:1px solid var(--line)}
.leg{display:flex;flex-wrap:wrap;gap:14px;padding:10px 16px;color:var(--muted)}
.d{display:inline-block;width:11px;height:11px;border-radius:50%;margin-right:5px;vertical-align:-1px}
.s{display:inline-block;width:10px;height:10px;margin-right:5px;vertical-align:-1px;border:2px solid #222;background:#fff}
</style></head><body>
<header><h1>__TITLE__</h1><p>Circles = device detections (ids match the review CSV). Squares = tags pressed in the car. Click a marker for details.</p></header>
<div id="map"></div>
<div class="leg"><span><i class="d" style="background:#d62728"></i>POTHOLE</span><span><i class="d" style="background:#ff8c00"></i>BREAKER</span><span><i class="d" style="background:#888"></i>ROUGHBURST</span><span><i class="s"></i>tag (P/B/R/O)</span></div>
<script>
const P=__PTS__, T=__TRACK__;
const map=L.map('map'); L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap'}).addTo(map);
if(T.length){const ln=L.polyline(T,{color:'#3b6fd8',weight:3,opacity:.6}).addTo(map);map.fitBounds(ln.getBounds(),{padding:[20,20]});}else{map.setView([18.52,73.85],13);}
const C={POTHOLE:'#d62728',BREAKER:'#ff8c00',ROUGHBURST:'#888'};
P.forEach(p=>{ if(!p.lat) return;
 if(p.kind==='det'){ L.circleMarker([p.lat,p.lon],{radius:8,color:'#fff',weight:1.5,fillColor:C[p.label],fillOpacity:.95}).addTo(map)
   .bindPopup(`<b>#${p.id} ${p.label}</b> at ${p.t}<br>key tag matched: ${p.tag||'none'}<br>heave ${p.hd} ms`)
   .bindTooltip(String(p.id),{permanent:true,direction:'right',offset:[6,0]}); }
 else { const ic=L.divIcon({html:`<div style="width:16px;height:16px;background:#fff;border:2px solid #222;font:bold 10px sans-serif;color:#222;text-align:center;line-height:16px">${p.label[0]}</div>`,className:'',iconSize:[20,20]});
   L.marker([p.lat,p.lon],{icon:ic}).addTo(map).bindPopup(`<b>Tag ${p.label}</b> at ${p.t}`); } });
</script></body></html>"""


def write_review(prefix, segs, evs, marks, args):
    """Review sheet (one row per detection) + interactive map, as used for the Kia ride."""
    rows, pts, track = [], [], []
    multi = len(segs) > 1
    nid = 0
    for si, (seg, es, ms) in enumerate(zip(segs, evs, marks)):
        t0 = seg["raw"][0][0] if seg["raw"] else (seg["summary"][0]["t"] if seg["summary"] else 0)
        def mmss(ms_):
            x = max(0, int(ms_) - t0) // 1000
            return (f"S{si+1} " if multi else "") + f"{x//60:02d}:{x%60:02d}"
        pairs, _ = match(es, ms, args.tag_before, args.tag_after)
        tag_of = {i: m["label"] for m, i in pairs if i is not None}
        for i, e in enumerate(es):
            nid += 1
            rows.append([nid, mmss(e["t"]), f"{e['lat']:.6f}", f"{e['lon']:.6f}", e["label"],
                         tag_of.get(i, "-"), int(e["heaveDurMs"]), ""])
            pts.append(dict(kind="det", id=nid, label=e["label"], lat=e["lat"], lon=e["lon"], t=mmss(e["t"]),
                            tag=tag_of.get(i, ""), hd=int(e["heaveDurMs"])))
        for m in ms:
            pts.append(dict(kind="tag", label=m["label"], lat=m["lat"], lon=m["lon"], t=mmss(m["t"])))
        track += [[r["lat"], r["lon"]] for r in seg["summary"][::10] if r["lat"] or r["lon"]]
    with open(prefix + "_review.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "time_mm:ss", "lat", "lon", "device_label", "your_key_tag", "heaveDur_ms",
                    "ACTUAL (optional, noted after drive)"])
        w.writerows(rows)
    title = "RoadSense ride: " + prefix.replace("\\", "/").split("/")[-1]
    html = MAP_HTML.replace("__TITLE__", title).replace("__PTS__", json.dumps(pts)).replace("__TRACK__", json.dumps(track))
    with open(prefix + "_ride_map.html", "w") as f:
        f.write(html)


if __name__ == "__main__":
    main()
