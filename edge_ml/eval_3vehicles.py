#!/usr/bin/env python3
"""
eval_3vehicles.py -- cross-vehicle evaluation for the RoadSense paper.

Usage:  python3 eval_3vehicles.py hatchback.txt sedan.txt suv.txt --params best.json --out results

Protocol (all decisions fixed before looking at scores):
  * Detection parameters are FROZEN (best.json, tuned on an earlier hatchback test ride).
  * Each drive is analysed from its FINAL calibration onward (earlier data was
    recorded under a calibration the crew discarded by recalibrating).
  * Ground truth = in-vehicle key tags.  'b' -> BREAKER.  'p' and 'r' -> SURFACE
    (pothole / local undulation / rough patch): the crew could not consistently
    separate small undulations into pothole vs rough at tagging time, so they are
    one ground-truth group. Same-group tags < 2 s apart are one anomaly.
  * Device classes map: BREAKER -> BREAKER;  POTHOLE, ROUGHBURST -> SURFACE.
  * A tag is matched to the nearest unmatched device event that STARTS within
    [tag - 5 s, tag + 1.5 s] (tags lag the physical crossing by human reaction time).
  * Baseline for comparison: identical pipeline with the adaptive calibration
    removed -- thresholds frozen in absolute units at the values calibrated on the
    tuning vehicle (the 'fixed threshold transferred across vehicles' approach).
  * Cross-vehicle consistency: device events from all vehicles clustered by GPS.
"""
import argparse, csv, json, math, sys, os
from collections import defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import roadsense_analyze as ra

TAG_BEFORE, TAG_AFTER, MERGE_MS = 5000, 1500, 2000
GROUP = {"BREAKER": "BREAKER", "POTHOLE": "SURFACE", "ROUGH": "SURFACE", "ROUGHBURST": "SURFACE"}


def hav(a, b):
    R = 6371000.0
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    d = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * R * math.asin(math.sqrt(d))


ORIG_BASELINE = ra.Baseline


class StaticFactory:
    """Replaces Baseline objects inside replay(): fixed absolute thresholds, no adaptation."""
    def __init__(self, cfg):
        self.cfg = cfg; self.i = 0

    def __call__(self):
        mu, sig = self.cfg[self.i % 3]; self.i += 1
        b = ORIG_BASELINE()
        def finish(fa, fr, b=b, mu=mu, sig=sig):
            b.mu, b.sigma, b.var, b.ready = mu, sig, sig * sig, True
            return True
        b.finish = finish
        b.adapt = lambda x, a: None
        return b


def run_drive(seg, P, static_cfg=None):
    orig = ra.Baseline
    if static_cfg:
        ra.Baseline = StaticFactory(static_cfg)
    try:
        ev, summ = ra.replay(seg["raw"], seg["kcal"], P, ra.make_gps_lookup(seg))
    finally:
        ra.Baseline = orig
    t_start = calib_end(seg, P)
    ev = [e for e in ev if e["t"] >= t_start]
    summ = [r for r in summ if r["t"] >= t_start]
    return ev, summ, t_start


def calib_end(seg, P):
    k = max(seg["kcal"]) if seg["kcal"] else seg["raw"][0][0]
    return k + P["GRAV_CALIB_MS"] + P["STAT_CALIB_MS"]


def gt_from_tags(marks, t_start):
    out = []
    for m in sorted(marks, key=lambda m: m["t"]):
        if m["t"] < t_start or m["label"] not in GROUP:
            continue
        g = GROUP[m["label"]]
        if out and out[-1]["group"] == g and m["t"] - out[-1]["t_last"] < MERGE_MS:
            out[-1]["t_last"] = m["t"]; continue
        out.append(dict(t=m["t"], t_last=m["t"], lat=m["lat"], lon=m["lon"], group=g, raw=m["label"]))
    return out


def match(gt, ev):
    used, pairs = set(), []
    for g in gt:
        best = None
        for i, e in enumerate(ev):
            if i in used:
                continue
            if g["t"] - TAG_BEFORE <= e["t"] <= g["t"] + TAG_AFTER:
                if best is None or abs(e["t"] - g["t"]) < abs(ev[best]["t"] - g["t"]):
                    best = i
        if best is not None:
            used.add(best)
        pairs.append((g, best))
    return pairs, used


def score(gt, ev):
    pairs, used = match(gt, ev)
    r = dict(n_gt={"BREAKER": 0, "SURFACE": 0}, det={"BREAKER": 0, "SURFACE": 0},
             cls=defaultdict(int), n_ev=len(ev), untagged=len(ev) - len(used))
    for g, i in pairs:
        r["n_gt"][g["group"]] += 1
        if i is not None:
            r["det"][g["group"]] += 1
            r["cls"][(g["group"], GROUP[ev[i]["label"]])] += 1
    n_gt = sum(r["n_gt"].values()); n_det = sum(r["det"].values())
    r["recall"] = n_det / n_gt if n_gt else 0
    r["precision_vs_tags"] = len(used) / len(ev) if ev else 0
    ok = r["cls"][("BREAKER", "BREAKER")] + r["cls"][("SURFACE", "SURFACE")]
    r["class_acc"] = ok / n_det if n_det else 0
    tpB = r["cls"][("BREAKER", "BREAKER")]
    predB = sum(1 for e in ev if e["label"] == "BREAKER")
    r["breaker_recall"] = tpB / r["n_gt"]["BREAKER"] if r["n_gt"]["BREAKER"] else 0
    r["breaker_precision"] = tpB / predB if predB else 0
    r["surface_recall"] = r["cls"][("SURFACE", "SURFACE")] / r["n_gt"]["SURFACE"] if r["n_gt"]["SURFACE"] else 0
    r["pairs"] = pairs
    return r


def class_f1(gt, ev):
    pairs, used = match(gt, ev)
    f = []
    for c in ("BREAKER", "SURFACE"):
        tp = sum(1 for g, i in pairs if g["group"] == c and i is not None and GROUP[ev[i]["label"]] == c)
        ngt = sum(1 for g in gt if g["group"] == c)
        npred = sum(1 for e in ev if GROUP[e["label"]] == c)
        pr = tp / npred if npred else 0; rc = tp / ngt if ngt else 0
        f.append(2 * pr * rc / (pr + rc) if pr + rc else 0)
    return sum(f) / 2


def tune(seg, P0):
    import itertools
    t0 = calib_end(seg, P0); gt = gt_from_tags(seg["marks"], t0)
    best = None
    grid = list(itertools.product([3, 4, 5, 6, 7], [0, 2, 3, 4, 6], [4, 5, 6, 8, 12], [2.0, 3.0, 4.0]))
    for n, (ke, kl, kt, kh) in enumerate(grid):
        if n % 75 == 0:
            print(f"  tuning {n}/{len(grid)}", flush=True)
        Q = dict(P0, K_EVENT=ke, K_LOCAL=kl, K_HEAVE_TRIG=kt, K_HEAVE=kh)
        ev, _, _ = run_drive(seg, Q)
        for hd in range(200, 951, 50):
            for rh in (3, 4, 6, 99):
                R = dict(Q, HEAVE_DUR_BREAKER_MS=hd, ROUGH_BURST_HITS=rh)
                for e in ev:
                    e["label"] = ra.classify(e, R)
                f = class_f1(gt, ev)
                key = (round(f, 4), -len(ev))
                if best is None or key > best[0]:
                    best = (key, R)
    return best[1], best[0][0]


def rough_check(marks, summ, t_start):
    """Fraction of r-tags with a YELLOW/RED roughness row within [-5 s, +2 s]."""
    rt = [m for m in marks if m["label"] == "ROUGH" and m["t"] >= t_start]
    hit = 0
    for m in rt:
        if any(m["t"] - 5000 <= s["t"] <= m["t"] + 2000 and s["label"] in ("YELLOW", "RED") for s in summ):
            hit += 1
    import random
    rnd = random.Random(0); lo, hi = summ[0]["t"], summ[-1]["t"]; chance = 0
    for _ in range(2000):
        t = rnd.uniform(lo, hi)
        chance += any(t - 5000 <= s_["t"] <= t + 2000 and s_["label"] in ("YELLOW", "RED") for s_ in summ[::5])
    tiers = defaultdict(int)
    for s in summ:
        tiers[s["label"] if s["label"] in ("GREEN", "YELLOW", "RED") else "EVENT"] += 1
    n = sum(tiers.values())
    return dict(r_tags=len(rt), r_hit=hit, chance=chance / 2000, share={k: v / n for k, v in tiers.items()})


def cluster_events(all_ev, radius):
    """Greedy GPS clustering of events from several vehicles."""
    clusters = []
    for veh, e in all_ev:
        if not e["lat"]:
            continue
        p = (e["lat"], e["lon"])
        best = None
        for c in clusters:
            d = hav(p, c["c"])
            if d <= radius and (best is None or d < best[0]):
                best = (d, c)
        if best:
            c = best[1]; c["m"].append((veh, e))
            n = len(c["m"]); c["c"] = ((c["c"][0] * (n - 1) + p[0]) / n, (c["c"][1] * (n - 1) + p[1]) / n)
        else:
            clusters.append(dict(c=p, m=[(veh, e)]))
    return clusters


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--params", required=True)
    ap.add_argument("--static-from", default="0.11890,0.07448,0.01701,0.11187,0.02654",
                    help="mu_j,sig_j,sig_h,mu_r,sig_r of the tuning vehicle (C line of the tuning ride)")
    ap.add_argument("--radius", type=float, default=20.0)
    ap.add_argument("--out", default="results")
    ap.add_argument("--tune-on", help="log of the training vehicle: re-tune params on it, then freeze for all")
    a = ap.parse_args()
    P = dict(ra.DEFAULTS); P.update(json.load(open(a.params)))
    tuned_note = f"params from {a.params} (frozen)"
    if a.tune_on:
        tseg = max(ra.parse_log(a.tune_on), key=lambda s: len(s["raw"]))
        P, f = tune(tseg, P)
        C = tseg["ccal"][-1]
        a.static_from = ",".join(C[1:6])
        tuned_note = f"params tuned on {os.path.basename(a.tune_on)} (train macro-F1 {f:.2f}), frozen for all vehicles; static baseline = its calibration in absolute units"
        json.dump({k: P[k] for k in ("K_EVENT", "K_LOCAL", "K_HEAVE_TRIG", "K_HEAVE", "HEAVE_DUR_BREAKER_MS", "ROUGH_BURST_HITS")},
                  open(os.path.join(os.path.dirname(a.out) or ".", "best_3veh.json"), "w"), indent=2)
    mj, sj, sh, mr, sr = map(float, a.static_from.split(","))
    static_cfg = [(mj, sj), (0.0, sh), (mr, sr)]

    rows, all_ev, per = [], [], {}
    for path in a.logs:
        veh = os.path.basename(path).rsplit(".", 1)[0]
        seg = max(ra.parse_log(path), key=lambda s: len(s["raw"]))
        ev, summ, t0 = run_drive(seg, P)
        evS, _, _ = run_drive(seg, P, static_cfg)
        gt = gt_from_tags(seg["marks"], t0)
        sA, sS = score(gt, ev), score(gt, evS)
        sA["f1"], sS["f1"] = class_f1(gt, ev), class_f1(gt, evS)
        rc = rough_check(seg["marks"], summ, t0)
        C = [c for c in seg["ccal"]][-1]
        per[veh] = dict(seg=seg, ev=ev, gt=gt, sA=sA, t0=t0)
        all_ev += [(veh, e) for e in ev]
        rows.append((veh, sA, sS, rc, C, len(seg["raw"]) / 100.0, (seg["raw"][-1][0] - t0) / 1000))

    out = []
    def pr(s=""):
        print(s); out.append(s)
    pr("RoadSense v3 -- 3-vehicle evaluation (tags as ground truth)")
    pr(tuned_note)
    pr(f"params: {json.dumps({k: P[k] for k in ('K_EVENT','K_LOCAL','K_HEAVE_TRIG','K_HEAVE','HEAVE_DUR_BREAKER_MS','ROUGH_BURST_HITS')})}")
    pr("")
    pr("Per-vehicle calibration (8 s on smooth road) -> the vehicle-specific noise floor the thresholds scale with")
    pr(f"{'vehicle':<10}{'mu_jerk(g)':>11}{'sig_jerk':>10}{'sig_heave':>10}{'analysed s':>11}")
    for veh, sA, sS, rc, C, dur, an in rows:
        pr(f"{veh:<10}{float(C[1]):>11.3f}{float(C[2]):>10.3f}{float(C[3]):>10.4f}{an:>11.0f}")
    pr("")
    hdr = f"{'vehicle':<10}{'method':<10}{'GT B/S':>8}{'events':>7}{'det.recall':>11}{'B recall':>9}{'B prec':>8}{'S recall':>9}{'class acc':>10}{'untagged':>9}{'macroF1':>8}"
    pr("Detection and classification against in-vehicle tags")
    pr(hdr)
    tot = defaultdict(lambda: defaultdict(float))
    for veh, sA, sS, rc, C, dur, an in rows:
        for name, s in (("adaptive", sA), ("static", sS)):
            pr(f"{veh:<10}{name:<10}{s['n_gt']['BREAKER']:>4}/{s['n_gt']['SURFACE']:<3}{s['n_ev']:>7}{s['recall']:>11.2f}"
               f"{s['breaker_recall']:>9.2f}{s['breaker_precision']:>8.2f}{s['surface_recall']:>9.2f}{s['class_acc']:>10.2f}{s['untagged']:>9}{s['f1']:>8.2f}")
            t = tot[name]
            for k in ("BREAKER", "SURFACE"):
                t["gt" + k] += s["n_gt"][k]; t["det" + k] += s["det"][k]
            t["okB"] += s["cls"][("BREAKER", "BREAKER")]; t["okS"] += s["cls"][("SURFACE", "SURFACE")]
            t["predB"] += sum(1 for e in (per[veh]["ev"] if name == "adaptive" else []) if e["label"] == "BREAKER")
            t["ev"] += s["n_ev"]; t["untagged"] += s["untagged"]
    pr("")
    for name in ("adaptive", "static"):
        t = tot[name]; gt = t["gtBREAKER"] + t["gtSURFACE"]; det = t["detBREAKER"] + t["detSURFACE"]
        pr(f"ALL 3 vehicles, {name:<8}: tags {int(gt)}  detection recall {det/gt:.2f}  "
           f"breaker recall {t['okB']/t['gtBREAKER']:.2f}  surface recall {t['okS']/t['gtSURFACE']:.2f}  "
           f"class acc {(t['okB']+t['okS'])/det if det else 0:.2f}  events {int(t['ev'])}  untagged {int(t['untagged'])}")
    pr("")
    pr("Road-roughness tier vs r-tags (YELLOW/RED within -5..+2 s of the tag)")
    for veh, sA, sS, rc, C, dur, an in rows:
        sh_ = rc["share"]
        pr(f"  {veh:<10} r-tags {rc['r_tags']:>3}  flagged {rc['r_hit']:>3} ({rc['r_hit']/rc['r_tags'] if rc['r_tags'] else 0:.0%})   "
           f"chance level {rc['chance']:.0%}   time share GREEN {sh_.get('GREEN',0):.0%} YELLOW {sh_.get('YELLOW',0):.0%} RED {sh_.get('RED',0):.0%}")
    pr("")
    # cross-vehicle consistency
    vehs = [r[0] for r in rows]
    cl = cluster_events(all_ev, a.radius)
    cnt = defaultdict(int); cnt_b = defaultdict(int)
    for c in cl:
        n = len(set(v for v, _ in c["m"])); cnt[n] += 1
        if any(e["label"] == "BREAKER" for _, e in c["m"]):
            cnt_b[n] += 1
    pr(f"Cross-vehicle consistency: device events clustered by GPS (radius {a.radius:.0f} m)")
    pr(f"  locations: {len(cl)}   seen by 3/3 vehicles: {cnt[3]}   2/3: {cnt[2]}   1/3: {cnt[1]}")
    pr(f"  locations containing a BREAKER label: 3/3 {cnt_b[3]}, 2/3 {cnt_b[2]}, 1/3 {cnt_b[1]}")
    # label agreement inside multi-vehicle clusters
    agree = tot_m = 0
    for c in cl:
        vs = {}
        for v, e in c["m"]:
            vs.setdefault(v, GROUP[e["label"]])
        if len(vs) >= 2:
            tot_m += 1; agree += len(set(vs.values())) == 1
    pr(f"  class agreement among vehicles at shared locations: {agree}/{tot_m}")
    # GT breaker locations: which vehicles detected each
    pr("")
    pr("Per tagged BREAKER (clustered across vehicles by tag GPS, 40 m): detected+labelled BREAKER by")
    bt = []
    for v in vehs:
        for g, i in per[v]["sA"]["pairs"]:
            if g["group"] == "BREAKER" and g["lat"]:
                bt.append((v, g, i))
    bcl = []
    for v, g, i in bt:
        p = (g["lat"], g["lon"])
        for c in bcl:
            if hav(p, c["c"]) < 40:
                c["m"].append((v, g, i)); break
        else:
            bcl.append(dict(c=p, m=[(v, g, i)]))
    for k, c in enumerate(sorted(bcl, key=lambda c: min(x[1]["t"] for x in c["m"]))):
        res = []
        for v in vehs:
            ms = [x for x in c["m"] if x[0] == v]
            if not ms:
                res.append(f"{v}:not tagged"); continue
            v_, g, i = ms[0]
            lab = per[v]["ev"][i]["label"] if i is not None else "MISSED"
            res.append(f"{v}:{lab}")
        pr(f"  breaker {k+1} ({c['c'][0]:.5f},{c['c'][1]:.5f}): " + ", ".join(res))

    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, "summary.txt"), "w").write("\n".join(out) + "\n")
    with open(os.path.join(a.out, "consensus_locations.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["location_id", "lat", "lon", "n_vehicles"] + [f"{v}_label" for v in vehs])
        for k, c in enumerate(sorted(cl, key=lambda c: -len(set(v for v, _ in c["m"]))), 1):
            labs = {}
            for v, e in c["m"]:
                labs.setdefault(v, e["label"])
            w.writerow([k, f"{c['c'][0]:.6f}", f"{c['c'][1]:.6f}", len(labs)] + [labs.get(v, "") for v in vehs])
    print(f"\nwrote {a.out}/summary.txt and {a.out}/consensus_locations.csv")


if __name__ == "__main__":
    main()
