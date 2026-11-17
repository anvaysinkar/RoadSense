#!/usr/bin/env python3
"""
ml_experiment.py -- Option 1 (ML filter on a loose trigger) and
Option 2 (RAW vs ABC-normalised features), leave-one-vehicle-out.

Pipeline under test
  loose trigger (K_LOCAL = 2, everything else frozen from best_3veh.json)
    -> per-event features (rs_features.py)
    -> classifier: NONE / BREAKER / SURFACE
    -> events predicted NONE are dropped, the rest keep the predicted class
Evaluation = the paper's protocol (eval_3vehicles.score): tags as ground truth,
match window [tag-5 s, tag+1.5 s], p/r tags = SURFACE.
Baselines: the rule-based detector at K_LOCAL = 6 (paper) and K_LOCAL = 2.

Training labels: each candidate matched to a tag gets that tag's group.
Unmatched candidates far from every tag are NONE. Unmatched candidates close
to a tag (within [tag-8 s, tag+3 s]) are AMBIGUOUS (second hit of the same
anomaly, or a neighbour the tagger did not press for) and are left out of
TRAINING only; the test vehicle is always scored on all of its candidates.
Untagged real undulations still end up as NONE, so precision is a lower bound.
"""
import json, os, sys, csv
import numpy as np
from collections import defaultdict
import roadsense_analyze as ra
import eval_3vehicles as ev3
import rs_features as rf
from sklearn.ensemble import RandomForestClassifier
from sklearn.tree import DecisionTreeClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline

VEH = ["hatchback", "sedan", "suv"]
CLASSES = ["NONE", "BREAKER", "SURFACE"]
OUT = "results"
os.makedirs(OUT, exist_ok=True)
P = dict(ra.DEFAULTS); P.update(json.load(open("best_3veh.json")))
LOOSE = dict(P, K_LOCAL=2.0)
AMB_BEFORE, AMB_AFTER = 8000, 3000
THR = 0.5          # keep an event if P(NONE) < THR (fixed before testing)


def load():
    data = {}
    for v in VEH:
        seg = max(ra.parse_log(f"../datasets/raw_drives/{v}.txt"), key=lambda s: len(s["raw"]))
        t0 = ev3.calib_end(seg, P)
        gt = ev3.gt_from_tags(seg["marks"], t0)
        cand = [e for e in rf.run(seg, LOOSE) if e["t"] >= t0]
        rule6 = [e for e in rf.run(seg, P) if e["t"] >= t0]
        pairs, used = ev3.match(gt, cand)
        y = ["NONE"] * len(cand)
        for g, i in pairs:
            if i is not None:
                y[i] = g["group"]
        amb = np.array([y[i] == "NONE" and any(g["t"] - AMB_BEFORE <= e["t"] <= g["t"] + AMB_AFTER for g in gt)
                        for i, e in enumerate(cand)])
        data[v] = dict(seg=seg, gt=gt, cand=cand, y=np.array(y), amb=amb, rule6=rule6)
    return data


def X_of(events, fs):
    return np.array([[e["feat"][k] for k in rf.FEATURES[fs]] for e in events], float)


MODELS = {
    "tree_d4": lambda s: DecisionTreeClassifier(max_depth=4, min_samples_leaf=3, class_weight="balanced", random_state=s),
    "forest": lambda s: RandomForestClassifier(n_estimators=40, max_depth=6, min_samples_leaf=2,
                                               class_weight="balanced_subsample", random_state=s),
    "logreg": lambda s: make_pipeline(StandardScaler(), LogisticRegression(C=0.5, class_weight="balanced", max_iter=3000)),
}
SEEDS = {"tree_d4": [0], "forest": [0, 1, 2, 3, 4], "logreg": [0]}


def relabel(cand, keep_lab):
    out = []
    for e, lab in zip(cand, keep_lab):
        if lab == "NONE":
            continue
        d = dict(e); d["label"] = "BREAKER" if lab == "BREAKER" else ("ROUGHBURST" if e["nHits"] >= 3 else "POTHOLE")
        out.append(d)
    return out


def metrics(gt, events):
    s = ev3.score(gt, events)
    return dict(events=s["n_ev"], untagged=s["untagged"], det_recall=s["recall"],
                B_rec=s["breaker_recall"], B_prec=s["breaker_precision"], S_rec=s["surface_recall"],
                class_acc=s["class_acc"], macroF1=ev3.class_f1(gt, events),
                _cls=dict(s["cls"]), _ngt=s["n_gt"], _det=s["det"])


def pooled(rows):
    """Pool counts over the 3 held-out vehicles (micro), like the 'All' row of Table II."""
    ngt = defaultdict(int); det = defaultdict(int); cls = defaultdict(int); ev = un = predB = 0
    for r in rows:
        for k, v in r["_ngt"].items(): ngt[k] += v
        for k, v in r["_det"].items(): det[k] += v
        for k, v in r["_cls"].items(): cls[k] += v
        ev += r["events"]; un += r["untagged"]; predB += r["_predB"]
    okB, okS = cls[("BREAKER", "BREAKER")], cls[("SURFACE", "SURFACE")]
    d = sum(det.values())
    return dict(events=ev, untagged=un, det_recall=d / sum(ngt.values()),
                B_rec=okB / ngt["BREAKER"], B_prec=okB / predB if predB else 0,
                S_rec=okS / ngt["SURFACE"], class_acc=(okB + okS) / d if d else 0)


def with_predB(m, events):
    m["_predB"] = sum(1 for e in events if e["label"] == "BREAKER"); return m


def lovo(data, fs, model, seed, thr=None):
    """Train on two vehicles, test on the third. thr: keep an event if P(NONE) < thr
    (None = plain argmax)."""
    res, probs = {}, {}
    for test in VEH:
        tr = [v for v in VEH if v != test]
        X = np.vstack([X_of(data[v]["cand"], fs)[~data[v]["amb"]] for v in tr])
        y = np.concatenate([data[v]["y"][~data[v]["amb"]] for v in tr])
        clf = MODELS[model](seed).fit(X, y)
        Xt = X_of(data[test]["cand"], fs)
        pr = clf.predict_proba(Xt); cl = list(clf.classes_)
        probs[test] = (pr, cl)
        res[test] = decide(data[test], pr, cl, thr)
    return res, probs


def decide(d, pr, cl, thr):
    iN, iB, iS = cl.index("NONE"), cl.index("BREAKER"), cl.index("SURFACE")
    lab = []
    for p in pr:
        if thr is None:
            lab.append(cl[int(np.argmax(p))])
        else:
            lab.append("NONE" if p[iN] >= thr else ("BREAKER" if p[iB] >= p[iS] else "SURFACE"))
    evs = relabel(d["cand"], lab)
    return with_predB(metrics(d["gt"], evs), evs)


def fmt(m):
    return (f"{m['events']:>6}{m['untagged']:>9}{m['det_recall']:>8.2f}{m['B_rec']:>7.2f}{m['B_prec']:>7.2f}"
            f"{m['S_rec']:>7.2f}{m['class_acc']:>8.2f}")


HDR = f"{'':<28}{'events':>6}{'untagged':>9}{'det.rec':>8}{'B.rec':>7}{'B.prec':>7}{'S.rec':>7}{'cls.acc':>8}"


def main():
    data = load()
    lines = []
    def pr(s=""):
        print(s); lines.append(s)
    pr("RoadSense ML -- leave-one-vehicle-out (train on 2 vehicles, test on the 3rd)")
    pr(f"candidates from loose trigger K_LOCAL=2: " + ", ".join(
        f"{v} {len(data[v]['cand'])} ({sum(data[v]['y']=='BREAKER')} B, {sum(data[v]['y']=='SURFACE')} S, {sum(data[v]['y']=='NONE')} none)"
        for v in VEH))
    pr("tags per drive (B/S): " + ", ".join(f"{v} {sum(g['group']=='BREAKER' for g in data[v]['gt'])}/"
                                            f"{sum(g['group']=='SURFACE' for g in data[v]['gt'])}" for v in VEH))
    pr("")

    # baselines
    base = {}
    for name, key in (("rules K_LOCAL=6 (paper)", "rule6"), ("rules K_LOCAL=2", "cand")):
        rows = []
        for v in VEH:
            evs = data[v][key]
            rows.append(with_predB(metrics(data[v]["gt"], evs), evs))
        base[name] = rows

    # ML, argmax decision
    table = {}
    for fs in ("raw", "norm"):
        for model in MODELS:
            per_seed = []
            for s in SEEDS[model]:
                res, _ = lovo(data, fs, model, s, THR)
                per_seed.append(res)
            table[(fs, model)] = per_seed

    pr("training excludes ambiguous near-tag candidates: " + ", ".join(f"{v} {int(data[v]['amb'].sum())}" for v in VEH))
    pr(f"ML decision: keep event if P(NONE) < {THR}; class = more likely of BREAKER / SURFACE")
    pr("")
    pr("Per held-out vehicle")
    pr(HDR)
    for v in VEH:
        pr(f"-- test vehicle: {v}")
        for name, rows in base.items():
            pr(f"  {name:<26}" + fmt(rows[VEH.index(v)]))
        for (fs, model), per_seed in table.items():
            m = per_seed[0][v]
            pr(f"  ML {model:<8} {fs:<5} feats   " + fmt(m) + (f"   (seed 0 of {len(per_seed)})" if len(per_seed) > 1 else ""))
    pr("")
    pr("Pooled over the 3 held-out vehicles (each vehicle predicted by a model that never saw it)")
    pr(HDR)
    summary = {}
    for name, rows in base.items():
        m = pooled(rows); summary[name] = m
        pr(f"  {name:<26}" + fmt(m))
    for (fs, model), per_seed in table.items():
        ms = [pooled([r[v] for v in VEH]) for r in per_seed]
        m = {k: float(np.mean([x[k] for x in ms])) for k in ms[0]}
        summary[f"ML {model} {fs}"] = m
        spread = ""
        if len(ms) > 1:
            spread = f"   seeds: S.rec {min(x['S_rec'] for x in ms):.2f}-{max(x['S_rec'] for x in ms):.2f}, untagged {min(x['untagged'] for x in ms)}-{max(x['untagged'] for x in ms)}"
        pr(f"  ML {model:<8} {fs:<5} feats   " + fmt({**m, 'events': round(m['events']), 'untagged': round(m['untagged'])}) + spread)
    pr("")

    # operating curves: forest, raw vs norm, threshold on P(NONE)
    curves = {}
    for fs in ("raw", "norm"):
        pts = []
        probs = [lovo(data, fs, "forest", s)[1] for s in SEEDS["forest"]]
        for thr in [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]:
            ms = [pooled([decide(data[v], *pb[v], thr) for v in VEH]) for pb in probs]
            m = {k: float(np.mean([x[k] for x in ms])) for k in ms[0]}; m["thr"] = thr; pts.append(m)
        curves[fs] = pts
    rule_pts = []
    for kl in (6, 5, 4, 3, 2):
        rows = []
        for v in VEH:
            seg = data[v]["seg"]; t0 = ev3.calib_end(seg, P)
            evs = [e for e in rf.run(seg, dict(P, K_LOCAL=float(kl))) if e["t"] >= t0]
            rows.append(with_predB(metrics(data[v]["gt"], evs), evs))
        m = pooled(rows); m["k_L"] = kl; rule_pts.append(m)
    pr("Operating points (pooled LOVO). Rules: sweep K_LOCAL.  ML forest: sweep the P(NONE) cut-off (mean of 5 seeds).")
    pr(f"{'':<28}{'events':>6}{'untagged':>9}{'det.rec':>8}{'B.rec':>7}{'B.prec':>7}{'S.rec':>7}{'cls.acc':>8}")
    for m in rule_pts:
        pr(f"  rules K_LOCAL={m['k_L']:<12}" + fmt(m))
    for fs in ("raw", "norm"):
        for m in curves[fs]:
            pr(f"  forest {fs:<4} P(NONE)<{m['thr']:<8}" + fmt({**m, 'events': round(m['events']), 'untagged': round(m['untagged'])}))
    pr("")

    # feature importance of the deployable model (norm forest trained on all 3 vehicles)
    X = np.vstack([X_of(data[v]["cand"], "norm")[~data[v]["amb"]] for v in VEH])
    y = np.concatenate([data[v]["y"][~data[v]["amb"]] for v in VEH])
    imp = np.mean([MODELS["forest"](s).fit(X, y).feature_importances_ for s in SEEDS["forest"]], axis=0)
    order = np.argsort(-imp)
    pr("Feature importance (norm forest, all vehicles, mean of 5 seeds):")
    pr("  " + ", ".join(f"{rf.FEATURES['norm'][i]} {imp[i]:.3f}" for i in order))

    open(os.path.join(OUT, "ml_summary.txt"), "w").write("\n".join(lines) + "\n")
    json.dump(dict(rule_pts=rule_pts, curves=curves, summary=summary), open(os.path.join(OUT, "ml_curves.json"), "w"),
              indent=1, default=float)
    # dataset export
    with open(os.path.join(OUT, "ml_dataset.csv"), "w", newline="") as f:
        w = csv.writer(f)
        cols = rf.FEATURES["raw"] + [c for c in rf.FEATURES["norm"] if c not in rf.FEATURES["raw"]]
        w.writerow(["vehicle", "t_ms", "lat", "lon", "rule_label", "y", "ambiguous"] + cols)
        for v in VEH:
            for e, yy, am in zip(data[v]["cand"], data[v]["y"], data[v]["amb"]):
                w.writerow([v, e["t"], f"{e['lat']:.6f}", f"{e['lon']:.6f}", e["label"], yy, int(am)] + [f"{e['feat'][c]:.6g}" for c in cols])
    print(f"\nwrote {OUT}/ml_summary.txt, ml_curves.json, ml_dataset.csv")


if __name__ == "__main__":
    main()
