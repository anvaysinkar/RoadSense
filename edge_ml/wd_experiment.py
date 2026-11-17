#!/usr/bin/env python3
"""
wd_experiment.py -- Steps 2-5 of the whole-drive plan.

Every 1.4 s window of every drive gets P(NONE), P(BREAKER), P(SURFACE).
Windows with anomaly score 1 - P(NONE) >= thr are merged into events
(consecutive windows, gap <= 0.4 s); an event's time is its strongest window's
anchor and its class the larger of summed P(BREAKER) / P(SURFACE).
Events are then scored exactly like the paper (eval_3vehicles.score: tag matched
to an event starting in [tag - 5 s, tag + 1.5 s]), pooled over the three
held-out vehicles (leave-one-vehicle-out).

Baselines on the same scoring:
  * the firmware's rule detector at K_LOCAL = 6 (paper), 4 and 2
  * one-feature threshold: score = jerk peak z-score, class by heave duration
"""
import json, os
import numpy as np
import roadsense_analyze as ra
import eval_3vehicles as ev3
import ml_experiment as M
import wd_dataset as W
from sklearn.ensemble import RandomForestClassifier, HistGradientBoostingClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline
from sklearn.utils.class_weight import compute_sample_weight

VEH = W.VEH
OUT = W.OUT
THRS = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95]
SEEDS = [0, 1, 2]


def make(name, seed):
    if name == "forest":
        return RandomForestClassifier(n_estimators=150, max_depth=8, min_samples_leaf=3,
                                      class_weight="balanced_subsample", n_jobs=-1, random_state=seed)
    if name == "boosting":
        return HistGradientBoostingClassifier(max_depth=4, learning_rate=0.08, max_iter=200,
                                              l2_regularization=1.0, random_state=seed)
    return make_pipeline(StandardScaler(), LogisticRegression(C=0.3, class_weight="balanced", max_iter=5000))


def fit(name, seed, X, y):
    m = make(name, seed)
    if name == "boosting":
        m.fit(X, y, sample_weight=compute_sample_weight("balanced", y))
    else:
        m.fit(X, y)
    return m


def events_from_scores(wins, a, pB, pS, thr):
    """merge windows with anomaly score >= thr into events"""
    evs, cur = [], None
    for w, ai, b, s in zip(wins, a, pB, pS):
        if ai < thr:
            continue
        t = w["t_anchor"]
        if cur is not None and t - cur["_last"] <= 400:
            cur["_last"] = t; cur["_B"] += b; cur["_S"] += s
            if ai > cur["_a"]: cur["_a"], cur["t"], cur["lat"], cur["lon"] = ai, t, w["lat"], w["lon"]
        else:
            if cur is not None: evs.append(cur)
            cur = dict(t=t, lat=w["lat"], lon=w["lon"], _last=t, _B=b, _S=s, _a=ai)
    if cur is not None: evs.append(cur)
    for e in evs:
        e["label"] = "BREAKER" if e["_B"] >= e["_S"] else "POTHOLE"
        e["nHits"] = 0
    return evs


def score_drive(d, evs):
    return M.with_predB(M.metrics(d["gt"], evs), evs)


def pooled_curve(data, probs, thrs):
    pts = []
    for thr in thrs:
        rows = []
        for v in VEH:
            pr, cl = probs[v]
            iN, iB, iS = cl.index("NONE"), cl.index("BREAKER"), cl.index("SURFACE")
            evs = events_from_scores(data[v]["wins"], 1 - pr[:, iN], pr[:, iB], pr[:, iS], thr)
            rows.append(score_drive(data[v], evs))
        m = M.pooled(rows); m["thr"] = thr; m["per_vehicle"] = rows; pts.append(m)
    return pts


def lovo_probs(data, name, fs, seed):
    probs = {}
    for test in VEH:
        tr = [v for v in VEH if v != test]
        Xtr = np.vstack([W.X(data[v]["wins"], fs)[data[v]["y"] != "IGNORE"] for v in tr])
        ytr = np.concatenate([data[v]["y"][data[v]["y"] != "IGNORE"] for v in tr])
        m = fit(name, seed, Xtr, ytr)
        probs[test] = (m.predict_proba(W.X(data[test]["wins"], fs)), list(m.classes_))
    return probs


def mean_pts(list_of_pts):
    out = []
    for group in zip(*list_of_pts):
        m = {k: float(np.mean([g[k] for g in group])) for k in group[0] if k not in ("per_vehicle",)}
        m["per_vehicle"] = group[0]["per_vehicle"]
        out.append(m)
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    data = W.build()
    lines = []
    def pr(s=""):
        print(s); lines.append(s)
    tags = sum(len(data[v]["gt"]) for v in VEH)
    pr("Whole-drive ML, leave-one-vehicle-out (train on 2 vehicles, test on the 3rd), events scored like the paper")
    pr("windows per drive: " + ", ".join(f"{v} {len(data[v]['wins'])}" for v in VEH) + f";  tags {tags} (18 breakers)")
    pr("training windows: " + ", ".join(f"{v} " + "/".join(f"{k} {int((data[v]['y']==k).sum())}" for k in ('NONE','SURFACE','BREAKER')) for v in VEH))
    pr("")
    H = f"{'':<34}{'events':>6}{'untagged':>9}{'det.rec':>8}{'B.rec':>7}{'B.prec':>7}{'S.rec':>7}{'cls.acc':>8}"
    curves = {}

    # --- baselines: rule detector ---
    rule_pts = []
    for kl in (6, 5, 4, 3, 2):
        rows = []
        for v in VEH:
            seg = data[v]["seg"]
            evs, _ = ra.replay(seg["raw"], seg["kcal"], dict(W.P, K_LOCAL=float(kl)), ra.make_gps_lookup(seg))
            evs = [e for e in evs if e["t"] >= data[v]["t_an"]]
            rows.append(score_drive(data[v], evs))
        m = M.pooled(rows); m["k_L"] = kl; m["per_vehicle"] = rows; rule_pts.append(m)
    curves["rules"] = rule_pts
    # --- baseline: one-feature threshold (jerk peak z), class from heave duration ---
    one = []
    for T in (4, 5, 6, 7, 8, 10, 12, 15):
        rows = []
        for v in VEH:
            wins = data[v]["wins"]
            a = np.array([1.0 if w["jpk_z"] >= T else 0.0 for w in wins])
            pB = np.array([1.0 if w["dur"] >= 450 else 0.0 for w in wins]); pS = 1 - pB
            rows.append(score_drive(data[v], events_from_scores(wins, a, pB, pS, 0.5)))
        m = M.pooled(rows); m["T"] = T; m["per_vehicle"] = rows; one.append(m)
    curves["one_feature"] = one

    # --- models ---
    models = {}
    for name in ("forest", "boosting", "logreg"):
        for fs in ("norm", "raw"):
            seeds = SEEDS if name != "logreg" else [0]
            pts = mean_pts([pooled_curve(data, lovo_probs(data, name, fs, s), THRS) for s in seeds])
            models[f"{name}_{fs}"] = pts
            print(f"  done {name} {fs}", flush=True)
    curves.update(models)

    pr("Operating points (pooled over the 3 held-out drives)")
    pr(H)
    for m in rule_pts:
        pr(f"  rules  K_LOCAL={m['k_L']:<22}" + M.fmt(m))
    for m in one:
        pr(f"  one feature  jerk z >= {m['T']:<12}" + M.fmt(m))
    for key, pts in models.items():
        pr(f"-- {key}")
        for m in pts:
            pr(f"  {key:<16} thr {m["thr"]:<12.2f}" + M.fmt({**m, 'events': round(m['events']), 'untagged': round(m['untagged'])}))
    pr("")

    # --- headline: match the paper rule's false-alarm budget and the k_L=4 budget ---
    def at_budget(pts, budget):
        ok = [p for p in pts if p["untagged"] <= budget]
        return max(ok, key=lambda p: p["det_recall"]) if ok else None
    pr("Best tag recall at a fixed false-alarm budget (untagged events, 3 drives pooled)")
    pr(H)
    for budget, ref in ((40, "≈ rules K_LOCAL=6"), (135, "≈ rules K_LOCAL=4")):
        pr(f"budget ≤ {budget} untagged ({ref})")
        for key, pts in [("rules", rule_pts), ("one_feature", one)] + list(models.items()):
            p = at_budget(pts, budget)
            if p:
                pr(f"  {key:<32}" + M.fmt({**p, 'events': round(p['events']), 'untagged': round(p['untagged'])}))
    pr("")
    # feature importance, norm forest on all drives
    Xa = np.vstack([W.X(data[v]["wins"], "norm")[data[v]["y"] != "IGNORE"] for v in VEH])
    ya = np.concatenate([data[v]["y"][data[v]["y"] != "IGNORE"] for v in VEH])
    imp = np.mean([fit("forest", s, Xa, ya).feature_importances_ for s in SEEDS], axis=0)
    pr("Feature importance (norm forest, all drives): " + ", ".join(f"{W.FEATURES['norm'][i]} {imp[i]:.3f}" for i in np.argsort(-imp)))
    open(f"{OUT}/wd_summary.txt", "w").write("\n".join(lines) + "\n")
    strip = lambda pts: [{k: v for k, v in p.items() if k != "per_vehicle"} | {"per_vehicle": [{k2: v2 for k2, v2 in r.items() if not k2.startswith("_")} for r in p["per_vehicle"]]} for p in pts]
    json.dump({k: strip(v) for k, v in curves.items()}, open(f"{OUT}/wd_curves.json", "w"), indent=1, default=float)
    print(f"wrote {OUT}/wd_summary.txt and wd_curves.json")


if __name__ == "__main__":
    main()
