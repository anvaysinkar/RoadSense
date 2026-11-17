#!/usr/bin/env python3
"""Compare the firmware simulation (tests/esp32_firmware_sim, the real .ino fed with the logs)
against the Python pipeline + exported forest. Same events, same labels expected."""
import numpy as np, ml_experiment as M, export_model as X
from sklearn.ensemble import RandomForestClassifier
data = M.load()
Xtr = np.vstack([M.X_of(data[v]["cand"], X.FS)[~data[v]["amb"]] for v in M.VEH])
ytr = np.concatenate([data[v]["y"][~data[v]["amb"]] for v in M.VEH])
clf = RandomForestClassifier(n_estimators=X.N_TREES, max_depth=X.DEPTH, min_samples_leaf=X.LEAF,
                             class_weight="balanced_subsample", random_state=X.SEED).fit(Xtr, ytr)
cl = list(clf.classes_)
tot = same_t = same_lab = 0; maxdp = 0
for v in M.VEH:
    fw = {}
    for ln in open(f"results/fw_{v}.txt"):
        p = ln.strip().split(",")
        if p[0] == "E": fw.setdefault(int(p[1]), {})["label"] = p[4]
        if p[0] == "Q": fw.setdefault(int(p[1]), {})["p"] = [float(p[2]), float(p[3]), float(p[4])]
    cand = data[v]["cand"]
    pr = clf.predict_proba(M.X_of(cand, X.FS))
    n_ok = n_t = 0
    for e, pp in zip(cand, pr):
        tot += 1
        if e["t"] not in fw: continue
        n_t += 1
        pN, pB, pS = pp[cl.index("NONE")], pp[cl.index("BREAKER")], pp[cl.index("SURFACE")]
        lab = "NONE" if pN >= 0.5 else ("BREAKER" if pB >= pS else ("ROUGHBURST" if e["nHits"] >= 3 else "POTHOLE"))
        n_ok += fw[e["t"]]["label"] == lab
        maxdp = max(maxdp, max(abs(a - b) for a, b in zip(fw[e["t"]]["p"], [pB, pN, pS])))
    same_t += n_t; same_lab += n_ok
    print(f"{v:<10} python events after final calibration {len(cand)}, found in firmware {n_t}, identical label {n_ok}")
print(f"ALL: {same_lab}/{tot} identical; max probability difference {maxdp:.3f} (firmware prints 3 decimals)")
