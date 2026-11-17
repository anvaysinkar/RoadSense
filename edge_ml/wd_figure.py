import json, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
c = json.load(open("results/wholedrive/wd_curves.json"))
S = dict(bg="#fcfcfb", ink="#0b0b0b", muted="#52514e", grid="#e6e5e1", norm="#2a78d6", raw="#eb6834", rules="#1baf7a")
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": S["muted"], "xtick.color": S["muted"], "ytick.color": S["muted"], "axes.labelcolor": S["ink"]})
fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.9), facecolor=S["bg"], gridspec_kw=dict(width_ratios=[1.15, 1]))
ax = axs[0]; ax.set_facecolor(S["bg"])
for key, col, name in (("rules", S["rules"], "rule detector (k_L 6 → 2)"), ("forest_norm", S["norm"], "forest, ABC-normalised features"), ("forest_raw", S["raw"], "forest, raw-unit features")):
    pts = sorted(c[key], key=lambda p: p["untagged"])
    ax.plot([p["untagged"] for p in pts], [p["det_recall"] for p in pts], color=col, lw=2, marker="o", ms=5, mec=S["bg"], mew=1.4, label=name)
ax.set_xlabel("untagged events (3 test drives, pooled)"); ax.set_ylabel("share of tagged anomalies found"); ax.set_ylim(0, 1.02)
ax.grid(color=S["grid"]); ax.set_axisbelow(True); [ax.spines[s].set_visible(False) for s in ("top", "right")]
ax.legend(frameon=False, loc="lower right", fontsize=8)
ax.set_title("Recall vs. false alarms", loc="left", fontsize=10, color=S["ink"])
ax = axs[1]; ax.set_facecolor(S["bg"])
pick = [("rules", lambda p: p["k_L"] == 4, S["rules"], "rules k_L=4"), ("forest_norm", lambda p: abs(p["thr"] - 0.3) < 1e-6, S["norm"], "forest, normalised"), ("forest_raw", lambda p: abs(p["thr"] - 0.5) < 1e-6, S["raw"], "forest, raw")]
cars = ["Hatchback", "Sedan", "SUV"]; tags = [30, 40, 34]; w = 0.26
for i, (key, sel, col, name) in enumerate(pick):
    p = [q for q in c[key] if sel(q)][0]
    vals = [r["events"] for r in p["per_vehicle"]]
    xs = [k + (i - 1) * w for k in range(3)]
    ax.bar(xs, vals, width=w - 0.03, color=col, label=name)
    for x, v in zip(xs, vals): ax.text(x, v + 2, str(v), ha="center", fontsize=8, color=S["ink"])
for k, t in enumerate(tags):
    ax.plot([k - 0.42, k + 0.42], [t, t], color=S["ink"], lw=1.2, ls="--")
ax.text(2.45, tags[2], "tagged", fontsize=7.5, color=S["muted"], va="center")
ax.set_xticks(range(3)); ax.set_xticklabels(cars); ax.set_ylabel("events per drive")
ax.grid(axis="y", color=S["grid"]); ax.set_axisbelow(True); [ax.spines[s].set_visible(False) for s in ("top", "right")]
ax.legend(frameon=False, fontsize=8, loc="upper left")
ax.set_title("Same false-alarm budget (~130), per test car", loc="left", fontsize=10, color=S["ink"])
fig.text(0.01, 0.01, "Leave-one-vehicle-out: each drive is scored by a model trained only on the other two. Dashed: tagged anomalies in that drive.", fontsize=7.5, color=S["muted"])
fig.tight_layout(rect=(0, 0.05, 1, 1)); fig.savefig("results/wholedrive/wd_results.png", dpi=170, facecolor=S["bg"]); print("ok")
