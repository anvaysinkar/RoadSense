import json, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
d = json.load(open("results/ml_curves.json"))
S = dict(surface="#fcfcfb", text="#0b0b0b", muted="#52514e", grid="#e6e5e1",
         norm="#2a78d6", raw="#eb6834", rules="#1baf7a")
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": S["muted"],
                     "axes.labelcolor": S["text"], "xtick.color": S["muted"], "ytick.color": S["muted"]})
fig, axs = plt.subplots(1, 2, figsize=(9.2, 3.6), facecolor=S["surface"])
series = [("rules (sweep $k_L$ 6→2)", d["rule_pts"], S["rules"], "k_L"),
          ("ML forest, ABC-normalised features", d["curves"]["norm"], S["norm"], "thr"),
          ("ML forest, raw-unit features", d["curves"]["raw"], S["raw"], "thr")]
for ax, key, yl in ((axs[0], "det_recall", "share of tags detected"), (axs[1], "B_prec", "speed-breaker precision")):
    ax.set_facecolor(S["surface"])
    for name, pts, c, lab in series:
        x = [p["untagged"] for p in pts]; y = [p[key] for p in pts]
        ax.plot(x, y, color=c, lw=2, marker="o", ms=5, mec=S["surface"], mew=1.5, label=name, zorder=3)
        for p in pts:
            if (lab == "thr" and abs(p["thr"] - 0.5) < 1e-9) or (lab == "k_L" and p["k_L"] in (6,)):
                ax.plot(p["untagged"], p[key], marker="o", ms=9, mfc="none", mec=c, mew=1.6, zorder=4)
    ax.grid(color=S["grid"], lw=0.8); ax.set_axisbelow(True)
    for sp in ("top", "right"): ax.spines[sp].set_visible(False)
    ax.set_xlabel("untagged events (3 drives, pooled)"); ax.set_ylabel(yl); ax.set_ylim(0, 1.02)
axs[0].legend(frameon=False, loc="lower right", fontsize=8, labelcolor=S["text"])
axs[0].set_title("Recall vs. false-alarm cost", loc="left", color=S["text"], fontsize=10)
axs[1].set_title("Breaker precision", loc="left", color=S["text"], fontsize=10)
fig.text(0.01, 0.01, "Leave-one-vehicle-out: each drive scored by a model trained on the other two. Rings: paper rule ($k_L$=6) and ML default cut-off P(NONE)<0.5.",
         fontsize=7.5, color=S["muted"])
fig.tight_layout(rect=(0, 0.05, 1, 1))
fig.savefig("results/ml_operating_points.png", dpi=170, facecolor=S["surface"])
print("ok")
