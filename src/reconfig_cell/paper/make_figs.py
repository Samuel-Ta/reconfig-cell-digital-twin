#!/usr/bin/env python3
"""Regenerate the paper's two data figures at print quality (500 dpi, readable fonts).

    python3 make_figs.py

Reads the committed CSVs; writes figs/gate_scatter.png + figs/phase3_gen.png.
Reviewer request: labels/legends/axis titles readable, >=500 dpi.
"""
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))
FIGS = os.path.join(HERE, "figs")
DPI = 500

plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 9.5, "axes.labelsize": 9.5,
    "xtick.labelsize": 8.5, "ytick.labelsize": 8.5, "legend.fontsize": 8.5,
})


def rows(path):
    with open(os.path.join(REPO, path)) as f:
        return list(csv.DictReader(f))


def pearson(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    dx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    dy = math.sqrt(sum((y - my) ** 2 for y in ys))
    return num / (dx * dy)


def fitline(ax, xs, ys, **kw):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    a = my - b * mx
    xr = [min(xs), max(xs)]
    ax.plot(xr, [a + b * x for x in xr], **kw)


# ── Fig. 2: the validation gate scatter (val_hb) ─────────────────────────────────────
def gate_scatter():
    data = rows("val_hb/motion.csv")
    s = [float(r["surrogate"]) for r in data]
    mn = [float(r["motion_min"]) for r in data]
    mean = [float(r["motion_mean"]) for r in data]
    std = [float(r["motion_std"]) for r in data]
    lab = [r["label"].replace("_n3", "") for r in data]

    fig, ax = plt.subplots(figsize=(3.5, 2.7))
    ax.errorbar(s, mean, yerr=std, fmt="o", color="#bbbbbb", ms=5, capsize=3,
                lw=1, label="mean $\\pm$ std (10 plans)", zorder=1)
    ax.scatter(s, mn, color="#d95f02", s=38, zorder=3,
               label="min (converged-optimal)")
    fitline(ax, s, mn, color="#d95f02", lw=1.4, alpha=0.8, zorder=2)
    for x, y, t in zip(s, mn, lab):
        ax.annotate(t, (x, y), fontsize=7.5, xytext=(4, -9), textcoords="offset points")
    ax.set_xlabel("surrogate $T_1$ (canonical joint travel, rad)")
    ax.set_ylabel("real RRT$^*$ motion $T_2$ (s)")
    ax.set_title(f"$r={pearson(s, mn):.3f}$, $\\rho=0.943$ ($n={len(s)}$, min estimator)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", framealpha=0.9)
    fig.tight_layout()
    out = os.path.join(FIGS, "gate_scatter.png")
    fig.savefig(out, dpi=DPI)
    print("wrote", out)


# ── Fig. 3: generalization at scale (phase 3) ────────────────────────────────────────
def phase3_gen():
    ur5 = rows("phase3_ur5/opt_summary_n3.csv")
    ur10 = rows("phase3_ur10/opt_summary_n3.csv")
    v5 = rows("phase3_ur5_val/motion.csv")
    v10 = rows("phase3_ur10_val/motion.csv")
    c5, c10 = "#1f77b4", "#d95f02"

    fig, axes = plt.subplots(2, 2, figsize=(7.0, 4.6))

    ax = axes[0][0]
    ax.hist([float(r["sa_cost"]) for r in ur5], bins=12, alpha=0.65, color=c5, label="UR5")
    ax.hist([float(r["sa_cost"]) for r in ur10], bins=12, alpha=0.65, color=c10, label="UR10")
    ax.set_xlabel("SA surrogate cost (rad)")
    ax.set_ylabel("configs")
    ax.set_title("(a) optimized cost ($n{=}50$/arm)")
    ax.legend()
    ax.grid(alpha=0.3)

    ax = axes[0][1]
    for data, c, name in ((ur5, c5, "UR5"), (ur10, c10, "UR10")):
        imp = sorted(float(r["impr_vs_baseline_pct"]) for r in data)
        wins = sum(1 for v in imp if v > 0)
        ax.plot(range(len(imp)), imp, marker="o", ms=2.5, lw=1.2, color=c,
                label=f"{name}: SA wins {wins}/{len(imp)}")
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xlabel("seed (sorted)")
    ax.set_ylabel("SA impr. vs random (%)")
    ax.set_title("(b) SA vs equal-budget random")
    ax.legend()
    ax.grid(alpha=0.3)

    for ax, data, c, name in ((axes[1][0], v5, c5, "UR5"), (axes[1][1], v10, c10, "UR10")):
        xs = [float(r["surrogate"]) for r in data]
        ys = [float(r["motion_median"]) for r in data]
        ax.scatter(xs, ys, color=c, s=26)
        fitline(ax, xs, ys, color=c, lw=1.3, alpha=0.8)
        ax.set_xlabel("surrogate $T_1$ (rad)")
        ax.set_ylabel("real motion $T_2$, median (s)")
        panel = "c" if name == "UR5" else "d"
        ax.set_title(f"({panel}) {name}: $r={pearson(xs, ys):.2f}$ ($n={len(xs)}$)")
        ax.grid(alpha=0.3)

    fig.tight_layout()
    out = os.path.join(FIGS, "phase3_gen.png")
    fig.savefig(out, dpi=DPI)
    print("wrote", out)


if __name__ == "__main__":
    gate_scatter()
    phase3_gen()
