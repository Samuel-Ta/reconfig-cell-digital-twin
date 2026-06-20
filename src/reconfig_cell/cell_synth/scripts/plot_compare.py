#!/usr/bin/env python3
"""plot_compare.py — figures for the fair optimizer comparison.

    python3 plot_compare.py <out_dir> [n_stations]

Reads compare_convergence_n{N}.csv and compare_summary_n{N}.csv (written by
compare_optimizers) and emits:
  * figs/optimizer_convergence.png — mean best-so-far vs #evals, one line per method (+std band)
  * figs/optimizer_quality.png      — final mean best surrogate cost per method (bar, error=std)
Pure post-processing of measured CSVs; computes no new numbers.
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

METHOD_ORDER = ["Random", "SA", "GA", "CMA-ES", "PSO", "BO"]
COLORS = {"Random": "#999999", "SA": "#d62728", "GA": "#1f77b4",
          "CMA-ES": "#2ca02c", "PSO": "#9467bd", "BO": "#ff7f0e"}


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.expanduser("~/reconfig_ws/src/reconfig_cell/opt_compare")
    n = sys.argv[2] if len(sys.argv) > 2 else "3"
    figs = os.path.join(out_dir, "figs")
    os.makedirs(figs, exist_ok=True)

    seedf = os.path.join(out_dir, f"compare_seedcurves_n{n}.csv")
    summ = os.path.join(out_dir, f"compare_summary_n{n}.csv")

    # ---- convergence: median + IQR over FEASIBLE seeds (blank cells = still infeasible) ----
    import numpy as np
    with open(seedf) as f:
        sr = csv.DictReader(f)
        cols = [c for c in sr.fieldnames if c != "eval"]
        data = {c: [] for c in cols}
        evals = []
        for row in sr:
            evals.append(int(row["eval"]))
            for c in cols:
                v = row[c]
                data[c].append(float(v) if v not in ("", None) else np.nan)
    # group seed columns by method (col name is "<method>_s<seed>")
    bymethod = {}
    for c in cols:
        m = c.rsplit("_s", 1)[0]
        bymethod.setdefault(m, []).append(c)
    methods = [m for m in METHOD_ORDER if m in bymethod] + \
              [m for m in bymethod if m not in METHOD_ORDER]

    evals = np.array(evals, dtype=float)
    BIG = 100.0                                       # best-so-far before first feasibility
    feas_vals = [v for c in cols for v in data[c] if not np.isnan(v)]
    ylo = (min(feas_vals) - 0.2) if feas_vals else 4.5
    yhi = (np.percentile(feas_vals, 97) + 0.3) if feas_vals else 9.0
    plt.figure(figsize=(6.4, 4.2))
    plotted = False
    for m in methods:
        arr = np.array([data[c] for c in bymethod[m]], dtype=float)   # (seeds, evals)
        nfeas = int(np.sum(~np.isnan(arr[:, -1])))
        if nfeas == 0:
            continue                                                  # e.g. Random: never feasible
        filled = np.where(np.isnan(arr), BIG, arr)                    # pre-feasible = penalty
        # each seed's best-so-far is monotone non-increasing => so is the cross-seed median.
        med = np.median(filled, axis=0)
        q1 = np.percentile(filled, 25, axis=0)
        q3 = np.percentile(filled, 75, axis=0)
        c = COLORS.get(m, None)
        lbl = m + (f" ({nfeas}/{arr.shape[0]} feas)" if nfeas < arr.shape[0] else "")
        plt.plot(evals, med, label=lbl, color=c, lw=1.8)
        plt.fill_between(evals, q1, q3, color=c, alpha=0.12)
        plotted = True
    plt.ylim(ylo, yhi)                                # clip to the feasible band
    plt.xlabel("surrogate evaluations")
    plt.ylabel("best-so-far surrogate cost (median, IQR band)")
    plt.title("Optimizer convergence on the validated surrogate")
    if plotted:
        plt.legend(frameon=False, ncol=2, fontsize=9)
    plt.grid(alpha=0.3)
    plt.tight_layout()
    p1 = os.path.join(figs, "optimizer_convergence.png")
    plt.savefig(p1, dpi=160)
    print("wrote", p1)

    # ---- final-quality bars ----
    with open(summ) as f:
        srows = {r["method"]: r for r in csv.DictReader(f)}
    # skip methods with no feasible solution (NaN best_mean), e.g. Random
    ms = [m for m in methods if m in srows and srows[m]["best_mean"] == srows[m]["best_mean"]
          and srows[m]["best_mean"].lower() != "nan"]
    means = [float(srows[m]["best_mean"]) for m in ms]
    stds = [float(srows[m]["best_std"]) for m in ms]
    plt.figure(figsize=(6.0, 4.0))
    bars = plt.bar(ms, means, yerr=stds, capsize=4,
                   color=[COLORS.get(m, "#555") for m in ms])
    for b, v in zip(bars, means):
        plt.text(b.get_x() + b.get_width() / 2, v, f"{v:.2f}",
                 ha="center", va="bottom", fontsize=9)
    plt.ylabel("final best surrogate cost (mean ± std)")
    plt.title("Solution quality by optimizer (lower is better)")
    plt.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    p2 = os.path.join(figs, "optimizer_quality.png")
    plt.savefig(p2, dpi=160)
    print("wrote", p2)


if __name__ == "__main__":
    main()
