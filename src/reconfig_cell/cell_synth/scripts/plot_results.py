#!/usr/bin/env python3
"""plot_results.py — figures for Rung 3 (no ROS needed).

    python3 plot_results.py convergence out.png traj1.csv [traj2.csv ...]
    python3 plot_results.py correlation out.png pairs.csv   # cols: label,surrogate,real_cycle[,std]

convergence: SA best-so-far cost vs iteration (one line per spec).
correlation: scatter of deterministic surrogate vs measured real cycle time + Pearson r.
"""
import csv
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def convergence(out, csvs):
    plt.figure(figsize=(7, 4.5))
    for p in csvs:
        rows = list(csv.DictReader(open(p)))
        xs = [int(r["iter"]) for r in rows]
        ys, run = [], None
        for r in rows:                       # plot best-so-far (monotone) for readability
            v = float(r["cost"])
            run = v if run is None else min(run, v)
            ys.append(run)
        plt.plot(xs, ys, lw=1.3, label=p.split("/")[-1].replace("sa_traj_", "").replace(".csv", ""))
    plt.xlabel("SA iteration"); plt.ylabel("best surrogate cost so far")
    plt.title("Rung 3 — SA convergence (joint-travel surrogate)")
    plt.legend(fontsize=7, ncol=2); plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig(out, dpi=130); print(f"wrote {out}")


def pearson(xs, ys):
    n = len(xs); mx = statistics.mean(xs); my = statistics.mean(ys)
    cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sx = sum((x - mx) ** 2 for x in xs) ** 0.5
    sy = sum((y - my) ** 2 for y in ys) ** 0.5
    return cov / (sx * sy) if sx and sy else float("nan")


def correlation(out, pairs_csv):
    rows = list(csv.DictReader(open(pairs_csv)))
    xs = [float(r["surrogate"]) for r in rows]
    ys = [float(r["real_cycle"]) for r in rows]
    errs = [float(r.get("std", 0) or 0) for r in rows]
    r = pearson(xs, ys)
    plt.figure(figsize=(6, 5))
    plt.errorbar(xs, ys, yerr=errs, fmt="o", ms=6, capsize=3)
    for r_ in rows:
        plt.annotate(r_.get("label", ""), (float(r_["surrogate"]), float(r_["real_cycle"])),
                     fontsize=6, xytext=(4, 4), textcoords="offset points")
    # least-squares line
    if len(xs) > 1:
        b = pearson(xs, ys) * (statistics.pstdev(ys) / statistics.pstdev(xs))
        a = statistics.mean(ys) - b * statistics.mean(xs)
        lo, hi = min(xs), max(xs)
        plt.plot([lo, hi], [a + b * lo, a + b * hi], "r--", lw=1,
                 label=f"fit (Pearson r={r:.3f})")
        plt.legend()
    plt.xlabel("deterministic joint-travel surrogate")
    plt.ylabel("measured real cycle time [s]")
    plt.title(f"Rung 3 — surrogate vs real (r={r:.3f}, n={len(xs)})")
    plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig(out, dpi=130); print(f"wrote {out}  Pearson r={r:.4f}  n={len(xs)}")


def main():
    mode, out = sys.argv[1], sys.argv[2]
    if mode == "convergence":
        convergence(out, sys.argv[3:])
    elif mode == "correlation":
        correlation(out, sys.argv[3])
    else:
        sys.exit("usage: plot_results.py convergence|correlation out.png ...")


if __name__ == "__main__":
    main()
