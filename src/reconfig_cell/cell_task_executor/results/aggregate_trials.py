#!/usr/bin/env python3
"""Aggregate within-config trial CSVs into one per-config summary (spec §10).

    python3 aggregate_trials.py <config_label> out.csv in1.csv [in2.csv ...]

Used when a config's trials are collected in more than one launch (long continuous
Gazebo runs degrade after ~35-40 min — the rg2 finger joint states drop and MoveIt
stalls waiting for a complete robot state, so config_2's 30 runs are split across two
shorter sub-batches). This merges the sub-batch CSVs (re-indexing run_idx), writes the
combined CSV, and prints the SAME summary trial_runner prints, computed over all
non-warmup rows. Descriptive per-config only — never cross-compared between configs.
"""
import csv
import statistics
import sys


def load(paths):
    rows = []
    for p in paths:
        with open(p) as f:
            for r in csv.DictReader(f):
                rows.append(r)
    return rows


def fnum(s):
    return [float(x) for x in s.split(";") if x != ""]


def main():
    label, out_csv, ins = sys.argv[1], sys.argv[2], sys.argv[3:]
    rows = load(ins)
    scored = [r for r in rows if r["warmup"] == "0"]
    n_warmup = len(rows) - len(scored)

    # write merged CSV (re-indexed run_idx, warmups kept for provenance)
    cols = ["run_idx", "config", "seed", "warmup", "n_stations", "per_op_plan_time",
            "per_op_exec_time", "full_cycle_time", "success", "failure_cause"]
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for i, r in enumerate(rows):
            r = dict(r); r["run_idx"] = i
            w.writerow({k: r[k] for k in cols})

    n = len(scored)
    ok = [r for r in scored if r["success"] == "1"]
    n_stations = scored[0]["n_stations"] if scored else "?"
    seeds = [r["seed"] for r in scored]

    print("=" * 64)
    print(f"  WITHIN-CONFIG RESULTS — {label}  ({n_stations} stations, N={n} scored, "
          f"{n_warmup} warmup excluded)  [aggregated from {len(ins)} sub-batches]")
    print("=" * 64)
    print(f"  task success      : {len(ok)}/{n}" + (f"  ({100*len(ok)/n:.1f}%)" if n else ""))
    causes = {}
    for r in scored:
        if r["success"] != "1":
            causes[r["failure_cause"]] = causes.get(r["failure_cause"], 0) + 1
    if causes:
        print("  failure causes    : " + ", ".join(f"{k}={v}" for k, v in sorted(causes.items())))
    print(f"  seeds vary        : {len(set(seeds))} unique of {len(seeds)} "
          f"(min {min(seeds)}, max {max(seeds)})")

    def block(lbl, vals):
        if not vals:
            print(f"  {lbl:<18}: (no successful runs)")
            return
        print(f"  {lbl:<18}: mean {statistics.mean(vals):.2f}  std {statistics.pstdev(vals):.2f}  "
              f"median {statistics.median(vals):.2f}  min {min(vals):.2f}  max {max(vals):.2f}")

    cyc = [float(r["full_cycle_time"]) for r in ok]
    n_ops = len(fnum(ok[0]["per_op_plan_time"])) if ok else None
    per_op = [c / n_ops for c in cyc] if n_ops else []
    plan_tot = [sum(fnum(r["per_op_plan_time"])) for r in ok]
    exec_tot = [sum(fnum(r["per_op_exec_time"])) for r in ok]
    print("  (successful runs)")
    block("per-op cycle [s]", per_op)   # PRIMARY
    block("full cycle [s]", cyc)
    block("total plan [s]", plan_tot)
    block("total exec [s]", exec_tot)
    print("  NOTE: per-config descriptive results only — NOT cross-compared with the other")
    print("        config (3 lanes inherently does more work than 2).")
    print("=" * 64)
    print(f"  merged CSV: {out_csv}")


if __name__ == "__main__":
    main()
