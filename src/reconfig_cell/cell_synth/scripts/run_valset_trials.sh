#!/usr/bin/env bash
# Rung 3 surrogate-vs-real: measure REAL cycle time for each validation config.
# Each config runs in its OWN fresh Gazebo boot (so the ~25-40 min sim degradation never
# accumulates across configs). Reuses synth_trials.launch.py (synth-pose conveyors + the
# existing trial_runner). Produces val_out/pairs.csv (label,surrogate,real_cycle,std,n).
#
#   ./run_valset_trials.sh [trials_per_config]   (default 3)
set -u
WS=$HOME/reconfig_ws
VAL=$WS/src/reconfig_cell/val_out
GEN=$VAL/generated
TRIALS=${1:-3}
source /opt/ros/jazzy/setup.bash 2>/dev/null
source $WS/install/setup.bash 2>/dev/null

[ -f "$VAL/valset_surrogate.csv" ] || { echo "run valset first (no valset_surrogate.csv)"; exit 1; }

tail -n +2 "$VAL/valset_surrogate.csv" | while IFS=, read -r label surrogate source config; do
  name="${config%.yaml}"
  ros2 run cell_generator generate --config "$VAL/$config" --out "$GEN" >/dev/null 2>&1
  csv="$VAL/realtrials_${name}.csv"
  log="/tmp/realtrial_${name}.log"
  echo "=== $label (surrogate $surrogate): $TRIALS trials, fresh sim ==="
  timeout 1200 ros2 launch cell_synth synth_trials.launch.py \
    cfg:="$VAL/$config" scene:="$GEN/$name/scene.yaml" task:="$GEN/$name/task.yaml" \
    trials:="$TRIALS" warmup:=1 csv:="$csv" gazebo_gui:=true >"$log" 2>&1
  pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f "move_group" 2>/dev/null
  sleep 3
  echo "   -> $(grep -ac , "$csv" 2>/dev/null) csv rows; batch: $(grep -ac 'batch complete' "$log") "
done

# merge surrogate + measured full_cycle (mean over successful, non-warmup runs) -> pairs.csv
python3 - "$VAL" <<'PY'
import csv, glob, os, statistics, sys
val = sys.argv[1]
sur = {r["label"]: r for r in csv.DictReader(open(os.path.join(val, "valset_surrogate.csv")))}
out = []
for label, r in sur.items():
    name = r["config"][:-5]
    p = os.path.join(val, f"realtrials_{name}.csv")
    cyc = []
    if os.path.exists(p):
        for row in csv.DictReader(open(p)):
            if row.get("warmup") == "0" and row.get("success") == "1":
                cyc.append(float(row["full_cycle_time"]))
    if cyc:
        out.append(dict(label=label, surrogate=r["surrogate"],
                        real_cycle=round(statistics.mean(cyc), 3),
                        std=round(statistics.pstdev(cyc), 3) if len(cyc) > 1 else 0.0,
                        n=len(cyc)))
        print(f"  {label}: surrogate {r['surrogate']}  real {out[-1]['real_cycle']}s (n={len(cyc)})")
    else:
        print(f"  {label}: NO successful real runs (excluded)")
with open(os.path.join(val, "pairs.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["label", "surrogate", "real_cycle", "std", "n"])
    w.writeheader(); w.writerows(out)
print(f"wrote {os.path.join(val,'pairs.csv')} with {len(out)} pairs")
PY
