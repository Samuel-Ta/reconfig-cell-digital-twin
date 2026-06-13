# Within-config validation (spec §10) — trial runner

`trial_runner` loops the generated relay task N times per config, resets to an identical
state between runs, logs each run to CSV, and prints per-config summary stats. Results are
**descriptive per-config only** and are **never cross-compared** (config_2's 3 lanes
inherently does more work than config_1's 2).

## Run it

Bring up the cell once and loop the task (run on a machine with a working display — the
gz_ros2_control controller_manager needs the full Gazebo bringup):

```bash
# config_1 (2 lanes)
ros2 launch cell_bringup cell_trials.launch.py config:=config_1 trials:=30 \
    csv:=/tmp/trials_config_1.csv

# config_2 (3 lanes)
ros2 launch cell_bringup cell_trials.launch.py config:=config_2 trials:=30 \
    csv:=/tmp/trials_config_2.csv
```

Args: `trials` (default 30, min 20), `warmup` (default 1, excluded from stats),
`op_timeout` (s, per-operation planning budget), `run_timeout` (s, whole run),
`seed_base` (first run's seed; each run uses `seed_base + run_idx`), `gazebo_gui`
(true/false). The launch passes the cell's `robot_mount` so success is checked against
the real sink world position.

## Protocol (built so the numbers survive review)

- **N**: 30 trials + 1 warmup per config; the warmup row is logged with `warmup=1` and
  excluded from stats (first run after launch is slow from planner/IK caching).
- **Seed**: `random.seed(seed)` is set per run with a **different, logged** seed each run.
  This seeds the IK multi-seed sampler (the dominant randomness here); OMPL's own RNG is
  left **unseeded** so it varies too. No single fixed seed — variance is real, not faked.
- **Reset**: every run starts identical — arm to `up`, part removed + re-spawned at source,
  grasp weld detached.
- **Timeouts**: a per-op planning timeout and a per-run timeout; either trips
  `failure_cause=timeout` (a timeout is a failure, never a hang).
- **Planning vs execution time are logged separately** (planning is the variable part).

## CSV schema

`run_idx, config, seed, warmup, n_stations, per_op_plan_time, per_op_exec_time,
full_cycle_time, success, failure_cause`

- `per_op_plan_time` / `per_op_exec_time`: `;`-separated per-operation seconds.
- `success`: 1 only if the carried part ends within `SINK_TOL_XY` (0.18 m) of the final sink.
- `failure_cause` ∈ `{plan_fail, grasp_fail, place_fail, timeout, none}`.

## Summary stats (printed per config, two separate tables)

success count / N; and for **per-op cycle time** (primary unit, more stable than full
cycle) plus full-cycle, total-plan, total-exec time: mean, std, median, min, max. Median is
included because planning-time distributions are right-skewed and the mean alone misleads.

## Results

**TBD — not yet run end-to-end.** The runner is built, installed, and the launch wiring is
verified; the full 30×2 batch must be run on a machine with a working display (the headless
CI/sandbox here does not start the gz_ros2_control controller_manager). Numbers are left
TBD rather than fabricated. Fill the two tables below from the printed summaries:

### config_1 (2 lanes, N=TBD)
| metric | value |
|---|---|
| task success | TBD / TBD |
| per-op cycle [s] | mean TBD, std TBD, median TBD, min TBD, max TBD |
| full cycle [s] | TBD |
| total plan / exec [s] | TBD / TBD |
| failure causes | TBD |

### config_2 (3 lanes, N=TBD)
| metric | value |
|---|---|
| task success | TBD / TBD |
| per-op cycle [s] | mean TBD, std TBD, median TBD, min TBD, max TBD |
| full cycle [s] | TBD |
| total plan / exec [s] | TBD / TBD |
| failure causes | TBD |
