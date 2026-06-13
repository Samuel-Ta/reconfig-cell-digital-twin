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

## Results (measured on the GUI machine; raw CSVs in `results/`)

Per-config descriptive only — **NOT cross-compared** (config_2's 3 lanes inherently does
more work than config_1's 2). One warmup run per launch excluded from stats. Every seed
unique within each config. Numbers are read verbatim from the printed summaries
(`results/summary_config_{1,2}.txt`).

### config_1 (2 lanes, N=30, seeds 1000–1030) — `results/trials_config_1.csv`
| metric | value |
|---|---|
| task success | **29 / 30 (96.7%)** |
| per-op cycle [s] *(primary)* | mean 34.93, std 2.23, median 33.91, min 33.33, max 42.98 |
| full cycle [s] | mean 69.87, std 4.46, median 67.82, min 66.67, max 85.96 |
| total plan / exec [s] | 0.20 / 12.85 (means) |
| failure causes | place_fail = 1 |

### config_2 (3 lanes, N=27, seeds 1001–1016 + 2001–2011) — `results/trials_config_2.csv`
| metric | value |
|---|---|
| task success | **27 / 27 (100%)** |
| per-op cycle [s] *(primary)* | mean 29.36, std 0.98, median 29.34, min 28.22, max 33.18 |
| full cycle [s] | mean 117.43, std 3.92, median 117.35, min 112.86, max 132.71 |
| total plan / exec [s] | 0.38 / 22.19 (means) |
| failure causes | none |

**Collection note (honesty).** config_1 ran as one clean 31-run batch. config_2 was
collected in two healthy sub-batches — part A (16, seeds 1001–1016) + part B (11, seeds
2001–2011) = **27 scored**, above the stated minimum of 20 but short of 30. Reason: on this
machine a Gazebo session reliably degrades after ~25–40 min of continuous running — the
`rg2_finger_joint{1,2}` states stop publishing (`Missing … complete state of the robot is
not yet known` + `Host unreachable`), so MoveIt can no longer get a complete robot state and
runs fast-fail at the first motion. A top-up "part C" launched into an already-degraded
machine and produced only infrastructure `plan_fail`s; it was **discarded**
(`trials_config_2_partC_DISCARDED_simdegraded.csv`) rather than counted, since those are
harness failures, not task outcomes. The 27 reported runs were all collected while the sim
was healthy. To reach a clean 30, run part C from a fresh boot:
`ros2 launch cell_bringup cell_trials.launch.py config:=config_2 trials:=3 seed_base:=3000
csv:=.../trials_config_2_partC.csv` then re-run `aggregate_trials.py`.

**Planning vs execution (why we logged them separately).** Planning time is tiny and very
stable (config_1 mean 0.20 s, config_2 0.38 s) while execution carries essentially all the
cycle-time variance — lumping them would have made the planner look far more variable than it
is. The remaining bulk of each cycle is the deliberate reset/settle/attach-detach sleeps.
