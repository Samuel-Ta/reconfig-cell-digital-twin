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

Clean re-run on the DetachableJoint detach-reliability-fixed code (HEAD), each config
collected as **N=30 scored** across fresh-booted, healthy sub-batches (no degraded runs
kept). Per-config descriptive only — **NOT cross-compared** (config_2's 3 lanes inherently
does more work than config_1's 2). One warmup run per launch excluded from stats. Every seed
unique within each config. Numbers are read verbatim from the printed summaries
(`results/summary_config_{1,2}.txt`).

### config_1 (2 lanes, N=30, seeds 1001–1015 + 2001–2015) — `results/trials_config_1.csv`
| metric | value |
|---|---|
| task success | **28 / 30 (93.3%)** |
| per-op cycle [s] *(primary)* | mean 42.07, std 1.67, median 42.19, min 39.10, max 45.79 |
| full cycle [s] | mean 84.14, std 3.34, median 84.38, min 78.20, max 91.58 |
| total plan / exec [s] | 0.25 / 13.78 (means) |
| failure causes | grasp_fail = 1, place_fail = 1 |

### config_2 (3 lanes, N=30, seeds 1001–1007 + 2001–2010 + 3001–3010 + 4001–4003) — `results/trials_config_2.csv`
| metric | value |
|---|---|
| task success | **29 / 30 (96.7%)** |
| per-op cycle [s] *(primary)* | mean 34.53, std 1.63, median 34.39, min 31.86, max 36.94 |
| full cycle [s] | mean 138.11, std 6.51, median 137.55, min 127.44, max 147.75 |
| total plan / exec [s] | 0.47 / 25.71 (means) |
| failure causes | place_fail = 1 |

**Collection note (honesty).** On this machine a Gazebo session reliably degrades after
~25–40 min of continuous running — the `rg2_finger_joint{1,2}` states stop publishing
(`Missing … complete state of the robot is not yet known` + `Host unreachable`), so MoveIt
can no longer get a complete robot state and runs fast-fail at the first motion. To keep
every scored run inside a healthy window, each config was collected as short sub-batches
from fresh boots, each with a distinct `seed_base` so all 30 seeds stay unique, then merged
with `aggregate_trials.py`: config_1 = part A (15, seeds 1001–1015) + part B (15, seeds
2001–2015); config_2 = part A (7, 1001–1007) + part B (10, 2001–2010) + part C (10,
3001–3010) + part D (3, 4001–4003). Any batch that showed the degradation signature was
discarded and re-collected, never counted (discarded CSVs retained with a `_DISCARDED_*`
suffix for transparency). Earlier pre-fix provisional CSVs are kept as
`trials_config_*_provisional.csv`.

**Did the detach fix change the failure count?** The DetachableJoint fix (confirmed detach +
defensive pre-spawn reset + per-run "gripper clear at start" log) did **not** reduce the
genuine task-failure count — config_1's 2 failures and config_2's 1 are real grasp/place IK
outcomes, not weld artifacts. What it changed is **run independence**: every one of the 60
scored runs logged `gripper clear at start: True` (0 dirty starts), and observed failures no
longer cascade — e.g. config_2 part C run 8 (place_fail, seed 3007) was immediately followed
by a clean success, where pre-fix a leftover weld could carry a failure into the next run.
The numbers above are therefore independent per-run outcomes, not a contaminated sequence.

**Planning vs execution (why we logged them separately).** Planning time is tiny and very
stable (config_1 mean 0.20 s, config_2 0.38 s) while execution carries essentially all the
cycle-time variance — lumping them would have made the planner look far more variable than it
is. The remaining bulk of each cycle is the deliberate reset/settle/attach-detach sleeps.
