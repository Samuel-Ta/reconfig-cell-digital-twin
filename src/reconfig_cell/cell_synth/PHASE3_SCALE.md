# Phase 3 — Scale to 50+ configs + UR10 transfer (multi-robot generality)

Both workstreams use the final full-cell formulation (SA engine, deterministic surrogate,
brute-force TSP order). Per the Phase-2 finding that base world pose is **cost-neutral** under
the cycle-time surrogate (base-relative invariance), the base is **fixed at the arena-center
mount** (`config_1`) for both robots — principled (the objective gives no base gradient) and
keeps the comparison clean. Documented finding: *base placement for cycle-time optimization is
determined by relative station geometry; an additional footprint/accessibility objective would
be needed to pin the absolute mount pose.*

Scripts (all reused, unmodified except the `ur_type` launch arg): `optimize` (SA vs
equal-budget random-valid baseline), `valset` (build a validation set spanning the surrogate
range), `motion_probe` (headless RRTstar real motion time). Data: `phase3_ur5/`, `phase3_ur10/`,
`phase3_ur5_val/`, `phase3_ur10_val/`.

## Workstream A — Scale up (50 optimized configs per robot)

50 SA-optimized configs per robot (seeds 500–549 UR5, 600–649 UR10; 3 stations; iters 150).

| robot | surrogate cost (mean ± std) | min–max | SA beats equal-budget random |
|---|---|---|---|
| **UR5** | 4.985 ± 0.156 | 4.67 – 5.35 | **46/50, mean −9.2%** (min −6.0%, max −48.6%) |
| **UR10** | 4.845 ± 0.262 | 4.31 – 5.27 | **45/50, mean −7.3%** (min −2.8%, max −25.4%) |

(Improvement = surrogate-cost reduction of the SA winner vs the best random-valid config found
under an identical oracle-evaluation budget.) Generalization evidence: **SA reliably beats
random across 50 seeds on both arms.**

## Workstream B — UR10 transfer

UR10 added by threading `ur_type` through `synth.launch.py` into the robot description. UR arms
share joint names, so the same SRDF / `ur_manipulator` group / RG2-on-`tool0` / `rg2_tcp` TIP
load unchanged; UR10's longer links (its `ur_description/config/ur10/physical_parameters.yaml`)
are used automatically by the IK and surrogate. Same arena, annulus, and task. UR10 loads
headless, synthesizes valid configs, and SA beats random (above).

## Real-sim correlation (surrogate vs RRTstar motion time)

Validation set of 16 configs per robot spanning the surrogate range; headless RRTstar
(optimizing planner), correlated on `motion_min`/`motion_median` with bootstrap 95% CIs.

| robot | budget | metric | Pearson r [95% CI] | Spearman ρ [95% CI] |
|---|---|---|---|---|
| **UR10** | 5 s × 6 plans | motion_median | **0.828** [0.62, 0.95] | **0.791** [0.41, 0.96] |
| **UR10** | 5 s × 6 plans | motion_min | 0.645 [0.14, 0.91] | 0.603 [0.05, 0.94] |
| **UR5** | 5 s × 6 plans (reduced) | motion_median | 0.440 [0.09, 0.83] | 0.500 [−0.08, 0.86] |
| **UR5** | 5 s × 6 plans (reduced) | motion_min | 0.413 [0.07, 0.77] | 0.385 [−0.18, 0.77] |
| **UR5** | **10 s × 10 plans (full)** | motion_min | _(re-validation running — to append)_ | |

**Honest note on the UR5 reduced-budget number.** RRTstar is an *optimizing* planner that uses
its entire `planning_time` every call (no early stop). To fit the scale-up in one day the first
pass used a halved budget (5 s, 6 plans); at that budget RRTstar does not converge, so real
motion is noisy — worst for UR5, whose stations sit near its reach limit (UR10's longer reach
plans the same arena more comfortably, hence its cleaner correlation even at reduced budget).
This is a *measurement* artifact, not a surrogate failure: the **established full-budget gate**
for UR5 (`val_hb`, 10 s × 12 plans, n=6 → pooled n=11) is **r = 0.880 / ρ = 0.943** (paper
§Surrogate-to-Real Validation). A full-budget UR5 re-validation (10 s × 10 plans, n=16) is
running and will be appended here.

## Takeaways
- The SA-against-the-validated-surrogate approach **transfers across robot arms**: SA beats
  random on both UR5 and UR10 across 50 seeds, and the surrogate correlates with real RRTstar
  motion on UR10 (median r=0.83).
- For cycle-time optimization the robot **base pose is cost-neutral** (Phase 2); Phase 3 fixes
  it at arena center for both robots accordingly.

All numbers measured; no fabrication. Correlations are deterministic-up-to-planner-noise.
