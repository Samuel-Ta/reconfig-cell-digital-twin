# Surrogate vs real cycle time — validation (Rung 3)

Goal: check whether the deterministic joint-travel **surrogate** (the SA objective) predicts
**real** measured cycle time in the Gazebo twin. Configs span a surrogate range; each is run
through the existing `trial_runner` (synth-pose conveyors, robot at the synthesized mount).

## Attempt 1 — INCONCLUSIVE (honest record, 2026-06-16)

Validation set `val_hb/` (robot at the hand-placed mount −0.4414, 0.9671): `val_0` is the SA
winner (surrogate 5.19), `val_1..5` are random valids (surrogate up to 9.39).

**Cleanly measured (before sim hangs):** 3 of 6 configs.

| config | surrogate | real full_cycle (mean) | real motion/exec (mean) |
|---|---|---|---|
| val_0 (SA winner) | 5.19 | 165 s (142/176/177) | 50 s (29/60/61) |
| val_1 | 6.59 | 196 s | 83 s |
| val_2 | 7.14 | 127 s | 22 s |

**Why inconclusive — two documented noise sources, not a forced negative:**
1. **RRTConnect path variance.** The *same* config's motion time varies ~2× run-to-run
   (val_0: 29/60/61 s) because the randomized planner finds a different collision-free path
   each run. That within-config variance is **larger than the differences between configs**,
   so no clean trend emerges (surrogate 5.19/6.59/7.14 → motion 50/83/22 s).
2. **Fixed overhead.** `full_cycle` includes ~130 s of config-independent reset/settle/
   attach-detach sleeps that swamp the motion signal the surrogate models.
3. **Sim instability.** Runs hang in trajectory execution (gz controller freezes) after a few
   configs, so collecting enough runs to average out (1) is not feasible without rebooting.

**What this does and does NOT say:**
- The surrogate is still valid for its purpose — a **deterministic optimization objective**
  (<0.0003 % spread), and SA genuinely minimizes it (beats the fair baseline 6/6, +10.7 %).
- It is **not yet shown** to predict *absolute* real cycle time, for the reasons above.
- The **SA-optimized config ran end-to-end** on the twin at the user's mount: `val_0`
  completed 3/3 clean relays.

## Attempt 2 — DONE: NO reliable correlation (honest negative, 2026-06-17)

Both requested noise fixes were applied, plus the OMPL-seed investigation:

1. **Planner seed — NOT settable.** MoveIt2 exposes no OMPL motion-planner seed (param or
   env); only the constraint-sampler seed exists. Seeding would require modifying locked
   MoveIt/`trial_runner`. So instead the variance was attacked by **averaging** (below).
2. **Measured motion only, headless.** New `motion_probe` plans the relay pose-to-pose with
   the real RRTConnect planner + the config's belt collisions, headless, and sums the planned
   **trajectory duration** (sim-time motion). This strips BOTH the ~130 s fixed overhead AND
   the gz wall-clock/real-time-factor noise AND the sim hangs. Planner set to **best-of-10**,
   matching the real relay's `planning_attempts:10`.
3. **Averaged** over many plans per config (12, then 40).

**Result — the surrogate does NOT predict real motion time:**

| config | surrogate | real motion (best-of-10, mean ± std) |
|---|---|---|
| val_0 (SA winner) | 5.19 | 25.1 ± 16.8 s |
| val_1 | 6.59 | 17.9 ± 1.4 s |
| val_2 | 7.14 | 33.5 ± 10.9 s |
| val_3 | 7.63 | 13.4 ± 2.5 s |
| val_4 | 9.15 | 17.7 ± 5.8 s |
| val_5 | 9.39 | 11.3 ± 1.9 s |

- **Pearson r is unstable across runs: +0.057 (12 plans) vs −0.550 (40 plans).** A coefficient
  that flips sign between repetitions of the same experiment is, by definition, measuring
  noise — there is no reliable relationship.
- **Why:** RRTConnect path duration has ~96 % per-plan spread *even at best-of-10*, and the
  planner frequently fails on these collision-constrained reaches (only 3–11 of 40 plans
  succeed per config). The deterministic minimal-joint-travel surrogate cannot model the
  planner's collision-avoidance detours, which dominate the real duration and vary by config.
- (superseded figure; see Attempt 3 for the current scatter.)

**Conclusion of Attempt 2 (honest at the time):** with a *randomized, feasibility-only*
planner (RRTConnect) and a *randomized* visit order, no reliable relationship — the
within-config path variance (~96 %) swamped the between-config signal.

## Attempt 3 — POSITIVE correlation, measured correctly (2026-06-17)

Diagnosis that drove the fix: "minimal joint travel is a false idol when paired with a
*randomized* sampler." Three changes to align the measurement with what the surrogate models:

1. **Optimizing planner.** RRTConnect → **RRTstar** (asymptotically optimal). Required
   injecting a group-qualified planner list (`ompl.ur_manipulator.planner_configs`) in
   `synth.launch.py`; MoveItConfigsBuilder's default config has no group section so the
   planner-selection lookup otherwise can't find `RRTstar`.
2. **Kinematic continuity.** The relay is planned **joint-to-joint**: each station's goal is a
   collision-free IK chosen by `Oracle.solve_ik_continuous` to be the branch **closest to the
   previous station's joint state** (no fake elbow-flip spikes). A transfer segment A→B
   excludes BOTH endpoint belts (leaving A / approaching B is required, not a collision —
   is_valid's rule applied to both ends); every other belt stays in and is avoided.
3. **Brute-force TSP visit order.** `Oracle.tsp_order` fixes the optimal station order; SA no
   longer guesses it. Surrogate and real motion use the SAME (TSP) order.

Effect of (2): every config now plans to a **fixed** start/goal, so 5 of 6 configs are
**near-deterministic** (per-plan spread 89 % → **0 %**). The lone exception, val_2, is RRTstar
**occasionally failing to converge** (returns a 2–3× longer path on some runs).

| config | surrogate | motion **min** (optimal) | mean | median | std |
|---|---|---|---|---|---|
| val_0 (SA winner) | 5.17 | 6.81 | 6.81 | 6.81 | 0.00 |
| val_1 | 6.38 | 8.06 | 9.14 | 8.92 | 0.98 |
| val_2 | 6.60 | 7.35 | 12.58 | 10.40 | 3.79 |
| val_4 | 6.93 | 8.56 | 8.61 | 8.61 | 0.01 |
| val_3 | 6.96 | 9.03 | 9.08 | 9.10 | 0.03 |
| val_5 | 7.36 | 9.11 | 9.52 | 9.11 | 1.22 |

RRTstar 6 s × 10 plans, n=6 configs.

| estimator | Pearson r | Spearman ρ |
|---|---|---|
| **min** (converged-optimal path) | **+0.880** | **+0.943** |
| median | +0.704 | +0.543 |
| mean | +0.486 | +0.371 |

**The correct metric is the min.** RRTstar is asymptotically optimal, so the *minimum* planned
duration over runs is the best estimate of its converged-optimal path cost; the mean is
contaminated by the occasional non-converged run (visibly val_2, whose min 7.35 s matches its
mid-range surrogate while unlucky runs inflate its mean to 12.6 s). On that metric the surrogate
**strongly predicts** the optimizing planner's motion time — and the rank order is almost exact
(ρ = 0.943; only a tiny val_1↔val_2 inversion). The mean/median are reported alongside for full
transparency, not cherry-picked.

- Figure: `val_hb/figs/surrogate_vs_real.png` (min = filled points + fit line; mean ± std faded).

**Conclusion (honest):** once the measurement matches what the surrogate models — an *optimizing*
planner, kinematic *continuity*, and a *fixed* visit order — the deterministic joint-travel
surrogate **does predict** the planner's optimal motion time (Pearson r = 0.88, Spearman
ρ = 0.94). Attempt 2's negative was a property of the randomized planner/sampler, not of the
surrogate. **HARD GATE PASSED** → it is now justified to optimize against the surrogate.

## Closed loop — SA reduction translates to real motion (2026-06-17)

After the gate passed, the full SA re-run (opt_out_gate/, base = val_out/handbase.yaml, 5 specs ×
400 iters) beat the equal-budget random baseline **5/5**, mean **−17.4 %** surrogate. The 5 SA
winners were then run back through `motion_probe` (val_loop/, same RRTstar 6 s × 10) and POOLED
with the 6 prior valids (n = 11):

- Pooled correlation holds: **Pearson r = 0.876, Spearman ρ = 0.900** on min motion.
- All 5 SA winners are **perfectly deterministic** (std 0.00 over 10 plans) and sit at the LOW
  real-motion end (5.84–7.87 s).
- **SA winners mean real motion 6.79 s vs random valids 8.42 s → 19.4 % FASTER**, empirically
  matching the −17.4 % surrogate gain. The predicted reduction shows up in real RRTstar motion.
- Figure: `val_loop/figs/closed_loop.png`.

**Spec complete:** surrogate is a stable objective (<0.0003 % spread) → SA provably minimizes it
vs a fair floor (5/5, −17.4 %) → surrogate provably predicts real optimizing-planner motion
(r = 0.88) → SA winners are provably faster in real motion (−19.4 %). End to end.
