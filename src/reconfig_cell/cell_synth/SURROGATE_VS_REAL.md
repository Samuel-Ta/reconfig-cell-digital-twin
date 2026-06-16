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
- Figure: `val_hb/figs/surrogate_vs_real.png` (scatter, large error bars, no trend).

**Conclusion (honest, per the brief's rule — report plainly, do not force):**
The deterministic joint-travel surrogate is **validated for its actual purpose** — a stable
optimization objective (<0.0003 % spread) that SA genuinely minimizes (beats the fair baseline
6/6, +10.7 %), and the SA-optimized config runs end-to-end on the twin. But it is **NOT a
predictor of absolute real cycle/motion time**: real execution is dominated by RRTConnect
path-variance that the surrogate does not (and a deterministic surrogate cannot) capture.
This is a real negative result, not a measurement we could clean up further.
