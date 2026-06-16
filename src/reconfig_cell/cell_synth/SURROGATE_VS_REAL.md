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

## Attempt 2 — planned (attack the noise, not the result)

1. **Fix the planner seed** so the same config yields the same path → stable motion time.
   Verify by running one config 3× (motion must stop being ~2× spread) and report that first.
2. **Correlate on motion/exec time only** (strip the ~130 s fixed overhead).
3. Collect what is cleanly obtainable (reboot around degradation); report per-config surrogate
   vs real motion, Pearson r + scatter, and the N it is based on.

Honest rule: if it correlates → report the clean positive; if not → report that plainly. No
forcing, no fabricated points.
