# Phase 2 — Full-cell synthesis: robot base pose in the decision space

Extends the Phase-1 optimizer so the **robot base pose `(x_b, y_b, ψ_b)`** is searched jointly
with the station placements. Stations are sampled in the UR5 reach annulus **relative to the
sampled base** (base-frame polar `(r, θ)`); the base pose then places the whole cell in the
world. SA is the engine (the Phase-1 justified choice), visit order is fixed by brute-force TSP,
and the surrogate / validity oracle / schema are unchanged.

**Code.** `cell_synth/scripts/synth_fullcell`. The only locked-code change is making the
generator **base-yaw aware** (`cell_generator/scripts/generate`: `base_xy`/`spawn_world`/
`emit_scene` rotate by `robot_mount.yaw`, which defaults to 0 → every prior config regenerates
**byte-identically**, verified). The oracle and surrogate already read `robot_mount` from the
doc, so they see the *same* base transform for a candidate — consistency by construction; no
oracle/surrogate edit needed. The twin launch now spawns the robot with `-Y` at the base yaw.

Run: `ros2 launch cell_synth synth.launch.py exe:=synth_fullcell n_specs:=6 base_seed:=300
n_stations:=3 iters:=140 n_ik:=20 fix_base:={0,1} arena_half:=0.6 out_dir:=<repo>/fullcell_out`.

## Sanity check (fix_base:=1 — base frozen at the known-good mount)
Reduces to the Phase-1 fixed-mount SA. Result (config_1 mount, 6 seeds): **6.066 ± 0.724**
(min 5.280), 6/6 feasible, base displacement 0.000 m on every seed. Phase-1 SA was
**5.910 ± 1.027**. The two overlap well inside their std → **the extension reproduces Phase 1
within noise**; nothing broke before opening the base search.

## Full-cell synthesis (fix_base:=0 — base + stations jointly optimized, 6 seeds)

| seed | cost | base (x, y, ψ) | base disp (m) | Δyaw (rad) |
|---|---|---|---|---|
| 300 | 5.775 | (-0.202, -0.355, 0.362) | 0.573 | 0.362 |
| 301 | 5.553 | (-0.279, -0.347, -1.165) | 0.512 | 1.165 |
| 302 | **5.125** | (-1.107, -1.128, 0.844) | 0.635 | 0.844 |
| 303 | 5.128 | (-0.611, -0.787, -0.152) | 0.167 | 0.152 |
| 304 | — (no feasible) | — | — | — |
| 305 | 6.485 | (-0.880, -0.603, 0.015) | 0.187 | 0.015 |
| **mean** | **5.613 ± 0.503** | — | **0.415** (0.17–0.64) | up to 1.17 |

Fixed mount was `(-0.697, -0.643)`; the arena box was `±0.6 m` around it on the warehouse floor.

### What it shows (honest)
1. **The optimizer does move the base** — mean 0.415 m (up to 0.64 m) and yaw up to ~1.17 rad;
   it does not keep it at the fixed point.
2. **But moving the base does not lower the surrogate cost.** Full-cell 5.613 ± 0.503 vs
   fixed-base 6.066 ± 0.724 overlap; the small difference is search noise plus the one dropped
   (infeasible) seed, not a real gain.
3. **This is structural, not luck.** The cycle-time surrogate (base-frame joint travel) *and*
   every validity constraint (reach, clearance ≥0.18 m, no-tilt, base keep-out, station bounds)
   are **base-relative — invariant under a rigid motion of the whole cell.** So the set of
   achievable, valid base-frame layouts is identical whether the base is fixed or free; the base
   world pose is a **degenerate (cost-neutral) dimension**, pinned only by the world arena box.
   SA drifts it freely because no cost gradient holds it.

**Takeaway for the paper:** for *cycle time*, **where you mount the robot does not matter —
only the station geometry relative to it.** Mount placement should be chosen on other grounds
(floor space, walls, cabling, cable-management), and the station optimizer run relative to
whatever mount is chosen. This both validates the Phase-1 fixed-mount formulation and bounds
what base-pose search can buy (nothing, for cycle time).

## Twin
One synthesized full-cell config (non-zero base pose) realized via the yaw-aware generator and
run on the full Gazebo twin (`synth_trials.launch.py`, robot spawned with `-Y`). See the commit
message / run log for the executed seed and outcome.

All numbers measured, deterministic-up-to-IK-noise (~3e-4 %; per-seed costs wobble a few %
between runs, conclusions stable). Data in `fullcell_out/`.
