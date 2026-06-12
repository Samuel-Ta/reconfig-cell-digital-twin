# reference_baseline_conveyor — rigid baseline for the CONVEYOR cell (§10 effort metric)

This is the **rigid / monolithic** comparison point for the declarative framework, built
on the **same conveyor cell** the framework demo uses — so the whole paper is one
consistent cell (no box-fixture vs conveyor mismatch).

It is constructed the locked way (spec §10):

1. **Freeze** the framework's generated output for conveyor **config_1** into `config_1/`
   as three hand-maintained artifacts — `world.sdf` (conveyor spawns, WORLD frame),
   `scene.yaml` (MoveIt BOX collisions, base_link), `task.yaml` (pick/place ops with the
   derived top-down targets as **hardcoded literals**). The generator is then set aside.
2. **Reconfigure** config_1 → config_2 **by hand** in those frozen files: add conveyor_3
   and the new hop. `config_2/` is the post-edit snapshot.
3. **Measure** that hand edit against the framework's single-file append.

The frozen literals are byte-identical to the framework's generator output for the same
poses (verified: conveyor_3's base_link scene pose `{0.7706, 0.2804, …}` and task target
`{0.516875008, 0.188076502, …}` match `cell_description/generated/config_2/`), so the two
sides describe the *same* physical cell — the only difference measured is **structural
coupling**, not modelling choices.

## Counting methodology (honesty rules)

The headline metric is **structural coupling**, NOT raw line count:
(a) artifacts/files touched, (b) distinct edit sites, (c) **synchronized copies of each
new pose** that must be kept consistent by hand across world/scene/task, (d) task-wiring
edits. Wall-clock time is secondary context only.

**Excluded from the count on BOTH sides (explicitly, so it is on record):**
- **Static visual-mesh boilerplate.** The conveyor `<include><uri>…DeliveryRobotWithConveyor</uri>`
  is identical scenery in the framework world and in this hand baseline. It is *not*
  reconfiguration work and does **not** inflate the baseline. Only what actually changes
  to add a lane is counted: the conveyor **pose**, its **synchronized pose-copies**, the
  **collision primitive**, and the **task ops**.
- **Comment lines** (`#`, `<!-- -->`). Counted as 0 on both sides.

Numbers below are **measured from the actual diffs** (`diff config_1 config_2`), not
estimated. Any unmeasured value is left **TBD**.

## Measured result — adding one lane (config_1 → config_2)

| Structural metric | Framework (1 YAML) | Rigid baseline (conveyor) |
|---|---:|---:|
| **Artifacts / files touched** | **1** (`config_2.yaml`) | **3** (`world.sdf`, `scene.yaml`, `task.yaml`) |
| **Distinct edit sites (diff hunks)** | 2 (append station + append ops, same file) | 4 (1 `<world name>` modify + 3 lane appends across 3 files) |
| **Synchronized copies of the NEW conveyor pose** (hand-kept-consistent, different frames) | **1** (single source; the other 2 frames are *derived* by the generator) | **3** — WORLD `<pose>0.0736 -0.3626 … -3.1036>`, base_link BOX `{0.7706, 0.2804, …}`, base_link target `{0.516875, 0.188077, …}`; **3 different frames, 3 independent hand transforms** |
| **Re-stated copies of an EXISTING pose** (re-grasp pick) | 0 (generator emits it) | 1 (conveyor_2's literal duplicated in `task.yaml`) |
| **Task-wiring ops added** | 2 (same file as the topology) | 2 (separate `task.yaml`) |
| **Modified lines** | **0** (append-only, INVARIANT 5) | 1 (`<world name>` config_1→config_2) |
| **Functional lines added** (excl. comments + mesh scenery) | **3** | ~10 |
| Wall-clock reconfig time | seconds (one append) | TBD (longer — 3 frames hand-computed) |

### The un-dismissable claim
To add one lane, the framework author writes the conveyor's pose **once**
(`config_2.yaml:15`); the generator derives the scene-collision pose and the grasp target.
The rigid baseline author must write that pose as **three hand-synchronized copies in
three different coordinate frames** (world, scene, task), each needing its own transform
(robot-mount subtraction; inward-shift + grasp-z; slab-z + yaw→quaternion) — plus
re-state conveyor_2's pose a fourth time for the re-grasp. Any one of those copies drifting
out of sync silently breaks the cell. That cross-file, cross-frame synchronization is the
cost the single-source-of-truth design removes.

## Files
- `config_1/{world.sdf, scene.yaml, task.yaml}` — frozen config_1 (the rigid baseline).
- `config_2/{world.sdf, scene.yaml, task.yaml}` — the same three files hand-edited to add
  the lane. `diff -r config_1 config_2` reproduces the baseline-side numbers above.

The box-fixture `reference_baseline/` package is kept for now and will be retired once
these conveyor numbers are confirmed clean.
