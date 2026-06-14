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
edits.

**Effort is reported as countable edit operations, never as wall-clock time.** A stopwatch
number is skill-dependent (an expert and a novice produce wildly different times for the
*same* edit) and therefore not reproducible. We instead report reconfiguration effort as
**discrete, countable edit operations** — edit sites, files touched, hand-derived coordinate
transforms, and synchronized value-copies. Those are **expert-independent** and **verifiable
directly from the diffs** (`diff -r config_1 config_2`), so anyone can reproduce the exact
integers. (The earlier TBD wall-clock row and its stopwatch protocol are superseded by this
and removed; they remain in git history.)

**Excluded from the count on BOTH sides (explicitly, so it is on record):**
- **Static visual-mesh boilerplate.** The conveyor `<include><uri>…DeliveryRobotWithConveyor</uri>`
  is identical scenery in the framework world and in this hand baseline. It is *not*
  reconfiguration work and does **not** inflate the baseline. Only what actually changes
  to add a lane is counted: the conveyor **pose**, its **synchronized pose-copies**, the
  **collision primitive**, and the **task ops**.
- **Comment lines** (`#`, `<!-- -->`). Counted as 0 on both sides.

Numbers below are **measured from the actual diffs** (`diff config_1 config_2`), not
estimated. Any unmeasured value is left **TBD**. The "functional lines added" counts are
reproduced by dropping comment lines (`#`, `<!--`), blank lines, and the static mesh
`<uri>` line from the `>` side of each file's diff (the `<world name>` change is a
*modification*, counted in its own row, not as an addition):

```
for f in world.sdf scene.yaml task.yaml; do
  diff config_1/$f config_2/$f | grep '^>' | sed 's/^> //' \
    | grep -vE '^\s*$|^\s*#|<!--|<uri>|<world name=' | wc -l
done
# -> world 5, scene 3, task 2  = 10
```

## Measured result — adding one lane (config_1 → config_2)

| Structural metric | Framework (1 YAML) | Rigid baseline (conveyor) |
|---|---:|---:|
| **Artifacts / files touched** | **1** (`config_2.yaml`) | **3** (`world.sdf`, `scene.yaml`, `task.yaml`) |
| **Distinct edit sites (diff hunks)** | 2 (append station + append ops, same file) | 4 (1 `<world name>` modify + 3 lane appends across 3 files) |
| **Synchronized copies of the NEW conveyor pose** (hand-kept-consistent, different frames) | **1** (single source; the other 2 frames are *derived* by the generator) | **3** — WORLD `<pose>0.0736 -0.3626 … -3.1036>`, base_link BOX `{0.7706, 0.2804, …}`, base_link target `{0.516875, 0.188077, …}`; **3 different frames, 3 independent hand transforms** |
| **Re-stated copies of an EXISTING pose** (re-grasp pick) | 0 (generator emits it) | 1 (conveyor_2's literal duplicated in `task.yaml`) |
| **Task-wiring ops added** | 2 (same file as the topology) | 2 (separate `task.yaml`) |
| **Modified lines** | **0** (append-only, INVARIANT 5) | 1 (`<world name>` config_1→config_2) |
| **Functional lines added** (excl. comments + mesh scenery) | **3** | **10** (world 5 + scene 3 + task 2) |

### The un-dismissable claim
To add one lane, the framework author writes the conveyor's pose **once**
(`config_2.yaml:15`); the generator derives the scene-collision pose and the grasp target.
The rigid baseline author must write that pose as **three hand-synchronized copies in
three different coordinate frames** (world, scene, task), each needing its own transform
(robot-mount subtraction; inward-shift + grasp-z; slab-z + yaw→quaternion) — plus
re-state conveyor_2's pose a fourth time for the re-grasp. Any one of those copies drifting
out of sync silently breaks the cell. That cross-file, cross-frame synchronization is the
cost the single-source-of-truth design removes.

### Reconfiguration effort as edit operations (supporting context — NOT the headline)

The structural-coupling table above stays the headline. This supporting table recasts the
**same** config_1 → config_2 change as countable **edit operations** — replacing the old
wall-clock measure with an expert-independent count that is reproducible directly from the
diffs. Every cell is an **exact integer read from the actual files** (`diff -r config_1
config_2`), no estimates and no times.

| Measure (adding one lane, config_1 → config_2) | Framework (1 YAML + generator) | Rigid baseline (3 hand files) |
|---|---:|---:|
| **Edit operations** (distinct diff hunks) | **2** | **4** |
| **Files touched** | **1** | **3** |
| **Hand-derived coordinate transforms** | **0** | **2** |
| **Synchronized value-copies** (pose instances hand-kept consistent) | **1** | **4** |

**How each integer is derived from the diffs (verify, don't trust):**
- **Edit operations** = number of distinct change hunks in `diff config_1/* config_2/*`.
  *Framework:* 2 hunks in `config_2.yaml` — append the `conveyor_3` station line, append the
  two task ops (`config_1.yaml→config_2.yaml`: hunks `14a15` + `17a19,20`). The framework
  then runs `cell_generator` **once** — a single mechanical command that authors **no**
  values (it derives the scene/task literals), so it adds no hand-edit and no transform.
  *Baseline:* 4 hunks across 3 files — `world.sdf` 2 (`25c25` rename `<world name>` +
  `39a…` insert the `conveyor_3` `<include>`), `scene.yaml` 1 (insert the BOX), `task.yaml`
  1 (insert the re-grasp pick + place).
- **Files touched** = `config_2.yaml` only (1) vs `world.sdf` + `scene.yaml` + `task.yaml` (3).
- **Hand-derived coordinate transforms** = new-lane pose values a human must compute by hand.
  *Framework:* 0 — the generator derives every frame from the one authored `{x,y,yaw}`.
  *Baseline:* 2 — the world pose `0.0736 -0.3626 … -3.1036` (`world.sdf:47`) must be
  hand-transformed into (a) the base_link BOX `{0.7706, 0.2804, …, qz −0.99982, qw 0.0190}`
  for `scene.yaml:18` (robot-mount subtraction + slab-z + yaw→quaternion) and (b) the
  base_link grasp target `{0.516875008, 0.188076502, …}` for `task.yaml:16` (mount
  subtraction + inward-shift + grasp-z) — two independent hand transforms.
- **Synchronized value-copies** = hand-written pose instances that must agree or the cell
  silently breaks. *Framework:* 1 — `conveyor_3`'s pose is written once
  (`config_2.yaml:15`); the re-grasp references `conveyor_2` by `id` (no value copied).
  *Baseline:* 4 — the new `conveyor_3` pose appears as **3** instances (`world.sdf:47`,
  `scene.yaml:18`, `task.yaml:16`) **plus** `conveyor_2`'s literal re-stated for the re-grasp
  pick (`task.yaml:15`, a 4th copy of an existing pose at `task.yaml:11`).

Net: adding one lane costs the framework **2 appends + 1 mechanical generator run, 1 file,
0 transforms, 1 authored pose**; the rigid baseline costs **4 hand edits across 3 files,
2 hand-derived transforms, and 4 mutually-consistent pose copies**. The gap is structural,
not a matter of typing speed — which is exactly why it is reported as operations, not seconds.

## Files
- `config_1/{world.sdf, scene.yaml, task.yaml}` — frozen config_1 (the rigid baseline).
- `config_2/{world.sdf, scene.yaml, task.yaml}` — the same three files hand-edited to add
  the lane. `diff -r config_1 config_2` reproduces the baseline-side numbers above.

The box-fixture `reference_baseline/` package is kept for now and will be retired once
these conveyor numbers are confirmed clean.
