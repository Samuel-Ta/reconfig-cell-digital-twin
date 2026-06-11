# reference_baseline — frozen config_1 monolithic baseline

This is the **rigid reference baseline** (spec §10): the hand-maintained
config_1 cell as **three separate artifacts** that must be kept in sync by hand.
It is the comparison point for the Phase-7 effort metric — the "rigid" side that
the declarative framework (one YAML + generator) is measured against.

## The three frozen artifacts (`config_1/`)

| File | Spec output | Holds |
|------|-------------|-------|
| `world.sdf`  | §7 #1 | Gazebo station fixtures + source parts (4 fixtures, 2 cubes) |
| `scene.yaml` | §7 #2 | MoveIt collision objects (the 4 fixtures, as BOX) |
| `task.yaml`  | §7 #3 | Pick/place ops with **hardcoded** target pose literals |

The same pick/place geometry (each station pose, each fixture size, each derived
target) appears **independently in all three files**. On the rigid path there is
no single source of truth — that duplication is the point.

## What is deliberately NOT here

- **The warehouse decorative backdrop.** It lives once in
  `cell_bringup/worlds/warehouse_backdrop.sdf` as a SEPARATE, constant base-world
  that the station models are spawned into at launch. It is identical across
  config_1 and config_2, so it **never appears in the config-to-config diff** and
  never pollutes the effort metric. Do not copy it into these files.
- **The UR5 + gripper.** Spawned by the bringup launch on top of the scene
  (INVARIANT 1: stations are environment, not part of the robot).

## Reconfiguring to config_2 (the cost the metric measures)

On this rigid path, adding lane 3 means hand-editing **all three files**:

1. `world.sdf` — append `source_3` (0.54, 0.0) + `sink_3` (0.32, 0.0) fixtures + `cube_3`.
2. `scene.yaml` — append `source_3` + `sink_3` collision objects.
3. `task.yaml` — append `pick source_3` + `place sink_3` with new hand-computed
   target literals.

Three artifacts, scattered edit regions, three synchronized copies of each new
pose. The framework path (Phase C) does the same reconfiguration as a single
append to one YAML, then regenerates. Measure both; report numbers only after
building (spec §10 — do not fabricate counts).

## Verification status

- Geometry/poses are the locked spec §8 config_1 values.
- Lane 1 pick→place verified end-to-end in Phase A (cube placed at sink_1 exactly:
  0.250, 0.550, 0.140). Lane 2 is the mirror across y=0; its targets are IK-reachable
  (checked at freeze). Full 2-lane execution runs via the config-agnostic
  `cell_task_executor` in Phase D, which consumes a `task.yaml` of this same schema.
