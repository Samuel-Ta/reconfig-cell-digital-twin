# cell_synth — Rung 1: automatic valid-config synthesis

Additive synthesis layer on top of the reconfigurable cell. It **invents** cell
configurations (station layouts) and keeps only the ones that are reachable,
collision-free, non-overlapping and in-bounds — by **reusing the existing IK guard as the
validity oracle and the existing generator as the realizer**. Nothing about the robot,
gripper, world, generator, guard, or schema is changed (synthesis spec INVARIANTs 1–6).

Rung 1 finds *valid* configs only. **No optimization** — the joint-travel surrogate is
computed and logged for later (Rung 3), never optimized on.

## What it reuses (imported, never reimplemented)

- **IK guard** `cell_scene_manager/scene_manager` → imported as `GUARD`. Its `reachable()`
  IK constants (`GROUP`, `TIP`, `UP_SEED`, `IK_SEED_TRIES`) and `build_collision_object()`
  are called verbatim; the oracle's reach check *is* the guard's `set_from_ik`.
- **Generator** `cell_generator/generate` → imported as `GEN`. Its `validate()` +
  `emit_scene()`/`emit_task()` + derivations realize a candidate into the locked
  scene/task schema. No geometry is re-derived here.

Both are loaded from their installed entry scripts via `SourceFileLoader` (they are
extensionless and `__main__`-guarded), so importing runs no `main()` and edits neither file.

## The oracle (headless)

`Oracle.is_valid(config_doc) -> (bool, reason)` runs with **no Gazebo, no controller_manager,
no GL** — it builds a standalone `moveit.core.PlanningScene` straight from the robot model
(MoveItPy loads only the model + kinematics from launch params). Checks, in order:

1. **arena bounds** — each station within `arena_half` (1.20 m) of the robot mount;
2. **non-overlap** — physical conveyor footprints (`0.80×0.40`, the chassis — deliberately
   smaller than the generator's conservative `1.0×0.5` MoveIt grasp slab, whose adjacent
   copies legally overlap in config_1/config_2) vs each other and a base keep-out;
3. **reach + collision** — the guard's IK on each generator-derived target, gated on the
   planning-scene collision check against the robot body + the *other* stations' slabs
   (the grasped fixture is excluded — approaching it is required, not a collision).

## The proposer (constrained sampling)

`Proposer(base_doc, seed)` samples station centers in a base-frame annulus `[0.60, 0.85] m`
(area-uniform) so the inward-shifted grasp target lands inside the UR5 reach envelope →
high hit-rate (not uniform over the arena). Seedable and reproducible; robot_mount/belt/part
are copied unchanged from the base config (`config_1`).

## Run it

```bash
# STEP 1 — prove the oracle is headless + correct (config_1 valid, config_invalid invalid)
ros2 launch cell_synth synth.launch.py exe:=oracle_smoke

# synthesize K valid configs (logs seed/attempts/hit-rate/wall-time + surrogate)
ros2 launch cell_synth synth.launch.py exe:=synthesize k:=5 n_stations:=3 seed:=7 \
    out_dir:=<dir>

# end-to-end sanity: run a synthesized config on the EXISTING twin (one full relay)
ros2 run cell_generator generate --config <synth.yaml> --out <gen>
ros2 launch cell_synth synth_demo.launch.py cfg:=<synth.yaml> \
    scene:=<gen>/<name>/scene.yaml task:=<gen>/<name>/task.yaml
```

## Measured Rung-1 results (this machine, headless)

- **Oracle headless** — MoveItPy + oracle up in **0.1 s** with `DISPLAY` unset, no Gazebo,
  no controller_manager. Verdicts match the known guard: `config_1` → **valid**,
  `config_invalid` → **invalid**.
- **Synthesis** (`n_stations=3`, biased annulus): all runs emitted **5/5** valid configs.
  | seed | valid | attempts | hit-rate | ms/attempt |
  |---|---|---|---|---|
  | 7  | 5/5 | 32 | 0.156 | ~5 |
  | 11 | 5/5 | 38 | 0.132 | ~3–6 |
  Reproducible: re-running seed 7 reproduces the same attempts `(9,4,16,1,2)` and the same
  emitted configs. (The logged *surrogate* varies run-to-run because the IK solver has its
  own internal RNG; the accept/reject verdict does not.)
- **End-to-end** — `synth_s7_n3_3` (3 conveyors at synthesized poses) ran a full relay on
  the existing twin: IK guard passed all 3 stations, then
  pick conveyor_1 → place conveyor_2 → re-grasp conveyor_2 → **place conveyor_3**, every
  grasp confirmed. (`task_executor`/`move_group` exit −11 = the documented moveit_py
  shutdown segfault, after all ops complete — same as the locked N=30 harness.)

Raw artifacts in `../synth_out/`: the synthesized configs, `synth_summary_s*_n*.csv`, and
the demo's generated scene/task.
