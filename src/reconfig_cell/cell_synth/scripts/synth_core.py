#!/usr/bin/env python3
"""cell_synth core — Rung 1 valid-config synthesis (oracle + proposer + surrogate).

This module REUSES, never reimplements:
  * the IK reachability guard  (cell_scene_manager/scene_manager) — imported as `GUARD`,
    its `reachable()`/`build_collision_object()` and its IK constants (GROUP/TIP/seeds)
    are called verbatim. We do NOT re-derive IK.
  * the generator              (cell_generator/generate)         — imported as `GEN`,
    its `validate()`/`emit_scene()`/`emit_task()`/derivations realize a candidate into the
    locked scene/task schema. We do NOT re-derive geometry.

The oracle is the guard's IK-feasibility check, plus the collision/overlap/bounds checks
the spec's constraint model names (the guard applies collision objects but does not test
overlap/bounds; those are assembled here, additively). Everything runs HEADLESS: a
standalone `moveit.core.planning_scene.PlanningScene` built straight from the robot model —
no Gazebo, no controller_manager, no /joint_states monitor.

NOTHING here optimizes (Rung 1). The joint-travel-time surrogate is computed + logged only.
"""
import importlib.machinery
import importlib.util
import math
import os
import random

import yaml
from geometry_msgs.msg import Pose
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_state import RobotState


# ── reuse: import the locked guard + generator from their installed scripts ──────────
def _load(path, name):
    # the guard/generator entry scripts are extensionless, so use an explicit source loader
    loader = importlib.machinery.SourceFileLoader(name, path)
    spec = importlib.util.spec_from_loader(name, loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)               # runs imports only; main() is __main__-guarded
    return mod


def _script_path(pkg, script):
    """Resolve an installed (or source) path to another package's entry script."""
    from ament_index_python.packages import get_package_prefix
    p = os.path.join(get_package_prefix(pkg), "lib", pkg, script)
    if os.path.exists(p):
        return p
    # source-tree fallback (symlink-install / running from a build tree)
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.normpath(os.path.join(here, "..", "..", pkg, "scripts", script))
    if os.path.exists(src):
        return src
    raise FileNotFoundError(f"cannot locate {pkg}/{script} (looked at {p} and {src})")


GUARD = _load(_script_path("cell_scene_manager", "scene_manager"), "cell_guard")
GEN = _load(_script_path("cell_generator", "generate"), "cell_gen")


# ── small geometry helpers (NEW: bounds + station-station non-overlap) ───────────────
def _obb_corners(cx, cy, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2.0, width / 2.0
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def _axes(corners):
    out = []
    for i in range(4):
        (x0, y0), (x1, y1) = corners[i], corners[(i + 1) % 4]
        ex, ey = x1 - x0, y1 - y0
        n = math.hypot(ex, ey) or 1.0
        out.append((-ey / n, ex / n))          # outward normal of an edge
    return out


def _obb_overlap(a, b):
    """Separating-Axis-Theorem overlap test for two oriented rectangles (footprints)."""
    for ax, ay in _axes(a) + _axes(b):
        amin = min(px * ax + py * ay for px, py in a)
        amax = max(px * ax + py * ay for px, py in a)
        bmin = min(px * ax + py * ay for px, py in b)
        bmax = max(px * ax + py * ay for px, py in b)
        if amax < bmin or bmax < amin:
            return False                       # found a separating axis -> disjoint
    return True


def _to_pose(tp):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(tp["x"]), float(tp["y"]), float(tp["z"])
    pose.orientation.x, pose.orientation.y = float(tp["qx"]), float(tp["qy"])
    pose.orientation.z, pose.orientation.w = float(tp["qz"]), float(tp["qw"])
    return pose


# ── config assembly + realization (REUSES the generator) ─────────────────────────────
def make_cell(doc, name="candidate"):
    """Build the generator's internal `cell` dict from a config doc (mirrors generate.main)."""
    return {"name": name, **doc["cell"], "robot_mount": doc["robot_mount"],
            "belt": doc["belt"], "part": doc["part"],
            "stations": doc["stations"], "task": doc["task"]}


def realize(cell):
    """Run the generator on a cell -> (scene_spec, task_spec, by_id). Raises on invalid schema."""
    by_id = GEN.validate(cell)                  # generator's own validator (may SystemExit)
    scene_spec = yaml.safe_load(GEN.emit_scene(cell, by_id))
    task_spec = yaml.safe_load(GEN.emit_task(cell, by_id))
    return scene_spec, task_spec, by_id


# ── the validity oracle ──────────────────────────────────────────────────────────────
# Physical conveyor footprint for station-station NON-OVERLAP, taken from the actual
# DeliveryRobotWithConveyor collision (two 0.53x0.51 belt boxes at x=+/-0.2 -> ~0.93x0.51).
# Calibrated against the real cell: at this size both reference configs (config_1/config_2,
# ~0.92 m apart at 90 deg) pass, while tightly-packed synthesized layouts (e.g. conveyors
# 0.44 m apart and parallel) are correctly rejected as physically stacking. The earlier
# 0.8x0.4 was too small (let conveyors stack); 1.0x0.5 (the MoveIt grasp slab) too big
# (rejected the valid references).
PHYS_FOOT_LEN, PHYS_FOOT_WID = 0.93, 0.51

DEFAULTS = dict(
    arena_half=1.20,        # world half-extent (m) around the robot mount, per axis
    base_keepout=0.20,      # half-width (m) of a square keep-out at the robot pedestal
    check_collision=True,   # gate IK solutions on planning-scene collisions (excl. grasped fixture)
)


class Oracle:
    """`is_valid(config_doc) -> (bool, reason)` — headless. Reuses GUARD's IK + GEN's geometry."""

    def __init__(self, moveit, arena_half=DEFAULTS["arena_half"],
                 base_keepout=DEFAULTS["base_keepout"],
                 check_collision=DEFAULTS["check_collision"]):
        self.model = moveit.get_robot_model()
        self.scene = PlanningScene(self.model)  # standalone scene from the model (no monitor)
        self.arena_half = float(arena_half)
        self.base_keepout = float(base_keepout)
        self.check_collision = bool(check_collision)
        self.frame = self.scene.planning_frame  # base_link

    def _apply_objects(self, cos, exclude_id=None):
        """Reset the scene and apply every collision object except `exclude_id`."""
        self.scene.remove_all_collision_objects()
        for co in cos:
            if co.id != exclude_id:
                self.scene.apply_collision_object(co)
        self.scene.current_state.update()

    # --- the guard's IK, extended with the collision gate the constraint model names ---
    def _reach_collision_free(self, pose):
        """True iff a joint-limit-respecting IK solution exists that is also collision-free
        against the CURRENT scene. Same seed strategy + constants as GUARD.reachable
        (UP_SEED first, then random over joint limits) — the IK call itself is the guard's."""
        for i in range(GUARD.IK_SEED_TRIES):
            st = RobotState(self.model)
            if i == 0:
                st.set_joint_group_positions(GUARD.GROUP, list(GUARD.UP_SEED))
            else:
                st.set_joint_group_positions(
                    GUARD.GROUP, [random.uniform(-math.pi, math.pi) for _ in range(6)])
            st.update()
            if st.set_from_ik(GUARD.GROUP, pose, GUARD.TIP, 0.05):
                if not self.check_collision or not self.scene.is_state_colliding(
                        st, GUARD.GROUP):
                    return True
        return False

    def is_valid(self, doc, name="candidate"):
        cell = make_cell(doc, name)
        m = cell["robot_mount"]
        stations = cell["stations"]

        # (1) arena bounds — every station inside the arena box around the mount
        for s in stations:
            dx, dy = s["pose"]["x"] - m["x"], s["pose"]["y"] - m["y"]
            if abs(dx) > self.arena_half or abs(dy) > self.arena_half:
                return False, f"bounds:{s['id']} outside arena"

        # (2) non-overlap — physical conveyor footprints vs each other AND vs the base keep-out
        foot = [(_obb_corners(s["pose"]["x"], s["pose"]["y"], s["pose"]["yaw"],
                              PHYS_FOOT_LEN, PHYS_FOOT_WID), s["id"]) for s in stations]
        base_box = _obb_corners(m["x"], m["y"], 0.0,
                                2 * self.base_keepout, 2 * self.base_keepout)
        for corners, sid in foot:
            if _obb_overlap(corners, base_box):
                return False, f"overlap:{sid} on robot base"
        for i in range(len(foot)):
            for j in range(i + 1, len(foot)):
                if _obb_overlap(foot[i][0], foot[j][0]):
                    return False, f"overlap:{foot[i][1]}~{foot[j][1]}"

        # (3) realize via the generator, then the guard's IK oracle on each derived target.
        # Collision is checked against the robot body + the OTHER stations' slabs, EXCLUDING
        # the slab currently being grasped (approaching the target fixture is required, not a
        # collision) — this is "collision against the already-present scene" per the spec.
        try:
            scene_spec, task_spec, _ = realize(cell)
        except SystemExit as e:
            return False, f"schema:{e}"
        cos = [GUARD.build_collision_object(entry, self.frame)
               for entry in scene_spec.get("collision_objects", [])]

        checked = set()
        for op in task_spec.get("ops", []):
            sid = op["station"]
            if sid in checked:
                continue
            checked.add(sid)
            self._apply_objects(cos, exclude_id=sid if self.check_collision else None)
            if not self._reach_collision_free(_to_pose(op["target_pose"])):
                return False, f"reach:{sid} unreachable or colliding"
        return True, "valid"

    # --- Rung-1 surrogate (kept for provenance): chains from one seed -> NONDETERMINISTIC
    # when a chained seed is poor and the IK solver random-restarts. Superseded by
    # surrogate_det for optimization. ---
    def joint_travel_surrogate(self, doc, name="candidate"):
        """Sum of slowest-joint angular travel across the pick->place target sequence.
        One IK solution per target (UP_SEED start); proxy for trajectory time. Logged only."""
        cell = make_cell(doc, name)
        _, task_spec, _ = realize(cell)
        prev = list(GUARD.UP_SEED)
        total = 0.0
        for op in task_spec.get("ops", []):
            st = RobotState(self.model)
            st.set_joint_group_positions(GUARD.GROUP, prev)
            st.update()
            if not st.set_from_ik(GUARD.GROUP, _to_pose(op["target_pose"]), GUARD.TIP, 0.05):
                return None                      # incomplete -> honestly report TBD
            q = list(st.get_joint_group_positions(GUARD.GROUP))
            total += max(abs(a - b) for a, b in zip(q, prev))
            prev = q
        return total

    # --- DETERMINISTIC surrogate (Rung 3 objective). For each target, pick the
    # MIN-slowest-joint-travel IK solution over a FIXED seed bank (same seeds every call).
    # The min over a diverse fixed bank is stable even though a single solve can restart,
    # because the best branch is reliably found by several seeds -> same score in/out. ---
    def _canonical_ik(self, pose, seedbank):
        """The CANONICAL IK solution for a target: the one closest to the elbow-up home
        UP_SEED over a fixed seed bank. set_from_ik is nondeterministic when it restarts,
        and picking by travel-from-prev jumps between elbow-up/down branches; anchoring to
        UP_SEED keeps every target in the SAME (elbow-up) branch the executor uses, so the
        chosen joint vector is stable to within IK tolerance -> deterministic score."""
        up = list(GUARD.UP_SEED)
        best = None
        for seed in seedbank:
            st = RobotState(self.model)
            st.set_joint_group_positions(GUARD.GROUP, seed)
            st.update()
            if st.set_from_ik(GUARD.GROUP, pose, GUARD.TIP, 0.05):
                q = list(st.get_joint_group_positions(GUARD.GROUP))
                d = sum((a - b) ** 2 for a, b in zip(q, up))     # distance to home branch
                if best is None or d < best[0]:
                    best = (d, q)
        return None if best is None else best[1]

    def _seedbank(self, n_ik, ik_seed):
        rng = random.Random(ik_seed)             # local RNG -> identical bank every call
        return [list(GUARD.UP_SEED)] + [
            [rng.uniform(-math.pi, math.pi) for _ in range(6)] for _ in range(n_ik - 1)]

    def surrogate_det(self, doc, name="candidate", task_spec=None, n_ik=40, ik_seed=20240615):
        """Deterministic joint-travel surrogate over the FULL ordered pick->place sequence.
        Sum of slowest-joint travel between consecutive CANONICAL (elbow-up) IK solutions,
        starting from home. Order-dependent (visit order changes the consecutive pairs).
        Returns None if any target is unreachable (caller treats as invalid)."""
        if task_spec is None:
            _, task_spec, _ = realize(make_cell(doc, name))
        seedbank = self._seedbank(n_ik, ik_seed)
        prev = list(GUARD.UP_SEED)
        total = 0.0
        for op in task_spec.get("ops", []):
            q = self._canonical_ik(_to_pose(op["target_pose"]), seedbank)
            if q is None:
                return None
            total += max(abs(a - b) for a, b in zip(q, prev))
            prev = q
        return total


# ── the proposer (constrained sampling, biased to the UR5 reach annulus) ─────────────
class Proposer:
    """Seedable sampler of candidate configs. Station centers are drawn in a base-frame
    annulus [r_min, r_max] so the derived (inward-shifted) grasp target lands inside the
    UR5 reach envelope -> high valid-hit rate (NOT uniform over the arena)."""

    def __init__(self, base_doc, seed, r_min=0.60, r_max=0.85,
                 th_lo=-math.pi, th_hi=math.pi, yaw_radial=True):
        self.base = base_doc
        self.seed = seed
        self.rng = random.Random(seed)
        self.r_min, self.r_max = float(r_min), float(r_max)
        self.th_lo, self.th_hi = float(th_lo), float(th_hi)
        self.yaw_radial = yaw_radial
        self.mount = base_doc["robot_mount"]

    def _station(self, idx):
        # sqrt-uniform radius for area-uniform sampling within the annulus
        u = self.rng.random()
        r = math.sqrt(self.r_min**2 + u * (self.r_max**2 - self.r_min**2))
        th = self.rng.uniform(self.th_lo, self.th_hi)
        bx, by = r * math.cos(th), r * math.sin(th)              # base frame
        wx, wy = bx + self.mount["x"], by + self.mount["y"]      # world frame
        # belt facing the robot (radial) by default, else random orientation
        yaw = math.atan2(-by, -bx) if self.yaw_radial else self.rng.uniform(-math.pi, math.pi)
        return {"id": f"conveyor_{idx}", "type": "conveyor",
                "pose": {"x": round(wx, 6), "y": round(wy, 6), "yaw": round(yaw, 6)}}

    @staticmethod
    def _relay_task(n):
        """Existing relay pattern: pick c1, place c2, pick c2, place c3, ... (carry-through)."""
        ops = [{"op": "pick", "from": "conveyor_1"}, {"op": "place", "to": "conveyor_2"}]
        for k in range(3, n + 1):
            ops += [{"op": "pick", "from": f"conveyor_{k-1}"},
                    {"op": "place", "to": f"conveyor_{k}"}]
        return ops

    def sample(self, n_stations):
        """Draw one full candidate config doc in the locked schema."""
        return {
            "cell": dict(self.base["cell"]),
            "robot_mount": dict(self.base["robot_mount"]),
            "belt": dict(self.base["belt"]),
            "part": dict(self.base["part"]),
            "stations": [self._station(i + 1) for i in range(n_stations)],
            "task": self._relay_task(n_stations),
        }


# ── visit order <-> task, and SA state helpers ──────────────────────────────────────
def relay_from_order(order):
    """Carry-through relay over an ordered station list: pick o0, place o1, pick o1, ..."""
    ops = [{"op": "pick", "from": order[0]}, {"op": "place", "to": order[1]}]
    for k in range(2, len(order)):
        ops += [{"op": "pick", "from": order[k - 1]}, {"op": "place", "to": order[k]}]
    return ops


def doc_from_state(base, stations, order):
    """Build a schema config doc from SA decision variables (station poses + visit order)."""
    return {"cell": dict(base["cell"]), "robot_mount": dict(base["robot_mount"]),
            "belt": dict(base["belt"]), "part": dict(base["part"]),
            "stations": [{"id": s["id"], "type": "conveyor",
                          "pose": dict(s["pose"])} for s in stations],
            "task": relay_from_order(order)}


# ── Rung 3: simulated annealing over (station poses x visit order) ───────────────────
class Annealer:
    """SA minimizing the deterministic joint-travel surrogate over the mixed decision
    space: continuous station (x,y,yaw) nudges + discrete visit-order swaps. Reuses the
    Rung-1 oracle for validity (every accepted state stays valid) and surrogate_det as the
    objective. Seedable; logs the cost trajectory. NO change to any locked package."""

    def __init__(self, oracle, base_doc, seed, n_stations, iters=400, t0=2.0,
                 cooling=0.99, step_xy=0.10, step_yaw=0.5, p_cont=0.7, p_jump=0.35,
                 r_min=0.60, r_max=0.85, n_ik=40, init_tries=12):
        self.o = oracle
        self.base = base_doc
        self.rng = random.Random(seed)
        self.n = n_stations
        self.iters, self.t0, self.cooling = iters, t0, cooling
        self.step_xy, self.step_yaw, self.p_cont = step_xy, step_yaw, p_cont
        self.p_jump, self.r_min, self.r_max = p_jump, r_min, r_max
        self.n_ik, self.init_tries = n_ik, init_tries
        self.hp = dict(iters=iters, t0=t0, cooling=cooling, step_xy=step_xy,
                       step_yaw=step_yaw, p_cont=p_cont, p_jump=p_jump, n_ik=n_ik,
                       init_tries=init_tries, seed=seed, n_stations=n_stations)

    def _cost(self, stations, order):
        """Validity-gated objective: invalid -> None (rejected); valid -> det surrogate."""
        doc = doc_from_state(self.base, stations, order)
        ok, _ = self.o.is_valid(doc, "sa")
        if not ok:
            return None
        return self.o.surrogate_det(doc, "sa", n_ik=self.n_ik)

    def _initial(self):
        """Start from the BEST of `init_tries` random valids (reuse Rung-1 sampler) so SA
        does not begin (and get cooled) in a poor basin; order = identity."""
        prop = Proposer(self.base, seed=self.rng.randint(0, 10**9))
        best = None
        seen = 0
        for _ in range(2000):
            if seen >= self.init_tries and best is not None:
                break
            doc = prop.sample(self.n)
            ok, _ = self.o.is_valid(doc, "sa0")
            if not ok:
                continue
            c = self.o.surrogate_det(doc, "sa0", n_ik=self.n_ik)
            if c is None:
                continue
            seen += 1
            if best is None or c < best[2]:
                stations = [{"id": s["id"], "pose": dict(s["pose"])} for s in doc["stations"]]
                order = [s["id"] for s in doc["stations"]]
                best = (stations, order, c)
        if best is None:
            raise RuntimeError("could not find a valid initial state")
        return best

    def _resample_pose(self):
        """Fresh station pose in the UR5 reach annulus (global jump move), belt radial."""
        u = self.rng.random()
        r = math.sqrt(self.r_min**2 + u * (self.r_max**2 - self.r_min**2))
        th = self.rng.uniform(-math.pi, math.pi)
        bx, by = r * math.cos(th), r * math.sin(th)
        m = self.base["robot_mount"]
        return {"x": round(bx + m["x"], 6), "y": round(by + m["y"], 6),
                "yaw": round(math.atan2(-by, -bx), 6)}

    def _neighbor(self, stations, order, frac):
        """Propose a neighbor. Move aggressiveness anneals with temperature fraction
        `frac`=T/T0: EARLY (frac~1) global station 'jumps' + large nudges explore; LATE
        (frac~0) jumps fade and nudges shrink so SA refines the best basin. Plus discrete
        visit-order swaps throughout. Explore-early/exploit-late is what lets SA beat the
        random-valid floor (which has neither refinement nor order-opt)."""
        stations = [{"id": s["id"], "pose": dict(s["pose"])} for s in stations]
        order = list(order)
        if self.n >= 3 and self.rng.random() > self.p_cont:
            i, j = self.rng.sample(range(self.n), 2)          # discrete: swap visit order
            order[i], order[j] = order[j], order[i]
            return stations, order, "swap"
        s = self.rng.choice(stations)
        if self.rng.random() < self.p_jump * frac:             # global jump (fades when cool)
            s["pose"] = self._resample_pose()
            return stations, order, "jump"
        scale = 0.25 + 0.75 * frac                             # local nudge shrinks when cool
        s["pose"]["x"] = round(s["pose"]["x"] + self.rng.gauss(0, self.step_xy * scale), 6)
        s["pose"]["y"] = round(s["pose"]["y"] + self.rng.gauss(0, self.step_xy * scale), 6)
        s["pose"]["yaw"] = round(s["pose"]["yaw"] + self.rng.gauss(0, self.step_yaw * scale), 6)
        return stations, order, "nudge"

    def run(self):
        stations, order, cost = self._initial()
        best = (cost, [dict(s) for s in stations], list(order))
        traj = [cost]
        T = self.t0
        accepts = 0
        stall = 0                                # iters since last improvement to `best`
        reheats = 0
        for _ in range(self.iters):
            ns, no, _ = self._neighbor(stations, order, frac=T / self.t0)
            nc = self._cost(ns, no)
            improved = False
            if nc is not None:
                d = nc - cost
                if d < 0 or self.rng.random() < math.exp(-d / max(T, 1e-9)):
                    stations, order, cost = ns, no, nc
                    accepts += 1
                    if nc < best[0] - 1e-9:
                        best = (nc, [dict(s) for s in stations], list(order))
                        improved = True
            stall = 0 if improved else stall + 1
            if stall >= 45:                       # reheat to escape a stuck basin
                T = min(self.t0, T * 4.0)
                stall = 0
                reheats += 1
                # restart from the best-so-far when reheating (focus exploration there)
                cost = best[0]
                stations = [dict(s) for s in best[1]]
                order = list(best[2])
            else:
                T *= self.cooling
            traj.append(cost)
        return dict(best_cost=best[0], best_stations=best[1], best_order=best[2],
                    init_cost=traj[0], traj=traj, accepts=accepts, reheats=reheats, hp=self.hp)


# ── emit a synthesized config in the existing schema (drop-in to the generator) ──────
def write_config_yaml(doc, path, provenance):
    m, b, p = doc["robot_mount"], doc["belt"], doc["part"]
    lines = [f"# SYNTHESIZED by cell_synth (Rung 1). {provenance}",
             "# Drop-in to cell_generator in the LOCKED schema; robot_mount/belt/part are",
             "# copied unchanged from the base config (robot/gripper/world are not touched).",
             f"cell: {{robot: {doc['cell']['robot']}, base_frame: {doc['cell']['base_frame']}}}",
             f"robot_mount: {{x: {m['x']}, y: {m['y']}, z: {m['z']}}}",
             f"belt: {{top_z: {b['top_z']}, inward: {b['inward']}}}",
             f"part: {{name: {p['name']}, geometry: {{type: {p['geometry']['type']}, "
             f"size: {list(p['geometry']['size'])}}}}}",
             "stations:"]
    for s in doc["stations"]:
        sp = s["pose"]
        lines.append(f"  - {{id: {s['id']}, type: {s['type']}, "
                     f"pose: {{x: {sp['x']}, y: {sp['y']}, yaw: {sp['yaw']}}}}}")
    lines.append("task:")
    for op in doc["task"]:
        if op["op"] == "pick":
            lines.append(f"  - {{op: pick,  from: {op['from']}}}")
        else:
            lines.append(f"  - {{op: place, to:   {op['to']}}}")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
