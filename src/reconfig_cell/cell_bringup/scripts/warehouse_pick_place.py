#!/usr/bin/env python3
"""
Warehouse cell — conveyor->conveyor pick->place using moveit_py + the
DetachableJoint grasp-fix.

This is the warehouse-cell analogue of step2_pick_place.py: same hand-driven
demonstrator and the same hard-won IK/grasp machinery, but the source and sink
are two of the three delivery conveyors arced around the cell (see
warehouse_cell.launch.py + cell_in_warehouse.world) instead of the static lane
fixtures. It picks a cube off one conveyor belt and places it on another.

Configs (which conveyor is source vs sink) — default mapping:
  config_1:  source = delivery_conveyor_1   ->  sink = delivery_conveyor_3
  config_2:  source = delivery_conveyor_3   ->  sink = delivery_conveyor_1
Select with --config 1|2 (default 1).

Frames / geometry (all hand-measured against the running sim):
  The robot (model name "ur5_rg2", here the arm-only UR5) is spawned at world
  (CELL_X, CELL_Y) with yaw 0, so base_link == world translated by that offset:
      base = world + (+0.697, +0.643)            (no rotation)
  The conveyor is the Open-RMF DeliveryRobotWithConveyor: its belt collision box
  (cube005_collider_box, size z=0.4) is centred at z=0.5 in the conveyor link
  (link at model origin), so the belt TOP face sits at world z = 0.70.
  There is no gripper geometry below tool0 (RG2 is cosmetic in Gazebo and the
  grasp is the DetachableJoint to wrist_3_link), so the grasp frame is tool0
  itself: ATTACH_OFFSET = 0 and the cube welds at the flange.

  The belt top (0.70) at the conveyor-centre radius (~0.60 m) is at the very edge
  of the UR5's reach for a top-down grasp, so we target a point shifted INWARD
  (toward the base) from the belt centre — still on the belt, but reachable.

Grasp/release is the DetachableJoint on cube_part_rg2.sdf (child_model=ur5_rg2):
  grasp   = publish gz.msgs.Empty to /cube_1/attach
  release = publish gz.msgs.Empty to /cube_1/detach

Prereq: warehouse_cell.launch.py is already running (Gazebo + warehouse world +
controllers + robot_state_publisher + move_group). The cube is spawned by this
script unless --no-spawn.
"""

import math
import random
import subprocess
import sys
import time

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import Pose, PoseStamped
from moveit.planning import MoveItPy

# ── cell + conveyor geometry ──────────────────────────────────────────────────
# Robot spawn pose in the world (must match warehouse_cell.launch.py CELL_X/Y, yaw 0).
CELL_X, CELL_Y = -0.697, -0.643

# Delivery conveyors: world (x, y) of the model origin == belt centre in plan view.
CONVEYORS_WORLD = {
    1: (-1.44596, -0.364014),  # user-placed: bearing 160 deg, R=0.80
    2: (-0.697,    0.177),     # moved IN to bearing 90 deg, R=0.82 (reachable for the relay)
    3: ( 0.0736,  -0.3626),    # user-placed: bearing  20 deg, R=0.82
}

BELT_TOP_Z = 0.70            # belt top face, world z
ROBOT_MOUNT_Z = 0.40         # UR5 base raised onto a pedestal (world z of base_link).
                             # Targets are expressed in base_link, so a higher mount
                             # LOWERS the belt's z in base frame -> brings the 0.70 m
                             # belt into the floor-UR5's top-down reach envelope.
CUBE_HALF = 0.02             # 40 mm cube
INWARD = 0.27                # shift target toward base to the belt's NEAR edge:
                             # conveyors now at R=0.82, so pick at 0.82-0.27=0.55 m
                             # (within the ~0.60 m top-down reach to the 0.70 m belt)

ATTACH_OFFSET = 0.0          # tool0 IS the grasp frame (no gripper below it)
APPROACH_CLEAR = 0.15        # approach/retreat clearance above target

CUBE_CENTER_Z = BELT_TOP_Z + CUBE_HALF - ROBOT_MOUNT_Z   # belt-frame z, base_link: 0.32
GRASP_TOOL0_Z = CUBE_CENTER_Z + ATTACH_OFFSET            # tool0 z at the grasp (base frame)
CUBE_SPAWN_WORLD_Z = BELT_TOP_Z + CUBE_HALF              # cube sits on the belt in WORLD frame: 0.72
                                                          # (gz spawns/reports in world frame, not base)

# config -> conveyor PATH the box travels: pick at path[i], place at path[i+1].
# config_2 = config_1 with one hop appended (append-only reconfiguration story).
CONFIGS = {
    1: [1, 2],       # one hop:  conv1 -> conv2          (conv3 unused)
    2: [1, 2, 3],    # relay:    conv1 -> conv2 -> conv3
}

CUBE_NAME = "cube_1"

# top-down: tool0 +z points to world -z  => roll = pi  => quat (1,0,0,0)
TOPDOWN_QUAT = (1.0, 0.0, 0.0, 0.0)  # (x, y, z, w)


def world_to_base(wx, wy):
    """Conveyor world (x, y) -> robot base_link (x, y). Robot at (CELL_X,CELL_Y), yaw 0."""
    return (wx - CELL_X, wy - CELL_Y)


def belt_target(conveyor_id):
    """base_link (x, y) on the belt of `conveyor_id`, shifted INWARD toward the base
    so the 0.70 m belt top stays within the UR5's top-down reach."""
    bx, by = world_to_base(*CONVEYORS_WORLD[conveyor_id])
    r = math.hypot(bx, by)
    if r < 1e-6:
        return bx, by
    # unit vector from belt centre toward the base origin, scaled by INWARD
    return (bx - INWARD * bx / r, by - INWARD * by / r)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def topdown_quat(yaw):
    """tool0 +z points to world -z (roll=pi), spun by `yaw` about world z."""
    q_roll = (1.0, 0.0, 0.0, 0.0)                       # Rx(pi)
    q_yaw = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))  # Rz(yaw)
    return quat_mul(q_yaw, q_roll)


def gz_pub(topic, times=3, gap=0.4):
    """Publish empty gz msgs (attach/detach trigger). Repeated because a single
    publish can be missed depending on sim-step timing (esp. with the GUI running)."""
    for _ in range(times):
        subprocess.run(
            ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""],
            check=False,
        )
        time.sleep(gap)


# RG2 finger open/close (gz JointPositionController, topics /rg2/fingerN_cmd, gz.msgs.Double).
# Sign (confirmed visually): POSITIVE spreads the fingers OPEN, NEGATIVE folds them IN.
# Both controllers actuate symmetrically (~equal, opposite yaw), so we don't tune a precise
# grip angle: we OPEN to clear the cube, then drive BOTH hard toward fully-closed and let
# the cube itself arrest them. The box has collision, so the fingers fold until they press
# its faces; the near finger nudges the box until the far one also contacts -> the box
# SELF-CENTERS and both fingers clamp symmetrically, then the weld holds it for the carry.
#   OPEN  = +0.65 -> fingers spread to descend around the 40 mm cube
#   CLOSE = -1.10 -> firm fold; the cube (not the angle) is the mechanical limit
FINGER_OPEN = 0.65
FINGER_CLOSE = -1.10


def set_fingers(cmd, settle=1.0):
    """Drive both RG2 finger position controllers to `cmd` rad and let them settle.

    Publish each command SEVERAL times (like gz_pub): a single gz-topic publish is
    routinely missed depending on sim-step timing, which previously left ONE finger
    on its old command -> only one finger folded. Repeating to BOTH topics makes the
    open/close land on both fingers so they actuate symmetrically."""
    for _ in range(4):
        for t in ("/rg2/finger1_cmd", "/rg2/finger2_cmd"):
            subprocess.run(["gz", "topic", "-t", t, "-m", "gz.msgs.Double", "-p", f"data: {cmd}"],
                           check=False)
        time.sleep(0.15)
    time.sleep(settle)


def cube_z():
    """Current cube z (m), or None."""
    import re
    r = subprocess.run(["gz", "model", "-m", CUBE_NAME, "-p"],
                       capture_output=True, text=True, timeout=8)
    m = re.search(r"\[\s*[-\d.]+\s+[-\d.]+\s+([-\d.]+)\s*\]", r.stdout)
    return float(m.group(1)) if m else None


def spawn_cube(logger, sx, sy):
    from ament_index_python.packages import get_package_share_directory
    import os
    sdf = os.path.join(
        get_package_share_directory("cell_bringup"), "models", "cube_part_rg2.sdf"
    )
    # idempotent: drop any cube from a previous run and WAIT until it is actually
    # gone before respawning (a too-short delay races `create` against a not-yet-
    # removed cube_1 -> name clash -> stale cube stays, fresh one missing).
    def cube_exists():
        r = subprocess.run(["gz", "model", "-m", CUBE_NAME, "-p"],
                           capture_output=True, text=True, timeout=8)
        return "Pose" in r.stdout
    if cube_exists():
        # ros_gz_sim remove takes the name as ROS param `entity_name`, NOT a -name flag.
        subprocess.run(["ros2", "run", "ros_gz_sim", "remove",
                        "--ros-args", "-p", f"entity_name:={CUBE_NAME}"],
                       check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(20):
            time.sleep(0.3)
            try:
                if not cube_exists():
                    break
            except Exception:
                break
    # Spawn the cube at the grasp point (gripper already descended there); the gz
    # DetachableJoint auto-attaches on the first update, so spawning it in place
    # captures a ~zero offset weld rather than flinging the part from a far pose.
    # NOTE: spawn pose is in the WORLD frame (ros_gz_sim create), so convert the
    # base-frame grasp (sx,sy) back to world.
    wx, wy = sx + CELL_X, sy + CELL_Y
    logger.info(f"Spawning {CUBE_NAME} at world ({wx:.3f},{wy:.3f},{CUBE_SPAWN_WORLD_Z}) from {sdf}")
    subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-file", sdf, "-name", CUBE_NAME,
         "-x", str(wx), "-y", str(wy), "-z", str(CUBE_SPAWN_WORLD_Z)],
        check=True,
    )
    time.sleep(1.0)


def move_to_named(arm, robot, logger, name):
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=name)
    return _plan_exec(arm, robot, logger, f"named:{name}")


def solve_ik(model, psm, robot, x, y, z, yaw, seed, logger, label, nominal=None):
    """Collision-checked top-down IK -> goal RobotState (None on failure).

    We command JOINT-SPACE goals from our own IK rather than Cartesian pose goals:
    move_group's pose-goal IK tends to pick joint configs that this gz position
    interface can't hold within goal tolerance (the arm never settles -> the
    controller aborts on goal_time). Seeding IK from the previous config keeps the
    chosen solution benign and continuous. Prefer an ELBOW-UP, tool-down posture
    (shoulder_lift negative): elbow-down solutions swing the lower links into the
    ground plane (the robot is floor-mounted) and jam. Among collision-free,
    un-wrapped, elbow-up solutions, pick the least wrapped (or closest to nominal).
    """
    from moveit.core.robot_state import RobotState

    qx, qy, qz, qw = topdown_quat(yaw)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    pose.orientation.x, pose.orientation.y = qx, qy
    pose.orientation.z, pose.orientation.w = qz, qw

    SHOULDER_LIFT = 1  # index in ur_manipulator joint order
    best_q = best_fallback = None
    best_score = best_fb_score = float("inf")
    tries = 120
    for i in range(tries):
        state = RobotState(model)
        if i == 0:
            state.set_joint_group_positions("ur_manipulator", list(seed))
        else:
            state.set_joint_group_positions(
                "ur_manipulator",
                [random.uniform(-math.pi, math.pi) for _ in range(6)])
        state.update()
        if not state.set_from_ik("ur_manipulator", pose, "rg2_tcp", 0.05):
            continue
        state.update()
        q = list(state.get_joint_group_positions("ur_manipulator"))
        with psm.read_only() as scene:
            if scene.is_state_colliding(robot_state=state,
                                        joint_model_group_name="ur_manipulator",
                                        verbose=False):
                continue
        if nominal is not None:
            score = sum((a - b) ** 2 for a, b in zip(q, nominal))
        else:
            score = max(abs(v) for v in q)
        if score < best_fb_score:                      # any valid solution
            best_fb_score, best_fallback = score, q
        if q[SHOULDER_LIFT] <= -0.5 and score < best_score:  # elbow-up only
            best_score, best_q = score, q

    if best_q is None:
        best_q, best_score = best_fallback, best_fb_score
    if best_q is None:
        logger.error(f"IK FAILED (no collision-free solution) for {label}")
        return None
    logger.info(f"IK {label}: {[round(v,3) for v in best_q]} (max|q|={best_score:.2f})")
    goal = RobotState(model)
    goal.set_joint_group_positions("ur_manipulator", best_q)
    goal.update()
    return goal


def move_to_state(arm, robot, logger, goal_state, label):
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=goal_state)
    return _plan_exec(arm, robot, logger, label)


def _plan_exec(arm, robot, logger, label, attempts=6):
    # RRTConnect is randomized; a returned path can still be rejected by the
    # ValidateSolution adapter (dense self-collision recheck). Re-plan a few times.
    result = None
    for i in range(1, attempts + 1):
        logger.info(f"Planning -> {label} (try {i}/{attempts})")
        result = arm.plan()
        if result:
            break
    if not result:
        logger.error(f"PLAN FAILED for {label} after {attempts} attempts")
        return False
    logger.info(f"Executing -> {label}")
    robot.execute(result.trajectory, controllers=[])
    time.sleep(0.5)
    return True


UP_SEED = {
    "shoulder_pan_joint": 0.0, "shoulder_lift_joint": -1.5708, "elbow_joint": -1.5708,
    "wrist_1_joint": -1.5708, "wrist_2_joint": 0.0, "wrist_3_joint": 0.0,
}


def run_diag(moveit, logger, path):
    """Sweep top-down wrist yaw to find a collision-free IK for each conveyor on the path."""
    from moveit.core.robot_state import RobotState

    model = moveit.get_robot_model()
    psm = moveit.get_planning_scene_monitor()
    targets = {}
    for cid in path:
        cx, cy = belt_target(cid)
        targets[f"conv{cid}"] = (cx, cy, GRASP_TOOL0_Z)

    for name, t in targets.items():
        logger.info(f"=== {name} base target ({t[0]:.3f},{t[1]:.3f},{t[2]:.3f}) ===")
        for deg in range(0, 360, 30):
            yaw = math.radians(deg)
            qx, qy, qz, qw = topdown_quat(yaw)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = t[0], t[1], t[2]
            pose.orientation.x, pose.orientation.y = qx, qy
            pose.orientation.z, pose.orientation.w = qz, qw

            state = RobotState(model)
            state.set_joint_group_positions("ur_manipulator",
                                            [UP_SEED[j] for j in UP_SEED])
            state.update()
            ok = state.set_from_ik("ur_manipulator", pose, "rg2_tcp", 0.1)
            colliding = None
            if ok:
                with psm.read_only() as scene:
                    colliding = scene.is_state_colliding(
                        robot_state=state, joint_model_group_name="ur_manipulator",
                        verbose=False)
            logger.info(f"  yaw={deg:3d}  ik={'OK ' if ok else 'NO '}  "
                        f"self_collision={colliding}")
    return 0


def _tokens():
    """Flatten argv into individual tokens. The launch passes everything via a single
    `args:=` string (one argv element), so 'in sys.argv' membership and pairwise parsing
    both break for multi-token args like '--config 2 --ikvals' -> re-split on whitespace."""
    return " ".join(sys.argv[1:]).split()


def parse_config(argv):
    cfg = 1
    for i, a in enumerate(argv):
        if a == "--config" and i + 1 < len(argv):
            cfg = int(argv[i + 1])
        elif a.startswith("--config="):
            cfg = int(a.split("=", 1)[1])
    if cfg not in CONFIGS:
        raise SystemExit(f"--config must be one of {sorted(CONFIGS)} (got {cfg})")
    return cfg


def main():
    tokens = _tokens()
    spawn = "--no-spawn" not in tokens
    cfg = parse_config(tokens)
    path = CONFIGS[cfg]                       # conveyor IDs the box travels through

    rclpy.init()
    logger = get_logger("warehouse_pick_place")
    logger.info(f"config_{cfg}: conveyor path " + " -> ".join(f"conv{c}" for c in path))

    moveit = MoveItPy(node_name="warehouse_pick_place")
    arm = moveit.get_planning_component("ur_manipulator")
    logger.info("MoveItPy up; ur_manipulator planning component ready.")

    if "--diag" in tokens:
        return run_diag(moveit, logger, path)

    model = moveit.get_robot_model()
    psm = moveit.get_planning_scene_monitor()
    seed = [UP_SEED[j] for j in UP_SEED]

    def ik(x, y, z, label, from_seed, nominal=None):
        st = solve_ik(model, psm, moveit, x, y, z, 0.0, from_seed, logger, label,
                      nominal=nominal)
        if st is None:
            return None, None
        return st, list(st.get_joint_group_positions("ur_manipulator"))

    if "--ikvals" in tokens:
        for cid in path:
            cx, cy = belt_target(cid)
            for lbl, z in ((f"conv{cid}.approach", GRASP_TOOL0_Z + APPROACH_CLEAR),
                           (f"conv{cid}.descend", GRASP_TOOL0_Z)):
                st, q = ik(cx, cy, z, lbl, seed)
                if q is not None:
                    logger.info(f"IKVALS {lbl}: {[round(v,4) for v in q]}")
                else:
                    logger.error(f"IKVALS {lbl}: UNREACHABLE")
        return 0

    def pick(conv_id, seed_in, do_spawn):
        """Descend on conv_id's belt and grasp the cube (spawning it there on the first
        hop), verifying the lift. Returns the retreat joint config (next seed) or None.

        The DetachableJoint auto-attaches at an uncontrolled instant, so we DETACH to
        clear it, let the cube settle, then explicitly ATTACH (one clean weld). gz-topic
        attach is timing-racy -> verify by retreating and checking the cube lifted."""
        sx, sy = belt_target(conv_id)
        pa_st, pa_q = ik(sx, sy, GRASP_TOOL0_Z + APPROACH_CLEAR, f"pick{conv_id}.approach", seed_in)
        if pa_st is None:
            return None
        pd_st, _ = ik(sx, sy, GRASP_TOOL0_Z, f"pick{conv_id}.descend", pa_q, nominal=pa_q)
        if pd_st is None:
            return None
        set_fingers(FINGER_OPEN)                   # open before descending onto the cube
        if not move_to_state(arm, moveit, logger, pa_st, f"pick{conv_id}.approach"):
            return None
        if not move_to_state(arm, moveit, logger, pd_st, f"pick{conv_id}.descend"):
            return None
        if do_spawn:
            spawn_cube(logger, sx, sy)
        gz_pub(f"/{CUBE_NAME}/detach")
        time.sleep(1.0)
        set_fingers(FINGER_CLOSE)                   # close the fingers to pinch the cube
        lifted_z = BELT_TOP_Z + 0.10
        for attempt in range(1, 4):
            logger.info(f"GRASP attempt {attempt} at conv{conv_id}")
            gz_pub(f"/{CUBE_NAME}/attach")
            time.sleep(1.0)
            if not move_to_state(arm, moveit, logger, pa_st, f"pick{conv_id}.retreat"):
                return None
            z = cube_z()
            logger.info(f"  cube z after retreat = {z} (lifted if > {lifted_z:.2f})")
            if z is not None and z > lifted_z:
                logger.info(f"GRASP confirmed at conv{conv_id}")
                return pa_q
            if attempt < 3:
                logger.warn("grasp not confirmed; re-descending to retry")
                if not move_to_state(arm, moveit, logger, pd_st, f"pick{conv_id}.descend"):
                    return None
        logger.error(f"GRASP FAILED at conv{conv_id} after retries")
        return None

    def place(conv_id, seed_in):
        """Carry to conv_id, descend, release, retreat. Returns retreat seed or None."""
        dx, dy = belt_target(conv_id)
        sa_st, sa_q = ik(dx, dy, GRASP_TOOL0_Z + APPROACH_CLEAR, f"place{conv_id}.approach",
                         seed_in, nominal=seed_in)
        if sa_st is None:
            return None
        sd_st, _ = ik(dx, dy, GRASP_TOOL0_Z, f"place{conv_id}.descend", sa_q, nominal=sa_q)
        if sd_st is None:
            return None
        if not move_to_state(arm, moveit, logger, sa_st, f"place{conv_id}.approach"):
            return None
        if not move_to_state(arm, moveit, logger, sd_st, f"place{conv_id}.descend"):
            return None
        logger.info(f"RELEASE at conv{conv_id}: detaching cube")
        gz_pub(f"/{CUBE_NAME}/detach")
        set_fingers(FINGER_OPEN)                    # open the fingers to release
        time.sleep(1.0)
        if not move_to_state(arm, moveit, logger, sa_st, f"place{conv_id}.retreat"):
            return None
        return sa_q

    try:
        set_fingers(FINGER_OPEN)                       # start with the gripper open
        if not move_to_named(arm, moveit, logger, "up"):
            return 1
        # Walk the path: pick at path[i], place at path[i+1]. The SAME cube is carried
        # the whole way (spawned only at the first station), so config_2 (1->2->3) just
        # re-picks at conv2 where config_1's place left it.
        seed_cur = seed
        nhops = len(path) - 1
        for i in range(nhops):
            src_id, dst_id = path[i], path[i + 1]
            logger.info(f"=== HOP {i+1}/{nhops}: conv{src_id} -> conv{dst_id} ===")
            seed_cur = pick(src_id, seed_cur, do_spawn=(i == 0 and spawn))
            if seed_cur is None:
                return 1
            seed_cur = place(dst_id, seed_cur)
            if seed_cur is None:
                return 1

        if not move_to_named(arm, moveit, logger, "up"):
            return 1
        logger.info(f"WAREHOUSE config_{cfg} COMPLETE (path " +
                    " -> ".join(f"conv{c}" for c in path) + ")")
        return 0
    finally:
        moveit.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
