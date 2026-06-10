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
    1: (-1.22, -0.34),
    2: (-0.697, -0.043),
    3: (-0.18, -0.34),
}

BELT_TOP_Z = 0.70            # belt top face, world z (== base z; base is at z=0)
CUBE_HALF = 0.02             # 40 mm cube
INWARD = 0.15                # shift target toward base from belt centre (reach)

ATTACH_OFFSET = 0.0          # tool0 IS the grasp frame (no gripper below it)
APPROACH_CLEAR = 0.15        # approach/retreat clearance above target

CUBE_CENTER_Z = BELT_TOP_Z + CUBE_HALF          # cube resting on belt: 0.72
GRASP_TOOL0_Z = CUBE_CENTER_Z + ATTACH_OFFSET   # tool0 z at the grasp: 0.72

# config -> (source conveyor id, sink conveyor id)
CONFIGS = {
    1: (1, 3),
    2: (3, 1),
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
    logger.info(f"Spawning {CUBE_NAME} at world ({wx:.3f},{wy:.3f},{CUBE_CENTER_Z}) from {sdf}")
    subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-file", sdf, "-name", CUBE_NAME,
         "-x", str(wx), "-y", str(wy), "-z", str(CUBE_CENTER_Z)],
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
        if not state.set_from_ik("ur_manipulator", pose, "tool0", 0.05):
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


def run_diag(moveit, logger, src, dst):
    """Sweep top-down wrist yaw to find a collision-free IK for the conveyor targets."""
    from moveit.core.robot_state import RobotState

    model = moveit.get_robot_model()
    psm = moveit.get_planning_scene_monitor()
    sx, sy = belt_target(src)
    dx, dy = belt_target(dst)
    targets = {
        f"PICK(c{src})":  (sx, sy, GRASP_TOOL0_Z),
        f"PLACE(c{dst})": (dx, dy, GRASP_TOOL0_Z),
    }

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
            ok = state.set_from_ik("ur_manipulator", pose, "tool0", 0.1)
            colliding = None
            if ok:
                with psm.read_only() as scene:
                    colliding = scene.is_state_colliding(
                        robot_state=state, joint_model_group_name="ur_manipulator",
                        verbose=False)
            logger.info(f"  yaw={deg:3d}  ik={'OK ' if ok else 'NO '}  "
                        f"self_collision={colliding}")
    return 0


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
    spawn = "--no-spawn" not in sys.argv
    cfg = parse_config(sys.argv)
    src, dst = CONFIGS[cfg]

    rclpy.init()
    logger = get_logger("warehouse_pick_place")
    logger.info(f"config_{cfg}: pick conveyor_{src} -> place conveyor_{dst}")

    moveit = MoveItPy(node_name="warehouse_pick_place")
    arm = moveit.get_planning_component("ur_manipulator")
    logger.info("MoveItPy up; ur_manipulator planning component ready.")

    if "--diag" in sys.argv:
        return run_diag(moveit, logger, src, dst)

    model = moveit.get_robot_model()
    psm = moveit.get_planning_scene_monitor()
    seed = [UP_SEED[j] for j in UP_SEED]

    sx, sy = belt_target(src)
    dx, dy = belt_target(dst)
    logger.info(f"PICK base target  ({sx:.3f},{sy:.3f},{GRASP_TOOL0_Z:.3f})")
    logger.info(f"PLACE base target ({dx:.3f},{dy:.3f},{GRASP_TOOL0_Z:.3f})")

    def ik(x, y, z, label, from_seed, nominal=None):
        st = solve_ik(model, psm, moveit, x, y, z, 0.0, from_seed, logger, label,
                      nominal=nominal)
        if st is None:
            return None, None
        return st, list(st.get_joint_group_positions("ur_manipulator"))

    if "--ikvals" in sys.argv:
        for lbl, t in (("pick.approach", (sx, sy, GRASP_TOOL0_Z + APPROACH_CLEAR)),
                       ("pick.descend", (sx, sy, GRASP_TOOL0_Z)),
                       ("place.approach", (dx, dy, GRASP_TOOL0_Z + APPROACH_CLEAR)),
                       ("place.descend", (dx, dy, GRASP_TOOL0_Z))):
            st, q = ik(t[0], t[1], t[2], lbl, seed)
            if q is not None:
                logger.info(f"IKVALS {lbl}: {[round(v,4) for v in q]}")
            else:
                logger.error(f"IKVALS {lbl}: UNREACHABLE")
        return 0

    try:
        # clear start (named 'up' posture).
        if not move_to_named(arm, moveit, logger, "up"):
            return 1

        # Pre-solve the whole motion as collision-checked joint configs, chaining
        # each IK seed off the previous so the arm moves through continuous,
        # holdable configurations.
        pa_st, pa_q = ik(sx, sy, GRASP_TOOL0_Z + APPROACH_CLEAR, "pick.approach", seed)
        if pa_st is None:
            return 1
        pd_st, _ = ik(sx, sy, GRASP_TOOL0_Z, "pick.descend", pa_q, nominal=pa_q)
        if pd_st is None:
            return 1
        sa_st, sa_q = ik(dx, dy, GRASP_TOOL0_Z + APPROACH_CLEAR, "place.approach", pa_q, nominal=pa_q)
        if sa_st is None:
            return 1
        sd_st, _ = ik(dx, dy, GRASP_TOOL0_Z, "place.descend", sa_q, nominal=sa_q)
        if sd_st is None:
            return 1

        # ── PICK at source conveyor ───────────────────────────────────────
        if not move_to_state(arm, moveit, logger, pa_st, "pick.approach"):
            return 1
        if not move_to_state(arm, moveit, logger, pd_st, "pick.descend"):
            return 1
        # Cube appears on the source belt, right where the descended flange sits.
        # The DetachableJoint auto-attaches on spawn at an uncontrolled instant; we
        # DETACH first to clear it and let the cube settle, then explicitly ATTACH so
        # exactly one weld is captured with the flange in place. Attach over gz topics
        # is timing-racy, so VERIFY the grasp by retreating and checking the cube
        # lifted, retrying the attach if it didn't.
        if spawn:
            spawn_cube(logger, sx, sy)
        gz_pub(f"/{CUBE_NAME}/detach")   # clear the spawn-time auto-attach
        time.sleep(1.0)

        lifted_z = BELT_TOP_Z + 0.10     # "lifted" threshold after retreat
        grasped = False
        for attempt in range(1, 4):
            logger.info(f"GRASP attempt {attempt}: attaching cube via DetachableJoint")
            gz_pub(f"/{CUBE_NAME}/attach")
            time.sleep(1.0)
            if not move_to_state(arm, moveit, logger, pa_st, "pick.retreat"):
                return 1
            z = cube_z()
            logger.info(f"  cube z after retreat = {z} (lifted if > {lifted_z:.2f})")
            if z is not None and z > lifted_z:
                grasped = True
                logger.info("GRASP confirmed")
                break
            if attempt < 3:
                logger.warn("grasp not confirmed; re-descending to retry")
                if not move_to_state(arm, moveit, logger, pd_st, "pick.descend"):
                    return 1
        if not grasped:
            logger.error("GRASP FAILED after retries")
            return 1

        # ── PLACE at sink conveyor ────────────────────────────────────────
        if not move_to_state(arm, moveit, logger, sa_st, "place.approach"):
            return 1
        if not move_to_state(arm, moveit, logger, sd_st, "place.descend"):
            return 1
        logger.info("RELEASE: detaching cube")
        gz_pub(f"/{CUBE_NAME}/detach")
        time.sleep(1.0)
        if not move_to_state(arm, moveit, logger, sa_st, "place.retreat"):
            return 1

        if not move_to_named(arm, moveit, logger, "up"):
            return 1

        logger.info(f"WAREHOUSE config_{cfg} PICK->PLACE COMPLETE "
                    f"(conveyor_{src} -> conveyor_{dst})")
        return 0
    finally:
        moveit.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
