#!/usr/bin/env python3
"""
Step 2 — hand-driven single-lane pick -> place using moveit_py + the
DetachableJoint grasp-fix.

This is the *hand-built* Step-2 demonstrator (spec §6 step 2), NOT framework
code: poses are computed inline here so we can iterate on one reliable lane
before the generator/nodes exist. Nothing here is config-agnostic yet.

Pipeline per op:
  approach (clearance above target) -> descend to target -> grasp/release ->
  retreat to approach.

Grasp/release is the DetachableJoint on the cube model (see models/cube_part.sdf):
  grasp   = publish gz.msgs.Empty to /cube_1/attach
  release = publish gz.msgs.Empty to /cube_1/detach
Finger actuation is cosmetic under grasp-fix (spec §4) and is intentionally
omitted here.

Prereq: step2_bringup.launch.py is already running (Gazebo + controllers +
robot_state_publisher). The cube is spawned by this script unless --no-spawn.

Geometry (hand-placed, mirrors lane 1 of config_1.yaml):
  source_1 fixture: 0.18^2 x 0.10, base on ground -> top face z = 0.10
  cube: 0.04 cube resting on source top -> center z = 0.12
  sink_1 fixture:   0.24^2 x 0.12, base on ground -> top face z = 0.12
The gripper attach frame sits 0.08 m along tool0 +z (gripper.urdf.xacro), and a
top-down grasp orients tool0 +z toward world -z (roll = pi), so:
  tool0_z = (attach-target world z) + 0.08
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

# ── hand-computed targets (base_link frame, metres) ───────────────────────────
ATTACH_OFFSET = 0.08          # gripper_attach_link distance along tool0 +z
APPROACH_CLEAR = 0.15         # global approach/retreat clearance (single value)

# top-down: tool0 +z points to world -z  => roll = pi  => quat (1,0,0,0)
TOPDOWN_QUAT = (1.0, 0.0, 0.0, 0.0)  # (x, y, z, w)

CUBE_CENTER_Z_SOURCE = 0.12   # cube center resting on source top face
PLACE_CUBE_CENTER_Z_SINK = 0.14  # cube center resting on sink top face (0.12 + 0.02)

PICK = {"x": 0.45, "y": 0.30, "z": CUBE_CENTER_Z_SOURCE + ATTACH_OFFSET}
PLACE = {"x": 0.25, "y": 0.55, "z": PLACE_CUBE_CENTER_Z_SINK + ATTACH_OFFSET}

CUBE_NAME = "cube_1"


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


def make_pose(x, y, z):
    p = PoseStamped()
    p.header.frame_id = "base_link"
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = float(z)
    p.pose.orientation.x = TOPDOWN_QUAT[0]
    p.pose.orientation.y = TOPDOWN_QUAT[1]
    p.pose.orientation.z = TOPDOWN_QUAT[2]
    p.pose.orientation.w = TOPDOWN_QUAT[3]
    return p


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


# Gripper finger positions (finger_left_joint, m): -0.03 fully open, 0.0 closed.
# OPEN puts the fingertips at x=+/-0.045 so the 0.04 cube clears them on descent;
# the DetachableJoint (not finger friction) does the actual holding, so we keep
# the gripper open throughout and never ram the part.
GRIPPER_OPEN = -0.03


def gripper_command(position, logger):
    logger.info(f"Gripper -> position {position}")
    subprocess.run(
        ["ros2", "action", "send_goal", "/gripper_controller/gripper_cmd",
         "control_msgs/action/GripperCommand",
         f"{{command: {{position: {position}, max_effort: 20.0}}}}"],
        check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def spawn_cube(logger):
    from ament_index_python.packages import get_package_share_directory
    import os
    sdf = os.path.join(
        get_package_share_directory("cell_bringup"), "models", "cube_part.sdf"
    )
    # idempotent: drop any cube from a previous run and WAIT until it is actually
    # gone before respawning. A too-short delay lets `create` race a not-yet-removed
    # cube_1 (name clash), leaving the stale cube in place and the fresh one missing.
    def cube_exists():
        r = subprocess.run(["gz", "model", "-m", CUBE_NAME, "-p"],
                           capture_output=True, text=True, timeout=8)
        return "Pose" in r.stdout
    if cube_exists():
        # NOTE: ros_gz_sim remove takes the name as a ROS param `entity_name`, NOT a
        # `-name` flag (the flag is silently ignored -> stale cubes pile up across runs).
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
    # Spawn the cube at the grasp point (cube resting on the source feeder, which is
    # exactly where the descended gripper's attach frame sits). The gz DetachableJoint
    # auto-attaches on the first update; spawning it here (gripper already in place)
    # makes that weld capture a ~zero offset, so the part is held cleanly instead of
    # being flung from a far wrist pose.
    logger.info(f"Spawning {CUBE_NAME} at grasp point from {sdf}")
    subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-file", sdf, "-name", CUBE_NAME,
         "-x", str(PICK["x"]), "-y", str(PICK["y"]),
         "-z", str(CUBE_CENTER_Z_SOURCE)],
        check=True,
    )
    time.sleep(1.0)


def move_to_named(arm, robot, logger, name):
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=name)
    return _plan_exec(arm, robot, logger, f"named:{name}")


def move_to_pose(arm, robot, logger, pose, label):
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")
    return _plan_exec(arm, robot, logger, label)


def solve_ik(model, psm, x, y, z, yaw, seed, logger, label, nominal=None):
    """Collision-checked top-down IK -> goal RobotState (None on failure).

    We command JOINT-SPACE goals from our own IK rather than Cartesian pose goals:
    move_group's pose-goal IK tends to pick joint configs that this gz position
    interface can't hold within goal tolerance (the arm never settles -> the
    controller aborts on goal_time). Seeding IK from the previous config keeps the
    chosen solution benign and continuous, matching the configs the arm holds well.
    """
    from moveit.core.robot_state import RobotState

    qx, qy, qz, qw = topdown_quat(yaw)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    pose.orientation.x, pose.orientation.y = qx, qy
    pose.orientation.z, pose.orientation.w = qz, qw

    # KDL is a local solver and readily converges to WRAPPED solutions (a joint at
    # e.g. -3.3 or -4.7 rad) that are within the +/-2pi limits but physically
    # contorted and untrackable. Try many seeds (the supplied seed first, then
    # random restarts) and keep the collision-free solution that is least wrapped
    # (smallest max |joint|), i.e. closest to a clean centred configuration.
    # Prefer an ELBOW-UP, tool-down posture: the upper arm stays raised
    # (shoulder_lift negative) and only the wrist descends to the part. Elbow-down
    # solutions (shoulder_lift >= ~0) swing the lower links into the ground plane
    # in Gazebo (the robot is floor-mounted at the work surface level) and jam.
    # Among collision-free, un-wrapped, elbow-up solutions, pick the least wrapped.
    SHOULDER_LIFT = 1  # index in ur_manipulator joint order
    best_q = best_fallback = None
    best_score = best_fb_score = float("inf")
    tries = 120
    for i in range(tries):
        state = RobotState(model)
        if i == 0:
            state.set_joint_group_positions("ur_manipulator", list(seed))
        else:
            # random restart seeded in [-pi, pi] -> explores IK branches
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
        # score: stay close to `nominal` (continuity with the previous config) if
        # given, else least wrapped. Lower is better in both cases.
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


def run_diag(moveit, logger):
    """Sweep top-down wrist yaw to find a collision-free IK for the targets."""
    from moveit.core.robot_state import RobotState

    model = moveit.get_robot_model()
    psm = moveit.get_planning_scene_monitor()
    targets = {"PICK": PICK, "PLACE": PLACE}

    for name, t in targets.items():
        logger.info(f"=== {name} target ({t['x']},{t['y']},{t['z']}) ===")
        for deg in range(0, 360, 30):
            yaw = math.radians(deg)
            qx, qy, qz, qw = topdown_quat(yaw)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = t["x"], t["y"], t["z"]
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


def main():
    spawn = "--no-spawn" not in sys.argv
    rclpy.init()
    logger = get_logger("step2_pick_place")

    moveit = MoveItPy(node_name="step2_pick_place")
    arm = moveit.get_planning_component("ur_manipulator")
    logger.info("MoveItPy up; ur_manipulator planning component ready.")

    if "--diag" in sys.argv:
        return run_diag(moveit, logger)

    if "--ikvals" in sys.argv:
        model = moveit.get_robot_model()
        psm = moveit.get_planning_scene_monitor()
        seed = [UP_SEED[j] for j in UP_SEED]
        for lbl, t in (("pick.approach", (PICK["x"], PICK["y"], PICK["z"] + APPROACH_CLEAR)),
                       ("pick.descend", (PICK["x"], PICK["y"], PICK["z"]))):
            st = solve_ik(model, psm, t[0], t[1], t[2], 0.0, seed, logger, lbl)
            if st is not None:
                q = list(st.get_joint_group_positions("ur_manipulator"))
                logger.info(f"IKVALS {lbl}: {[round(v,4) for v in q]}")
        return 0

    model = moveit.get_robot_model()
    psm = moveit.get_planning_scene_monitor()
    seed = [UP_SEED[j] for j in UP_SEED]

    def ik(x, y, z, label, from_seed, nominal=None):
        st = solve_ik(model, psm, x, y, z, 0.0, from_seed, logger, label, nominal=nominal)
        if st is None:
            return None, None
        return st, list(st.get_joint_group_positions("ur_manipulator"))

    try:
        # clear start. Fingers spawn OPEN (URDF initial_value) and the grasp uses
        # the DetachableJoint, so the gripper controller is never commanded here
        # (its action server is unreliable; see gripper.urdf.xacro note).
        if not move_to_named(arm, moveit, logger, "up"):
            return 1

        # Pre-solve the whole motion as collision-checked joint configs, chaining
        # each IK seed off the previous so the arm moves through continuous,
        # holdable configurations (see solve_ik docstring).
        # approach: best elbow-up posture (no nominal). descend/place: stay close to
        # the previous config so the wrist doesn't flip branches mid-motion.
        pa_st, pa_q = ik(PICK["x"], PICK["y"], PICK["z"] + APPROACH_CLEAR, "pick.approach", seed)
        if pa_st is None:
            return 1
        pd_st, _ = ik(PICK["x"], PICK["y"], PICK["z"], "pick.descend", pa_q, nominal=pa_q)
        if pd_st is None:
            return 1
        sa_st, sa_q = ik(PLACE["x"], PLACE["y"], PLACE["z"] + APPROACH_CLEAR, "place.approach", pa_q, nominal=pa_q)
        if sa_st is None:
            return 1
        sd_st, _ = ik(PLACE["x"], PLACE["y"], PLACE["z"], "place.descend", sa_q, nominal=sa_q)
        if sd_st is None:
            return 1

        # ── PICK at source_1 ──────────────────────────────────────────────
        if not move_to_state(arm, moveit, logger, pa_st, "pick.approach"):
            return 1
        if not move_to_state(arm, moveit, logger, pd_st, "pick.descend"):
            return 1
        # Part appears at the source feeder, right where the descended gripper sits.
        # The cube's DetachableJoint auto-attaches on spawn at an uncontrolled instant;
        # we DETACH first to clear that joint and let the cube settle on the feeder,
        # then explicitly ATTACH so exactly one weld is captured with the gripper in
        # place. Attach/detach over gz topics is timing-racy, so we VERIFY the grasp by
        # retreating and checking the cube lifted, retrying the attach if it didn't.
        if spawn:
            spawn_cube(logger)
        gz_pub(f"/{CUBE_NAME}/detach")   # clear the spawn-time auto-attach
        time.sleep(1.0)

        grasped = False
        for attempt in range(1, 4):
            logger.info(f"GRASP attempt {attempt}: attaching cube via DetachableJoint")
            gz_pub(f"/{CUBE_NAME}/attach")
            time.sleep(1.0)
            if not move_to_state(arm, moveit, logger, pa_st, "pick.retreat"):
                return 1
            z = cube_z()
            logger.info(f"  cube z after retreat = {z} (lifted if > 0.18)")
            if z is not None and z > 0.18:
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

        # ── PLACE at sink_1 ───────────────────────────────────────────────
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

        logger.info("STEP 2 PICK->PLACE COMPLETE")
        return 0
    finally:
        moveit.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
