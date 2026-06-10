#!/usr/bin/env python3
"""
Step 2 manual pick->place test (one hand-built lane).

Drives the UR5 tool0 through a top-down pick at source_1, a DetachableJoint
grasp, a transfer to sink_1, and a release. Grasp/release are triggered by
publishing gz.msgs.Empty to the cube's /cube_1/{attach,detach} gz topics.

Run AFTER step2_bringup.launch.py is up and the cube is spawned:
    LD_PRELOAD=~/reconfig_ws/fastcdr_compat.so python3 step2_pick_place_test.py
"""

import subprocess
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint,
    OrientationConstraint, BoundingVolume,
)
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

# Top-down orientation: 180 deg about X so tool0 +z points down (-world z).
TOPDOWN = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


def gz_pub(topic):
    """Publish an empty gz message to trigger attach/detach."""
    subprocess.run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""],
        check=False, capture_output=True,
    )


class Step2Test(Node):
    def __init__(self):
        super().__init__("step2_pick_place_test",
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def move_tool0(self, label, x, y, z, quat=TOPDOWN):
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose = Pose(position=Point(x=x, y=y, z=z), orientation=quat)

        # Position constraint: small box around target at tool0.
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.02, 0.02, 0.02]))
        pc.constraint_region.primitive_poses.append(ps.pose)
        pc.weight = 1.0

        # Orientation constraint: top-down with modest tolerance.
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool0"
        oc.orientation = quat
        oc.absolute_x_axis_tolerance = 0.2
        oc.absolute_y_axis_tolerance = 0.2
        oc.absolute_z_axis_tolerance = 0.2
        oc.weight = 1.0

        req = MotionPlanRequest(
            group_name="ur_manipulator",
            num_planning_attempts=20,
            allowed_planning_time=10.0,
            max_velocity_scaling_factor=0.4,
            max_acceleration_scaling_factor=0.4,
            goal_constraints=[Constraints(position_constraints=[pc],
                                          orientation_constraints=[oc])],
        )
        goal = MoveGroup.Goal(request=req)

        self.get_logger().info(f"[{label}] planning tool0 -> ({x:.2f}, {y:.2f}, {z:.2f})")
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f"[{label}] goal REJECTED")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=60.0)
        r = rf.result()
        if not r:
            self.get_logger().error(f"[{label}] no result (timeout)")
            return False
        ec = r.result.error_code.val
        ok = ec == 1
        self.get_logger().info(f"[{label}] {'OK' if ok else 'FAIL'} (code={ec})")
        return ok

    def run(self):
        if not self.client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("move_action server unavailable")
            return False

        steps = [
            ("pre-grasp",  lambda: self.move_tool0("pre-grasp",  0.45, 0.30, 0.32)),
            ("grasp",      lambda: self.move_tool0("grasp",      0.45, 0.30, 0.20)),
            ("ATTACH",     lambda: (gz_pub("/cube_1/attach"), True)[1]),
            ("lift",       lambda: self.move_tool0("lift",       0.45, 0.30, 0.35)),
            ("pre-place",  lambda: self.move_tool0("pre-place",  0.25, 0.55, 0.35)),
            ("place",      lambda: self.move_tool0("place",      0.25, 0.55, 0.24)),
            ("DETACH",     lambda: (gz_pub("/cube_1/detach"), True)[1]),
            ("retreat",    lambda: self.move_tool0("retreat",    0.25, 0.55, 0.40)),
        ]
        for name, fn in steps:
            if name in ("ATTACH", "DETACH"):
                self.get_logger().info(f">>> {name} (gz topic)")
            if not fn():
                self.get_logger().error(f"Sequence stopped at '{name}'")
                return False
        self.get_logger().info("=== STEP 2 PICK->PLACE COMPLETE ===")
        return True


def main():
    rclpy.init()
    node = Step2Test()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
