"""
Step 2 pick->place runner.

Run this AFTER step2_bringup.launch.py is up (Gazebo + controllers +
robot_state_publisher + move_group). It launches the hand-driven moveit_py
demonstrator (scripts/step2_pick_place.py) with the full MoveIt config as
parameters and the fastcdr LD_PRELOAD shim applied (see memory: fastcdr-shim).

    ros2 launch cell_bringup step2_pick_place.launch.py

There are no collision objects in Step 2, so this moveit_py instance plans in an
empty scene + robot; the fixtures live only in Gazebo and the planned path
clears them via the approach/retreat clearance.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bringup_pkg = get_package_share_directory("cell_bringup")
    controllers_yaml = os.path.join(bringup_pkg, "config", "ros2_controllers.yaml")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="cell_bringup")
        .robot_description(
            file_path="urdf/ur5_with_gripper.urdf.xacro",
            mappings={
                "name": "ur",
                "ur_type": "ur5",
                "tf_prefix": "",
                "simulation_controllers": controllers_yaml,
            },
        )
        .robot_description_semantic(file_path="config/moveit/ur5_with_gripper.srdf")
        .robot_description_kinematics(file_path="config/moveit/kinematics.yaml")
        .joint_limits(file_path="config/moveit/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    shim = os.path.join(os.path.expanduser("~"), "reconfig_ws", "fastcdr_compat.so")
    env = {"LD_PRELOAD": shim} if os.path.exists(shim) else {}

    # moveit_cpp (the engine behind moveit_py) reads pipeline names from
    # `planning_pipelines.pipeline_names` (see moveit_cpp.hpp PlanningPipelineOptions),
    # whereas MoveItConfigsBuilder.to_dict() emits the flat `planning_pipelines: [ompl]`
    # list that the move_group node expects. Rewrite that one key for moveit_py.
    params = moveit_config.to_dict()
    params["planning_pipelines"] = {"pipeline_names": params["planning_pipelines"]}

    pick_place_node = Node(
        package="cell_bringup",
        executable="step2_pick_place.py",
        name="step2_pick_place",
        output="screen",
        additional_env=env,
        arguments=[LaunchConfiguration("args")],
        parameters=[
            params,
            {
                "use_sim_time": True,
                # The arm tracks the goal accurately but lags mid-path (sim RTF ~0.67),
                # so it settles slower than moveit's default 1.2x duration monitor allows
                # -> spurious TIMED_OUT that cancels the move mid-flight. Stop monitoring
                # duration and just wait for the controller's own goal_time verdict.
                "trajectory_execution.execution_duration_monitoring": False,
                "trajectory_execution.allowed_execution_duration_scaling": 10.0,
                "trajectory_execution.allowed_goal_duration_margin": 10.0,
                # Default plan request params consumed by PlanningComponent.plan()
                # with no args (moveit_py reads the `plan_request_params` namespace).
                "plan_request_params": {
                    "planning_pipeline": "ompl",
                    "planner_id": "RRTConnectkConfigDefault",
                    "planning_attempts": 10,
                    "planning_time": 5.0,
                    "max_velocity_scaling_factor": 0.3,
                    "max_acceleration_scaling_factor": 0.3,
                },
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "args", default_value="",
            description="Extra argv for the demonstrator (e.g. --diag, --no-spawn)."),
        pick_place_node,
    ])
