"""Headless launch for Rung 1 synthesis (oracle + proposer).

    ros2 launch cell_synth synth.launch.py exe:=oracle_smoke
    ros2 launch cell_synth synth.launch.py exe:=synthesize k:=5 n_stations:=3 seed:=7 out_dir:=/path

This brings up ONLY the synthesis node with the MoveIt parameters (robot_description +
semantic + kinematics + joint_limits + OMPL). Deliberately NO Gazebo, NO controller_manager,
NO move_group node — the oracle is pure kinematics/geometry (IK + collision) and MoveItPy
embeds its own move_group. use_sim_time is FALSE (no /clock headless). This is what lets the
oracle try thousands of candidates fast and avoids the flaky physics sim entirely.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    exe = LaunchConfiguration("exe").perform(context)
    bringup_pkg = get_package_share_directory("cell_bringup")
    controllers_yaml = os.path.join(bringup_pkg, "config", "ur5_rg2_controllers.yaml")

    ur_type = LaunchConfiguration("ur_type").perform(context)   # Phase 3: ur5 | ur10 ...
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="cell_bringup")
        .robot_description(file_path="urdf/ur5_rg2_arm.urdf.xacro",
                           mappings={"name": "ur", "ur_type": ur_type, "tf_prefix": "",
                                     "simulation_controllers": controllers_yaml})
        .robot_description_semantic(file_path="config/moveit/ur5_rg2.srdf")
        .robot_description_kinematics(file_path="config/moveit/kinematics.yaml")
        .joint_limits(file_path="config/moveit/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit/ur5_rg2_moveit_controllers.yaml")
        .to_moveit_configs()
    )
    mp_params = moveit_config.to_dict()
    mp_params["planning_pipelines"] = {"pipeline_names": mp_params["planning_pipelines"]}
    # MoveItConfigsBuilder loads moveit's DEFAULT ompl config (global planner list, no group
    # section), so the group-qualified key 'ur_manipulator[<planner>]' the planner-selection
    # lookup needs is absent. Inject the group's planner list so motion_probe can request an
    # OPTIMIZING planner (RRTstar) by name. Only the (headless) motion_probe plans; the other
    # synth exes are IK-only, so this is inert for them.
    if "ompl" in mp_params:
        mp_params["ompl"]["ur_manipulator"] = {
            "default_planner_config": "RRTConnect",
            "planner_configs": ["RRTConnect", "RRTstar", "PRMstar", "TRRT"],
        }

    shim = os.path.join(os.path.expanduser("~"), "reconfig_ws", "fastcdr_compat.so")
    env = {"LD_PRELOAD": shim} if os.path.exists(shim) else {}

    # forward CLI args for synthesize (k/seed/max_attempts) and optimize (n_specs/base_seed/iters/n_ik)
    fwd = [f"{n}:={LaunchConfiguration(n).perform(context)}"
           for n in ("k", "n_stations", "seed", "max_attempts", "base_config", "out_dir",
                     "n_specs", "base_seed", "iters", "n_ik",
                     "fix_base", "arena_half",
                     "quality", "min_gap", "min_ang",
                     "opt_dir", "n_pool", "n_pick",
                     "valset", "n_plans", "planner", "planning_time")]

    node = Node(package="cell_synth", executable=exe, name="cell_synth", output="screen",
                additional_env=env, arguments=fwd,
                parameters=[mp_params, {"use_sim_time": False}])
    done = RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[LogInfo(msg="[cell_synth] node exited; tearing down launch"), Shutdown()]))
    return [LogInfo(msg=f"[cell_synth] headless run: exe={exe} (no Gazebo / no controllers)"),
            node, done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("exe", default_value="oracle_smoke",
                              description="oracle_smoke | synthesize"),
        DeclareLaunchArgument("k", default_value="5"),
        DeclareLaunchArgument("n_stations", default_value="3"),
        DeclareLaunchArgument("seed", default_value="1"),
        DeclareLaunchArgument("max_attempts", default_value="300"),
        DeclareLaunchArgument("base_config", default_value="config_1"),
        DeclareLaunchArgument("out_dir", default_value=""),
        DeclareLaunchArgument("n_specs", default_value="5"),
        DeclareLaunchArgument("base_seed", default_value="100"),
        DeclareLaunchArgument("iters", default_value="400"),
        DeclareLaunchArgument("n_ik", default_value="40"),
        DeclareLaunchArgument("fix_base", default_value="0"),
        DeclareLaunchArgument("arena_half", default_value="0.6"),
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("quality", default_value="1"),
        DeclareLaunchArgument("min_gap", default_value="0.18"),
        DeclareLaunchArgument("min_ang", default_value="50"),
        DeclareLaunchArgument("opt_dir", default_value=""),
        DeclareLaunchArgument("n_pool", default_value="50"),
        DeclareLaunchArgument("n_pick", default_value="6"),
        DeclareLaunchArgument("valset", default_value=""),
        DeclareLaunchArgument("n_plans", default_value="12"),
        DeclareLaunchArgument("planner", default_value="RRTstar"),
        DeclareLaunchArgument("planning_time", default_value="10.0"),
        OpaqueFunction(function=launch_setup),
    ])
