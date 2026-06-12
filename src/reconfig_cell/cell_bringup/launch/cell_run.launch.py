"""Run a GENERATED warehouse config through the framework nodes (spec §6 step 5).

Run AFTER warehouse_cell.launch.py is up (Gazebo + warehouse world + ur5_rg2 +
controllers + move_group). This launches the two config-AGNOSTIC framework nodes for
the chosen config:

    ros2 launch cell_bringup cell_run.launch.py config:=config_1
    ros2 launch cell_bringup cell_run.launch.py config:=config_2

  cell_scene_manager  applies the generated scene + IK reachability guard, then EXITS
                      0 (pass) / nonzero (a conveyor out of reach).
  cell_task_executor  starts ONLY if the guard passed (gated on the exit code) and runs
                      the generated relay task.

Switching config_1 -> config_2 is purely `config:=config_2` — same launch, same nodes,
no code edits (INVARIANT 2). The artifacts come from cell_generator.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration
    config = LaunchConfiguration("config").perform(context)

    bringup_pkg = get_package_share_directory("cell_bringup")
    desc_pkg = get_package_share_directory("cell_description")
    controllers_yaml = os.path.join(bringup_pkg, "config", "ur5_rg2_controllers.yaml")

    gen_dir = os.path.join(desc_pkg, "generated", config)
    scene_yaml = os.path.join(gen_dir, "scene.yaml")
    task_yaml = os.path.join(gen_dir, "task.yaml")
    for f in (scene_yaml, task_yaml):
        if not os.path.exists(f):
            raise RuntimeError(f"generated artifact missing: {f}\nrun: ros2 run cell_generator "
                               f"generate --config <cell_description>/{config}.yaml --out "
                               f"<cell_description>/generated")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="cell_bringup")
        .robot_description(file_path="urdf/ur5_rg2_arm.urdf.xacro",
                           mappings={"name": "ur", "ur_type": "ur5", "tf_prefix": "",
                                     "simulation_controllers": controllers_yaml})
        .robot_description_semantic(file_path="config/moveit/ur5_rg2.srdf")
        .robot_description_kinematics(file_path="config/moveit/kinematics.yaml")
        .joint_limits(file_path="config/moveit/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit/ur5_rg2_moveit_controllers.yaml")
        .to_moveit_configs()
    )

    shim = os.path.join(os.path.expanduser("~"), "reconfig_ws", "fastcdr_compat.so")
    env = {"LD_PRELOAD": shim} if os.path.exists(shim) else {}

    params = moveit_config.to_dict()
    params["planning_pipelines"] = {"pipeline_names": params["planning_pipelines"]}
    common = {
        "use_sim_time": True,
        "trajectory_execution.execution_duration_monitoring": False,
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        "trajectory_execution.allowed_goal_duration_margin": 10.0,
        "plan_request_params": {
            "planning_pipeline": "ompl", "planner_id": "RRTConnectkConfigDefault",
            "planning_attempts": 10, "planning_time": 5.0,
            "max_velocity_scaling_factor": 0.3, "max_acceleration_scaling_factor": 0.3,
        },
    }

    scene_manager = Node(package="cell_scene_manager", executable="scene_manager",
                         name="cell_scene_manager", output="screen", additional_env=env,
                         arguments=[scene_yaml, task_yaml], parameters=[params, common])
    task_executor = Node(package="cell_task_executor", executable="task_executor",
                         name="cell_task_executor", output="screen", additional_env=env,
                         arguments=[task_yaml], parameters=[params, common])

    # GATE: run the executor only if the guard exited 0; otherwise abort.
    def on_guard_exit(event, ctx):
        if event.returncode == 0:
            return [LogInfo(msg="[cell_run] IK guard PASSED -> starting cell_task_executor"),
                    task_executor]
        return [LogInfo(msg=f"[cell_run] IK guard FAILED (rc={event.returncode}); "
                            f"executor NOT started"), Shutdown()]

    return [
        LogInfo(msg=f"[cell_run] config={config}  scene={scene_yaml}"),
        scene_manager,
        RegisterEventHandler(OnProcessExit(target_action=scene_manager, on_exit=on_guard_exit)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="config_1",
                              description="generated config to run (config_1 | config_2)"),
        OpaqueFunction(function=launch_setup),
    ])
