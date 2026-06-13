"""Within-config validation run (spec §10): bring up the cell ONCE, then loop the
generated relay task N times via trial_runner, logging CSV + per-config summary stats.

    ros2 launch cell_bringup cell_trials.launch.py config:=config_1 trials:=30
    ros2 launch cell_bringup cell_trials.launch.py config:=config_2 trials:=30 gazebo_gui:=false

Identical bringup to cell_warehouse.launch.py (warehouse world + conveyor sync + UR5/RG2
+ controllers + move_group + IK guard); the ONLY difference is the guard gates
trial_runner (loop + measure) instead of task_executor (run once). Configs are scored
SEPARATELY and never cross-compared.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
    IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler,
    Shutdown, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, IfElseSubstitution, LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

PICKNPLACE_URDF = os.path.expanduser("~/picknplace_ws/src/picknplace/urdf")
CELL_X, CELL_Y = "-0.697", "-0.643"
ALL_CONVEYORS = (1, 2, 3)


def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration("config").perform(context)
    trials = LaunchConfiguration("trials").perform(context)
    warmup = LaunchConfiguration("warmup").perform(context)
    op_timeout = LaunchConfiguration("op_timeout").perform(context)
    run_timeout = LaunchConfiguration("run_timeout").perform(context)
    seed_base = LaunchConfiguration("seed_base").perform(context)
    csv_path = LaunchConfiguration("csv").perform(context) or f"/tmp/trials_{config}.csv"
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    bringup_pkg = get_package_share_directory("cell_bringup")
    desc_pkg = get_package_share_directory("cell_description")
    world_path = os.path.join(bringup_pkg, "worlds", "cell_arranged.world")
    controllers_yaml = os.path.join(bringup_pkg, "config", "ur5_rg2_controllers.yaml")
    xacro_file = os.path.join(bringup_pkg, "urdf", "ur5_rg2_arm.urdf.xacro")

    gen_dir = os.path.join(desc_pkg, "generated", config)
    scene_yaml = os.path.join(gen_dir, "scene.yaml")
    task_yaml = os.path.join(gen_dir, "task.yaml")
    cfg_yaml = os.path.join(desc_pkg, f"{config}.yaml")
    for f in (scene_yaml, task_yaml, cfg_yaml):
        if not os.path.exists(f):
            raise RuntimeError(f"missing artifact: {f} (run cell_generator for {config})")

    with open(cfg_yaml) as fh:
        doc = yaml.safe_load(fh)
    used = {int(s["id"].split("_")[-1]) for s in doc["stations"]}
    to_remove = [n for n in ALL_CONVEYORS if n not in used]
    mount = doc["robot_mount"]

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " name:=ur", " ur_type:=ur5", " tf_prefix:=",
        " simulation_controllers:=", controllers_yaml,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

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

    mp_params = moveit_config.to_dict()
    mp_params["planning_pipelines"] = {"pipeline_names": mp_params["planning_pipelines"]}
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

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[robot_description, {"use_sim_time": True}])
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": IfElseSubstitution(
            gazebo_gui, if_value=[" -r -v 4 ", world_path],
            else_value=[" -s -r -v 4 ", world_path])}.items())
    spawn_robot = Node(package="ros_gz_sim", executable="create", output="screen",
                       arguments=["-string", robot_description_content, "-name", "ur5_rg2",
                                  "-x", CELL_X, "-y", CELL_Y, "-z", "0.40", "-allow_renaming", "true"])
    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen")

    remove_actions = [ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "remove", "--ros-args",
             "-p", f"entity_name:=delivery_conveyor_{n}"], output="screen")
        for n in to_remove]

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
               output="screen")
    arm = Node(package="controller_manager", executable="spawner",
               arguments=["joint_trajectory_controller", "-c", "/controller_manager"], output="screen")

    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                      additional_env=env, parameters=[moveit_config.to_dict(), {
                          "use_sim_time": True,
                          "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
                          "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
                          "warehouse_port": 0}])
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
                arguments=["-d", os.path.join(bringup_pkg, "rviz", "step1.rviz")], additional_env=env,
                condition=IfCondition(launch_rviz),
                parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics, moveit_config.joint_limits,
                            {"use_sim_time": True}])

    scene_manager = Node(package="cell_scene_manager", executable="scene_manager",
                         name="cell_scene_manager", output="screen", additional_env=env,
                         arguments=[scene_yaml, task_yaml], parameters=[mp_params, common])
    trial_runner = Node(package="cell_task_executor", executable="trial_runner",
                        name="trial_runner", output="screen", additional_env=env,
                        arguments=[task_yaml, "--config", config, "--trials", trials,
                                   "--warmup", warmup, "--csv", csv_path,
                                   "--op-timeout", op_timeout, "--run-timeout", run_timeout,
                                   "--seed-base", seed_base,
                                   "--mount-x", str(mount["x"]), "--mount-y", str(mount["y"])],
                        parameters=[mp_params, common])

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])
    after_robot = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot, on_exit=[TimerAction(period=2.0, actions=[jsb] + remove_actions)]))
    after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm]))
    after_arm = RegisterEventHandler(OnProcessExit(
        target_action=arm, on_exit=[move_group, rviz, TimerAction(period=8.0, actions=[scene_manager])]))

    def on_guard_exit(event, ctx):
        if event.returncode == 0:
            return [LogInfo(msg="[cell_trials] IK guard PASSED -> starting trial_runner"), trial_runner]
        return [LogInfo(msg=f"[cell_trials] IK guard FAILED (rc={event.returncode}); aborting"),
                Shutdown()]
    gate = RegisterEventHandler(OnProcessExit(target_action=scene_manager, on_exit=on_guard_exit))
    # when the trial batch finishes, tear the whole launch down
    after_trials = RegisterEventHandler(OnProcessExit(
        target_action=trial_runner, on_exit=[LogInfo(msg="[cell_trials] batch complete"), Shutdown()]))

    return [LogInfo(msg=f"[cell_trials] config={config} trials={trials} (+{warmup} warmup) "
                        f"conveyors {sorted(used)} csv={csv_path}"),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", PICKNPLACE_URDF),
            rsp, gz_launch, clock_bridge, delayed_spawn, after_robot, after_jsb, after_arm,
            gate, after_trials]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="config_1"),
        DeclareLaunchArgument("trials", default_value="30"),
        DeclareLaunchArgument("warmup", default_value="1"),
        DeclareLaunchArgument("op_timeout", default_value="15.0"),
        DeclareLaunchArgument("run_timeout", default_value="180.0"),
        DeclareLaunchArgument("seed_base", default_value="1000",
                              description="first run's seed; run i uses seed_base+i (vary across sub-batches)"),
        DeclareLaunchArgument("csv", default_value=""),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
