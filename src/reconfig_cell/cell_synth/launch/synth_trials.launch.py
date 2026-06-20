"""Real cycle-time measurement for a SYNTHESIZED config (Rung 3 surrogate-vs-real check).

    ros2 launch cell_synth synth_trials.launch.py cfg:=<synth.yaml> \
        scene:=<scene.yaml> task:=<task.yaml> trials:=5 csv:=<out.csv>

= synth_demo's conveyor placement (remove the baked conveyors, spawn them at the
SYNTHESIZED poses) + the EXISTING cell_task_executor/trial_runner timed loop (the same
harness used for the locked N=30 results). Logs full_cycle_time per run to CSV so the
DETERMINISTIC surrogate can be correlated against measured real cycle time. Nothing in any
locked package changes; trial_runner and the guard are reused unmodified.
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
ALL_CONVEYORS = (1, 2, 3, 4, 5)
CONVEYOR_URI = "https://fuel.gazebosim.org/1.0/Open-RMF/models/DeliveryRobotWithConveyor"


def _conveyor_sdf(name):
    # NOTE: pose is applied by `create` via -x/-y/-z/-Y flags, NOT here. ros_gz_sim create
    # IGNORES a <pose> embedded in -string SDF, which silently spawned everything at the
    # world origin (0,0,0) — the cause of the cube falling through to the floor.
    return (f"<?xml version='1.0'?><sdf version='1.8'>"
            f"<include><uri>{CONVEYOR_URI}</uri><name>{name}</name>"
            f"<static>true</static></include></sdf>")


BELT_THK = 0.10


def _belt_slab_sdf(name):
    """A STATIC belt slab matching the MoveIt belt box in scene.yaml (full 1.0x0.5 footprint).
    Pose applied by `create` flags (see _conveyor_sdf note). The cube's grasp point is 0.27 m
    inward from the station centre, well within this 1.0x0.5 slab, so it lands ON the belt."""
    return (f"<?xml version='1.0'?><sdf version='1.8'><model name='{name}'><static>true</static>"
            f"<link name='l'>"
            f"<collision name='c'><geometry><box><size>1.0 0.5 {BELT_THK}</size></box></geometry></collision>"
            f"<visual name='v'><geometry><box><size>1.0 0.5 {BELT_THK}</size></box></geometry>"
            f"<material><ambient>0.30 0.30 0.34 1</ambient><diffuse>0.35 0.35 0.4 1</diffuse></material>"
            f"</visual></link></model></sdf>")


def launch_setup(context, *args, **kwargs):
    cfg_yaml = LaunchConfiguration("cfg").perform(context)
    scene_yaml = LaunchConfiguration("scene").perform(context)
    task_yaml = LaunchConfiguration("task").perform(context)
    trials = LaunchConfiguration("trials").perform(context)
    warmup = LaunchConfiguration("warmup").perform(context)
    seed_base = LaunchConfiguration("seed_base").perform(context)
    csv_path = LaunchConfiguration("csv").perform(context) or "/tmp/synth_trials.csv"
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    bringup_pkg = get_package_share_directory("cell_bringup")
    world_path = os.path.join(bringup_pkg, "worlds", "cell_arranged.world")
    controllers_yaml = os.path.join(bringup_pkg, "config", "ur5_rg2_controllers.yaml")
    xacro_file = os.path.join(bringup_pkg, "urdf", "ur5_rg2_arm.urdf.xacro")
    for f in (cfg_yaml, scene_yaml, task_yaml):
        if not os.path.exists(f):
            raise RuntimeError(f"missing artifact: {f}")

    with open(cfg_yaml) as fh:
        doc = yaml.safe_load(fh)
    stations = doc["stations"]
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
    # spawn the robot at the config's robot_mount (so the cell can be relocated; the
    # generated grasp targets are derived relative to this same mount)
    mx, my, mz = str(mount["x"]), str(mount["y"]), str(mount["z"])
    myaw = str(mount.get("yaw", 0.0))      # Phase-2: spawn at the config's base yaw (absent->0)
    spawn_robot = Node(package="ros_gz_sim", executable="create", output="screen",
                       arguments=["-string", robot_description_content, "-name", "ur5_rg2",
                                  "-x", mx, "-y", my, "-z", mz, "-Y", myaw,
                                  "-allow_renaming", "true"])
    # move the (free, static) pedestal under the relocated robot base
    move_pedestal = ExecuteProcess(
        cmd=["gz", "service", "-s", "/world/default/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
             "--req", f'name: "robot_pedestal", position: {{x: {mx}, y: {my}, z: {float(mz)/2}}}'],
        output="screen")
    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen")

    remove_actions = [ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "remove", "--ros-args",
             "-p", f"entity_name:=delivery_conveyor_{n}"], output="screen")
        for n in ALL_CONVEYORS]
    top_z = doc["belt"]["top_z"]
    # pose is passed as -x/-y/-z/-Y FLAGS (create ignores <pose> in -string SDF)
    spawn_conveyors = [Node(package="ros_gz_sim", executable="create", output="screen",
                            arguments=["-string", _conveyor_sdf(s["id"]),
                                       "-x", str(s["pose"]["x"]), "-y", str(s["pose"]["y"]),
                                       "-z", "0.0", "-Y", str(s["pose"]["yaw"])])
                       for s in stations]
    spawn_pads = [Node(package="ros_gz_sim", executable="create", output="screen",
                       arguments=["-string", _belt_slab_sdf(f"{s['id']}_belt"),
                                  "-x", str(s["pose"]["x"]), "-y", str(s["pose"]["y"]),
                                  "-z", str(top_z - BELT_THK / 2), "-Y", str(s["pose"]["yaw"])])
                  for s in stations]

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
    scene_manager = Node(package="cell_scene_manager", executable="scene_manager",
                         name="cell_scene_manager", output="screen", additional_env=env,
                         arguments=[scene_yaml, task_yaml], parameters=[mp_params, common])
    trial_runner = Node(package="cell_task_executor", executable="trial_runner",
                        name="trial_runner", output="screen", additional_env=env,
                        arguments=[task_yaml, "--config", os.path.basename(cfg_yaml), "--trials", trials,
                                   "--warmup", warmup, "--csv", csv_path,
                                   "--op-timeout", "15.0", "--run-timeout", "200.0",
                                   "--seed-base", seed_base,
                                   "--mount-x", str(mount["x"]), "--mount-y", str(mount["y"])],
                        parameters=[mp_params, common])

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])
    after_robot = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[TimerAction(period=2.0, actions=[jsb] + remove_actions + [move_pedestal]),
                 TimerAction(period=4.0, actions=spawn_conveyors + spawn_pads)]))
    after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm]))
    after_arm = RegisterEventHandler(OnProcessExit(
        target_action=arm, on_exit=[move_group, TimerAction(period=8.0, actions=[scene_manager])]))

    def on_guard_exit(event, ctx):
        if event.returncode == 0:
            return [LogInfo(msg="[synth_trials] IK guard PASSED -> trial_runner"), trial_runner]
        return [LogInfo(msg=f"[synth_trials] IK guard FAILED (rc={event.returncode})"), Shutdown()]
    gate = RegisterEventHandler(OnProcessExit(target_action=scene_manager, on_exit=on_guard_exit))
    after_trials = RegisterEventHandler(OnProcessExit(
        target_action=trial_runner, on_exit=[LogInfo(msg="[synth_trials] batch complete"), Shutdown()]))

    return [LogInfo(msg=f"[synth_trials] cfg={os.path.basename(cfg_yaml)} trials={trials} csv={csv_path}"),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", PICKNPLACE_URDF),
            rsp, gz_launch, clock_bridge, delayed_spawn, after_robot, after_jsb, after_arm,
            gate, after_trials]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cfg"),
        DeclareLaunchArgument("scene"),
        DeclareLaunchArgument("task"),
        DeclareLaunchArgument("trials", default_value="5"),
        DeclareLaunchArgument("warmup", default_value="1"),
        DeclareLaunchArgument("seed_base", default_value="5000"),
        DeclareLaunchArgument("csv", default_value=""),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
