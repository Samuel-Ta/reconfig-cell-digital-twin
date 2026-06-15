"""End-to-end sanity run of a SYNTHESIZED config on the EXISTING twin (Rung 1 step 5).

    ros2 launch cell_synth synth_demo.launch.py cfg:=<synth.yaml> scene:=<scene.yaml> task:=<task.yaml>

Identical bringup to cell_bringup/cell_warehouse.launch.py — same warehouse world, same
UR5+RG2 robot, same controllers/move_group, the same config-AGNOSTIC IK guard
(cell_scene_manager) gating the same relay executor (cell_task_executor) on the generator's
artifacts. The ONLY additions (this is the synthesis layer, purely additive):
  * remove all three conveyors baked into cell_arranged.world (they sit at the config_1/2
    poses), then
  * spawn delivery_conveyor_1..N at the SYNTHESIZED world poses (same Fuel model, same
    <static> include as the world uses — just relocated), so the carried cube has belt
    support exactly where the generated targets expect it.
Nothing about the robot, gripper, world model, generator, guard, or executor is changed.
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
ALL_CONVEYORS = (1, 2, 3)
CONVEYOR_URI = "https://fuel.gazebosim.org/1.0/Open-RMF/models/DeliveryRobotWithConveyor"


def _conveyor_sdf(name, x, y, yaw):
    """One relocated conveyor (pose applied by create -x/-y/-z/-Y flags, NOT embedded in
    -string SDF — create ignores an embedded <pose>, which spawns everything at the origin)."""
    return (f"<?xml version='1.0'?><sdf version='1.8'>"
            f"<include><uri>{CONVEYOR_URI}</uri><name>{name}</name>"
            f"<static>true</static></include></sdf>")


BELT_THK = 0.10


def _belt_slab_sdf(name):
    """Static belt slab (1.0x0.5 footprint) matching scene.yaml; pose via create flags."""
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

    # remove ALL baked conveyors, then spawn the synthesized ones at their world poses
    remove_actions = [ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "remove", "--ros-args",
             "-p", f"entity_name:=delivery_conveyor_{n}"], output="screen")
        for n in ALL_CONVEYORS]
    top_z = doc["belt"]["top_z"]
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
    task_executor = Node(package="cell_task_executor", executable="task_executor",
                         name="cell_task_executor", output="screen", additional_env=env,
                         arguments=[task_yaml], parameters=[mp_params, common])

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])
    # after the robot: bring up jsb, remove baked conveyors, then (2s later) spawn synth ones
    after_robot = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[TimerAction(period=2.0, actions=[jsb] + remove_actions),
                 TimerAction(period=4.0, actions=spawn_conveyors + spawn_pads)]))
    after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm]))
    after_arm = RegisterEventHandler(OnProcessExit(
        target_action=arm, on_exit=[move_group, TimerAction(period=8.0, actions=[scene_manager])]))

    def on_guard_exit(event, ctx):
        if event.returncode == 0:
            return [LogInfo(msg="[synth_demo] IK guard PASSED -> starting executor"), task_executor]
        return [LogInfo(msg=f"[synth_demo] IK guard FAILED (rc={event.returncode}); aborting"),
                Shutdown()]
    gate = RegisterEventHandler(OnProcessExit(target_action=scene_manager, on_exit=on_guard_exit))
    after_exec = RegisterEventHandler(OnProcessExit(
        target_action=task_executor, on_exit=[LogInfo(msg="[synth_demo] relay finished"), Shutdown()]))

    return [LogInfo(msg=f"[synth_demo] cfg={os.path.basename(cfg_yaml)} "
                        f"conveyors at synthesized poses: "
                        f"{[(s['id'], round(s['pose']['x'],2), round(s['pose']['y'],2)) for s in stations]}"),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", PICKNPLACE_URDF),
            rsp, gz_launch, clock_bridge, delayed_spawn, after_robot, after_jsb, after_arm,
            gate, after_exec]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cfg"),
        DeclareLaunchArgument("scene"),
        DeclareLaunchArgument("task"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
