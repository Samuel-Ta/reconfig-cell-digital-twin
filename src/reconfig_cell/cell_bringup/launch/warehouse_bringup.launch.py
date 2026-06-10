"""
Warehouse-backdrop bringup: UR5 + spec parallel-jaw gripper in the picknplace
warehouse as a DECORATIVE BACKDROP (Option A — conveyors removed; see memory
warehouse-backdrop-constant).

This is the Step-1/Step-2 control + MoveIt + RViz stack, IDENTICAL to
step2_bringup.launch.py, with two changes only:
  (1) the Gazebo world is the warehouse backdrop (worlds/warehouse_backdrop.sdf:
      the picknplace warehouse with the 3 delivery conveyors stripped out, so it
      is visual scenery only — building visuals have no collisions, only the
      ground does), and
  (2) GZ_SIM_RESOURCE_PATH is extended so `model://workcell` (the building mesh)
      and its textures resolve from the picknplace package.

The robot is spawned at the WORLD ORIGIN, so base_link == world and every spec
§8 station pose (source_1 = 0.45,0.30, ...) is used unchanged — the proven Step-2
pick-place logic reuses verbatim. The warehouse building is pure decoration.

Override `world:=...` to point at a lane world (backdrop + box fixtures) for the
Step-2 pick-place; default is the bare backdrop for the Step-1 bringup test.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

# picknplace resources needed for the warehouse backdrop's `model://workcell` mesh.
PICKNPLACE_URDF = os.path.expanduser("~/picknplace_ws/src/picknplace/urdf")
PICKNPLACE_TEXTURES = os.path.expanduser(
    "~/picknplace_ws/src/picknplace/urdf/workcell/materials/textures"
)


def launch_setup(context, *args, **kwargs):
    tf_prefix   = LaunchConfiguration("tf_prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui  = LaunchConfiguration("gazebo_gui")

    bringup_pkg      = get_package_share_directory("cell_bringup")
    controllers_yaml = os.path.join(bringup_pkg, "config", "ros2_controllers.yaml")
    xacro_file       = os.path.join(bringup_pkg, "urdf", "ur5_with_gripper.urdf.xacro")
    default_world    = os.path.join(bringup_pkg, "worlds", "warehouse_backdrop.sdf")
    world_file       = LaunchConfiguration("world").perform(context) or default_world

    # ── robot description ──────────────────────────────────────────────────────
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        xacro_file,
        " name:=ur",
        " ur_type:=ur5",
        " tf_prefix:=", tf_prefix,
        " simulation_controllers:=", controllers_yaml,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ── MoveIt config via builder ──────────────────────────────────────────────
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

    # ── LD_PRELOAD shim for fastcdr missing serialize overloads ────────────────
    shim = os.path.join(os.path.expanduser("~"), "reconfig_ws", "fastcdr_compat.so")
    moveit_env = {"LD_PRELOAD": shim} if os.path.exists(shim) else {}

    # ── nodes ──────────────────────────────────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 4 ", world_file],
                else_value=[" -s -r -v 4 ", world_file],
            )
        }.items(),
    )

    # Robot at the world ORIGIN (base_link == world): keeps spec §8 poses unchanged.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_description_content,
            "-name",   "ur5_with_gripper",
            "-allow_renaming", "true",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )
    # NOTE: gripper_controller is intentionally NOT spawned. Its type
    # parallel_gripper_action_controller/GripperActionController segfaults the
    # gz-hosted controller_manager on activate (exit 139, takes the whole sim
    # down) -- same crash documented for warehouse_cell.launch.py. The gripper is
    # cosmetic: fingers spawn OPEN via the URDF state_interface initial_value and
    # are never actuated; the grasp is the DetachableJoint on the cube. So arm-only
    # control is all Step 1/2 need.

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        additional_env=moveit_env,
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
                "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
                "warehouse_port": 0,
            },
        ],
    )

    rviz_config = os.path.join(bringup_pkg, "rviz", "step1.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        additional_env=moveit_env,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_rviz),
        output="screen",
    )

    # ── sequencing (same chain as Step 1/2) ────────────────────────────────────
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])
    spawn_controllers_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot,
                      on_exit=[TimerAction(period=2.0, actions=[jsb_spawner])])
    )
    spawn_arm_gripper_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner,
                      on_exit=[arm_spawner])
    )
    moveit_after_arm = RegisterEventHandler(
        OnProcessExit(target_action=arm_spawner,
                      on_exit=[move_group_node, rviz_node])
    )

    return [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", PICKNPLACE_URDF),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", PICKNPLACE_TEXTURES),
        rsp_node,
        gz_launch,
        clock_bridge,
        delayed_spawn,
        spawn_controllers_after_robot,
        spawn_arm_gripper_after_jsb,
        moveit_after_arm,
    ]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("tf_prefix",   default_value="",     description="Joint name prefix"),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument("gazebo_gui",  default_value="true", description="Launch Gazebo GUI?"),
        DeclareLaunchArgument("world",       default_value="",
                              description="World file (default: warehouse_backdrop.sdf)."),
    ]
    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
