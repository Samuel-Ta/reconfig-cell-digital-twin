"""
Warehouse cell bringup (working path): the proven reconfig UR5 + gripper + MoveIt stack,
spawned at the workstation pose inside the picknplace warehouse world
(cell_in_warehouse.world: warehouse + 3 delivery conveyors arced within reach).

This is the reconfig Step-2 stack with two changes only: (1) the Gazebo world is the
picknplace warehouse cell, and (2) the robot is spawned at the cell pose (-0.697,-0.643)
where the static ur5_rg2 placeholder used to stand. Everything else — controllers, MoveIt,
RViz, the fastcdr shim — is identical to step2_bringup.

Pick/place is run afterwards with warehouse_pick_place.py (config_1 / config_2), which
picks a cube off one conveyor belt and places it on another.
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

PICKNPLACE_URDF = os.path.expanduser("~/picknplace_ws/src/picknplace/urdf")
CELL_WORLD = os.path.expanduser(
    "~/picknplace_ws/src/picknplace/worlds/cell_in_warehouse.world"
)
# Workstation pose: where the ur5_rg2 placeholder stood; conveyors are arced around it.
CELL_X, CELL_Y = "-0.697", "-0.643"


def launch_setup(context, *args, **kwargs):
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    bringup_pkg = get_package_share_directory("cell_bringup")
    # Default to the vendored, arranged world: conveyors fanned at R=0.82 + a 0.40 m
    # robot pedestal (see cell_arranged.world). Override world:= to use another.
    default_world = os.path.join(bringup_pkg, "worlds", "cell_arranged.world")
    world_path = LaunchConfiguration("world").perform(context) or default_world
    controllers_yaml = os.path.join(bringup_pkg, "config", "ur5_rg2_controllers.yaml")
    xacro_file = os.path.join(bringup_pkg, "urdf", "ur5_rg2_arm.urdf.xacro")

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " name:=ur", " ur_type:=ur5", " tf_prefix:=",
        " simulation_controllers:=", controllers_yaml,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="cell_bringup")
        .robot_description(
            file_path="urdf/ur5_rg2_arm.urdf.xacro",
            mappings={"name": "ur", "ur_type": "ur5", "tf_prefix": "",
                      "simulation_controllers": controllers_yaml},
        )
        .robot_description_semantic(file_path="config/moveit/ur5_rg2.srdf")
        .robot_description_kinematics(file_path="config/moveit/kinematics.yaml")
        .joint_limits(file_path="config/moveit/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit/ur5_rg2_moveit_controllers.yaml")
        .to_moveit_configs()
    )

    shim = os.path.join(os.path.expanduser("~"), "reconfig_ws", "fastcdr_compat.so")
    moveit_env = {"LD_PRELOAD": shim} if os.path.exists(shim) else {}

    rsp_node = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[robot_description, {"use_sim_time": True}],
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 4 ", world_path],
                else_value=[" -s -r -v 4 ", world_path],
            )
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-string", robot_description_content,
                   "-name", "ur5_rg2",
                   "-x", CELL_X, "-y", CELL_Y, "-z", "0.40",  # on the pedestal
                   "-allow_renaming", "true"],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )
    # NOTE: gripper_controller is intentionally NOT spawned — parallel_gripper_action_controller
    # segfaults the gz-hosted controller_manager on load, and the grasp is the DetachableJoint
    # (fingers cosmetic, never actuated), so the arm-only control is all we need.

    move_group_node = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen", additional_env=moveit_env,
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True,
             "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
             "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
             "warehouse_port": 0},
        ],
    )

    rviz_config = os.path.join(bringup_pkg, "rviz", "step1.rviz")
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_config], additional_env=moveit_env,
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits, {"use_sim_time": True}],
        condition=IfCondition(launch_rviz), output="screen",
    )

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])
    after_robot = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[TimerAction(period=2.0, actions=[jsb_spawner])]))
    after_jsb = RegisterEventHandler(OnProcessExit(
        target_action=jsb_spawner, on_exit=[arm_spawner]))
    after_arm = RegisterEventHandler(OnProcessExit(
        target_action=arm_spawner, on_exit=[move_group_node, rviz_node]))

    return [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", PICKNPLACE_URDF),
        rsp_node, gz_launch, clock_bridge,
        delayed_spawn, after_robot, after_jsb, after_arm,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("world", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
