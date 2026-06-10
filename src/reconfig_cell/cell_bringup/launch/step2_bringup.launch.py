"""
Step 2 bringup: UR5 + gripper in the hand-built ONE-LANE world
(source_1 feeder + sink_1 bin). Same control/MoveIt/RViz stack as Step 1; the
only change is the Gazebo world (step2_lane.world.sdf instead of empty.sdf).

The cube part is NOT launched here — spawn it after bringup with:
    ros2 run ros_gz_sim create -file <cell_bringup>/models/cube_part.sdf \
        -name cube_1 -x 0.45 -y 0.30 -z 0.12
so the DetachableJoint's child_model (the robot) already exists.

Success test (Step 2): a single pick -> place using the detachable-joint grasp.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
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


def launch_setup(context, *args, **kwargs):
    tf_prefix   = LaunchConfiguration("tf_prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui  = LaunchConfiguration("gazebo_gui")

    bringup_pkg      = get_package_share_directory("cell_bringup")
    controllers_yaml = os.path.join(bringup_pkg, "config", "ros2_controllers.yaml")
    xacro_file       = os.path.join(bringup_pkg, "urdf", "ur5_with_gripper.urdf.xacro")
    world_file       = os.path.join(bringup_pkg, "worlds", "step2_lane.world.sdf")

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
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

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

    # ── sequencing (same chain as Step 1) ──────────────────────────────────────
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])
    spawn_controllers_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot,
                      on_exit=[TimerAction(period=2.0, actions=[jsb_spawner])])
    )
    spawn_arm_gripper_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner,
                      on_exit=[arm_spawner, gripper_spawner])
    )
    moveit_after_arm = RegisterEventHandler(
        OnProcessExit(target_action=arm_spawner,
                      on_exit=[move_group_node, rviz_node])
    )

    return [
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
    ]
    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
