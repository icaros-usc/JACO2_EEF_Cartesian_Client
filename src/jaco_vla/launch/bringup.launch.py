"""Native Gen2 Cartesian control, observations, and an optional SSH tunnel."""

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def required_process(process, reason):
    """Register shutdown handling before starting a required process."""
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=process,
                on_exit=[EmitEvent(event=Shutdown(reason=reason))],
            )
        ),
        process,
    ]


def configure(context):
    # This callback already has a runtime context, so immediate values can
    # be read directly. Deferred substitutions remain in action definitions.
    config = context.launch_configurations
    sim = config["sim"]
    driver = Node(
        package="jaco_vla",
        executable="jaco_cartesian",
        name="jaco_cartesian",
        namespace="/",
        output="screen",
        parameters=[
            LaunchConfiguration("control_config"),
            {"sim": sim},
        ],
    )
    client_overrides = {
        "instruction": ParameterValue(
            LaunchConfiguration("instruction"), value_type=str
        ),
        "dry_run": config["dry_run"] == "true",
        "trajectory_dir": ParameterValue(config["trajectory_dir"], value_type=str),
    }
    if config["server_url"]:
        client_overrides["server_url"] = ParameterValue(
            config["server_url"], value_type=str
        )
    if config["replan_steps"]:
        steps = int(config["replan_steps"])
        if steps < 1:
            raise ValueError("replan_steps must be a positive integer")
        client_overrides["replan_steps"] = steps
    if config["gripper_xyz"].strip():
        xyz = yaml.safe_load(config["gripper_xyz"])
        if (
            not isinstance(xyz, list)
            or len(xyz) != 3
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                for value in xyz
            )
        ):
            raise ValueError("gripper_xyz must be [x, y, z] with three finite numbers")
        client_overrides["gripper_xyz"] = [float(value) for value in xyz]
    client = Node(
        package="jaco_vla",
        executable="vla_client",
        name="vla_client",
        namespace="/",
        output="screen",
        # Compression and video encoding happen after robot control has stopped.
        sigterm_timeout=(
            "300"
            if config["trajectory_dir"].strip().lower() not in ("", "none")
            else "5"
        ),
        parameters=[
            LaunchConfiguration("control_config"),
            client_overrides,
        ],
    )

    actions = []

    if config["launch_cameras"] == "true":
        with Path(config["camera_config"]).open(encoding="utf-8") as stream:
            cameras = yaml.safe_load(stream)
        if not isinstance(cameras, dict) or any(
            not isinstance(cameras.get(role), dict) for role in ("overview", "wrist")
        ):
            raise ValueError("camera_config must contain overview and wrist mappings")

        for role, name in (
            ("overview", "camera"),
            ("wrist", "wrist"),
        ):
            actions.append(
                Node(
                    package="realsense2_camera",
                    executable="realsense2_camera_node",
                    name=name,
                    namespace="camera",
                    parameters=[{"camera_name": name}, cameras[role]],
                    output="screen",
                    emulate_tty=True,
                )
            )

    if config["view_camera_images"] == "true":
        for topic in (
            "/camera/camera/color/image_raw",
            "/camera/wrist/color/image_raw",
        ):
            actions.append(
                Node(
                    package="image_tools",
                    executable="showimage",
                    ros_arguments=["--log-level", "warn"],
                    output="screen",
                    remappings=[("image", topic)],
                )
            )

    remote = config["ssh_target"]
    if remote:
        if remote.startswith("-") or any(char.isspace() for char in remote):
            raise ValueError("ssh_target must be a single SSH host alias or user@host")

        # The client URL must target localhost:8010 when this tunnel is used.
        tunnel = ExecuteProcess(
            cmd=["ssh", "-N", remote],
            output="screen",
        )
        actions.extend(required_process(tunnel, "SSH tunnel exited"))

    # Launch order does not guarantee readiness; the client waits for observations.
    actions.extend(required_process(driver, "Native Cartesian driver exited"))
    actions.extend(required_process(client, "VLA client exited"))
    return actions


def generate_launch_description():
    own = Path(get_package_share_directory("jaco_vla"))
    arguments = [
        DeclareLaunchArgument(
            "sim",
            default_value="mock",
            choices=["mock", "real"],
            description="Select mock or real hardware",
        ),
        DeclareLaunchArgument(
            "launch_cameras",
            default_value="true",
            choices=["true", "false"],
            description="Start the overview and wrist camera drivers; disable when drivers run separately",
        ),
        DeclareLaunchArgument(
            "view_camera_images",
            default_value="false",
            choices=["true", "false"],
            description="Open windows showing the raw overview and wrist camera streams",
        ),
        DeclareLaunchArgument(
            "ssh_target",
            default_value="",
            description="Optional SSH host alias or user@host for the tunnel",
        ),
        DeclareLaunchArgument(
            "instruction",
            default_value="",
            description="Task instruction passed to the VLA client",
        ),
        DeclareLaunchArgument(
            "server_url",
            default_value="",
            description="Override the OpenPI WebSocket URL from control.yaml",
        ),
        DeclareLaunchArgument(
            "replan_steps",
            default_value="",
            description="Override actions per prediction; use 1 for Nintendo control",
        ),
        DeclareLaunchArgument(
            "gripper_xyz",
            default_value="",
            description="Optional initial [x, y, z] in metres in the robot base frame; preserve starting orientation",
        ),
        DeclareLaunchArgument(
            "trajectory_dir",
            default_value="task0",
            description="Rollout dataset directory; None disables recording",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="false",
            choices=["true", "false"],
            description="Preview policy targets without sending motion goals",
        ),
        DeclareLaunchArgument(
            "control_config",
            default_value=str(own / "config" / "control.yaml"),
            description="Driver motion settings and policy client parameters",
        ),
        DeclareLaunchArgument(
            "camera_config",
            default_value=str(own / "config" / "cameras.yaml"),
            description="Overview and wrist camera parameter groups",
        ),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=configure)])
