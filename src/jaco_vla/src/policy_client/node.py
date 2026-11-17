"""Asynchronous ROS 2 VLA client with an explicit, fail-closed motion state machine."""

import json
import math
import signal
import time
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from jaco_vla_interfaces.action import CartesianMove

from .core import (
    action_to_base,
    apply_delta,
    finite_vector,
    normalize_quaternion,
    pose_to_world,
    proprio,
)
from .transport import PolicyConnection

# Driver messages use the SDK base frame; policy states and deltas use world.
BASE_FRAME = "j2n6s200_link_base"
# Fixed joint names published by the Jaco Gen2 6DOF/two-finger driver.
ARM_JOINTS = tuple(f"j2n6s200_joint_{i}" for i in range(1, 7))
# Policy order matches simulation: left/thumb, then right/index.
FINGER_JOINTS = ("j2n6s200_joint_finger_1", "j2n6s200_joint_finger_2")
# Startup positioning uses measured feedback, independent of timed-goal success.
INITIAL_POSITION_TOLERANCE_M = 0.005
INITIAL_ORIENTATION_TOLERANCE_RAD = math.radians(5)
INITIAL_POSE_TIMEOUT_S = 30.0

# Required ROS parameter types; values come from control.yaml and launch arguments.
PARAMETER_TYPES = {
    "server_url": Parameter.Type.STRING,
    "instruction": Parameter.Type.STRING,
    "dry_run": Parameter.Type.BOOL,
    "base_position_world": Parameter.Type.DOUBLE_ARRAY,
    "base_quaternion_world": Parameter.Type.DOUBLE_ARRAY,
    "action_translation_scale": Parameter.Type.DOUBLE,
    "action_rotation_scale": Parameter.Type.DOUBLE,
    "finger_open": Parameter.Type.DOUBLE_ARRAY,
    "finger_closed": Parameter.Type.DOUBLE_ARRAY,
    "replan_steps": Parameter.Type.INTEGER,
    "overview_warmup_s": Parameter.Type.DOUBLE,
    "wrist_warmup_s": Parameter.Type.DOUBLE,
    "server_timeout_s": Parameter.Type.DOUBLE,
    "sensor_timeout_s": Parameter.Type.DOUBLE,
    "motion_timeout_s": Parameter.Type.DOUBLE,
}


@dataclass(frozen=True)
class PendingInference:
    """Inference work, observation time (monotonic seconds), and stop epoch."""

    future: Future
    observed_at: float
    epoch: int


class VLAClient(Node):
    """Collect observations, request policy deltas, and execute one goal at a time."""

    def __init__(self):
        super().__init__("vla_client")
        self.config = {}
        for name, parameter_type in PARAMETER_TYPES.items():
            parameter = self.declare_parameter(
                name, parameter_type, ParameterDescriptor(read_only=True)
            )
            if parameter.type_ == Parameter.Type.NOT_SET:
                raise ValueError(
                    f"Missing required parameter '{name}'; "
                    "load control.yaml and supply launch arguments"
                )
            self.config[name] = parameter.value
        self.config["gripper_xyz"] = self.declare_parameter(
            "gripper_xyz",
            Parameter.Type.DOUBLE_ARRAY,
            ParameterDescriptor(read_only=True),
        ).value
        self._validate_config()
        directory = self.declare_parameter(
            "trajectory_dir", "None", ParameterDescriptor(read_only=True)
        ).value
        self.recorder = None
        if directory.strip().lower() not in ("", "none"):
            from .recording import RolloutRecorder

            self.recorder = RolloutRecorder(
                directory, self.config["instruction"], self.config
            )
            self.get_logger().info(
                f"Recording rollout to {self.recorder.dataset.dataset_dir}; "
                "the episode will be saved on shutdown"
            )
        self.policy = PolicyConnection(
            self.config["server_url"],
            self.config["server_timeout_s"],
        )
        self.action_plan = deque()
        self.enabled, self.fault, self.phase = False, "", "waiting"
        self.startup_pending, self.startup_reason = True, ""
        self.initial_target = None
        self.epoch = 0
        self.images, self.joints = {}, {}
        self.camera_started_at = {}
        self.joint_received_at = 0.0
        self.goal_handle = None
        self.goal_pending = False
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vla_infer")
        self.inference = None
        self.requests = self.responses = 0
        self.latency = 0.0
        self.motion = ActionClient(self, CartesianMove, "/jaco_cartesian/move")
        self.current_pose, self.pose_received_at = None, 0.0
        self.create_subscription(
            PoseStamped,
            "/jaco_cartesian/pose",
            self._pose_message,
            qos_profile_sensor_data,
        )
        self.heartbeat = self.create_publisher(Empty, "/jaco_cartesian/heartbeat", 10)
        self.create_subscription(
            JointState, "/joint_states", self._joints, qos_profile_sensor_data
        )
        for key, topic in (
            ("overview", "/camera/camera/color/image_raw"),
            ("wrist", "/camera/wrist/color/image_raw"),
        ):
            self.create_subscription(
                Image,
                topic,
                lambda msg, k=key: self._image(k, msg),
                qos_profile_sensor_data,
            )
        self.create_service(Trigger, "~/stop", self._stop_service)
        self.status_publisher = self.create_publisher(String, "~/status", 10)
        self.create_timer(0.005, self._tick)
        self.create_timer(1.0, self._status)
        self.get_logger().info(
            f"Waiting for startup readiness; dry_run={self.config['dry_run']}."
        )

    def _validate_config(self):
        if self.config["gripper_xyz"] is not None:
            try:
                self.config["gripper_xyz"] = finite_vector(
                    self.config["gripper_xyz"], 3
                )
            except ValueError as exc:
                raise ValueError(f"gripper_xyz: {exc}") from exc
        try:
            self.config["base_position_world"] = finite_vector(
                self.config["base_position_world"], 3
            )
        except ValueError as exc:
            raise ValueError(f"base_position_world: {exc}") from exc
        try:
            self.config["base_quaternion_world"] = normalize_quaternion(
                self.config["base_quaternion_world"]
            )
        except ValueError as exc:
            raise ValueError(f"base_quaternion_world: {exc}") from exc
        for key in (
            "action_translation_scale",
            "action_rotation_scale",
            "server_timeout_s",
            "sensor_timeout_s",
            "motion_timeout_s",
        ):
            if not math.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if self.config["replan_steps"] < 1:
            raise ValueError("replan_steps must be a positive integer")
        for key in ("overview_warmup_s", "wrist_warmup_s"):
            if not math.isfinite(self.config[key]) or self.config[key] < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
        for key in ("finger_open", "finger_closed"):
            self.config[key] = finite_vector(self.config[key], 2)
        for key in ("finger_open", "finger_closed"):
            if np.any(self.config[key] < 0) or np.any(self.config[key] > 1.51):
                raise ValueError(f"{key} exceeds finger limits [0, 1.51] radians")
        if np.any(
            np.abs(self.config["finger_closed"] - self.config["finger_open"]) < 1e-6
        ):
            raise ValueError("Open and closed finger calibration must differ")

    def _image(self, key, msg):
        """Keep only frames received after this camera's startup warm-up."""
        now = time.monotonic()
        warmup = self.config[f"{key}_warmup_s"]
        if key not in self.camera_started_at:
            self.camera_started_at[key] = now
            if warmup > 0:
                self.get_logger().info(f"Warming up {key} camera for {warmup:g} s")
        if now - self.camera_started_at[key] < warmup:
            return
        if key not in self.images and warmup > 0:
            self.get_logger().info(f"{key.capitalize()} camera warm-up complete")
        self.images[key] = (msg, now)

    def _joints(self, msg):
        if len(msg.name) != len(msg.position) or not np.isfinite(msg.position).all():
            return
        # Incomplete joint messages must not refresh the complete observation.
        candidate = dict(zip(msg.name, msg.position, strict=True))
        required = ARM_JOINTS + FINGER_JOINTS
        if not all(n in candidate for n in required):
            return
        self.joints = candidate
        self.joint_received_at = time.monotonic()

    def _pose_message(self, msg):
        self.current_pose, self.pose_received_at = msg, time.monotonic()

    def _pose(self):
        """Return fresh base-frame position (3,) in metres and xyzw quaternion (4,)."""
        if (
            self.current_pose is None
            or time.monotonic() - self.pose_received_at
            > self.config["sensor_timeout_s"]
        ):
            raise RuntimeError("Missing/stale native Cartesian pose")
        msg = self.current_pose
        if msg.header.frame_id != BASE_FRAME:
            raise RuntimeError(f"Native Cartesian pose must use frame {BASE_FRAME}")
        p, q = msg.pose.position, msg.pose.orientation
        return finite_vector([p.x, p.y, p.z], 3), normalize_quaternion(
            [q.x, q.y, q.z, q.w]
        )

    def _ready(self):
        """Require complete observations received within sensor_timeout_s.

        ROS header timestamps and cross-sensor synchronization are not checked.
        """
        now = time.monotonic()
        if (
            not self.joints
            or now - self.joint_received_at > self.config["sensor_timeout_s"]
        ):
            raise RuntimeError("Missing/stale complete joint state")
        for key in ("overview", "wrist"):
            if (
                key not in self.images
                or now - self.images[key][1] > self.config["sensor_timeout_s"]
            ):
                raise RuntimeError(f"Missing/stale {key} image")
        self._pose()
        if not self.motion.server_is_ready():
            raise RuntimeError("Native Cartesian action server unavailable")

    def _start_when_ready(self):
        """Start once after readiness; a later stop or fault requires a restart."""
        try:
            self._ready()
        except Exception as exc:
            reason = str(exc)
            if reason != self.startup_reason:
                self.get_logger().info(f"Waiting for startup readiness: {reason}")
                self.startup_reason = reason
            return
        self.action_plan.clear()
        self.startup_pending, self.startup_reason = False, ""
        self.enabled, self.phase = True, "idle"
        self.epoch += 1
        xyz = self.config["gripper_xyz"]
        if xyz is not None:
            if self.config["dry_run"]:
                self.get_logger().info(
                    f"DRY RUN initial gripper_xyz={xyz.tolist()}; no goals sent"
                )
            else:
                _, quaternion = self._pose()
                fingers = [self.joints[name] for name in FINGER_JOINTS]
                self.initial_target = (xyz, quaternion, fingers)
                self.initial_pose_deadline = time.monotonic() + INITIAL_POSE_TIMEOUT_S
                self.startup_reason = "initial gripper pose"
                self.get_logger().info(
                    f"Moving to initial base-frame gripper_xyz={xyz.tolist()} before inference"
                )
        if self.initial_target is None:
            self.get_logger().info("Startup ready; inference started automatically")

    def _move_to_initial_pose(self, now):
        """Repeat timed goals until feedback reaches the base-frame startup pose.

        Preserve the orientation and finger positions captured at startup. These
        positioning goals are neither policy actions nor recorded rollout steps.
        """
        target_position, target_quaternion, fingers = self.initial_target
        position, quaternion = self._pose()
        position_error = np.linalg.norm(target_position - position)
        # q and -q represent the same orientation.
        orientation_alignment = abs(np.dot(target_quaternion, quaternion))
        if (
            position_error <= INITIAL_POSITION_TOLERANCE_M
            and orientation_alignment >= math.cos(INITIAL_ORIENTATION_TOLERANCE_RAD / 2)
        ):
            self.initial_target, self.startup_reason = None, ""
            self.get_logger().info("Initial gripper pose reached; starting inference")
            return
        self._send_goal(target_position, target_quaternion, fingers, now)

    def _stop_service(self, req, res):
        self.stop("Operator stop")
        res.success, res.message = True, "Disabled; trajectory cancellation requested"
        return res

    def stop(self, reason):
        """Latch a stop, invalidate pending inference, and cancel an accepted goal."""
        self.startup_pending, self.startup_reason = False, ""
        self.enabled, self.fault, self.phase = False, str(reason), "idle"
        self.initial_target = None
        self.epoch += 1
        self.action_plan.clear()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.policy.close()
        self.get_logger().warn(str(reason))

    def _status(self):
        status = {
            "enabled": self.enabled,
            "dry_run": self.config["dry_run"],
            "phase": self.phase,
            "waiting_for": self.startup_reason,
            "fault": self.fault,
            "queue": len(self.action_plan),
            "requests": self.requests,
            "responses": self.responses,
            "inference_latency_s": round(self.latency, 3),
            "inference_pending": self.inference is not None,
        }
        self.status_publisher.publish(String(data=json.dumps(status)))

    def _poll_inference(self, now):
        """Install a completed policy response only if its execution epoch is current."""
        if self.inference is None:
            return
        request = self.inference
        future = request.future
        if not future.done():
            if (
                self.enabled
                and now - request.observed_at > self.config["server_timeout_s"] + 0.5
            ):
                self.stop("Inference deadline exceeded")
            return
        self.inference = None
        try:
            actions = future.result()
            if request.epoch != self.epoch or not self.enabled:
                return
            self.latency = now - request.observed_at
            self.responses += 1
            steps = self.config["replan_steps"]
            if len(actions) < steps:
                raise ValueError(
                    f"Expected at least {steps} actions for replan_steps; got {len(actions)}"
                )
            if self.recorder is not None:
                self.recorder.append_embedding(self.policy.embedding)
            self.action_plan.extend(actions[:steps])
        except Exception as exc:
            if request.epoch == self.epoch and self.enabled:
                self.stop(f"Inference failed: {exc}")

    def _request(self, now):
        """Request a new plan from latest observations after all previous goals finish."""
        if self.inference is not None or self.action_plan or self.phase == "executing":
            return
        state = self._policy_state(*self._pose())
        self.get_logger().info(
            "Sending policy state [x, y, z, rx, ry, rz, left_finger, right_finger] "
            "(world frame, m/rad): "
            f"{np.array2string(np.asarray(state), precision=6, suppress_small=True, max_line_width=180)}"
        )
        # Snapshot message references; subscription callbacks replace, never mutate.
        overview, wrist = (self.images[k][0] for k in ("overview", "wrist"))
        future = self.worker.submit(
            self.policy.infer,
            overview,
            wrist,
            state,
            self.config["instruction"],
        )
        self.inference = PendingInference(future, now, self.epoch)
        self.requests += 1

    def _policy_state(self, position, quaternion):
        """Return world-frame proprioception (8,) from a measured SDK base pose."""
        position, quaternion = pose_to_world(
            position,
            quaternion,
            self.config["base_position_world"],
            self.config["base_quaternion_world"],
        )
        fingers = [self.joints[n] for n in FINGER_JOINTS]
        return proprio(position, quaternion, fingers)

    def _tick(self):
        """Check stream activity and dispatch the next action when the driver is idle."""
        now = time.monotonic()
        try:
            self._poll_inference(now)
            if self.startup_pending:
                # Establish the driver's heartbeat before any startup motion.
                self.heartbeat.publish(Empty())
                self._start_when_ready()
                return
            if not self.enabled:
                return
            self._ready()
            if self.initial_target is not None and now > self.initial_pose_deadline:
                raise RuntimeError(
                    "Initial gripper pose was not reached before timeout"
                )
            if self.phase == "executing" and now > self.deadline:
                raise RuntimeError(f"{self.phase} deadline exceeded")
            self.heartbeat.publish(Empty())
            if self.phase == "executing":
                return
            if self.initial_target is not None:
                self._move_to_initial_pose(now)
                return
            if self.action_plan:
                self._dispatch(self.action_plan.popleft(), now)
            else:
                self._request(now)
        except Exception as exc:
            self.stop(f"Client fault: {exc}")

    def _dispatch(self, action, now):
        """Convert a world-frame policy delta into an absolute SDK base-frame goal."""
        action_log = (
            "policy action [dx, dy, dz, rx, ry, rz, grip] "
            "(world frame, policy units): "
            f"{np.array2string(action, precision=6, suppress_small=True, max_line_width=160)}"
        )
        position, quaternion = self._pose()
        step = None
        if self.recorder is not None and not self.config["dry_run"]:
            step = (
                self.images["overview"][0],
                self.images["wrist"][0],
                self._policy_state(position, quaternion),
                np.array(action, copy=True),
                time.monotonic(),
            )
        action = action_to_base(
            action,
            self.config["base_quaternion_world"],
            self.config["action_translation_scale"],
            self.config["action_rotation_scale"],
        )
        target_position, target_quaternion = apply_delta(position, quaternion, action)
        fingers = self.config["finger_open"] + ((action[6] + 1) / 2) * (
            self.config["finger_closed"] - self.config["finger_open"]
        )
        if self.config["dry_run"]:
            self.get_logger().info(f"DRY RUN {action_log}; no goals sent")
            return
        self.get_logger().info(f"Executing {action_log}")
        self._send_goal(target_position, target_quaternion, fingers, now, step)

    def _send_goal(self, target_position, target_quaternion, fingers, now, step=None):
        """Send one base-frame pose (metres, xyzw) and two finger angles (radians)."""
        goal = CartesianMove.Goal()
        goal.target.header.frame_id = BASE_FRAME
        goal.target.header.stamp = self.get_clock().now().to_msg()
        (
            goal.target.pose.position.x,
            goal.target.pose.position.y,
            goal.target.pose.position.z,
        ) = map(float, target_position)
        (
            goal.target.pose.orientation.x,
            goal.target.pose.orientation.y,
            goal.target.pose.orientation.z,
            goal.target.pose.orientation.w,
        ) = map(float, target_quaternion)
        goal.finger_positions = list(map(float, fingers))
        goal.timeout_s = self.config["motion_timeout_s"]
        self.phase, self.deadline = "executing", now + goal.timeout_s + 2.0
        self.goal_pending = True
        self.motion.send_goal_async(goal).add_done_callback(
            lambda future, epoch=self.epoch: self._accepted(future, epoch, step)
        )

    def _accepted(self, future, epoch, step=None):
        """Track an accepted goal; cancel it if a stop preceded its acceptance."""
        self.goal_pending = False
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                if epoch == self.epoch:
                    self.stop("cartesian goal rejected")
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(
                lambda result, epoch=epoch: self._result(result, epoch)
            )
            if epoch != self.epoch or not self.enabled:
                handle.cancel_goal_async()
            if step is not None:
                self.recorder.append(*step)
        except Exception as exc:
            self.stop(f"cartesian goal response failed: {exc}")

    def _result(self, future, epoch):
        self.goal_handle = None
        if epoch != self.epoch or not self.enabled:
            return
        try:
            result = future.result()
            if (
                result.status != GoalStatus.STATUS_SUCCEEDED
                or not result.result.success
            ):
                raise RuntimeError(f"status={result.status}: {result.result.message}")
            if not self.goal_pending:
                self.phase = "idle"
        except Exception as exc:
            self.stop(f"cartesian goal failed: {exc}")

    def close(self):
        """Keep ROS alive briefly to cancel accepted or late-arriving goals."""
        self.stop(self.fault or "Client shutdown")
        # ROS context must still be alive to send cancellation requests.
        end = time.monotonic() + 1.0
        while (
            rclpy.ok()
            and time.monotonic() < end
            and (self.goal_handle is not None or self.goal_pending)
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.worker.shutdown(wait=False, cancel_futures=True)
        self.motion.destroy()
        if self.recorder is not None:
            self.get_logger().info("Saving rollout trajectory")
            directory = self.recorder.save(self.fault)
            if directory is not None:
                self.get_logger().info(f"Saved rollout trajectory: {directory}")


def main(args=None):
    # Leave ROS alive during shutdown so close() can attempt goal cancellation.
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    shutdown_requested = False

    def request_shutdown(signum, frame):
        # Raising KeyboardInterrupt inside a ROS/pybind callback can become TypeError.
        nonlocal shutdown_requested
        shutdown_requested = True

    previous_handlers = {
        sig: signal.signal(sig, request_shutdown)
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    try:
        node = VLAClient()
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
