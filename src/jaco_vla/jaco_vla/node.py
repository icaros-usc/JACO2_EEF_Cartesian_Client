"""Asynchronous ROS 2 VLA client with an explicit, fail-closed motion state machine."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit_msgs.msg import JointConstraint
from tf2_ros import Buffer, TransformListener
from .core import (ActionQueue, apply_delta, finite_vector, gripper_fraction,
                   motion_duration, normalize_quaternion, proprio, nearest_continuous_angle)
from .transport import infer

ARM = [f'j2n6s200_joint_{i}' for i in range(1, 7)]
FINGERS = [f'j2n6s200_joint_finger_{i}' for i in (1, 2)]
DEFAULTS = {
    'server_url': 'http://127.0.0.1:8010/infer', 'instruction': '',
    'dry_run': True, 'contract_verified': False,
    'overview_topic': '/camera/camera/color/image_raw',
    'wrist_topic': '/camera/wrist/color/image_raw',
    'base_frame': 'j2n6s200_link_base', 'eef_frame': 'j2n6s200_end_effector',
    'move_group': 'jaco_arm', 'arm_controller': 'jaco_arm_position_controller',
    'hand_controller': 'hand_controller', 'translation_frame': 'base',
    'rotation_frame': 'tool', 'proprio_layout': 'legacy_openpi_10',
    'arm_joints': ARM, 'finger_joints': FINGERS,
    'continuous_joints': [ARM[i-1] for i in (1, 4, 5, 6)],
    'finger_open': [0.0, 0.0], 'finger_closed': [1.4, 1.4],
    'finger_lower': [0.0, 0.0], 'finger_upper': [1.51, 1.51],
    'max_chunk': 50, 'queue_capacity': 8, 'refill_threshold': 4,
    'action_ttl_s': 8.0, 'server_timeout_s': 3.0, 'sensor_timeout_s': 0.5,
    'max_sensor_skew_s': 0.15, 'service_timeout_s': 2.0,
    'minimum_duration_s': 0.5, 'max_motion_duration_s': 6.0,
    'arm_velocity_rad_s': 0.25, 'arm_acceleration_rad_s2': 0.4,
    'finger_velocity_rad_s': 0.5, 'finger_acceleration_rad_s2': 0.8,
    'max_joint_step_rad': 0.35, 'max_translation_m': 0.03, 'max_rotation_rad': 0.15,
    'workspace_min': [-0.8, -0.8, 0.05], 'workspace_max': [0.8, 0.8, 1.2],
    'collision_step_rad': 0.025, 'minimum_collision_samples': 10,
    'max_start_drift_rad': 0.02, 'settled_velocity_rad_s': 0.03,
    'require_force_sensor': True, 'force_topic': '/wireless_ft/ftSensor3',
    'max_force_n': 4.0, 'max_torque_nm': 1.0, 'image_size': 224,
    'publish_debug_images': False,
}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class VLAClient(Node):
    def __init__(self):
        super().__init__('vla_client')
        self.p = {}
        for name, default in DEFAULTS.items():
            self.p[name] = self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value
        self._validate_config()
        self.queue = ActionQueue(self.p['queue_capacity'], self.p['action_ttl_s'])
        self.enabled, self.fault, self.phase = False, '', 'idle'
        self.epoch = 0
        self.images, self.joints, self.velocities = {}, {}, {}
        self.js_time = self.js_stamp = 0.0
        self.force = None
        self.handles, self.sending = {}, set()
        self.pending = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='vla_http')
        self.inference = None
        self.last_frames = None
        self.requests = self.responses = 0
        self.latency = 0.0
        self.tf = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.arm = ActionClient(self, FollowJointTrajectory,
                                f"/{self.p['arm_controller']}/follow_joint_trajectory")
        self.hand = ActionClient(self, FollowJointTrajectory,
                                 f"/{self.p['hand_controller']}/follow_joint_trajectory")
        self.create_subscription(JointState, '/joint_states', self._joints, qos_profile_sensor_data)
        for key, topic in (('overview', self.p['overview_topic']), ('wrist', self.p['wrist_topic'])):
            self.create_subscription(Image, topic, lambda msg, k=key: self._image(k, msg), qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, self.p['force_topic'], self._force, qos_profile_sensor_data)
        self.create_service(SetBool, '~/enable', self._enable)
        self.create_service(Trigger, '~/stop', self._stop_service)
        self.status = self.create_publisher(String, '~/status', 10)
        self.debug = {k: self.create_publisher(Image, f'~/sent_{k}', 1)
                      for k in ('overview', 'wrist')} if self.p['publish_debug_images'] else {}
        self.create_timer(0.02, self._tick)
        self.create_timer(1.0, self._status)
        self.get_logger().info(f"Started disabled; dry_run={self.p['dry_run']}. Call ~/enable to begin.")

    def _validate_config(self):
        for key in ('server_timeout_s', 'sensor_timeout_s', 'max_sensor_skew_s', 'action_ttl_s',
                    'service_timeout_s', 'minimum_duration_s', 'max_motion_duration_s',
                    'arm_velocity_rad_s', 'arm_acceleration_rad_s2', 'finger_velocity_rad_s',
                    'finger_acceleration_rad_s2', 'max_joint_step_rad', 'max_translation_m',
                    'max_rotation_rad', 'collision_step_rad', 'max_start_drift_rad',
                    'settled_velocity_rad_s', 'max_force_n', 'max_torque_nm'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not 1 <= self.p['refill_threshold'] <= self.p['queue_capacity'] <= self.p['max_chunk'] <= 256:
            raise ValueError('Require 1 <= refill_threshold <= queue_capacity <= max_chunk <= 256')
        if not 2 <= self.p['minimum_collision_samples'] <= 100 or not 16 <= self.p['image_size'] <= 2048:
            raise ValueError('Invalid collision sample count or image size')
        if len(self.p['arm_joints']) != 6 or len(self.p['finger_joints']) != 2:
            raise ValueError('This package supports Jaco Gen2 6DOF with two fingers')
        names = self.p['arm_joints'] + self.p['finger_joints']
        if not set(self.p['continuous_joints']).issubset(self.p['arm_joints']):
            raise ValueError('continuous_joints must be a subset of arm_joints')
        if len(set(names)) != 8:
            raise ValueError('Joint names must be distinct')
        for key in ('finger_open', 'finger_closed', 'finger_lower', 'finger_upper'):
            self.p[key] = finite_vector(self.p[key], 2)
        for key in ('workspace_min', 'workspace_max'):
            self.p[key] = finite_vector(self.p[key], 3)
        if np.any(self.p['workspace_min'] >= self.p['workspace_max']):
            raise ValueError('Invalid workspace bounds')
        if np.any(self.p['finger_lower'] >= self.p['finger_upper']):
            raise ValueError('Invalid finger limits')
        for key in ('finger_open', 'finger_closed'):
            if np.any(self.p[key] < self.p['finger_lower']) or np.any(self.p[key] > self.p['finger_upper']):
                raise ValueError(f'{key} exceeds finger limits')
        gripper_fraction(self.p['finger_open'], self.p['finger_open'], self.p['finger_closed'])
        proprio([0, 0, 0], [0, 0, 0, 1], 0.0, self.p['proprio_layout'])
        if self.p['translation_frame'] not in ('base', 'tool') or self.p['rotation_frame'] not in ('base', 'tool'):
            raise ValueError('Invalid action frame')

    def _image(self, key, msg):
        self.images[key] = (msg, time.monotonic())

    def _joints(self, msg):
        if len(msg.name) != len(msg.position) or not np.isfinite(msg.position).all():
            return
        # Some publishers send only subsets. Keep per-message completeness rather
        # than letting old joints appear fresh when a different subset arrives.
        candidate = dict(zip(msg.name, msg.position))
        required = self.p['arm_joints'] + self.p['finger_joints']
        if not all(n in candidate for n in required):
            return
        self.joints = candidate
        self.velocities = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
        self.js_time, self.js_stamp = time.monotonic(), stamp_seconds(msg.header.stamp)

    def _force(self, msg):
        w = msg.wrench
        self.force = (np.array([w.force.x, w.force.y, w.force.z]),
                      np.array([w.torque.x, w.torque.y, w.torque.z]),
                      time.monotonic(), stamp_seconds(msg.header.stamp))

    def _check_stamp(self, stamp):
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if stamp <= 0 or age < -0.05 or age > self.p['sensor_timeout_s']:
            raise RuntimeError('Sensor/TF timestamp stale or clock mismatch')

    def _pose(self):
        tf = self.tf.lookup_transform(self.p['base_frame'], self.p['eef_frame'], Time())
        self._check_stamp(stamp_seconds(tf.header.stamp))
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), normalize_quaternion([q.x, q.y, q.z, q.w])

    def _ready(self):
        now = time.monotonic()
        if not self.joints or now-self.js_time > self.p['sensor_timeout_s']:
            raise RuntimeError('Missing/stale complete joint state')
        self._check_stamp(self.js_stamp)
        stamps = [self.js_stamp]
        for key in ('overview', 'wrist'):
            if key not in self.images or now-self.images[key][1] > self.p['sensor_timeout_s']:
                raise RuntimeError(f'Missing/stale {key} image')
            stamp = stamp_seconds(self.images[key][0].header.stamp)
            self._check_stamp(stamp)
            stamps.append(stamp)
        if max(stamps)-min(stamps) > self.p['max_sensor_skew_s']:
            raise RuntimeError('Images/joint state exceed maximum timestamp skew')
        self._pose()
        if not self.ik.service_is_ready() or not self.validity.service_is_ready():
            raise RuntimeError('MoveIt IK/state validity services unavailable')
        if not self.p['dry_run']:
            if not self.arm.server_is_ready() or not self.hand.server_is_ready():
                raise RuntimeError('Arm/hand trajectory action servers unavailable')
        if self.p['require_force_sensor']:
            if self.force is None or now-self.force[2] > self.p['sensor_timeout_s']:
                raise RuntimeError('Missing/stale force sensor')
            self._check_stamp(self.force[3])
            f, t = self.force[:2]
            if not np.isfinite(f).all() or not np.isfinite(t).all():
                raise RuntimeError('Invalid force sensor data')
            if np.linalg.norm(f) > self.p['max_force_n'] or np.linalg.norm(t) > self.p['max_torque_nm']:
                raise RuntimeError('Force/torque limit exceeded')

    def _settled(self):
        names = self.p['arm_joints'] + self.p['finger_joints']
        return all(n in self.velocities and math.isfinite(self.velocities[n]) and
                   abs(self.velocities[n]) <= self.p['settled_velocity_rad_s'] for n in names)

    def _enable(self, req, res):
        if not req.data:
            self.stop('Operator disabled')
            res.success, res.message = True, 'Disabled; trajectory cancellation requested'
            return res
        try:
            if self.enabled:
                raise RuntimeError('Already enabled')
            if self.handles or self.sending or self.inference is not None:
                raise RuntimeError('Wait for previous goals/inference to finish before re-enabling')
            if not self.p['instruction'].strip():
                raise RuntimeError('Set a non-empty instruction at launch')
            if not self.p['dry_run'] and not self.p['contract_verified']:
                raise RuntimeError('Verify policy contract/calibration, then set contract_verified=true')
            self._ready()
            if not self._settled():
                raise RuntimeError('Robot must be stationary with complete velocity feedback')
            self.queue.clear()
            self.last_frames = None
            self.enabled, self.fault, self.phase = True, '', 'idle'
            self.epoch += 1
            res.success, res.message = True, 'Enabled'
        except Exception as exc:
            res.success, res.message = False, str(exc)
        return res

    def _stop_service(self, req, res):
        self.stop('Operator stop')
        res.success, res.message = True, 'Disabled; trajectory cancellation requested'
        return res

    def stop(self, reason):
        self.enabled, self.fault, self.phase = False, str(reason), 'idle'
        self.epoch += 1
        self.queue.clear()
        if self.pending is not None:
            self.pending.cancel()
            self.pending = None
        for handle in list(self.handles.values()):
            handle.cancel_goal_async()
        self.get_logger().warn(str(reason))

    def _status(self):
        self.status.publish(String(data=json.dumps({
            'enabled': self.enabled, 'dry_run': self.p['dry_run'], 'phase': self.phase,
            'fault': self.fault, 'queue': len(self.queue.items), 'requests': self.requests,
            'responses': self.responses, 'inference_latency_s': round(self.latency, 3),
            'inference_pending': self.inference is not None})))

    def _poll_inference(self, now):
        if self.inference is None:
            return
        future, index, observed, generation, epoch = self.inference
        if not future.done():
            if self.enabled and now-observed > self.p['server_timeout_s']+0.5:
                self.stop('Inference deadline exceeded')
            return
        self.inference = None
        try:
            actions = future.result()
            if epoch != self.epoch or not self.enabled:
                return
            self.latency = now-observed
            self.responses += 1
            count = self.queue.install(actions, index, observed, generation, now)
            self.get_logger().info(f'Inference {self.latency:.3f}s; pending actions={count}')
        except Exception as exc:
            if epoch == self.epoch and self.enabled:
                self.stop(f'Inference failed: {exc}')

    def _request(self, now):
        if self.inference is not None or len(self.queue.items) >= self.p['refill_threshold']:
            return
        frames = tuple(stamp_seconds(self.images[k][0].header.stamp) for k in ('overview', 'wrist'))
        if self.last_frames is not None and any(a <= b for a, b in zip(frames, self.last_frames)):
            return
        p, q = self._pose()
        fingers = [self.joints[n] for n in self.p['finger_joints']]
        state = proprio(p, q, gripper_fraction(fingers, self.p['finger_open'], self.p['finger_closed']),
                        self.p['proprio_layout'])
        # Snapshot message references; subscription callbacks replace, never mutate.
        overview, wrist = (self.images[k][0] for k in ('overview', 'wrist'))
        future = self.worker.submit(infer, self.p['server_url'], overview, wrist, state,
                                    self.p['instruction'], self.p['image_size'],
                                    self.p['server_timeout_s'], self.p['max_chunk'])
        self.inference = (future, self.queue.dispatched, now, self.queue.generation, self.epoch)
        self.last_frames = frames
        self.requests += 1
        for key, publisher in self.debug.items():
            publisher.publish(self.images[key][0])

    def _tick(self):
        now = time.monotonic()
        try:
            self._poll_inference(now)
            if not self.enabled:
                return
            self._ready()
            if self.phase != 'idle' and now > self.deadline:
                raise RuntimeError(f'{self.phase} deadline exceeded')
            if self.phase in ('ik', 'collision'):
                if now-self.action_observed > self.p['action_ttl_s']:
                    raise RuntimeError('Action expired during planning')
                self._poll_plan(now)
                return
            if self.phase == 'executing':
                return
            if self.phase == 'settling':
                if self.js_time <= self.result_time or not self._settled():
                    return
                self.phase = 'idle'
            if not self._settled():
                raise RuntimeError('Unexpected motion at action boundary')
            self._request(now)
            if self.queue.items:
                self.action_observed = self.queue.items[0].observed_at
                action = self.queue.pop(now)
                self._start_ik(action, now)
        except Exception as exc:
            self.stop(f'Client fault: {exc}')

    def _start_ik(self, action, now):
        p, q = self._pose()
        target_p, target_q = apply_delta(p, q, action, self.p['translation_frame'],
            self.p['rotation_frame'], self.p['max_translation_m'], self.p['max_rotation_rad'],
            self.p['workspace_min'], self.p['workspace_max'])
        self.start = dict(self.joints)
        self.finger_target = self.p['finger_open'] + ((action[6]+1)/2)*(self.p['finger_closed']-self.p['finger_open'])
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name, ik.ik_link_name = self.p['move_group'], self.p['eef_frame']
        ik.avoid_collisions = True
        ik.timeout = Duration(seconds=0.5).to_msg()
        ik.robot_state.is_diff = True
        ik.robot_state.joint_state.name = list(self.start)
        ik.robot_state.joint_state.position = list(self.start.values())
        # IK must also consider the target gripper state.
        for name, value in zip(self.p['finger_joints'], self.finger_target):
            ik.robot_state.joint_state.position[ik.robot_state.joint_state.name.index(name)] = float(value)
        for name in self.p['arm_joints']:
            ik.constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(self.start[name]),
                tolerance_above=self.p['max_joint_step_rad'],
                tolerance_below=self.p['max_joint_step_rad'], weight=1.0))
        pose = PoseStamped()
        pose.header.frame_id = self.p['base_frame']
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, target_p)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, target_q)
        ik.pose_stamped = pose
        self.pending = self.ik.call_async(request)
        self.phase, self.deadline = 'ik', now+self.p['service_timeout_s']

    def _poll_plan(self, now):
        if not self.pending.done():
            return
        result = self.pending.result()
        self.pending = None
        if self.phase == 'ik':
            if result is None or result.error_code.val != 1:
                raise RuntimeError(f'IK failed: {None if result is None else result.error_code.val}')
            solution = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
            self.target = dict(self.start)
            for name in self.p['arm_joints']:
                if name not in solution or not math.isfinite(solution[name]):
                    raise RuntimeError(f'Incomplete/invalid IK joint {name}')
                if name in self.p['continuous_joints']:
                    solution[name] = nearest_continuous_angle(solution[name], self.start[name])
                # Bounded joints are never wrapped or clamped.
                if abs(solution[name]-self.start[name]) > self.p['max_joint_step_rad']:
                    raise RuntimeError('IK branch jump exceeds joint step limit')
                self.target[name] = solution[name]
            self.target.update(zip(self.p['finger_joints'], map(float, self.finger_target)))
            self.duration = max(motion_duration(
                [self.start[n] for n in names], [self.target[n] for n in names],
                self.p['minimum_duration_s'], self.p[v], self.p[a])
                for names, v, a in (
                    (self.p['arm_joints'], 'arm_velocity_rad_s', 'arm_acceleration_rad_s2'),
                    (self.p['finger_joints'], 'finger_velocity_rad_s', 'finger_acceleration_rad_s2')))
            if self.duration > self.p['max_motion_duration_s']:
                raise RuntimeError('Motion requires a duration above configured limit')
            travel = max(abs(self.target[n]-self.start[n]) for n in self.target)
            self.samples = max(self.p['minimum_collision_samples'], math.ceil(travel/self.p['collision_step_rad']))
            self.sample = 0
            self.phase = 'collision'
        else:
            if result is None or not result.valid:
                raise RuntimeError('MoveIt rejected an interpolated arm/hand state')
            self.sample += 1
        if self.sample <= self.samples:
            fraction = self.sample/self.samples
            request = GetStateValidity.Request()
            # Empty group checks the full robot, including arm-hand interactions.
            request.group_name = ''
            request.robot_state.is_diff = True
            request.robot_state.joint_state.name = list(self.target)
            request.robot_state.joint_state.position = [
                float(self.start[n]+fraction*(self.target[n]-self.start[n])) for n in self.target]
            self.pending = self.validity.call_async(request)
            self.deadline = now+self.p['service_timeout_s']
        else:
            self._send(now)

    def _send(self, now):
        self._ready()
        names = self.p['arm_joints'] + self.p['finger_joints']
        if not self._settled() or any(abs(self.joints[n]-self.start[n]) > self.p['max_start_drift_rad'] for n in names):
            raise RuntimeError('Robot moved while planning; refusing stale trajectory')
        if self.p['dry_run']:
            self.get_logger().info(f'DRY RUN validated target, duration={self.duration:.3f}s; no goals sent')
            self.phase = 'idle'
            return
        self.phase, self.deadline = 'executing', now+self.duration+3.0
        epoch = self.epoch
        start_stamp = (self.get_clock().now()+Duration(seconds=0.2)).to_msg()
        for key, client, joints in (('arm', self.arm, self.p['arm_joints']),
                                     ('hand', self.hand, self.p['finger_joints'])):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(joints)
            goal.trajectory.header.stamp = start_stamp
            # Shared future start time; rest-to-rest quintic interpolation.
            for values, seconds in ((self.start, 0.0), (self.target, self.duration)):
                point = JointTrajectoryPoint()
                point.positions = [float(values[n]) for n in joints]
                point.velocities = [0.0]*len(joints)
                point.accelerations = [0.0]*len(joints)
                point.time_from_start = Duration(seconds=seconds).to_msg()
                goal.trajectory.points.append(point)
            goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
            self.sending.add(key)
            client.send_goal_async(goal).add_done_callback(
                lambda f, k=key, e=epoch: self._accepted(f, k, e))

    def _accepted(self, future, key, epoch):
        self.sending.discard(key)
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                if epoch == self.epoch:
                    self.stop(f'{key} trajectory rejected')
                return
            self.handles[key] = handle
            handle.get_result_async().add_done_callback(
                lambda f, k=key, e=epoch: self._result(f, k, e))
            if epoch != self.epoch or not self.enabled:
                handle.cancel_goal_async()
        except Exception as exc:
            self.stop(f'{key} goal response failed: {exc}')

    def _result(self, future, key, epoch):
        self.handles.pop(key, None)
        if epoch != self.epoch or not self.enabled:
            return
        try:
            result = future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise RuntimeError(f'status={result.status}, code={result.result.error_code}: {result.result.error_string}')
            if not self.handles and not self.sending:
                self.phase, self.result_time = 'settling', time.monotonic()
                self.deadline = self.result_time+self.p['service_timeout_s']
        except Exception as exc:
            self.stop(f'{key} trajectory failed: {exc}')

    def close(self):
        self.stop('Client shutdown')
        # ROS context must still be alive to send cancellation requests.
        end = time.monotonic()+1.0
        while rclpy.ok() and time.monotonic() < end and (self.handles or self.sending):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.worker.shutdown(wait=False, cancel_futures=True)
        self.arm.destroy()
        self.hand.destroy()


def main(args=None):
    # Leave ROS alive during SIGINT so close() can attempt goal cancellation.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = VLAClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
