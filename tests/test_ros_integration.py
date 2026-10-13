"""Isolated ROS domain and fake services. No hardware drivers are started."""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import TransformStamped, WrenchStamped
from std_srvs.srv import SetBool
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from control_msgs.action import FollowJointTrajectory
from tf2_ros import TransformBroadcaster
from jaco_vla.node import VLAClient, ARM, FINGERS


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.js = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.images = [self.create_publisher(Image, t, qos_profile_sensor_data) for t in
                       ('/camera/camera/color/image_raw', '/camera/wrist/color/image_raw')]
        self.ft = self.create_publisher(WrenchStamped, '/wireless_ft/ftSensor3', qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.count = self.ik_count = self.valid_count = self.goals = self.cancels = 0
        self.publish_sensors = True
        self.collision_valid = True
        self.action_error = False
        self.positions = [0.0, 2.0, 2.0, 0.0, 0.0, 0.0, .7, .7]
        self.create_service(GetPositionIK, '/compute_ik', self.solve)
        self.create_service(GetStateValidity, '/check_state_validity', self.valid)
        self.actions = [ActionServer(self, FollowJointTrajectory, f'/{c}/follow_joint_trajectory',
                       execute_callback=self.execute, cancel_callback=self.cancel)
                       for c in ('jaco_arm_position_controller', 'hand_controller')]
        self.create_timer(.025, self.publish)

    def publish(self):
        if not self.publish_sensors:
            return
        self.count += 1
        stamp = self.get_clock().now().to_msg()
        self.js.publish(JointState(header=__import__('std_msgs.msg', fromlist=['Header']).Header(stamp=stamp),
                                  name=ARM+FINGERS, position=self.positions, velocity=[0.0]*8))
        for publisher in self.images:
            msg = Image()
            msg.header.stamp = stamp
            msg.width, msg.height, msg.step, msg.encoding = 2, 2, 6, 'rgb8'
            msg.data = bytes([64]*12)
            publisher.publish(msg)
        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        self.ft.publish(wrench)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'j2n6s200_link_base', 'j2n6s200_end_effector'
        tf.transform.translation.z = .3
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)

    def solve(self, request, response):
        self.ik_count += 1
        assert request.ik_request.avoid_collisions
        response.error_code.val = 1
        response.solution.joint_state.name = ARM
        response.solution.joint_state.position = self.positions[:6]
        return response

    def valid(self, request, response):
        self.valid_count += 1
        assert request.group_name == ''
        assert len(request.robot_state.joint_state.name) == 8
        response.valid = self.collision_valid
        return response

    def execute(self, handle):
        self.goals += 1
        assert len(handle.request.trajectory.points) == 2
        for point in handle.request.trajectory.points:
            assert all(x == 0.0 for x in point.velocities)
            assert all(x == 0.0 for x in point.accelerations)
        result = FollowJointTrajectory.Result()
        if self.action_error:
            handle.abort()
            result.error_code = -4
            result.error_string = 'Injected path tolerance violation'
        else:
            handle.succeed()
        return result

    def cancel(self, handle):
        self.cancels += 1
        return CancelResponse.ACCEPT

    def close(self):
        for action in self.actions:
            action.destroy()
        self.destroy_node()


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        payload = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        assert len(payload['state']) == 10
        assert payload['return_chunk'] and payload['image_b64'] and payload['wrist_image_b64']
        self.server.received.set()
        time.sleep(.4)
        body = json.dumps({'actions': [] if getattr(self.server, 'malformed', False) else [[0]*7 for _ in range(8)]}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass


@pytest.fixture
def harness(monkeypatch):
    # The test runner requires an isolated domain before importing/initializing ROS.
    import os
    assert os.environ.get('ROS_DOMAIN_ID') == '173'
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    server.received = threading.Event()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    original = Node.__init__
    def patched(self, *args, **kwargs):
        if args and args[0] == 'vla_client':
            kwargs['parameter_overrides'] = [Parameter('server_url', value=f'http://127.0.0.1:{server.server_port}/infer'),
                Parameter('instruction', value='mock task')]
        original(self, *args, **kwargs)
    monkeypatch.setattr(Node, '__init__', patched)
    rclpy.init()
    fake, client = FakeRobot(), VLAClient()
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    executor.add_node(client)
    def spin_until(predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        raise AssertionError(f'Timed out: enabled={client.enabled}, phase={client.phase}, fault={client.fault}')
    spin_until(lambda: 'overview' in client.images and client.joints and client.ik.service_is_ready())
    spin_until(lambda: client.tf.can_transform('j2n6s200_link_base', 'j2n6s200_end_effector', rclpy.time.Time()))
    def ready():
        try:
            client._ready()
            return True
        except Exception:
            return False
    spin_until(ready)
    yield fake, client, server, spin_until
    client.stop('Test complete')
    spin_until(lambda: not client.sending and not client.handles and
               (client.inference is None or client.inference[0].done()))
    client.worker.shutdown(wait=True, cancel_futures=True)
    executor.remove_node(client)
    executor.remove_node(fake)
    client.destroy_node()
    fake.close()
    executor.shutdown()
    rclpy.shutdown()
    server.shutdown()
    server.server_close()


def enable(client):
    response = client._enable(SetBool.Request(data=True), SetBool.Response())
    assert response.success, response.message


def test_http_does_not_block_sensor_callbacks_and_dry_run_has_no_goals(harness):
    fake, client, server, spin = harness
    enable(client)
    spin(lambda: server.received.is_set())
    before = fake.count
    spin(lambda: fake.count > before+6)
    assert client.inference is not None and not client.inference[0].done()
    spin(lambda: fake.valid_count >= 11 and client.phase == 'idle')
    assert client.enabled and fake.goals == 0


def test_stop_discards_late_inference(harness):
    fake, client, server, spin = harness
    enable(client)
    spin(lambda: server.received.is_set())
    client.stop('test stop during HTTP')
    spin(lambda: client.inference is None)
    assert not client.enabled and not client.queue.items
    assert fake.ik_count == 0 and fake.goals == 0


def test_stale_sensor_stops_client(harness):
    fake, client, server, spin = harness
    enable(client)
    fake.publish_sensors = False
    spin(lambda: not client.enabled)
    assert 'stale' in client.fault.lower()
    assert not client.queue.items


def test_collision_failure_never_sends_goal(harness):
    fake, client, server, spin = harness
    fake.collision_valid = False
    client.p['dry_run'], client.p['contract_verified'] = False, True
    spin(lambda: client.arm.server_is_ready() and client.hand.server_is_ready())
    enable(client)
    spin(lambda: not client.enabled)
    assert 'interpolated' in client.fault
    assert fake.goals == 0


def test_action_failure_does_not_advance_next_action(harness):
    fake, client, server, spin = harness
    fake.action_error = True
    client.p['dry_run'], client.p['contract_verified'] = False, True
    spin(lambda: client.arm.server_is_ready() and client.hand.server_is_ready())
    enable(client)
    spin(lambda: not client.enabled)
    assert 'trajectory failed' in client.fault
    assert client.queue.dispatched == 1
    assert not client.queue.items


def test_successful_mock_arm_and_hand_results_resume(harness):
    fake, client, server, spin = harness
    client.p['dry_run'], client.p['contract_verified'] = False, True
    spin(lambda: client.arm.server_is_ready() and client.hand.server_is_ready())
    enable(client)
    spin(lambda: fake.goals >= 4)
    assert client.enabled and client.queue.dispatched >= 2


def test_malformed_http_response_stops_without_goals(harness):
    fake, client, server, spin = harness
    server.malformed = True
    enable(client)
    spin(lambda: not client.enabled)
    assert 'Inference failed' in client.fault
    assert not client.queue.items and fake.goals == 0


def test_force_threshold_stops_and_clears(harness):
    fake, client, server, spin = harness
    enable(client)
    msg = WrenchStamped()
    msg.header.stamp = client.get_clock().now().to_msg()
    msg.wrench.force.x = 100.0
    client._force(msg)
    client._tick()
    assert not client.enabled and 'Force/torque limit' in client.fault
    assert not client.queue.items and fake.goals == 0
