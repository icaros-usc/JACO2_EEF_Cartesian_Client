"""Launch only the GenericSystem mock driver and check controller startup."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration

assert os.environ.get('ROS_DOMAIN_ID') == '173'
logfile = Path(os.environ['ROS_LOG_DIR'])/'launch.log'
with logfile.open('w') as log:
    process = subprocess.Popen(['ros2', 'launch', 'jaco_vla', 'bringup.launch.py',
        'sim:=mock', 'cameras:=false', 'rviz:=false', 'dry_run:=true', 'instruction:=mock test'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    try:
        rclpy.init()
        node = Node('mock_startup_check')
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        ik = node.create_client(GetPositionIK, '/compute_ik')
        validity = node.create_client(GetStateValidity, '/check_state_validity')
        joints = {}
        node.create_subscription(JointState, '/joint_states',
            lambda msg: joints.update(zip(msg.name, msg.position)), 10)
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        seen = []
        node.create_subscription(String, '/vla_client/status', lambda msg: seen.append(msg.data), 10)
        deadline = time.monotonic()+25
        expected = {'joint_state_broadcaster', 'jaco_arm_position_controller', 'hand_controller'}
        states = {}
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f'Launch exited early: {process.returncode}; inspect {logfile}')
            if client.wait_for_service(timeout_sec=.2):
                future = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=1)
                if future.done():
                    states = {c.name: c.state for c in future.result().controller}
                if all(states.get(n) == 'active' for n in expected) and seen and ik.service_is_ready() and validity.service_is_ready() and len(joints) >= 8 and buffer.can_transform('j2n6s200_link_base', 'j2n6s200_end_effector', Time()):
                    import json
                    status = json.loads(seen[-1])
                    assert status['enabled'] is False and status['dry_run'] is True
                    request = GetPositionIK.Request()
                    request.ik_request.group_name = 'jaco_arm'
                    request.ik_request.ik_link_name = 'j2n6s200_end_effector'
                    request.ik_request.avoid_collisions = True
                    request.ik_request.timeout = Duration(seconds=0.5).to_msg()
                    request.ik_request.robot_state.is_diff = True
                    request.ik_request.robot_state.joint_state.name = list(joints)
                    request.ik_request.robot_state.joint_state.position = list(joints.values())
                    tf = buffer.lookup_transform('j2n6s200_link_base', 'j2n6s200_end_effector', Time())
                    pose = PoseStamped()
                    pose.header.frame_id = 'j2n6s200_link_base'
                    pose.pose.position.x = tf.transform.translation.x
                    pose.pose.position.y = tf.transform.translation.y
                    pose.pose.position.z = tf.transform.translation.z
                    pose.pose.orientation = tf.transform.rotation
                    request.ik_request.pose_stamped = pose
                    future = ik.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                    assert future.done(), 'Actual MoveIt IK request timed out'
                    assert future.result().error_code.val == 1, f'Actual MoveIt IK failed: {future.result().error_code.val}'
                    print('PASS: all three mock controllers active; MoveIt collision-aware IK succeeded; VLA client disabled in dry-run mode')
                    break
            rclpy.spin_once(node, timeout_sec=.1)
        else:
            raise RuntimeError(f'Mock startup timeout: controllers={states}; inspect {logfile}')
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
