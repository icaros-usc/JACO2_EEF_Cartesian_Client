import math
from types import SimpleNamespace
import numpy as np
import pytest
from jaco_vla.core import (ActionQueue, apply_delta, parse_actions, motion_duration,
                          quaternion_rotvec, rotvec_quaternion, proprio, gripper_fraction)
from jaco_vla.transport import image_rgb, jpeg_b64


def test_late_chunk_replaces_overlap_without_replaying_deltas():
    q = ActionQueue(3, 4.0)
    old = np.array([[i, 0, 0, 0, 0, 0, 0] for i in range(5)], float)
    q.install(old, 0, 0.0, q.generation, 0.0)
    index = q.dispatched
    q.pop(0.1)
    q.pop(0.2)
    new = old.copy()
    new[:, 0] += 10
    assert q.install(new, index, 0.0, q.generation, 0.3) == 3
    assert q.pop(0.4)[0] == 12
    assert q.pop(0.4)[0] == 13
    assert q.pop(0.4)[0] == 14
    assert q.pop(0.4) is None


def test_stop_invalidates_inflight_response_and_expiry_rejects():
    q = ActionQueue(2, 1.0)
    gen = q.generation
    q.clear()
    assert q.install(np.zeros((2, 7)), 0, 0, gen, 0) == 0
    assert q.install(np.zeros((2, 7)), 0, 0, q.generation, 2) == 0
    q.install(np.zeros((2, 7)), 0, 0, q.generation, 0)
    with pytest.raises(ValueError, match='expired'):
        q.pop(2)
    assert not q.items


@pytest.mark.parametrize('actions', [[], [[0]*6], [[0]*8], [[math.nan]*7], [[0]*6+[2]], [0]*7])
def test_malformed_actions_fail(actions):
    with pytest.raises(ValueError):
        parse_actions({'actions': actions}, 50)


def test_delta_frames_are_explicit():
    q = rotvec_quaternion([0, 0, math.pi/2])
    action = [0.01, 0, 0, 0.1, 0, 0, 0]
    args = (0.03, 0.2, np.array([-1]*3), np.array([1]*3))
    p_base, r_tool = apply_delta([0, 0, 0], q, action, 'base', 'tool', *args)
    p_tool, r_base = apply_delta([0, 0, 0], q, action, 'tool', 'base', *args)
    assert np.allclose(p_base, [0.01, 0, 0])
    assert np.allclose(p_tool, [0, 0.01, 0])
    assert not np.allclose(r_tool, r_base)


def test_delta_rejects_oversized_or_out_of_workspace():
    args = ([0, 0, 0], [0, 0, 0, 1])
    with pytest.raises(ValueError, match='limits'):
        apply_delta(*args, [1, 0, 0, 0, 0, 0, 0], 'base', 'tool', .03, .2, [-1]*3, [1]*3)
    with pytest.raises(ValueError, match='workspace'):
        apply_delta(*args, [.01, 0, 0, 0, 0, 0, 0], 'base', 'tool', .03, .2, [-1]*3, [0]*3)


def test_quaternion_sign_does_not_change_state():
    q = rotvec_quaternion([0.1, -.2, .3])
    assert np.allclose(quaternion_rotvec(q), quaternion_rotvec(-q))
    assert np.allclose(quaternion_rotvec(q), [.1, -.2, .3])
    assert len(proprio([0]*3, q, .5, 'legacy_openpi_10')) == 10
    assert len(proprio([0]*3, q, .5, 'pose_grip_7')) == 7
    assert gripper_fraction([.7, .7], [0, 0], [1.4, 1.4]) == .5


def test_quintic_respects_peak_velocity_and_acceleration():
    distance, vmax, amax = .35, .25, .4
    duration = motion_duration([0], [distance], .1, vmax, amax)
    s = np.linspace(0, 1, 10001)
    v = distance/duration*(30*s**2-60*s**3+30*s**4)
    a = distance/duration**2*(60*s-180*s**2+120*s**3)
    assert abs(v).max() <= vmax+1e-8
    assert abs(a).max() <= amax+1e-8


def test_padded_bgr_image_and_invalid_encoding():
    msg = SimpleNamespace(width=1, height=2, step=5, encoding='bgr8', data=bytes([1, 2, 3, 99, 99, 4, 5, 6, 99, 99]))
    assert image_rgb(msg).tolist() == [[[3, 2, 1]], [[6, 5, 4]]]
    assert jpeg_b64(msg, 224)
    msg.encoding = '16UC1'
    with pytest.raises(ValueError, match='Unsupported'):
        image_rgb(msg)


def test_continuous_joint_wrap_preserves_nearby_representation():
    from jaco_vla.core import nearest_continuous_angle
    assert abs(nearest_continuous_angle(-math.pi+.01, math.pi-.01)-(math.pi+.01)) < 1e-8
    assert abs(nearest_continuous_angle(math.pi-.01, -math.pi+.01)-(-math.pi-.01)) < 1e-8
