"""ROS-independent policy contract and pose conversions."""

import numpy as np
from scipy.spatial.transform import Rotation


def finite_vector(value, size):
    """Return a finite float array of shape (size,), or raise ValueError."""
    vector = np.asarray(value, dtype=float)
    if vector.shape != (size,) or not np.isfinite(vector).all():
        raise ValueError(f"Expected {size} finite numbers")
    return vector


def normalize_quaternion(quaternion):
    """Validate and normalize one quaternion in ROS/SciPy [x, y, z, w] order."""
    quaternion = finite_vector(quaternion, 4)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-9:
        raise ValueError("Zero quaternion")
    return quaternion / norm


def pose_to_world(position, quaternion, base_position_world, base_quaternion_world):
    """Transform a gripper pose from SDK base to policy world coordinates.

    Positions are (3,) in metres and quaternions are (4,) in xyzw order.
    The base pose describes the base frame in world. Return world position (3,)
    and world gripper orientation as an xyzw quaternion (4,).
    """
    base_rotation = Rotation.from_quat(normalize_quaternion(base_quaternion_world))
    world_position = base_rotation.apply(finite_vector(position, 3)) + finite_vector(
        base_position_world, 3
    )
    world_rotation = base_rotation * Rotation.from_quat(
        normalize_quaternion(quaternion)
    )
    return world_position, world_rotation.as_quat()


def action_to_base(action, base_quaternion_world, translation_scale, rotation_scale):
    """Scale a world-frame policy delta (7,) and rotate it into the SDK base frame.

    Action is [dx, dy, dz, rx, ry, rz, grip]. The positive scalar scales convert
    policy translation units to metres and rotation-vector units to radians.
    The base quaternion (4,) is xyzw in world; grip is in [-1, 1].
    Return a new array; displacement deltas need no origin offset and grip is unchanged.
    """
    base_action = finite_vector(action, 7).copy()
    base_action[:3] *= translation_scale
    base_action[3:6] *= rotation_scale
    world_to_base = Rotation.from_quat(
        normalize_quaternion(base_quaternion_world)
    ).inv()
    base_action[:3] = world_to_base.apply(base_action[:3])
    base_action[3:6] = world_to_base.apply(base_action[3:6])
    return base_action


def apply_delta(position, quaternion, action):
    """Return an absolute pose from a finite policy action of shape (7,).

    Position is in metres in the SDK base frame;
    quaternion is [x, y, z, w]. Action is [dx, dy, dz, rx, ry, rz, grip],
    with translation in metres, a rotation vector in radians, and grip in
    [-1, 1]. Translation and rotation deltas both use the base frame. Return position
    (3,) and quaternion (4,). The policy server owns action magnitude constraints.
    """
    action = finite_vector(action, 7)
    translation = action[:3]
    rotation_vector = action[3:6]
    if not -1 <= action[6] <= 1:
        raise ValueError("Gripper policy value must be in [-1, 1]")
    orientation = Rotation.from_quat(normalize_quaternion(quaternion))
    target_position = finite_vector(position, 3) + translation
    delta_rotation = Rotation.from_rotvec(rotation_vector)
    # Left multiplication applies the rotation about the base axes.
    target_orientation = delta_rotation * orientation
    return target_position, target_orientation.as_quat()


def parse_actions(data):
    """Return finite actions (N, 7), clamping the normalized gripper to [-1, 1]."""
    if not isinstance(data, dict) or "actions" not in data:
        raise ValueError("Response must contain actions")
    actions = np.array(data["actions"], dtype=float, copy=True)
    if actions.ndim != 2 or actions.shape[1] != 7 or len(actions) < 1:
        raise ValueError("Expected a nonempty array of shape (N, 7)")
    if not np.isfinite(actions).all():
        raise ValueError("Non-finite actions")
    # Model predictions can slightly overshoot the open/closed endpoints.
    actions[:, 6] = np.clip(actions[:, 6], -1, 1)
    return actions


def proprio(position, quaternion, finger_positions):
    """Build [XYZ, rotation vector, left proximal joint, right proximal joint].

    Position (3,) is metres in the policy world frame; quaternion (4,) is xyzw
    in that same frame.
    Finger positions (2,) are measured radians in policy order (left, right).
    Return eight values with orientation represented as a rotation vector in radians.
    """
    quaternion = normalize_quaternion(quaternion)
    rotation_vector = Rotation.from_quat(quaternion).as_rotvec()
    # Retain the original policy adapter's near-identity cutoff.
    if np.linalg.norm(quaternion[:3]) < 1e-9:
        rotation_vector = np.zeros(3)
    # Preserve the policy's signed axis at exactly pi; SciPy canonicalizes this tie.
    if quaternion[3] == 0 and np.dot(rotation_vector, quaternion[:3]) < 0:
        rotation_vector = -rotation_vector
    pose = np.r_[finite_vector(position, 3), rotation_vector]
    return np.r_[pose, finite_vector(finger_positions, 2)].tolist()
