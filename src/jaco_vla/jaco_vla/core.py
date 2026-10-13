"""ROS-independent policy contract, geometry, limits and action scheduling."""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np


def finite_vector(value, size):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f'Expected {size} finite numbers')
    return a


def normalize_quaternion(q):
    q = finite_vector(q, 4)
    n = np.linalg.norm(q)
    if n < 1e-9:
        raise ValueError('Zero quaternion')
    return q / n


def quaternion_product(a, b):
    a, b = normalize_quaternion(a), normalize_quaternion(b)
    av, bv, aw, bw = a[:3], b[:3], a[3], b[3]
    return normalize_quaternion(np.r_[aw*bv + bw*av + np.cross(av, bv), aw*bw - av@bv])


def rotvec_quaternion(v):
    v = finite_vector(v, 3)
    theta = np.linalg.norm(v)
    if theta < 1e-9:
        return normalize_quaternion(np.r_[v * 0.5, 1.0])
    return np.r_[v * math.sin(theta/2)/theta, math.cos(theta/2)]


def quaternion_rotvec(q):
    q = normalize_quaternion(q)
    if q[3] < 0:
        q = -q
    n = np.linalg.norm(q[:3])
    return np.zeros(3) if n < 1e-9 else q[:3] * (2*math.atan2(n, q[3])/n)


def rotate(q, v):
    q = normalize_quaternion(q)
    v = finite_vector(v, 3)
    return v + 2*np.cross(q[:3], np.cross(q[:3], v) + q[3]*v)


def apply_delta(position, quaternion, action, translation_frame, rotation_frame,
                max_translation, max_rotation, workspace_min, workspace_max):
    a = finite_vector(action, 7)
    if np.linalg.norm(a[:3]) > max_translation or np.linalg.norm(a[3:6]) > max_rotation:
        raise ValueError('Policy delta exceeds configured Cartesian limits')
    if not -1 <= a[6] <= 1:
        raise ValueError('Gripper policy value must be in [-1, 1]')
    if translation_frame not in ('base', 'tool') or rotation_frame not in ('base', 'tool'):
        raise ValueError('Delta frames must be base or tool')
    q = normalize_quaternion(quaternion)
    p = finite_vector(position, 3) + (a[:3] if translation_frame == 'base' else rotate(q, a[:3]))
    dq = rotvec_quaternion(a[3:6])
    target_q = quaternion_product(q, dq) if rotation_frame == 'tool' else quaternion_product(dq, q)
    if np.any(p < workspace_min) or np.any(p > workspace_max):
        raise ValueError('Cartesian target outside workspace')
    return p, target_q


def parse_actions(data, max_chunk):
    if not isinstance(data, dict) or 'actions' not in data:
        raise ValueError('Response must contain actions')
    a = np.asarray(data['actions'], dtype=float)
    if a.ndim != 2 or a.shape[1] != 7 or not 1 <= len(a) <= max_chunk:
        raise ValueError(f'Expected 1..{max_chunk} rows of 7 actions')
    if not np.isfinite(a).all() or np.any(np.abs(a[:, 6]) > 1):
        raise ValueError('Non-finite actions or gripper values outside [-1,1]')
    return a


def gripper_fraction(positions, opened, closed):
    p = np.asarray(positions, dtype=float)
    lo, hi = np.asarray(opened, dtype=float), np.asarray(closed, dtype=float)
    if np.any(np.abs(hi-lo) < 1e-6):
        raise ValueError('Open and closed finger calibration must differ')
    return float(np.clip((p-lo)/(hi-lo), 0, 1).mean())


def proprio(position, quaternion, grip, layout):
    pose = np.r_[finite_vector(position, 3), quaternion_rotvec(quaternion)]
    if layout == 'pose_grip_7':
        return np.r_[pose, grip].tolist()
    if layout == 'legacy_openpi_10':
        # Policy-domain pseudo joints copied as a contract, not Jaco angles.
        opened = np.array([1.51, 0.93, 1.51, 0.93])
        closed = np.array([0.02, -0.83, 0.02, -0.83])
        return np.r_[pose, closed + (1-grip)*(opened-closed)].tolist()
    raise ValueError('Unknown proprio layout')


def nearest_continuous_angle(solution, seed):
    if not math.isfinite(solution) or not math.isfinite(seed):
        raise ValueError('Invalid continuous joint value')
    return seed + math.atan2(math.sin(solution-seed), math.cos(solution-seed))


def motion_duration(start, target, minimum, velocity, acceleration):
    # Rest-to-rest quintic: peak h'=1.875; peak |h''|=10/sqrt(3).
    delta = float(np.max(np.abs(np.asarray(target)-np.asarray(start))))
    if minimum <= 0 or velocity <= 0 or acceleration <= 0:
        raise ValueError('Timing limits must be positive')
    return max(minimum, 1.875*delta/velocity, math.sqrt((10/math.sqrt(3))*delta/acceleration))


@dataclass(frozen=True)
class QueuedAction:
    action: np.ndarray
    observed_at: float


class ActionQueue:
    """Replace overlapping pending deltas, aligned by actions dispatched since observation."""
    def __init__(self, capacity, ttl):
        if capacity < 1 or ttl <= 0:
            raise ValueError('Invalid queue limits')
        self.capacity, self.ttl = capacity, ttl
        self.items = deque()
        self.dispatched = 0
        self.generation = 0

    def clear(self):
        self.items.clear()
        self.generation += 1

    def install(self, actions, request_index, observed_at, generation, now):
        if generation != self.generation or now-observed_at > self.ttl:
            return 0
        skip = self.dispatched-request_index
        if skip < 0:
            raise ValueError('Request index ahead of execution')
        remaining = actions[skip:skip+self.capacity]
        self.items = deque(QueuedAction(a.copy(), observed_at) for a in remaining)
        return len(self.items)

    def pop(self, now):
        if not self.items:
            return None
        if now-self.items[0].observed_at > self.ttl:
            self.clear()
            raise ValueError('Action queue expired')
        self.dispatched += 1
        return self.items.popleft().action
