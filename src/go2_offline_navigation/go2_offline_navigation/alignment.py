"""Alignment confirmation independent of ROS; keeps large corrections gated."""
import math

import numpy as np


def matrix_from_pose(position, orientation):
    q = np.array([orientation.x, orientation.y, orientation.z, orientation.w], dtype=float)
    xyz = np.array([position.x, position.y, position.z], dtype=float)
    if not np.isfinite(q).all() or not np.isfinite(xyz).all() or abs(np.linalg.norm(q) - 1) > 0.001:
        raise ValueError('Non-finite pose or non-unit quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    result = np.eye(4)
    result[:3, :3] = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                     (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                     (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
    result[:3, 3] = xyz
    return result


def rotation_distance(a, b):
    return math.acos(float(np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1)/2, -1, 1)))


class AlignmentGate:
    def __init__(self, translation=0.2, rotation=0.15, recovery_frames=3, max_gap=3.0):
        if (not all(math.isfinite(value) and value > 0 for value in (translation, rotation, max_gap))
                or not isinstance(recovery_frames, int) or isinstance(recovery_frames, bool)
                or recovery_frames < 2):
            raise ValueError('Invalid correction gate parameters')
        self.translation, self.rotation = translation, rotation
        self.recovery_frames = recovery_frames
        self.max_gap = max_gap
        self.reset()

    def reset(self):
        self.accepted = None
        self.anchor = None
        self.count = 0
        self.last_stamp = -1.0

    def observe(self, stamp, matrix):
        if not math.isfinite(stamp) or stamp <= 0 or matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            self.reset()
            return False
        if stamp <= self.last_stamp:
            return False
        if self.last_stamp > 0 and stamp - self.last_stamp > self.max_gap:
            self.reset()
        self.last_stamp = stamp
        reference = self.accepted if self.accepted is not None else self.anchor
        if reference is not None and (
                np.linalg.norm(matrix[:3, 3] - reference[:3, 3]) > self.translation
                or rotation_distance(reference, matrix) > self.rotation):
            self.accepted = None
            self.anchor = matrix.copy()
            self.count = 1
            return False
        if self.accepted is None:
            if self.anchor is None:
                self.anchor = matrix.copy()
            self.count += 1
            if self.count < self.recovery_frames:
                return False
        self.accepted = matrix.copy()
        return True
