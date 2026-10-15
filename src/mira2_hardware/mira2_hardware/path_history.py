"""Bounded, thread-safe display history; no ROS or control dependencies."""
from collections import deque
import math
import threading


class PathHistory:
    def __init__(self, max_points=2000, step=0.05):
        if max_points < 1:
            raise ValueError('max_points must be positive')
        self._poses = deque(maxlen=max_points)
        self._step = step
        self._lock = threading.Lock()

    def append(self, pose):
        with self._lock:
            if self._poses:
                a, b = self._poses[-1].pose.position, pose.pose.position
                if math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) < self._step:
                    return False
            self._poses.append(pose)
            return True

    def snapshot(self):
        # Publishing happens outside the lock, so DDS cannot stall appends.
        with self._lock:
            return list(self._poses)
