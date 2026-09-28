"""Bounded quarantine of discontinuous EV measurements; never fabricate poses."""
import math
import numpy as np
from vio_bridge.vio_geometry import rotation


def unexplained_rotation(previous_quat, quat, dt, previous_omega, omega):
    """Return attitude change not explained by measured angular velocity."""
    angle = 2 * math.acos(float(np.clip(abs(np.dot(previous_quat, quat)), 0., 1.)))
    if not math.isfinite(dt) or dt <= 0:
        return angle
    expected = 0.5 * (np.linalg.norm(previous_omega) + np.linalg.norm(omega)) * dt
    return max(0.0, angle - expected)


class VioRecovery:
    def __init__(self, stable_time=.5, timeout=2., max_correction=.75,
                 max_angle_deg=20., sample_gap=.1, residual=.03,
                 max_source_gap=3., gap_max_correction=2.5, gap_max_angle_deg=90.):
        values = (stable_time, timeout, max_correction, max_angle_deg, sample_gap, residual,
                  max_source_gap, gap_max_correction, gap_max_angle_deg)
        if not all(math.isfinite(v) and v > 0 for v in values) or stable_time >= timeout:
            raise ValueError('VIO recovery limits must be positive; stable_time < timeout')
        self.stable_time, self.timeout = stable_time, timeout
        self.max_correction, self.max_angle = max_correction, math.radians(max_angle_deg)
        self.sample_gap, self.residual = sample_gap, residual
        self.max_source_gap = max_source_gap
        self.gap_max_correction = gap_max_correction
        self.gap_max_angle = math.radians(gap_max_angle_deg)
        self.anchor = self.previous = None
        self.source_gap = False
        self.first_candidate_stamp = None
        self.started = self.stable_since = None

    @property
    def active(self):
        return self.started is not None

    def begin(self, now, stamp, position, quat, velocity, omega, source_gap=False):
        self.started = now
        self.anchor = (stamp, position.copy(), quat.copy(), rotation(quat) @ velocity,
                       omega.copy())
        self.previous = self.stable_since = None
        self.source_gap = source_gap
        self.first_candidate_stamp = None

    def expired(self, now):
        return self.active and now - self.started > self.timeout

    def accept(self, now, stamp, pos, quat, velocity, omega):
        """Only admit bounded corrections followed by a stable fresh sequence."""
        if self.expired(now):
            return False
        at, ap, aq, av, aw = self.anchor
        source_interval = stamp-at
        if self.first_candidate_stamp is None and source_interval > self.max_source_gap:
            return False
        angle_residual = unexplained_rotation(aq, quat, source_interval, aw, omega)
        world_velocity = rotation(quat) @ velocity
        correction_limit = self.gap_max_correction if self.source_gap else self.max_correction
        angle_limit = self.gap_max_angle if self.source_gap else self.max_angle
        bounded = (source_interval > 0 and
                   np.linalg.norm(pos - (ap + av * source_interval)) <= correction_limit and
                   angle_residual <= angle_limit)
        continuous = False
        if bounded and self.previous is not None:
            pt, pp, pv = self.previous
            dt = stamp - pt
            continuous = (0 < dt <= self.sample_gap + 1e-9 and
                          np.linalg.norm(pos - pp - pv * dt) <= self.residual)
        if not bounded:
            self.previous = self.stable_since = None
            return False
        if self.first_candidate_stamp is None:
            self.first_candidate_stamp = stamp
        if not continuous:
            self.stable_since = stamp
        self.previous = (stamp, pos.copy(), world_velocity)
        return stamp - self.stable_since >= self.stable_time
