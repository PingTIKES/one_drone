"""MAVLink PX4 odometry -> ROS NWU/FLU, with no MAVLink or ROS dependency."""
import math
import numpy as np

FLIP = np.diag([1., -1., -1.])
DIAG = (0, 6, 11, 15, 18, 20)


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not .9 < np.linalg.norm(q) < 1.1:
        raise ValueError('INVALID_QUATERNION')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def covariance_diagonal(values, unknown):
    if len(values) != 21:
        raise ValueError('INVALID_COVARIANCE_LENGTH')
    diagonal = np.array([values[i] for i in DIAG], dtype=float)
    known = np.isfinite(diagonal) & (diagonal >= 0)
    return np.diag(np.where(known, diagonal, unknown)), bool(known.all())


def convert(message, unknown_variance=1000.):
    # MAV_FRAME_LOCAL_NED=1, LOCAL_FRD=20. ROS world is NWU (not ENU).
    if message.frame_id not in (1, 20):
        raise ValueError('UNSUPPORTED_POSE_FRAME')
    r = rotation(message.q)
    position = np.array([message.x, message.y, message.z], dtype=float)
    velocity = np.array([message.vx, message.vy, message.vz], dtype=float)
    omega = np.array([message.rollspeed, message.pitchspeed, message.yawspeed], dtype=float)
    if not all(np.isfinite(v).all() for v in (position, velocity, omega)):
        raise ValueError('NONFINITE_ODOMETRY')
    if message.child_frame_id == 12:  # MAV_FRAME_BODY_FRD
        velocity_transform = FLIP
    elif message.child_frame_id == message.frame_id:
        velocity_transform = FLIP @ r.T
    else:
        raise ValueError('UNSUPPORTED_VELOCITY_FRAME')
    # For F R F, the quaternion is [w, x, -y, -z]. Its global sign is immaterial.
    q = np.asarray(message.q, dtype=float)
    q = q / np.linalg.norm(q) * np.array([1., 1., -1., -1.])
    pc, pose_known = covariance_diagonal(message.pose_covariance, unknown_variance)
    vc, velocity_known = covariance_diagonal(message.velocity_covariance, unknown_variance)
    pose_transform = np.zeros((6, 6))
    pose_transform[:3, :3] = pose_transform[3:, 3:] = FLIP
    twist_transform = np.zeros((6, 6))
    twist_transform[:3, :3] = velocity_transform
    twist_transform[3:, 3:] = FLIP
    return dict(position=FLIP @ position, quaternion=q,
                velocity=velocity_transform @ velocity, omega=FLIP @ omega,
                pose_covariance=pose_transform @ pc @ pose_transform.T,
                twist_covariance=twist_transform @ vc @ twist_transform.T,
                pose_diagonal_known=pose_known, twist_diagonal_known=velocity_known)


class ClockSync:
    """Map PX4 sampling time using the offset provided by MAVROS timesync."""
    def __init__(self):
        self.offset_ns = None
        self.last_reply_at = -math.inf

    def stamp(self, remote_usec):
        if self.offset_ns is None:
            raise ValueError('WAIT_TIMESYNC')
        return int(remote_usec) * 1000 + self.offset_ns
