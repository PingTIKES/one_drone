"""Pose conversions and fixed local-world alignment for ROS odometry."""
import math
import numpy as np

from vio_bridge.vio_geometry import quaternion, rotation


FLIP = np.diag([1., -1., -1.])


class InitialYawAlignment:
    """Latch heading once, keeping gravity vertical and the source origin fixed.

    The rotation is applied to world-frame pose/covariance only. Body-frame
    linear and angular velocities do not change when world axes are renamed.
    Source resets must be reported by the caller, not concealed by relatching.
    """
    def __init__(self):
        self.rotation = None
        self.initial_yaw = None

    def apply(self, position, body_rotation, covariance):
        if self.rotation is None:
            if math.hypot(body_rotation[0, 0], body_rotation[1, 0]) < 1e-6:
                raise ValueError('initial body forward axis is vertical; yaw is undefined')
            self.initial_yaw = math.atan2(body_rotation[1, 0], body_rotation[0, 0])
            c, s = math.cos(self.initial_yaw), math.sin(self.initial_yaw)
            self.rotation = np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])
        axes = np.zeros((6, 6))
        axes[:3, :3] = axes[3:, 3:] = self.rotation
        return (self.rotation @ np.asarray(position),
                quaternion(self.rotation @ body_rotation),
                axes @ np.asarray(covariance).reshape(6, 6) @ axes.T)


def vio_body_pose(position_frd, orientation_frd):
    """PX4 EV FRD world/body -> OpenVINS z-up world/body FLU."""
    return FLIP @ np.asarray(position_frd, dtype=float), \
        FLIP @ rotation(orientation_frd) @ FLIP


def px4_body_pose(position_ned, orientation_ned):
    """PX4 NED/FRD local pose -> NWU/FLU local pose."""
    return FLIP @ np.asarray(position_ned, dtype=float), \
        FLIP @ rotation(orientation_ned) @ FLIP


class VioOdomAlignment:
    def __init__(self):
        self.rotation = self.translation = None

    def latch(self, vio_pose, px4_pose):
        if self.rotation is None:
            vio_position, vio_rotation = vio_pose
            px4_position, px4_rotation = px4_pose
            self.rotation = px4_rotation @ vio_rotation.T
            self.translation = px4_position - self.rotation @ vio_position

    def transform(self, pose):
        if self.rotation is None:
            raise ValueError('VIO/PX4 odom alignment is not initialized')
        position, body_rotation = pose
        return self.rotation @ position + self.translation, \
            quaternion(self.rotation @ body_rotation)
