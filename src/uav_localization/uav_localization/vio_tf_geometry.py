"""Align accepted OpenVINS body poses with PX4's local NWU odom frame."""
import numpy as np

from uav_localization.vio_geometry import quaternion, rotation


FLIP = np.diag([1., -1., -1.])


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
