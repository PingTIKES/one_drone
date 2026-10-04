"""Camera-aware yaw policy for an independently planned XYZ trajectory."""
from dataclasses import dataclass
import math

from flight_bridge.velocity_math import yaw_policy


@dataclass(frozen=True)
class YawDecision:
    translation_scale: float
    yaw_rate_flu: float
    mode: str
    bearing: float


def world_to_body(vector, orientation):
    q = orientation
    norm = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
    if norm < 1e-6:
        raise ValueError('invalid odometry orientation')
    w, x, y, z = q.w/norm, q.x/norm, q.y/norm, q.z/norm
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    c, s = math.cos(yaw), math.sin(yaw)
    return (c*vector[0] + s*vector[1],
            -s*vector[0] + c*vector[1], vector[2])


class YawManager:
    def __init__(self, soft_deg, hard_deg, reverse_deg, min_speed_scale,
                 max_yaw_rate, degraded_max_yaw_rate, rate_gain,
                 camera_hfov_deg, lookahead_time, outside_fov_speed_scale):
        self.hold_angle = math.radians(soft_deg)
        self.hard_angle = math.radians(hard_deg)
        self.reverse_angle = math.radians(reverse_deg)
        self.min_speed_scale = min_speed_scale
        self.max_yaw_rate = max_yaw_rate
        self.degraded_max_yaw_rate = degraded_max_yaw_rate
        self.rate_gain = rate_gain
        self.half_fov = math.radians(camera_hfov_deg) / 2
        self.lookahead_time = lookahead_time
        self.outside_fov_speed_scale = outside_fov_speed_scale
        yaw_policy(0., self.hold_angle, self.hard_angle, self.reverse_angle,
                   self.min_speed_scale, self.max_yaw_rate, self.rate_gain)
        if not (0 < camera_hfov_deg < 180 and lookahead_time >= 0 and
                0 <= outside_fov_speed_scale <= 1 and
                0 < degraded_max_yaw_rate <= max_yaw_rate):
            raise ValueError('invalid camera FOV or yaw policy parameters')

    def decide(self, world_velocity, acceleration, orientation, degraded=False,
               arrival_hold=False):
        if arrival_hold:
            return YawDecision(0., 0., 'ARRIVAL_HOLD', 0.)
        bx, by, _ = world_to_body(world_velocity, orientation)
        ax, ay, _ = world_to_body(acceleration, orientation)
        # Acceleration points to the near-future trajectory, not just the
        # instantaneous velocity. Ignore it at a near-zero speed to avoid
        # turning on numerical noise at the endpoint.
        look_x = bx + self.lookahead_time * ax
        look_y = by + self.lookahead_time * ay
        if math.hypot(bx, by) < 0.05:
            look_x, look_y = bx, by
        angle = math.atan2(look_y, look_x)
        scale, rate, mode = yaw_policy(
            angle, self.hold_angle, self.hard_angle, self.reverse_angle,
            self.min_speed_scale,
            self.degraded_max_yaw_rate if degraded else self.max_yaw_rate,
            self.rate_gain)
        if abs(angle) > self.half_fov and scale > self.outside_fov_speed_scale:
            scale, mode = self.outside_fov_speed_scale, 'YAW_FOV_LIMIT'
        return YawDecision(scale, rate, mode, angle)
