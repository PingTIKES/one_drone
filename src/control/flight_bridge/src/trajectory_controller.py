"""EGO position/velocity sample tracking, with no PX4 or yaw ownership."""
from dataclasses import dataclass
import math

from flight_bridge.velocity_math import arrival_deadband_active


@dataclass(frozen=True)
class TrackingResult:
    world_velocity: tuple
    acceleration: tuple
    arrival_hold: bool
    speed_limited: bool
    trajectory_id: int


class TrajectoryController:
    def __init__(self, position_gain, max_horizontal_speed, max_vertical_speed,
                 arrival_position_deadband, arrival_velocity_deadband,
                 arrival_exit_scale):
        self.position_gain = float(position_gain)
        self.max_horizontal_speed = float(max_horizontal_speed)
        self.max_vertical_speed = float(max_vertical_speed)
        self.arrival_position_deadband = float(arrival_position_deadband)
        self.arrival_velocity_deadband = float(arrival_velocity_deadband)
        self.arrival_exit_scale = float(arrival_exit_scale)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.position_gain, self.max_horizontal_speed,
                    self.max_vertical_speed)):
            raise ValueError('tracking gains and speeds must be positive')
        arrival_deadband_active(False, 0., 0., self.arrival_position_deadband,
                                self.arrival_velocity_deadband,
                                self.arrival_exit_scale)
        self.arrival_hold = False

    def reset(self):
        self.arrival_hold = False

    def track(self, command, odom, speed_scale=1.0):
        if not math.isfinite(speed_scale) or not 0 < speed_scale <= 1:
            raise ValueError('invalid tracking speed scale')
        actual = odom.pose.pose.position
        error = (command.position.x - actual.x,
                 command.position.y - actual.y,
                 command.position.z - actual.z)
        planned_speed = math.hypot(command.velocity.x, command.velocity.y)
        self.arrival_hold = arrival_deadband_active(
            self.arrival_hold, math.hypot(*error[:2]), planned_speed,
            self.arrival_position_deadband, self.arrival_velocity_deadband,
            self.arrival_exit_scale)
        world = [command.velocity.x + self.position_gain * error[0],
                 command.velocity.y + self.position_gain * error[1],
                 command.velocity.z + self.position_gain * error[2]]
        if self.arrival_hold:
            world[0] = world[1] = 0.0
        horizontal = math.hypot(world[0], world[1])
        limit = self.max_horizontal_speed * speed_scale
        limited = horizontal > limit
        if limited:
            world[0] *= limit / horizontal
            world[1] *= limit / horizontal
        world[2] = max(-self.max_vertical_speed, min(self.max_vertical_speed, world[2]))
        return TrackingResult(tuple(world),
                              (command.acceleration.x, command.acceleration.y, command.acceleration.z),
                              self.arrival_hold, limited, int(command.trajectory_id))
