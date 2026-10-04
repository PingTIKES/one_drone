"""Validate EGO samples before they can become PX4 setpoints.

This is a command-integrity check. Collision checking remains in EGO's 3-D
grid map; a valid command here does not imply an obstacle-free path.
"""
import math


class TrajectoryValidator:
    def __init__(self, max_sample_age, max_future_stamp, max_planned_speed,
                 max_planned_acceleration, max_tracking_error):
        limits = (max_sample_age, max_future_stamp, max_planned_speed,
                  max_planned_acceleration, max_tracking_error)
        if not all(math.isfinite(v) and v > 0 for v in limits):
            raise ValueError('trajectory validation limits must be positive and finite')
        self.max_sample_age = max_sample_age
        self.max_future_stamp = max_future_stamp
        self.max_planned_speed = max_planned_speed
        self.max_planned_acceleration = max_planned_acceleration
        self.max_tracking_error = max_tracking_error

    def validate(self, command, now, odom_position=None):
        stamp = command.header.stamp.sec + command.header.stamp.nanosec * 1e-9
        if command.header.frame_id != 'odom' or command.trajectory_flag != command.TRAJECTORY_STATUS_READY:
            return False, 'FRAME_OR_TRAJECTORY_FLAG'
        if not math.isfinite(stamp) or stamp <= 0 or now - stamp > self.max_sample_age or stamp - now > self.max_future_stamp:
            return False, 'TRAJECTORY_TIMESTAMP'
        position = (command.position.x, command.position.y, command.position.z)
        velocity = (command.velocity.x, command.velocity.y, command.velocity.z)
        acceleration = (command.acceleration.x, command.acceleration.y, command.acceleration.z)
        if not all(math.isfinite(v) for v in position + velocity + acceleration):
            return False, 'NONFINITE_TRAJECTORY'
        if math.hypot(*velocity[:2]) > self.max_planned_speed or math.hypot(*acceleration[:2]) > self.max_planned_acceleration:
            return False, 'TRAJECTORY_DYNAMIC_LIMIT'
        if odom_position is not None:
            actual = (odom_position.x, odom_position.y, odom_position.z)
            if all(math.isfinite(v) for v in actual) and math.dist(position, actual) > self.max_tracking_error:
                return False, 'TRAJECTORY_TRACKING_ERROR'
        return True, 'OK'
