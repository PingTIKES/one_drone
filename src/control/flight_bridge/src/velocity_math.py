"""Frame conversion for a FLU body command and PX4 local NED velocity."""
import math


def body_flu_to_ned(vx, vy, heading_ned):
    if not all(math.isfinite(v) for v in (vx, vy, heading_ned)):
        raise ValueError('nonfinite velocity or heading')
    c, s = math.cos(heading_ned), math.sin(heading_ned)
    return c * vx + s * vy, s * vx - c * vy


def required_forward_clearance(speed, reaction_time, deceleration, margin):
    """Conservative reaction plus braking distance for planar cruise."""
    if (not all(math.isfinite(v) for v in
                (speed, reaction_time, deceleration, margin)) or
            speed < 0 or reaction_time < 0 or deceleration <= 0 or margin < 0):
        raise ValueError('invalid stopping-distance input')
    return speed * reaction_time + speed * speed / (2.0 * deceleration) + margin


def arrival_deadband_active(active, position_error, planned_speed,
                            position_deadband, velocity_deadband, exit_scale):
    """Latch an arrival deadband with hysteresis for quiet endpoint hover."""
    values = (position_error, planned_speed, position_deadband,
              velocity_deadband, exit_scale)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite arrival-deadband input')
    if (position_error < 0 or planned_speed < 0 or position_deadband <= 0 or
            velocity_deadband <= 0 or exit_scale < 1):
        raise ValueError('invalid arrival-deadband input')
    scale = exit_scale if active else 1.0
    return (position_error <= position_deadband * scale and
            planned_speed <= velocity_deadband * scale)


def yaw_policy(angle, hold_angle, hard_angle, reverse_angle,
               minimum_speed_scale, max_yaw_rate, yaw_rate_gain):
    """Return translation scale, FLU yaw rate and the active yaw regime.

    Small path-direction changes use the quadrotor's lateral motion and keep
    the camera heading. Larger changes blend translation with a slow turn.
    Translation stops only when the requested motion is almost backwards,
    outside the forward camera's usable hemisphere.
    """
    values = (angle, hold_angle, hard_angle, reverse_angle,
              minimum_speed_scale, max_yaw_rate, yaw_rate_gain)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite yaw-policy input')
    if not (0 <= hold_angle < hard_angle < reverse_angle <= math.pi):
        raise ValueError('yaw angles must satisfy 0 <= hold < hard < reverse <= pi')
    if not (0 <= minimum_speed_scale <= 1 and max_yaw_rate > 0 and yaw_rate_gain > 0):
        raise ValueError('invalid yaw-policy scale or gain')

    magnitude = abs(angle)
    if magnitude <= hold_angle:
        return 1.0, 0.0, 'YAW_HOLD'

    sign = 1.0 if angle >= 0 else -1.0
    yaw_rate = sign * min(max_yaw_rate, yaw_rate_gain * (magnitude - hold_angle))
    if magnitude >= reverse_angle:
        return 0.0, yaw_rate, 'YAW_ALIGN_REVERSE'

    progress = min(1.0, (magnitude - hold_angle) / (hard_angle - hold_angle))
    speed_scale = max(minimum_speed_scale,
                      1.0 - (1.0 - minimum_speed_scale) * progress)
    return speed_scale, yaw_rate, ('YAW_ALIGN_SLOW' if magnitude >= hard_angle
                                    else 'YAW_ALIGN_MOVING')
