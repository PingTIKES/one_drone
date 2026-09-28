"""Frame conversion for a FLU body command and PX4 local NED velocity."""
import math


def body_flu_to_ned(vx, vy, heading_ned):
    if not all(math.isfinite(v) for v in (vx, vy, heading_ned)):
        raise ValueError('nonfinite velocity or heading')
    c, s = math.cos(heading_ned), math.sin(heading_ned)
    return c * vx + s * vy, s * vx - c * vy


def limit_horizontal(vx, vy, limit):
    if not all(math.isfinite(v) for v in (vx, vy, limit)) or limit <= 0:
        raise ValueError('invalid velocity limit')
    speed = math.hypot(vx, vy)
    scale = min(1., limit / speed) if speed else 1.
    return vx * scale, vy * scale


def constrain_body_velocity(vx, vy, limit, allow_reverse=False,
                            max_lateral_speed=0.0):
    """Apply the final body-frame motion envelope before sending to PX4."""
    if not all(math.isfinite(v) for v in
               (vx, vy, limit, max_lateral_speed)) or max_lateral_speed < 0:
        raise ValueError('invalid body velocity constraint')
    if not allow_reverse:
        vx = max(0.0, vx)
    vy = max(-max_lateral_speed, min(max_lateral_speed, vy))
    return limit_horizontal(vx, vy, limit)


def required_forward_clearance(speed, reaction_time, deceleration, margin):
    """Conservative reaction plus braking distance for planar cruise."""
    if (not all(math.isfinite(v) for v in
                (speed, reaction_time, deceleration, margin)) or
            speed < 0 or reaction_time < 0 or deceleration <= 0 or margin < 0):
        raise ValueError('invalid stopping-distance input')
    return speed * reaction_time + speed * speed / (2.0 * deceleration) + margin
