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
