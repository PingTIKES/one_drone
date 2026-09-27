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
