"""Bounded vertical feedback controller for flight before horizontal aiding."""
import math


class BootstrapHeightController:
    def __init__(self, hover, minimum, maximum, height_gain, velocity_gain,
                 integral_gain, climb_speed, thrust_slew):
        values = (hover, minimum, maximum, height_gain, velocity_gain,
                  integral_gain, climb_speed, thrust_slew)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('bootstrap controller parameters must be finite')
        if not (0 < minimum < hover < maximum < 1 and
                min(height_gain, velocity_gain, climb_speed, thrust_slew) > 0 and integral_gain >= 0):
            raise ValueError('invalid bootstrap thrust limits or feedback gains')
        self.hover, self.minimum, self.maximum = hover, minimum, maximum
        self.height_gain, self.velocity_gain, self.integral_gain = height_gain, velocity_gain, integral_gain
        self.climb_speed, self.thrust_slew = climb_speed, thrust_slew
        self.integral = 0.
        self.thrust = minimum

    def update(self, height_error, vertical_speed, dt):
        if not all(math.isfinite(v) for v in (height_error, vertical_speed, dt)) or dt <= 0:
            raise ValueError('invalid bootstrap feedback')
        dt = min(dt, .1)
        desired_speed = max(-self.climb_speed, min(self.climb_speed, self.height_gain * height_error))
        error = desired_speed - vertical_speed
        candidate = max(-.15, min(.15, self.integral + self.integral_gain * error * dt))
        raw = self.hover + self.velocity_gain * error + candidate
        # Do not accumulate against a saturated actuator or during spool-up.
        if self.minimum < raw < self.maximum and abs(raw - self.thrust) <= self.thrust_slew * dt:
            self.integral = candidate
        target = max(self.minimum, min(self.maximum, self.hover + self.velocity_gain * error + self.integral))
        delta = self.thrust_slew * dt
        self.thrust += max(-delta, min(delta, target - self.thrust))
        return self.thrust
