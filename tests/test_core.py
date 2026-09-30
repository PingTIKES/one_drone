"""Geometry invariants for the PX4 velocity adapter."""
import math
import unittest

from flight_bridge.velocity_math import (arrival_deadband_active,
                                         body_flu_to_ned)


class CoreGeometryTest(unittest.TestCase):
    def test_body_left_velocity_has_correct_ned_sign(self):
        north, east = body_flu_to_ned(0., 1., 0.)
        self.assertAlmostEqual(north, 0.)
        self.assertAlmostEqual(east, -1.)
        north, east = body_flu_to_ned(1., 0., math.pi / 2)
        self.assertAlmostEqual(north, 0., places=7)
        self.assertAlmostEqual(east, 1.)

    def test_arrival_deadband_requires_small_error_and_speed(self):
        self.assertTrue(arrival_deadband_active(
            False, .10, .05, .15, .10, 1.5))
        self.assertFalse(arrival_deadband_active(
            False, .16, .05, .15, .10, 1.5))
        self.assertFalse(arrival_deadband_active(
            False, .10, .11, .15, .10, 1.5))

    def test_arrival_deadband_uses_hysteresis(self):
        self.assertTrue(arrival_deadband_active(
            True, .20, .14, .15, .10, 1.5))
        self.assertFalse(arrival_deadband_active(
            True, .23, .14, .15, .10, 1.5))

if __name__ == '__main__':
    unittest.main()
