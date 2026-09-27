"""Geometry invariants for the PX4 velocity adapter."""
import math
import unittest

from flight_bridge.velocity_math import body_flu_to_ned, limit_horizontal


class CoreGeometryTest(unittest.TestCase):
    def test_body_left_velocity_has_correct_ned_sign(self):
        north, east = body_flu_to_ned(0., 1., 0.)
        self.assertAlmostEqual(north, 0.)
        self.assertAlmostEqual(east, -1.)
        north, east = body_flu_to_ned(1., 0., math.pi / 2)
        self.assertAlmostEqual(north, 0., places=7)
        self.assertAlmostEqual(east, 1.)

    def test_speed_limit_preserves_direction(self):
        self.assertEqual(limit_horizontal(.6, .8, .5), (.3, .4))


if __name__ == '__main__':
    unittest.main()
