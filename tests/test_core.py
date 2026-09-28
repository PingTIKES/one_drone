"""Geometry invariants for the PX4 velocity adapter."""
import math
import unittest

from flight_bridge.velocity_math import (body_flu_to_ned,
                                         constrain_body_velocity,
                                         limit_horizontal,
                                         required_forward_clearance)


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

    def test_forward_only_envelope_blocks_blind_motion(self):
        self.assertEqual(constrain_body_velocity(-1., .8, 1.5), (0., 0.))
        self.assertEqual(constrain_body_velocity(2., .4, 1.5), (1.5, 0.))

    def test_required_clearance_includes_reaction_and_braking(self):
        self.assertAlmostEqual(required_forward_clearance(1.5, .5, .6, .5), 3.125)


if __name__ == '__main__':
    unittest.main()
