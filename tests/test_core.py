"""Geometry invariants shared by map alignment and the PX4 velocity adapter."""
import math
import unittest

from one_drone_control.velocity_math import body_flu_to_ned, limit_horizontal
from one_drone_navigation.map_odom import align_map_to_odom


class CoreGeometryTest(unittest.TestCase):
    def test_manual_alignment_places_current_body_at_selected_map_pose(self):
        odom_body = (3.2, -1.1, math.radians(73))
        desired = (1.3, 9.4, math.radians(-20))
        x, y, yaw = align_map_to_odom(desired, odom_body)
        c, s = math.cos(yaw), math.sin(yaw)
        self.assertAlmostEqual(x + c * odom_body[0] - s * odom_body[1], desired[0])
        self.assertAlmostEqual(y + s * odom_body[0] + c * odom_body[1], desired[1])
        self.assertAlmostEqual(math.atan2(math.sin(yaw + odom_body[2]),
                                           math.cos(yaw + odom_body[2])), desired[2])

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
