"""Geometry invariants for the PX4 velocity adapter."""
import math
import unittest
import numpy as np

from depth_filter.depth_filters import DepthFilters
from flight_bridge.velocity_math import (arrival_deadband_active,
                                         body_flu_to_ned)
from vio_bridge.vio_quality import (feature_confidence, normalized_covariance,
                                    quality_state)


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

    def test_vio_quality_combines_covariance_features_and_motion(self):
        ratio = normalized_covariance(
            [.1, .1, .1], [.025, .025, .025], [.1, .1, .1], [1., .25, 1.])
        self.assertAlmostEqual(ratio, .1)
        self.assertAlmostEqual(feature_confidence(40, 20, 60), .5)
        confidence, reasons = quality_state(
            ratio, 40, 20, 60, 7., 6., .7, .6)
        self.assertLessEqual(confidence, .5)
        self.assertIn('FEATURES_LOW', reasons)
        self.assertIn('VELOCITY_CHANGE_HIGH', reasons)
        self.assertIn('ANGULAR_RATE_HIGH', reasons)

    def test_depth_filter_fills_small_hole_and_smooths_stable_depth(self):
        filters = DepthFilters(spatial_kernel=3, temporal_alpha=.5,
                               temporal_max_delta=2., hole_min_neighbors=5)
        first = np.ones((3, 3), np.float32)
        first[1, 1] = np.nan
        self.assertAlmostEqual(float(filters.apply(first)[1, 1]), 1.)
        second = np.full((3, 3), 2., np.float32)
        self.assertTrue(np.allclose(filters.apply(second), 1.5))

if __name__ == '__main__':
    unittest.main()
