"""Changing local world axes must preserve gravity and body control commands."""
import math
import numpy as np
import pytest
from vio_bridge.vio_tf_geometry import InitialYawAlignment
from vio_bridge.vio_geometry import rotation


def rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


@pytest.mark.parametrize('heading', [0., math.pi/2, -math.pi/2, 3.57])
def test_initial_heading_zero_and_forward_motion_is_positive_x(heading):
    align = InitialYawAlignment()
    forward = rz(heading) @ np.array([1., 0., 0.])
    p, q, _ = align.apply(forward, rz(heading), np.eye(6))
    np.testing.assert_allclose(p, [1., 0., 0.], atol=1e-12)
    np.testing.assert_allclose(rotation(q), np.eye(3), atol=1e-12)


def test_alignment_is_latched_and_later_turns_remain_visible():
    align = InitialYawAlignment()
    align.apply([0., 0., 2.], rz(1.), np.eye(6))
    _, q, _ = align.apply([0., 0., 2.], rz(1.4), np.eye(6))
    np.testing.assert_allclose(rotation(q), rz(.4), atol=1e-12)
    assert align.initial_yaw == pytest.approx(1.)


def test_gravity_and_real_roll_pitch_are_preserved():
    tilt = rotation([math.cos(.1), math.sin(.1), 0., 0.])
    align = InitialYawAlignment()
    p, q, _ = align.apply([0., 0., 2.], rz(1.2) @ tilt, np.eye(6))
    np.testing.assert_allclose(rotation(q), tilt, atol=1e-12)
    assert p[2] == 2.


def test_world_pose_covariance_and_control_body_vector_are_consistent():
    align = InitialYawAlignment()
    r = rz(math.pi/2)
    cov = np.diag([1., 2., 3., 4., 5., 6.])
    p, q, new_cov = align.apply([2., 3., 4.], r, cov)
    np.testing.assert_allclose(np.diag(new_cov), [2., 1., 3., 5., 4., 6.])
    np.testing.assert_allclose(r.T @ np.array([2., 3., 4.]), rotation(q).T @ p)


def test_vertical_initial_body_forward_axis_waits_for_defined_heading():
    align = InitialYawAlignment()
    with pytest.raises(ValueError):
        align.apply([0., 0., 0.], np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]]), np.eye(6))
    assert align.rotation is None
