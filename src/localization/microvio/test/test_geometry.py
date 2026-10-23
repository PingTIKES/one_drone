import math
from types import SimpleNamespace

import numpy as np
import pytest
from microvio.geometry import ClockSync, convert


def packet(**changes):
    covariance = [float('nan')] * 21
    for i in (0, 6, 11, 15, 18, 20):
        covariance[i] = .01
    fields = dict(frame_id=1, child_frame_id=1, q=[1., 0., 0., 0.],
                  x=1., y=2., z=-3., vx=4., vy=5., vz=-6.,
                  rollspeed=.1, pitchspeed=.2, yawspeed=.3,
                  pose_covariance=covariance, velocity_covariance=covariance,
                  quality=0, estimator_type=8, time_usec=10_000_000, reset_counter=6)
    fields.update(changes)
    return SimpleNamespace(**fields)


def test_ned_frd_to_nwu_flu():
    out = convert(packet())
    np.testing.assert_allclose(out['position'], [1., -2., 3.])
    np.testing.assert_allclose(out['velocity'], [4., -5., 6.])
    np.testing.assert_allclose(out['omega'], [.1, -.2, -.3])
    assert np.isfinite(out['pose_covariance']).all()


def test_world_velocity_rotates_into_body_before_ego_adapter():
    q = [math.sqrt(.5), 0., 0., math.sqrt(.5)]
    out = convert(packet(q=q, vx=1., vy=0., vz=0.))
    np.testing.assert_allclose(out['velocity'], [0., 1., 0.], atol=1e-12)
    np.testing.assert_allclose(out['quaternion'], [q[0], 0., 0., -q[3]])


def test_body_velocity_is_not_rotated_twice():
    q = [math.sqrt(.5), 0., 0., math.sqrt(.5)]
    out = convert(packet(q=q, child_frame_id=12, vx=1., vy=2., vz=3.))
    np.testing.assert_allclose(out['velocity'], [1., -2., -3.])


@pytest.mark.parametrize('changes', [dict(frame_id=4), dict(child_frame_id=20),
                                   dict(q=[0., 0., 0., 0.]), dict(x=float('nan'))])
def test_rejects_unsupported_or_invalid_inputs(changes):
    with pytest.raises(ValueError):
        convert(packet(**changes))


def test_unknown_covariance_is_not_filled_with_false_certainty():
    out = convert(packet(pose_covariance=[float('nan')]*21), unknown_variance=123.)
    assert not out['pose_diagonal_known']
    np.testing.assert_allclose(np.diag(out['pose_covariance']), [123.]*6)


def test_timesync_only_accepts_matching_low_rtt_replies():
    sync = ClockSync()
    assert not sync.reply(100, 10, 20, 0., .1)
    sync.request(1_000_000_000)
    assert sync.reply(100_000_000, 1_000_000_000, 1_020_000_000, 1., .1)
    assert sync.offset_ns == 910_000_000
    assert sync.stamp(110_000) == 1_020_000_000
    sync.request(2_000_000_000)
    assert not sync.reply(200_000_000, 2_000_000_000, 2_200_000_000, 2., .1)
