import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.parameter import Parameter
from microvio.microvio_node import Microvio
from test_geometry import packet


@pytest.fixture
def node():
    rclpy.init()
    n = Microvio()
    n.link = SimpleNamespace(close=lambda: None)
    n.heartbeat_at = n.estimator_at = time.monotonic()
    n.flags = 47
    n.sync.offset_ns = n.get_clock().now().nanoseconds - 10_000_000_000
    n.sync.last_reply_at = time.monotonic()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_real_node_constructs_and_quality_zero_is_unknown(node):
    node.on_odometry(packet())
    assert node.published_count == 1
    assert node.health() == ('VALID', 'PX4_ESTIMATE_VALID_MODULE_HEALTH_UNAVAILABLE')
    assert node.quality == 0


def test_local_frd_does_not_require_global_yaw_alignment(node):
    node.on_odometry(packet())
    node.flags = 830
    assert node.health()[0] == 'VALID'
    assert node.required_flags() == 46


def test_ned_reference_still_requires_global_attitude_flag(node):
    node.set_parameters([Parameter('external_vision_reference', value='ned')])
    node.on_odometry(packet())
    node.flags = 830
    assert node.health() == ('DEGRADED', 'ESTIMATOR_FLAGS_INCOMPLETE')


def test_local_frd_still_rejects_missing_horizontal_aiding(node):
    node.on_odometry(packet())
    node.flags = 830 & ~8
    assert node.health() == ('DEGRADED', 'ESTIMATOR_FLAGS_INCOMPLETE')


def test_reset_counter_and_clock_reboot_are_not_hidden(node):
    node.on_odometry(packet())
    node.on_odometry(packet(time_usec=10_001_000, reset_counter=7))
    assert node.reset_index == 1
    assert node.health()[1] == 'ESTIMATOR_RECENT_RESET'
    node.on_odometry(packet(time_usec=1_000_000, reset_counter=0))
    assert node.reset_index == 2 and node.sync.offset_ns is None
    assert node.health()[0] == 'INVALID'


def test_stale_samples_and_duplicate_stamps_never_make_new_tf(node):
    node.on_odometry(packet())
    node.on_odometry(packet())
    assert node.published_count == 1
    assert node.health()[0] == 'INVALID'
    node.odom_at = -math.inf
    assert node.health()[0] == 'INVALID'


def test_no_flight_commands_sent_by_default_poll(node):
    mav = Mock()
    node.link = SimpleNamespace(mav=mav, recv_match=lambda **kwargs: None, close=lambda: None)
    node.poll()
    assert {call[0] for call in mav.method_calls} == {'heartbeat_send', 'timesync_send'}
