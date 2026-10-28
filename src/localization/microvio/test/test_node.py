import math
import time
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
    n.connected = True
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


def test_mavros_is_the_only_serial_owner(node):
    assert not hasattr(node, 'connect')
    assert not hasattr(node, 'poll')
    assert not hasattr(node, 'link')
    from mavros_msgs.msg import State
    node.on_mavros_state(State(connected=True, armed=False, mode='POSCTL'))
    assert node.flight_mode == 'POSCTL'


def test_connection_health_survives_high_rate_raw_telemetry_burst(node):
    from mavros_msgs.msg import Mavlink, State
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    raw = node.create_publisher(Mavlink, '/uas1/mavlink_source',
                               QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
    state = node.create_publisher(State, '/mavros/state',
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic()+2.
    while (not raw.get_subscription_count() or not state.get_subscription_count()) and time.monotonic()<deadline:
        rclpy.spin_once(node, timeout_sec=.01)
    assert raw.get_subscription_count() and state.get_subscription_count()
    node.heartbeat_at = -math.inf
    state.publish(State(connected=True, armed=False, mode='ALTCTL'))
    for _ in range(100):raw.publish(Mavlink(sysid=1, compid=1, msgid=31))
    deadline = time.monotonic()+2.
    while not math.isfinite(node.heartbeat_at) and time.monotonic()<deadline:
        rclpy.spin_once(node, timeout_sec=.01)
    assert math.isfinite(node.heartbeat_at)
    node.on_odometry(packet())
    assert node.health()[0] == 'VALID'
    node.heartbeat_at -= float(node.p('heartbeat_timeout'))+1.
    assert node.health() == ('INVALID', 'PX4_HEARTBEAT_STALE')


def test_mavros_timesync_rejects_high_latency_and_clears_on_disconnect(node):
    from mavros_msgs.msg import State, TimesyncStatus
    before = node.sync.offset_ns
    node.on_timesync(TimesyncStatus(remote_timestamp_ns=10_000_000_000,
                                    estimated_offset_ns=1234, round_trip_time_ms=200.))
    assert node.sync.offset_ns == before
    node.on_timesync(TimesyncStatus(remote_timestamp_ns=10_000_000_000,
                                    estimated_offset_ns=1234, round_trip_time_ms=2.))
    assert node.sync.offset_ns == 1234
    node.on_mavros_state(State(connected=False))
    assert node.sync.offset_ns is None
    assert node.reset_index == 1


def test_dedicated_mavros_tof_rejects_stale_and_invalid_samples(node):
    from sensor_msgs.msg import Range
    node.range_pub = Mock()
    msg = Range(min_range=.02, max_range=12., range=.04)
    msg.header.stamp = node.get_clock().now().to_msg()
    node.on_mavros_range(msg)
    assert node.range_distance == pytest.approx(.04)
    assert node.ground_range == pytest.approx(.04)
    assert node.range_pub.publish.call_count == 1
    # A fresh ROS arrival cannot turn an old sample into fresh height feedback.
    msg.header.stamp.sec -= 2
    node.on_mavros_range(msg)
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.range = float('nan')
    node.on_mavros_range(msg)
    assert node.range_pub.publish.call_count == 1


def test_first_airborne_module_sample_preserves_fcu_baro_height(node):
    from mavros_msgs.msg import Altitude
    node.odom_pub = Mock()
    altitude = Altitude(monotonic=100.)
    altitude.header.stamp = node.get_clock().now().to_msg()
    node.on_altitude(altitude)
    assert node.ground_z is None and node.ground_range is None
    node.armed = True
    altitude.monotonic = 100.7
    node.on_altitude(altitude)
    # The first EV sample resets PX4 Z to zero after physical liftoff.
    node.on_odometry(packet(z=0.))
    out = node.odom_pub.publish.call_args.args[0]
    assert out.pose.pose.position.z == pytest.approx(.7, abs=1e-5)


def test_initial_heading_zero_does_not_freeze_later_body_turns(node):
    import numpy as np
    from microvio.geometry import rotation
    node.odom_pub = Mock()
    node.on_odometry(packet(q=[math.sqrt(.5), 0., 0., math.sqrt(.5)]))
    first = node.odom_pub.publish.call_args.args[0]
    q = first.pose.pose.orientation
    np.testing.assert_allclose(rotation([q.w, q.x, q.y, q.z]), np.eye(3), atol=1e-12)
    assert first.pose.pose.position.x == pytest.approx(0.)
    assert first.pose.pose.position.y == pytest.approx(0.)
    assert first.pose.pose.position.z == pytest.approx(0.)
    # Source yaw changes by -90 deg (NED), i.e. +90 deg in ROS.
    node.on_odometry(packet(time_usec=10_001_000))
    second = node.odom_pub.publish.call_args.args[0]
    q = second.pose.pose.orientation
    assert math.atan2(rotation([q.w, q.x, q.y, q.z])[1, 0],
                      rotation([q.w, q.x, q.y, q.z])[0, 0]) == pytest.approx(math.pi/2)


def test_constant_position_mode_is_reported_without_suppressing_odometry(node):
    node.flags = 431
    node.on_odometry(packet(q=[math.sqrt(.5), 0., 0., math.sqrt(.5)]))
    assert node.initial_yaw_alignment.initial_yaw is None
    assert node.provisional_alignment.initial_yaw == pytest.approx(-math.pi/2)
    assert node.published_count == 1
    assert node.health() == ('DEGRADED', 'ESTIMATOR_CONSTANT_POSITION')
    node.flags = 830
    node.on_odometry(packet(time_usec=10_001_000))
    assert node.initial_yaw_alignment.initial_yaw == pytest.approx(0.)
    assert node.published_count == 2
    assert node.reset_index == 1
    assert node.health()[1] == 'ESTIMATOR_RECENT_RESET'


def test_unknown_covariance_still_publishes_with_degraded_diagnostics(node):
    node.on_odometry(packet(pose_covariance=[float('nan')] * 21))
    assert node.published_count == 1
    assert node.health() == ('DEGRADED', 'POSE_COVARIANCE_UNKNOWN')


def test_airborne_initial_origin_preserves_tof_height_after_ekf_z_reset(node):
    import numpy as np
    node.odom_pub = Mock()
    node.flags = 165
    node.on_odometry(packet())
    assert node.initial_yaw_alignment.rotation is None
    node.armed = True
    node.ground_range, node.range_distance = .07, .77
    node.range_at = time.monotonic()
    node.flags = 46
    node.on_odometry(packet(time_usec=10_001_000, x=12., y=8., z=0.))
    out = node.odom_pub.publish.call_args.args[0]
    np.testing.assert_allclose([out.pose.pose.position.x, out.pose.pose.position.y,
                               out.pose.pose.position.z], [0., 0., .7], atol=1e-9)
    assert node.reset_index == 1


def test_unknown_covariance_cannot_mask_estimator_accelerometer_error(node):
    node.on_odometry(packet(pose_covariance=[float('nan')] * 21))
    node.flags |= 2048
    assert node.health() == ('INVALID', 'ESTIMATOR_ACCEL_ERROR')
    assert not node.aiding_ready()
