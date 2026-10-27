"""Yaw mode contracts, using mocked PX4 publishers; never sends flight commands."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.parameter import Parameter

from flight_bridge.flight_bridge import FlightBridge
from flight_bridge.yaw_manager import YawDecision


@pytest.fixture
def bridge():
    rclpy.init()
    n = FlightBridge()
    n.now = lambda: 100.
    n.px4_adapter = Mock()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def pose(n, heading, reset=0, delta=0., valid=True):
    m = VehicleLocalPosition(xy_valid=valid, z_valid=valid, heading=heading,
                             heading_reset_counter=reset, delta_heading=delta)
    n.on_position(m)


def ready_trajectory(n, arrival=False):
    n.map_valid = lambda: True
    n.cmd = object()
    n.cmd_at = n.ego_odom_at = 100.
    n.ego_odom = Odometry()
    n.ego_odom.pose.pose.orientation.w = 1.
    n.trajectory_controller.track = Mock(return_value=SimpleNamespace(
        world_velocity=(-.3, .2, .1), acceleration=(0., 0., 0.),
        arrival_hold=arrival, speed_limited=False, trajectory_id=12))
    n.yaw_manager.decide = Mock(return_value=YawDecision(.5, .2, 'DECOUPLED', .3))


def test_initial_heading_is_valid_latched_and_used_for_position_hold(bridge):
    n = bridge
    n.set_parameters([Parameter('yaw_control', value=False)])
    pose(n, 1., valid=False)
    assert n.initial_heading is None
    pose(n, .6)
    pose(n, -.2)
    n.send_position((1., 2., -1.))
    n.px4_adapter.position.assert_called_once_with((1., 2., -1.), .6)
    assert n.initial_heading == pytest.approx(.6)


@pytest.mark.parametrize('degraded,arrival', [(False, False), (True, False), (False, True)])
def test_fixed_heading_bypasses_path_yaw_and_fov_scaling(bridge, degraded, arrival):
    n = bridge
    n.set_parameters([Parameter('yaw_control', value=False)])
    pose(n, .6)
    pose(n, -.2)
    ready_trajectory(n, arrival)
    assert n.send_ego_command(degraded=degraded)
    n.yaw_manager.decide.assert_not_called()
    call = n.px4_adapter.velocity_body.call_args
    assert call.kwargs['yaw_ned'] == pytest.approx(.6)
    assert call.args[1] == 0.  # No trajectory-derived yaw-rate command.
    assert call.args[2] == pytest.approx(-.2)  # Velocity conversion still uses current heading.
    assert call.args[0][:2] == ((0., 0.) if arrival else (-.3, .2))
    assert n.translation_scale == (0. if arrival else 1.)
    assert n.yaw_mode == 'INITIAL_YAW_HOLD'


def test_enabled_mode_preserves_decoupled_policy(bridge):
    n = bridge
    pose(n, .6)
    pose(n, -.2)
    ready_trajectory(n)
    assert n.send_ego_command()
    n.yaw_manager.decide.assert_called_once()
    call = n.px4_adapter.velocity_body.call_args
    assert call.kwargs == {}
    assert call.args[1] == .2
    assert call.args[0][:2] == (-.15, .1)
    n.send_position((0., 0., -1.))
    assert n.px4_adapter.position.call_args.args[1] == pytest.approx(-.2)


def test_px4_heading_reset_updates_reference_once_without_relatching(bridge):
    pose(bridge, .6)
    pose(bridge, 1.7, reset=1, delta=1.1)
    assert bridge.initial_heading == pytest.approx(1.7)
    pose(bridge, 1.8, reset=1, delta=1.1)
    assert bridge.initial_heading == pytest.approx(1.7)


def test_fixed_yaw_reaches_px4_velocity_setpoint(bridge):
    from flight_bridge.px4_adapter import Px4Adapter
    adapter = Px4Adapter(bridge, 'yaw_test', 1)
    adapter.mode_pub = Mock()
    adapter.setpoint_pub = Mock()
    adapter.audit_pub = Mock()
    adapter.velocity_body((1., 0., .2), 0., math.pi/2, 12, 1., yaw_ned=.6)
    msg = adapter.setpoint_pub.publish.call_args.args[0]
    assert msg.yaw == pytest.approx(.6)
    assert msg.yawspeed == 0.
    assert msg.velocity == pytest.approx([0., 1., -.2], abs=1e-6)
    adapter.velocity_body((1., 0., 0.), .2, 0., 13, 1.)
    msg = adapter.setpoint_pub.publish.call_args.args[0]
    assert math.isnan(msg.yaw)
    assert msg.yawspeed == pytest.approx(-.2)
