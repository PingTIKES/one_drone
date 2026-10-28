"""Optional service takeoff remains separate from manual OFFBOARD takeover."""
from unittest.mock import Mock

import pytest
import rclpy
from flight_bridge.mavros_adapter import LocalPosition as VehicleLocalPosition
from mavros_msgs.msg import State
from std_msgs.msg import String
from std_srvs.srv import Trigger

from flight_bridge.flight_bridge import FlightBridge


@pytest.fixture
def bridge(monkeypatch):
    monkeypatch.setattr('flight_bridge.flight_bridge.MavrosAdapter', Mock())
    rclpy.init()
    node = FlightBridge()
    node.now = lambda: 100.
    node.on_position(VehicleLocalPosition(
        x=1., y=2., z=-.5, heading=.3))
    node.on_status(State(connected=True, armed=False, mode='POSCTL'))
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_explicit_takeoff_without_map_depth_or_health(bridge):
    n = bridge
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    for _ in range(20):
        n.tick()
    assert n.state == 'ARMING'
    n.px4_adapter.set_mode.assert_any_call('OFFBOARD')
    n.px4_adapter.arm.assert_not_called()
    # OFFBOARD can be selected before arming; wait for actual arm confirmation.
    n.on_status(State(connected=True, armed=False, mode='OFFBOARD'))
    for _ in range(20):
        n.tick()
    assert n.state == 'ARMING'
    n.px4_adapter.arm.assert_called()
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.tick()
    assert n.state == 'TAKEOFF'
    n.px4_adapter.position.assert_called_with((1., 2., -2.5), .3)
    n.on_position(VehicleLocalPosition(
        x=1., y=2., z=-2.5, heading=.3))
    n.tick()
    n.now = lambda: 101.1
    n.pose_at = n.status_at = 101.1
    n.tick()
    assert n.state == 'HOLD'
    n.tick()
    n.px4_adapter.position.assert_called_with((1., 2., -2.5), .3)
    n.px4_adapter.velocity_body.assert_not_called()


def test_takeoff_requires_position_to_compute_relative_height(bridge):
    bridge.position.x = float('nan')
    response = bridge.takeoff(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert 'fresh PX4 local position' in response.message
    bridge.px4_adapter.arm.assert_not_called()
    bridge.px4_adapter.set_mode.assert_not_called()


def test_rc_exit_cancels_service_takeoff_without_automatic_rearming(bridge):
    n = bridge
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.tick()
    assert n.state == 'TAKEOFF'
    n.px4_adapter.arm.reset_mock()
    n.px4_adapter.set_mode.reset_mock()
    n.on_status(State(connected=True, armed=True, mode='POSCTL'))
    n.tick()
    assert n.state == 'IDLE' and n.takeoff_target is None
    for _ in range(30):
        n.tick()
    n.px4_adapter.arm.assert_not_called()
    n.px4_adapter.set_mode.assert_not_called()


def test_unarmed_offboard_timeout_does_not_keep_retrying_arm(bridge):
    n = bridge
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    n.on_status(State(connected=True, armed=False, mode='OFFBOARD'))
    n.tick()
    assert n.state == 'ARMING'
    n.now = lambda: 116.
    n.pose_at = n.status_at = 116.
    n.px4_adapter.arm.reset_mock()
    n.tick()
    assert not n.takeoff_pending and n.takeoff_target is None
    assert n.trajectory_status == 'ARMING_TIMEOUT'
    for _ in range(30):
        n.tick()
    n.px4_adapter.arm.assert_not_called()
