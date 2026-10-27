"""Optional service takeoff remains separate from manual OFFBOARD takeover."""
from unittest.mock import Mock

import pytest
import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger

from flight_bridge.flight_bridge import FlightBridge


@pytest.fixture
def bridge(monkeypatch):
    monkeypatch.setattr('flight_bridge.flight_bridge.Px4Adapter', Mock())
    rclpy.init()
    node = FlightBridge()
    node.now = lambda: 100.
    node.on_position(VehicleLocalPosition(
        x=1., y=2., z=-.5, heading=.3, xy_valid=True, z_valid=True))
    node.on_status(VehicleStatus(arming_state=1, nav_state=2))
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_explicit_takeoff_without_map_depth_or_health(bridge):
    n = bridge
    assert not n.map_valid() and not n.depth_valid()
    n.on_vio(String(data='INVALID'))
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    for _ in range(20):
        n.tick()
    assert n.state == 'ARMING'
    n.px4_adapter.command.assert_any_call(176, 1., 6.)
    n.px4_adapter.command.assert_any_call(400, 1., 0.)
    # OFFBOARD can be selected before arming; wait for actual arm confirmation.
    n.on_status(VehicleStatus(arming_state=1, nav_state=14))
    n.tick()
    assert n.state == 'ARMING'
    n.on_status(VehicleStatus(arming_state=2, nav_state=14))
    n.tick()
    assert n.state == 'TAKEOFF'
    n.px4_adapter.position.assert_called_with((1., 2., -2.5), .3)
    n.on_position(VehicleLocalPosition(
        x=1., y=2., z=-2.5, heading=.3, xy_valid=True, z_valid=True))
    n.tick()
    n.now = lambda: 101.1
    n.pose_at = n.status_at = 101.1
    n.tick()
    assert n.state == 'HOLD'
    n.tick()
    n.px4_adapter.position.assert_called_with((1., 2., -2.5), .3)
    n.px4_adapter.velocity_body.assert_not_called()


def test_takeoff_requires_position_to_compute_relative_height(bridge):
    bridge.position.xy_valid = False
    response = bridge.takeoff(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert 'fresh PX4 local position' in response.message
    bridge.px4_adapter.command.assert_not_called()


def test_rc_exit_cancels_service_takeoff_without_automatic_rearming(bridge):
    n = bridge
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    n.on_status(VehicleStatus(arming_state=2, nav_state=14))
    n.tick()
    assert n.state == 'TAKEOFF'
    n.px4_adapter.command.reset_mock()
    n.on_status(VehicleStatus(arming_state=2, nav_state=2))
    n.tick()
    assert n.state == 'IDLE' and n.takeoff_target is None
    for _ in range(30):
        n.tick()
    n.px4_adapter.command.assert_not_called()
