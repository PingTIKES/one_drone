"""Takeoff without perception, with PX4 output mocked before node creation."""
from unittest.mock import Mock

import pytest
import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger

from flight_bridge.flight_bridge import FlightBridge
from flight_bridge.flight_supervisor import FlightSupervisor


@pytest.fixture
def bridge(monkeypatch):
    monkeypatch.setattr('flight_bridge.flight_bridge.Px4Adapter', Mock())
    rclpy.init()
    node = FlightBridge()
    node.now = lambda: 100.
    node.on_position(VehicleLocalPosition(
        x=1., y=2., z=-.5, heading=.3, xy_valid=True, z_valid=True))
    node.on_vio(String(data='VALID'))
    node.vio_valid_since = 90.
    node.on_status(VehicleStatus(arming_state=2, nav_state=14))
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_takeoff_reaches_altitude_without_map_depth_or_reset_ack(bridge):
    n = bridge
    # Neither depth nor a fused map has ever arrived. Planner reset ACKs
    # must not be prerequisites for a vertical takeoff either.
    n.reset_epoch = 'pending_reset'
    assert not n.reset_map_ready()
    assert not n.map_valid()
    assert not n.depth_valid()
    response = n.takeoff(Trigger.Request(), Trigger.Response())
    assert response.success
    assert n.state == 'PRESTREAM'
    for _ in range(20):
        n.tick()
    assert n.state == 'ARMING'
    n.px4_adapter.command.assert_any_call(176, 1., 6.)
    n.px4_adapter.command.assert_any_call(400, 1., 0.)
    n.tick()
    assert n.state == 'TAKEOFF'
    n.tick()
    n.px4_adapter.position.assert_called_with((1., 2., -2.5), .3)
    n.on_position(VehicleLocalPosition(
        x=1., y=2., z=-2.5, heading=.3, xy_valid=True, z_valid=True))
    n.tick()
    assert n.takeoff_reached_since == 100.
    n.now = lambda: 101.1
    n.pose_at = n.vio_at = n.status_at = 101.1
    n.tick()
    assert n.state == 'CRUISE'
    assert n.hold_target == (1., 2., -2.5)
    # Reaching altitude does not bypass the separate navigation checks.
    n.tick()
    assert n.state == 'HOLD'
    n.px4_adapter.position.assert_called_with((1., 2., -2.5), .3)
    n.px4_adapter.velocity_body.assert_not_called()


@pytest.mark.parametrize('missing', ['position', 'vio'])
def test_missing_localization_still_blocks_takeoff(bridge, missing):
    if missing == 'position':
        bridge.position.xy_valid = False
    else:
        bridge.on_vio(String(data='INVALID'))
    response = bridge.takeoff(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert bridge.state == 'IDLE'
    assert ('fresh PX4 local position' if missing == 'position'
            else 'stable VALID VIO') in response.message
    bridge.px4_adapter.command.assert_not_called()


def test_interrupted_takeoff_recovers_without_map_or_depth(bridge):
    n = bridge
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    n.state = 'TAKEOFF'
    n.on_vio(String(data='INVALID'))
    n.tick()
    assert n.state == 'HOLD'
    assert n.hold_resume_state == 'TAKEOFF'
    assert not n.can_auto_recover()
    n.on_vio(String(data='VALID'))
    n.vio_valid_since = 90.
    assert n.can_auto_recover()
    n.tick()
    assert n.state == 'TAKEOFF'
    n.px4_adapter.position.assert_called_with(n.takeoff_target, .3)


@pytest.mark.parametrize('field', ['status_at', 'failsafe', 'arming_state', 'nav_state'])
def test_takeoff_recovery_keeps_px4_state_checks(bridge, field):
    n = bridge
    n.state = 'TAKEOFF'
    n.enter_hold('Estimator interruption', auto_recover=True)
    assert n.can_auto_recover()
    if field == 'status_at':
        n.status_at = 90.
    else:
        setattr(n.status, field, True if field == 'failsafe' else 0)
    assert not n.can_auto_recover()


@pytest.mark.parametrize('map_ready,depth_valid', [(False, True), (True, False)])
def test_navigation_recovery_still_requires_map_and_depth(map_ready, depth_valid):
    assert not FlightSupervisor.may_auto_recover(
        'HOLD', True, map_ready, depth_valid, 'CRUISE',
        True, True, True, True, True, False)
