"""Manual OFFBOARD control contracts; all PX4 output is mocked."""
from unittest.mock import Mock

import pytest
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from flight_bridge.flight_bridge import FlightBridge


@pytest.fixture
def bridge(monkeypatch):
    monkeypatch.setattr('flight_bridge.flight_bridge.Px4Adapter', Mock())
    rclpy.init()
    n = FlightBridge()
    n.now = lambda: 100.
    n.on_position(VehicleLocalPosition(
        x=1., y=2., z=-1.5, heading=0., xy_valid=True, z_valid=True))
    n.on_status(VehicleStatus(arming_state=2, nav_state=14))
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.header.stamp.sec = 100
    odom.pose.pose.orientation.w = 1.
    n.on_ego_odom(odom)
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def command(stamp=100., speed=.3):
    msg = PositionCommand()
    msg.header.frame_id = 'odom'
    ns = round(stamp * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
    msg.velocity.x = speed
    return msg


def test_manual_offboard_latches_airborne_position_without_takeoff(bridge):
    n = bridge
    n.tick()
    assert n.state == 'HOLD'
    n.px4_adapter.position.assert_called_with((1., 2., -1.5), 0.)
    n.position.x = 1.1
    n.position.z = -1.4
    n.tick()
    n.px4_adapter.position.assert_called_with((1., 2., -1.5), 0.)
    n.px4_adapter.command.assert_not_called()
    assert n.takeoff_target is None
    assert not n.takeoff(Trigger.Request(), Trigger.Response()).success


@pytest.mark.parametrize('health', [None, 'INVALID', 'DEGRADED'])
def test_fresh_ego_runs_without_health_map_depth_or_resume_service(bridge, health):
    n = bridge
    if health is not None:
        n.on_vio(String(data=health))
    assert not n.map_valid() and not n.depth_valid()
    n.on_position_command(command())
    n.tick()
    assert n.state == 'CRUISE'
    assert n.px4_adapter.velocity_body.call_args.args[0] == pytest.approx((.3, 0., 0.))
    n.px4_adapter.command.assert_not_called()


def test_command_loss_latches_stop_then_fresh_command_resumes(bridge):
    n = bridge
    n.on_position_command(command())
    n.tick()
    n.now = lambda: 100.3
    n.pose_at = n.status_at = 100.3
    n.position.x = 1.5
    n.tick()
    assert n.state == 'HOLD'
    n.px4_adapter.position.assert_called_with((1.5, 2., -1.5), 0.)
    n.on_ego_odom(n.ego_odom)
    n.ego_odom.header.stamp.nanosec = 300000000
    n.on_position_command(command(100.3))
    n.tick()
    assert n.state == 'CRUISE'


def test_mode_exit_releases_tracking_and_reentry_latches_current_position(bridge):
    n = bridge
    n.on_position_command(command())
    n.tick()
    n.px4_adapter.velocity_body.reset_mock()
    n.on_status(VehicleStatus(arming_state=2, nav_state=2))
    n.tick()
    assert n.state == 'IDLE' and n.cmd is None
    n.px4_adapter.velocity_body.assert_not_called()
    n.px4_adapter.command.assert_not_called()
    n.position.x = 5.
    n.on_status(VehicleStatus(arming_state=2, nav_state=14))
    n.tick()
    n.px4_adapter.position.assert_called_with((5., 2., -1.5), 0.)
    assert n.state == 'HOLD'


def test_standby_stream_does_not_arm_or_change_mode(bridge):
    n = bridge
    n.on_status(VehicleStatus(arming_state=1, nav_state=2))
    n.tick()
    assert n.state == 'IDLE'
    n.px4_adapter.position.assert_called_with((1., 2., -1.5), 0.)
    n.px4_adapter.command.assert_not_called()


def test_rc_can_reenter_offboard_after_a_landing_request(bridge):
    n = bridge
    n.tick()
    assert n.land(Trigger.Request(), Trigger.Response()).success
    n.on_status(VehicleStatus(arming_state=2, nav_state=18))
    n.tick()
    n.on_status(VehicleStatus(arming_state=2, nav_state=14))
    n.tick()
    assert n.state == 'HOLD'
    n.now = lambda: 100.1
    n.on_position_command(command(100.1))
    n.tick()
    assert n.state == 'CRUISE'


@pytest.mark.parametrize('bad', ['command_nan', 'odom_nan', 'odom_old', 'quaternion_zero', 'position_stale'])
def test_unusable_inputs_fall_back_to_hold(bridge, bad):
    n = bridge
    msg = command()
    if bad == 'command_nan':
        msg.velocity.x = float('nan')
    elif bad == 'odom_nan':
        n.ego_odom.pose.pose.position.x = float('nan')
    elif bad == 'odom_old':
        n.ego_odom.header.stamp.sec = 90
    elif bad == 'quaternion_zero':
        n.ego_odom.pose.pose.orientation.w = 0.
    else:
        n.pose_at = 90.
    n.on_position_command(msg)
    n.tick()
    assert n.state == 'HOLD'
    n.px4_adapter.velocity_body.assert_not_called()
    if bad == 'position_stale':
        n.px4_adapter.velocity_ned.assert_called_with((0., 0., 0.))


def test_status_loss_does_not_keep_following_ego(bridge):
    n = bridge
    n.on_position_command(command())
    n.tick()
    n.status_at = 90.
    n.px4_adapter.velocity_body.reset_mock()
    n.tick()
    assert n.state == 'HOLD'
    n.position.x = 1.5
    n.tick()
    n.px4_adapter.position.assert_called_with((1., 2., -1.5), 0.)
    n.px4_adapter.velocity_body.assert_not_called()


def test_reset_discards_old_data_without_requiring_map_ack(bridge):
    n = bridge
    n.on_position_command(command())
    n.tick()
    n.on_vio_reset(Header(stamp=n.get_clock().now().to_msg(), frame_id='reset'))
    assert n.cmd is None and n.ego_odom is None
    # Use the same time base as the deterministic samples.
    n.reset_sensor_stamp = 100.
    n.now = lambda: 100.1
    n.pose_at = n.status_at = 100.1
    n.tick()
    assert n.state == 'HOLD'
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.header.stamp.sec, odom.header.stamp.nanosec = 100, 100000000
    odom.pose.pose.orientation.w = 1.
    n.on_ego_odom(odom)
    n.on_position_command(command(100.1))
    n.tick()
    assert n.state == 'CRUISE'
