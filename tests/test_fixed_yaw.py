"""Yaw mode contracts, using mocked PX4 publishers; never sends flight commands."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State
from std_srvs.srv import Trigger
from flight_bridge.mavros_adapter import LocalPosition as VehicleLocalPosition
from rclpy.parameter import Parameter
from quadrotor_msgs.msg import PositionCommand

from flight_bridge.flight_bridge import FlightBridge
from flight_bridge.yaw_manager import YawDecision
from flight_bridge.mavros_adapter import wrap


@pytest.fixture
def bridge(monkeypatch):
    monkeypatch.setattr('flight_bridge.flight_bridge.MavrosAdapter', Mock())
    rclpy.init()
    n = FlightBridge()
    n.now = lambda: 100.
    n.px4_adapter = Mock()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def pose(n, heading, valid=True):
    m = VehicleLocalPosition(x=0. if valid else math.nan, heading=heading)
    n.on_position(m)


def ready_trajectory(n, arrival=False):
    n.cmd = PositionCommand()
    n.cmd.header.frame_id = 'odom'
    n.cmd.header.stamp.sec = 100
    n.cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
    n.cmd_at = n.ego_odom_at = 100.
    n.ego_odom = Odometry()
    n.ego_odom.header.frame_id = 'odom'
    n.ego_odom.header.stamp.sec = 100
    n.ego_odom.pose.pose.orientation.w = 1.
    n.trajectory_controller.track = Mock(return_value=SimpleNamespace(
        world_velocity=(-.3, .2, .1), acceleration=(0., 0., 0.),
        arrival_hold=arrival, speed_limited=False))
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


@pytest.mark.parametrize('arrival', [False, True])
def test_fixed_heading_bypasses_path_yaw_and_fov_scaling(bridge, arrival):
    n = bridge
    n.set_parameters([Parameter('yaw_control', value=False)])
    pose(n, .6)
    pose(n, -.2)
    ready_trajectory(n, arrival)
    assert n.send_ego_command()
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


def test_fixed_yaw_reaches_px4_velocity_setpoint(bridge):
    from flight_bridge.mavros_adapter import MavrosAdapter
    adapter = MavrosAdapter(bridge, '/yaw_test')
    adapter.setpoint_pub = Mock()
    adapter.velocity_body((1., 0., .2), 0., math.pi/2, yaw_ned=.6)
    msg = adapter.setpoint_pub.publish.call_args.args[0]
    assert msg.yaw == pytest.approx(math.pi/2 - .6)
    assert msg.yaw_rate == 0.
    assert (msg.velocity.x, msg.velocity.y, msg.velocity.z) == pytest.approx([1., 0., .2], abs=1e-6)
    adapter.velocity_body((1., 0., 0.), .2, 0.)
    msg = adapter.setpoint_pub.publish.call_args.args[0]
    assert msg.type_mask & msg.IGNORE_YAW
    assert msg.yaw_rate == pytest.approx(.2)


def attitude(n, yaw_enu, stamp, yaw_rate=0.):
    n.now = lambda: stamp
    msg = Imu()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(stamp * 1e9), 10**9)
    msg.orientation.w, msg.orientation.z = math.cos(yaw_enu/2), math.sin(yaw_enu/2)
    msg.angular_velocity.z = yaw_rate
    n.on_attitude(msg)


def test_takeoff_latches_current_heading_after_ground_initialization(bridge):
    n = bridge
    n.set_parameters([Parameter('yaw_control', value=False)])
    pose(n, 0.)  # First PX4 pose, before visual yaw initialization.
    pose(n, -math.pi)
    n.on_status(State(connected=True, mode='POSCTL'))
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    n.send_position(n.takeoff_target)
    assert n.initial_heading == pytest.approx(-math.pi)
    assert n.px4_adapter.position.call_args.args[1] == pytest.approx(-math.pi)


@pytest.mark.parametrize('state', ['PRESTREAM', 'TAKEOFF', 'HOLD', 'CRUISE'])
@pytest.mark.parametrize('jump', [-math.pi, math.pi/2, -math.pi/2])
def test_frame_reset_preserves_physical_yaw_target_in_every_phase(bridge, state, jump):
    n = bridge
    n.set_parameters([Parameter('yaw_control', value=False)])
    attitude(n, .1, 100.)
    pose(n, math.pi/2-.1)
    n.state = state
    # Preserve an existing physical heading error as well as the reference.
    target_enu = .3
    n.initial_heading = wrap(math.pi/2-target_enu)
    attitude(n, wrap(.1+jump), 100.05)
    pose(n, wrap(math.pi/2-.1-jump))
    n.send_position((1., 2., -1.5))
    expected = wrap(math.pi/2-target_enu-jump)
    assert wrap(n.initial_heading-expected) == pytest.approx(0.)
    assert wrap(n.px4_adapter.position.call_args.args[1]-expected) == pytest.approx(0.)
    ready_trajectory(n)
    n.cmd.header.stamp.nanosec = n.ego_odom.header.stamp.nanosec = 50000000
    n.cmd_at = n.ego_odom_at = n.now()
    assert n.send_ego_command()
    assert wrap(n.px4_adapter.velocity_body.call_args.kwargs['yaw_ned']-expected) == pytest.approx(0.)
    # A second sample in the same frame must not compensate the reset twice.
    attitude(n, wrap(.1+jump), 100.10)
    assert wrap(n.initial_heading-expected) == pytest.approx(0.)


def test_real_rotation_does_not_move_fixed_yaw_reference(bridge):
    n = bridge
    attitude(n, .1, 100.)
    pose(n, math.pi/2-.1)
    reference = n.initial_heading
    # A physical turn larger than the reset threshold is explained by gyro.
    attitude(n, .5, 100.05, yaw_rate=8.)
    assert n.initial_heading == pytest.approx(reference)
    # Crossing +/-pi is also an ordinary physical turn.
    attitude(n, math.pi-.05, 100.60)
    attitude(n, -math.pi+.05, 100.65, yaw_rate=2.)
    assert n.initial_heading == pytest.approx(reference)


def test_pose_before_reset_imu_does_not_double_shift_takeoff_reference(bridge):
    n = bridge
    attitude(n, 0., 100.)
    pose(n, math.pi/2)
    n.now = lambda: 100.05
    pose(n, -math.pi/2)  # Pose receives the reset ahead of the IMU callback.
    n.on_status(State(connected=True, mode='POSCTL'))
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    attitude(n, math.pi, 100.05)
    assert wrap(n.initial_heading+math.pi/2) == pytest.approx(0.)


def test_stale_or_replayed_attitude_cannot_reapply_yaw_reset(bridge):
    n = bridge
    attitude(n, 0., 100.)
    pose(n, math.pi/2)
    attitude(n, math.pi, 100.)
    attitude(n, math.pi, 99.95)
    assert n.attitude_stamp == 100.
    attitude(n, 0., 100.05)
    assert n.initial_heading == pytest.approx(math.pi/2)


def test_manual_offboard_entry_and_reentry_use_current_direction(bridge):
    n = bridge
    n.set_parameters([Parameter('yaw_control', value=False)])
    pose(n, .2)
    pose(n, 1.)
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.tick()
    assert n.initial_heading == pytest.approx(1.)
    n.on_status(State(connected=True, armed=True, mode='POSCTL'))
    n.tick()
    pose(n, 2.)  # Pilot turns in manual mode before returning to OFFBOARD.
    n.standby_setpoint()
    assert n.px4_adapter.position.call_args.args[1] == pytest.approx(2.)
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.tick()
    assert n.initial_heading == pytest.approx(2.)
