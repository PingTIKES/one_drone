"""Takeoff with vertical feedback but no horizontal odometry, no hardware."""
import math
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, EstimatorStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Header
from std_srvs.srv import Trigger
from flight_bridge.flight_bridge import FlightBridge
from flight_bridge.bootstrap_takeoff import BootstrapHeightController


@pytest.fixture
def bridge(monkeypatch):
    monkeypatch.setattr('flight_bridge.flight_bridge.MavrosAdapter', Mock())
    rclpy.init(args=['--ros-args', '-p', 'bootstrap_takeoff:=true', '-p', 'localization_source:=microvio', '-p', 'bootstrap_localization_stable_time:=0.0'])
    n = FlightBridge()
    clock = [100.]
    n.now = lambda: clock[0]
    def feedback(z=0., aided=False):
        stamp = n.get_clock().now().to_msg()
        stamp.sec, stamp.nanosec = int(clock[0]), int((clock[0] % 1)*1e9)
        pose = PoseStamped();pose.header.stamp = stamp
        pose.pose.position.x = 0. if aided else math.nan
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.
        n.on_mavros_pose(pose)
        vel = TwistStamped();vel.header.stamp = stamp
        n.on_velocity(vel)
        imu = Imu();imu.header.stamp = stamp;imu.orientation.w = 1.
        n.on_attitude(imu)
        n.on_estimator(EstimatorStatus(velocity_vert_status_flag=True, pos_vert_abs_status_flag=True,
                                      velocity_horiz_status_flag=aided, pos_horiz_rel_status_flag=aided,
                                      const_pos_mode_status_flag=not aided))
        n.on_status(State(connected=True, armed=False, mode='POSCTL'))
    feedback()
    yield n, clock, feedback
    n.destroy_node();rclpy.shutdown()


def start_airborne(n):
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    for _ in range(25):n.tick()
    n.px4_adapter.set_mode.assert_called_with('OFFBOARD')
    # Mode confirmation precedes the only arm request.
    n.on_status(State(connected=True, mode='OFFBOARD'))
    for _ in range(20):n.tick()
    n.px4_adapter.arm.assert_called()
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.tick()
    assert n.state == 'BOOTSTRAP_TAKEOFF'


def test_no_odom_takeoff_handoff_and_decision_readiness(bridge):
    n, clock, feedback = bridge
    assert not n.pose_valid() and n.ego_odom is None
    start_airborne(n)
    clock[0] += .05
    feedback(.2)
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.tick()
    assert not n.navigation_ready
    assert n.px4_adapter.attitude.call_args.args[1] > 0
    n.px4_adapter.position.assert_not_called()
    # The origin reset that accompanies airborne initialization must not cancel takeoff.
    n.on_vio_reset(Header())
    assert n.takeoff_pending and n.bootstrap is not None
    feedback(.7, aided=True)
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.on_localization_ready(Bool(data=True))
    odom = Odometry();odom.header.frame_id = 'odom';odom.header.stamp.sec = 100
    odom.pose.pose.orientation.w = 1.
    n.on_ego_odom(odom)
    n.tick()
    assert n.state == 'TAKEOFF' and not n.navigation_ready
    # Airborne origin does not add another 2 m above the initialization height.
    n.px4_adapter.position.assert_called_with((0., 0., -2.), pytest.approx(math.pi/2))
    feedback(2., aided=True)
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.tick()
    clock[0] += 1.1
    feedback(2., aided=True)
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.on_localization_ready(Bool(data=True))
    odom.header.stamp.sec = 101;odom.header.stamp.nanosec = 150000000
    n.on_ego_odom(odom)
    n.tick()
    assert n.state == 'HOLD' and n.navigation_ready and not n.takeoff_pending


def test_no_vertical_feedback_is_not_blind_thrust(bridge):
    n, _, _ = bridge
    n.velocity_at = -math.inf
    assert not n.takeoff(Trigger.Request(), Trigger.Response()).success
    n.px4_adapter.attitude.assert_not_called()
    n.px4_adapter.arm.assert_not_called()
    n.px4_adapter.set_mode.assert_not_called()


def test_prearm_feedback_gap_waits_at_zero_thrust_then_restarts_prestream(bridge):
    n, clock, feedback = bridge
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    clock[0] += .6
    n.tick()
    assert n.state == 'BOOTSTRAP_PRESTREAM' and n.takeoff_pending
    n.px4_adapter.attitude.assert_called_with(n.bootstrap_yaw, 0.)
    n.px4_adapter.arm.assert_not_called()
    n.px4_adapter.set_mode.assert_not_called()
    feedback()
    n.on_status(State(connected=True, armed=False, mode='OFFBOARD'))
    for _ in range(24):n.tick()
    n.px4_adapter.arm.assert_not_called()
    n.tick()
    n.px4_adapter.arm.assert_called_once()


def test_prearm_feedback_loss_still_times_out_without_arming(bridge):
    n, clock, _ = bridge
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    clock[0] += 16.
    n.on_status(State(connected=True, armed=False, mode='OFFBOARD'))
    n.tick()
    assert n.state == 'IDLE' and not n.takeoff_pending and n.bootstrap is None
    n.px4_adapter.arm.assert_not_called()
    n.px4_adapter.set_mode.assert_not_called()
    # A delayed mode confirmation must not turn an aborted takeoff into HOLD.
    n.tick()
    assert n.state == 'IDLE' and not n.navigation_ready
    n.px4_adapter.arm.assert_not_called()


def test_explicit_retry_in_disarmed_offboard_is_allowed(bridge):
    n, _, _ = bridge
    n.on_status(State(connected=True, armed=False, mode='OFFBOARD'))
    n.tick()
    assert n.state == 'IDLE'
    n.px4_adapter.arm.assert_not_called()
    assert n.takeoff(Trigger.Request(), Trigger.Response()).success
    assert n.state == 'BOOTSTRAP_PRESTREAM'
    n.px4_adapter.arm.assert_not_called()


def test_stale_disarmed_offboard_status_does_not_report_airborne_hold(bridge):
    n, clock, _ = bridge
    n.on_status(State(connected=True, armed=False, mode='OFFBOARD'))
    n.tick()
    clock[0] += 2.
    n.tick()
    assert n.state == 'IDLE' and not n.navigation_ready
    n.px4_adapter.arm.assert_not_called()
    n.px4_adapter.set_mode.assert_not_called()


@pytest.mark.parametrize('fault', ['timeout', 'feedback', 'height'])
def test_bootstrap_fault_requests_land_without_navigation(bridge, fault):
    n, clock, feedback = bridge
    start_airborne(n)
    if fault == 'timeout':clock[0] += 21
    feedback(2. if fault == 'height' else .4)
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    if fault == 'feedback':n.velocity_at = -math.inf
    n.tick()
    assert n.state == 'LANDING' and not n.navigation_ready
    n.px4_adapter.set_mode.assert_called_with('AUTO.LAND')
    n.px4_adapter.position.assert_not_called()


def test_mode_exit_during_bootstrap_never_reenters(bridge):
    n, _, _ = bridge
    start_airborne(n)
    n.px4_adapter.arm.reset_mock()
    n.px4_adapter.set_mode.reset_mock()
    n.on_status(State(connected=True, armed=True, mode='ALTCTL'))
    n.tick()
    assert n.state == 'IDLE' and n.bootstrap is None
    for _ in range(30):n.tick()
    n.px4_adapter.arm.assert_not_called()
    n.px4_adapter.set_mode.assert_not_called()


def test_height_controller_converges_with_limited_thrust():
    c = BootstrapHeightController(.5, .1, .7, 1., .15, .08, .3, .3)
    h = v = 0.
    for _ in range(800):
        thrust = c.update(.7-h, v, .05)
        assert .1 <= thrust <= .7
        v += (19.62*(thrust-.5)-.5*v)*.05
        h = max(0., h+v*.05)
        if h == 0:v = max(0., v)
    assert abs(h-.7) < .05 and abs(v) < .05


def test_px4_yaw_frame_reset_does_not_command_a_physical_turn(bridge):
    n, clock, _ = bridge
    start_airborne(n)
    # FC yaw jumps by 90 degrees while the measured physical yaw rate is zero.
    clock[0] += .05
    imu = Imu();imu.header.stamp.sec = 100;imu.header.stamp.nanosec = 50000000
    imu.orientation.w, imu.orientation.z = math.sqrt(.5), math.sqrt(.5)
    n.on_attitude(imu)
    assert n.bootstrap_yaw == pytest.approx(math.pi/2)
    assert n.initial_heading == pytest.approx(0.)


def test_tof_takeoff_height_survives_px4_vertical_origin_change(bridge):
    from sensor_msgs.msg import Range
    n, clock, feedback = bridge
    n.set_parameters([Parameter('bootstrap_height_source', value='range')])
    def distance(value):
        msg = Range();msg.header.stamp.sec = 100
        msg.min_range, msg.max_range, msg.range = .02, 12., value
        n.on_range(msg)
    distance(.07)
    start_airborne(n)
    # At 0.7 m actual height, PX4 renames its Z origin to zero.
    feedback(0., aided=True)
    distance(.77)
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.on_localization_ready(Bool(data=True))
    odom = Odometry();odom.header.frame_id='odom';odom.header.stamp.sec=100
    odom.pose.pose.orientation.w=1.
    n.on_ego_odom(odom)
    n.tick()
    assert n.state == 'TAKEOFF'
    n.px4_adapter.position.assert_called_with((0., 0., pytest.approx(-1.3)), pytest.approx(math.pi/2))


def test_transient_aiding_does_not_trigger_position_takeover(bridge):
    n, clock, feedback = bridge
    n.set_parameters([Parameter('bootstrap_localization_stable_time', value=.5)])
    start_airborne(n)
    feedback(.7, aided=True)
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.on_localization_ready(Bool(data=True))
    odom = Odometry();odom.header.frame_id='odom';odom.header.stamp.sec=100
    odom.pose.pose.orientation.w=1.
    n.on_ego_odom(odom)
    n.tick()
    assert n.state == 'BOOTSTRAP_TAKEOFF'
    n.on_localization_ready(Bool(data=False))
    n.tick()
    assert n.bootstrap_ready_since is None
    n.px4_adapter.position.assert_not_called()


def test_reset_after_position_takeover_discards_old_takeoff_target(bridge):
    n, _, _ = bridge
    n.bootstrap = None
    n.takeoff_pending = True
    n.state = 'TAKEOFF'
    n.takeoff_target = (1., 2., -2.)
    n.on_status(State(connected=True, armed=True, mode='OFFBOARD'))
    n.on_vio_reset(Header())
    assert n.takeoff_target is None and not n.takeoff_pending
    assert n.state == 'LANDING'
    n.px4_adapter.set_mode.assert_called_with('AUTO.LAND')
