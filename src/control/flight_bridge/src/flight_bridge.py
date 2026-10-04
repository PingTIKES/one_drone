"""Explicit PX4 takeoff/hold/land service and EGO trajectory adapter.

No autonomous land command is sent on VIO or depth loss. A recoverable
estimator interruption holds PX4's current local position, then resumes after
VIO and PX4 are stable. PX4's own failsafe remains authoritative.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from flight_interfaces.msg import SystemStatus
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, String, Header
from std_srvs.srv import Trigger
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

from flight_bridge.px4_adapter import Px4Adapter
from flight_bridge.flight_supervisor import FlightSupervisor
from flight_bridge.trajectory_controller import TrajectoryController
from flight_bridge.trajectory_validator import TrajectoryValidator
from flight_bridge.velocity_math import required_forward_clearance
from flight_bridge.yaw_manager import YawManager, world_to_body


NAV_OFFBOARD = 14
ARMED = 2


class FlightBridge(Node):
    def __init__(self):
        super().__init__('flight_bridge')
        defaults = dict(px4_ns='px4_1', target_system=2, takeoff_altitude=2.,
                        max_horizontal_speed=.5, max_vertical_speed=.3,
                        max_yaw_rate=.35, position_gain=.8,
                        yaw_soft_limit_deg=30., yaw_hard_limit_deg=100.,
                        yaw_reverse_limit_deg=150., yaw_min_speed_scale=.25,
                        yaw_rate_gain=.8,
                        arrival_position_deadband=.15,
                        arrival_velocity_deadband=.10,
                        arrival_exit_scale=1.5,
                        degraded_speed_scale=.4,
                        degraded_max_yaw_rate=.10,
                        reaction_time=.5, assumed_braking_deceleration=.6,
                        safety_margin=.5, verified_forward_range=5.,
                        command_timeout=.2, pose_timeout=.5, vio_timeout=.5,
                        vio_stable_time=1., depth_heartbeat_timeout=.5, depth_grace=.8,
                        takeoff_tolerance=.2, takeoff_stable_time=1.,
                        auto_recover_from_hold=True, status_timeout=1.5,
                        planner_heartbeat_timeout=.75, map_timeout=.75,
                        camera_hfov_deg=87., yaw_lookahead_time=.3,
                        yaw_outside_fov_speed_scale=.25,
                        trajectory_max_sample_age=.5,
                        trajectory_max_future_stamp=.1,
                        trajectory_max_planned_speed=8.,
                        trajectory_max_planned_acceleration=12.,
                        trajectory_max_tracking_error=5.)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value
        required_clearance = required_forward_clearance(
            float(self.p('max_horizontal_speed')), float(self.p('reaction_time')),
            float(self.p('assumed_braking_deceleration')), float(self.p('safety_margin')))
        if required_clearance > float(self.p('verified_forward_range')):
            raise ValueError(
                f'configured cruise needs {required_clearance:.2f} m forward clearance, '
                f'but verified_forward_range is {float(self.p("verified_forward_range")):.2f} m')
        self.yaw_manager = YawManager(
            float(self.p('yaw_soft_limit_deg')),
            float(self.p('yaw_hard_limit_deg')),
            float(self.p('yaw_reverse_limit_deg')),
            float(self.p('yaw_min_speed_scale')),
            float(self.p('max_yaw_rate')),
            float(self.p('degraded_max_yaw_rate')),
            float(self.p('yaw_rate_gain')),
            float(self.p('camera_hfov_deg')),
            float(self.p('yaw_lookahead_time')),
            float(self.p('yaw_outside_fov_speed_scale')))
        self.trajectory_controller = TrajectoryController(
            self.p('position_gain'), self.p('max_horizontal_speed'),
            self.p('max_vertical_speed'), self.p('arrival_position_deadband'),
            self.p('arrival_velocity_deadband'), self.p('arrival_exit_scale'))
        self.trajectory_validator = TrajectoryValidator(
            self.p('trajectory_max_sample_age'),
            self.p('trajectory_max_future_stamp'),
            self.p('trajectory_max_planned_speed'),
            self.p('trajectory_max_planned_acceleration'),
            self.p('trajectory_max_tracking_error'))
        self.supervisor = FlightSupervisor()
        if not (0 < float(self.p('degraded_speed_scale')) <= 1 and
                0 < float(self.p('degraded_max_yaw_rate')) <= float(self.p('max_yaw_rate'))):
            raise ValueError('degraded speed/yaw limits must be positive and no greater than normal limits')
        self.required_clearance = required_clearance
        self.state = 'IDLE'
        self.position = None
        self.status = None
        self.pose_at = self.status_at = self.vio_at = self.cmd_at = self.obstacle_at = -math.inf
        self.last_obstacle_good = -math.inf
        self.vio_ok = self.obstacle_ok = False
        self.vio_state = 'INVALID'
        self.vio_valid_since = None
        self.cmd = None
        self.ego_odom = None
        self.ego_odom_at = -math.inf
        self.takeoff_target = self.hold_target = None
        self.takeoff_reached_since = None
        self.prestream_count = 0
        self.last_reset = None
        self.planner_instance = None
        self.map_stamp = -math.inf
        self.planner_at = -math.inf
        self.create_subscription(Header, '/ego/map_heartbeat', self.on_map_heartbeat, 1)
        self.reset_epoch = None
        self.map_epoch = None
        self.traj_epoch = None
        self.reset_command_cutoff = -math.inf
        self.reset_sensor_stamp = -math.inf
        reset_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Header, '/vio_reset_event', self.on_vio_reset, reset_qos)
        self.create_subscription(Header, '/ego/map_reset_ready', self.on_map_ready, reset_qos)
        self.create_subscription(Header, '/ego/trajectory_reset_ready', self.on_traj_ready, reset_qos)
        self.velocity_active = False
        self.command_clamped = False
        self.yaw_mode = 'YAW_HOLD'
        self.arrival_hold = False
        self.translation_scale = 1.0
        self.trajectory_status = 'WAITING_FOR_TRAJECTORY'
        self.hold_reason = ''
        self.hold_auto_recover = False
        self.hold_resume_state = None
        px4_ns = str(self.p('px4_ns')).strip('/')
        px4 = '/' + (px4_ns + '/' if px4_ns else '') + 'fmu/'
        qos = qos_profile_sensor_data
        self.px4_adapter = Px4Adapter(self, px4_ns, self.p('target_system'))
        self.state_pub = self.create_publisher(String, 'flight_state', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hold_reason_pub = self.create_publisher(String, 'flight_hold_reason', latched)
        self.safety_pub = self.create_publisher(String, 'flight_safety_status', latched)
        self.system_pub = self.create_publisher(SystemStatus, 'system_status', latched)
        self.hold_reason_pub.publish(String(data=''))
        self.safety_pub.publish(String(
            data=f'EGO_YAW_MANAGER max_speed={float(self.p("max_horizontal_speed")):.2f}m/s '
                 f'required_clearance={self.required_clearance:.2f}m'))
        self.create_subscription(VehicleLocalPosition, px4 + 'out/vehicle_local_position', self.on_position, qos)
        self.create_subscription(VehicleStatus, px4 + 'out/vehicle_status', self.on_status, qos)
        self.create_subscription(PositionCommand, '/ego/position_cmd', self.on_position_command, 20)
        self.create_subscription(Odometry, '/ego/odom', self.on_ego_odom, 20)
        self.create_subscription(String, 'vio_health', self.on_vio, 10)
        self.create_subscription(Bool, '/ego/depth_fresh', self.on_obstacle, 10)
        self.create_service(Trigger, 'takeoff', self.takeoff)
        self.create_service(Trigger, 'resume_navigation', self.resume)
        self.create_service(Trigger, 'land', self.land)
        self.create_timer(.05, self.tick)
        self.create_timer(1., self.publish_safety_status)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_valid(self):
        p = self.position
        return (p is not None and p.xy_valid and p.z_valid and
                all(math.isfinite(v) for v in (p.x, p.y, p.z, p.heading)) and
                0 <= self.now() - self.pose_at <= float(self.p('pose_timeout')))

    def vio_valid(self):
        return self.vio_ok and 0 <= self.now() - self.vio_at <= float(self.p('vio_timeout'))

    def vio_degraded(self):
        return (self.vio_state == 'DEGRADED' and
                0 <= self.now() - self.vio_at <= float(self.p('vio_timeout')))

    def vio_stable(self):
        return (self.vio_valid() and self.vio_valid_since is not None and
                self.now() - self.vio_valid_since >= float(self.p('vio_stable_time')))

    def depth_valid(self):
        # Software stereo can occasionally skip a frame. Require a recent
        # positive depth heartbeat, with a bounded grace period for one skip.
        return (0 <= self.now() - self.obstacle_at <= float(self.p('depth_heartbeat_timeout')) and
                0 <= self.now() - self.last_obstacle_good <= float(self.p('depth_grace')))

    def publish_safety_status(self):
        now = self.now()
        vio_age = now - self.vio_at if math.isfinite(self.vio_at) else math.inf
        depth_age = now - self.last_obstacle_good if math.isfinite(self.last_obstacle_good) else math.inf
        self.safety_pub.publish(String(
            data=f'ego_tracking=true '
                 f'max_speed={float(self.p("max_horizontal_speed")):.2f}m/s '
                 f'required_clearance={self.required_clearance:.2f}m '
                 f'command_clamped={self.command_clamped} '
                 f'yaw_mode={self.yaw_mode} translation_scale={self.translation_scale:.2f} '
                 f'vio_state={self.vio_state} '
                 f'hold_auto_recover={self.hold_auto_recover} '
                 f'vio_stable={self.vio_stable()} vio_age={vio_age:.3f}s '
                 f'depth_valid={self.depth_valid()} depth_age={depth_age:.3f}s '
                 f'trajectory_status={self.trajectory_status}'))

    def publish_system_status(self):
        msg = SystemStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.flight_state = self.state
        msg.vio_state = self.vio_state
        msg.hold_reason = self.hold_reason
        msg.px4_position_valid = self.pose_valid()
        msg.px4_offboard = (self.status is not None and
                           self.status.nav_state == NAV_OFFBOARD and
                           self.status.arming_state == ARMED and
                           0 <= self.now() - self.status_at <= float(self.p('status_timeout')))
        msg.depth_valid = self.depth_valid()
        msg.map_valid = self.map_valid()
        msg.trajectory_valid = (self.cmd is not None and
                                0 <= self.now() - self.cmd_at <= float(self.p('command_timeout')) and
                                self.trajectory_status == 'OK')
        self.system_pub.publish(msg)

    def on_position(self, msg):
        resets = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        reset_changed = (self.last_reset is not None and resets != self.last_reset and
                         self.state in ('TAKEOFF', 'CRUISE', 'HOLD'))
        self.last_reset = resets
        self.position = msg
        # PX4 publishes its boot-relative uORB timestamp. Freshness must use
        # ROS callback arrival time in both SITL and hardware deployments.
        self.pose_at = self.now()
        if reset_changed:
            self.hold_target = (msg.x, msg.y, msg.z)
            if self.state == 'HOLD':
                self.hold_resume_state = 'CRUISE'
            self.enter_hold('PX4 local estimate reset; waiting for stable automatic recovery',
                            auto_recover=True)

    def on_status(self, msg):
        self.status = msg
        self.status_at = self.now()

    def on_vio_reset(self, msg):
        self.reset_epoch = msg.frame_id
        self.reset_sensor_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.ego_odom = None
        self.ego_odom_at = -math.inf
        self.vio_ok = False
        self.vio_state = "INVALID"
        self.vio_at = -math.inf
        self.reset_command_cutoff = self.now()
        self.cmd = None
        self.cmd_at = -math.inf
        self.trajectory_controller.reset()
        self.trajectory_status = 'VIO_RESET'
        self.vio_valid_since = None
        if self.state in ('TAKEOFF', 'CRUISE'):
            self.enter_hold('VIO reset: rebuilding map; old goal discarded', auto_recover=True)
        if self.state == 'HOLD' and self.hold_auto_recover:
            self.hold_resume_state = 'CRUISE'

    def map_valid(self):
        return (self.map_stamp > 0 and -0.02 <= self.now() - self.map_stamp <= float(self.p('map_timeout')) and
                time.monotonic() - self.planner_at <= float(self.p('planner_heartbeat_timeout')))

    def on_map_heartbeat(self, msg):
        changed = self.planner_instance is not None and self.planner_instance != msg.frame_id
        self.planner_instance = msg.frame_id
        self.map_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.planner_at = time.monotonic()
        if changed:
            self.cmd = None
            self.cmd_at = -math.inf
            self.trajectory_controller.reset()
            self.trajectory_status = 'PLANNER_RESTARTED'
            self.reset_command_cutoff = self.now()
            if self.state == 'CRUISE':
                self.enter_hold('Planner restarted; waiting for rebuilt map', auto_recover=True)

    def on_map_ready(self, msg):
        self.map_epoch = msg.frame_id

    def on_traj_ready(self, msg):
        self.traj_epoch = msg.frame_id

    def reset_map_ready(self):
        return self.reset_epoch is None or (self.map_epoch == self.reset_epoch and
                                           self.traj_epoch == self.reset_epoch)

    def on_position_command(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.reset_map_ready() or not self.map_valid() or stamp <= self.reset_command_cutoff:
            return
        actual = self.ego_odom.pose.pose.position if self.ego_odom is not None else None
        valid, reason = self.trajectory_validator.validate(msg, self.now(), actual)
        if not valid:
            self.trajectory_status = reason
            return
        self.cmd = msg
        self.cmd_at = self.now()
        self.trajectory_status = 'OK'

    def on_ego_odom(self, msg):
        if msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 <= self.reset_sensor_stamp:
            return
        self.ego_odom = msg
        self.ego_odom_at = self.now()

    def on_vio(self, msg):
        valid = msg.data == 'VALID'
        if valid and not self.vio_ok:
            self.vio_valid_since = self.now()
        elif msg.data == 'INVALID':
            self.vio_valid_since = None
        self.vio_state = msg.data
        self.vio_ok = valid
        self.vio_at = self.now()

    def on_obstacle(self, msg):
        self.obstacle_ok = bool(msg.data)
        self.obstacle_at = self.now()
        if self.obstacle_ok:
            self.last_obstacle_good = self.obstacle_at

    def takeoff(self, _request, response):
        missing = self.supervisor.takeoff_blockers(
            self.state, self.pose_valid(), self.vio_stable(),
            self.reset_map_ready() and self.map_valid())
        response.success = not missing
        response.message = ('Starting PX4 Offboard takeoff' if response.success else
                            'Takeoff blocked: ' + ', '.join(missing))
        if response.success:
            p = self.position
            self.takeoff_target = (p.x, p.y, p.z - float(self.p('takeoff_altitude')))
            self.hold_target = (p.x, p.y, p.z)
            self.prestream_count = 0
            self.state = 'PRESTREAM'
        return response

    def resume(self, _request, response):
        response.success = (self.state == 'HOLD' and self.reset_map_ready() and self.map_valid() and self.depth_valid() and self.pose_valid() and self.vio_stable() and
                            self.status is not None and
                            self.status.nav_state == NAV_OFFBOARD)
        response.message = ('Flight control resumed; submit a new goal after VIO reset' if response.success else
                            'Waiting for stable VIO, PX4 position and Offboard')
        if response.success:
            self.state = 'CRUISE'
            self.cmd_at = -math.inf
            self.hold_auto_recover = False
            self.hold_resume_state = None
            self.hold_reason = ''
            self.hold_reason_pub.publish(String(data=''))
        return response

    def land(self, _request, response):
        response.success = self.state in ('TAKEOFF', 'CRUISE', 'HOLD')
        response.message = 'Land commanded' if response.success else 'Not airborne in this controller'
        if response.success:
            self.vehicle_command(21)
            self.state = 'LANDING'
        return response

    def vehicle_command(self, command, p1=0., p2=0.):
        self.px4_adapter.command(command, p1, p2)

    def send_position(self, target):
        self.velocity_active = False
        self.px4_adapter.position(target,
                                  self.position.heading if self.pose_valid() else math.nan)

    def send_ego_command(self, degraded=False):
        if (not self.reset_map_ready() or not self.map_valid() or
                not 0 <= self.now() - self.cmd_at <= float(self.p("command_timeout")) or
                self.cmd is None or self.ego_odom is None or
                not 0 <= self.now() - self.ego_odom_at <= float(self.p('pose_timeout'))):
            return False
        speed_scale = float(self.p('degraded_speed_scale')) if degraded else 1.0
        tracking = self.trajectory_controller.track(self.cmd, self.ego_odom, speed_scale)
        self.arrival_hold = tracking.arrival_hold
        decision = self.yaw_manager.decide(
            tracking.world_velocity, tracking.acceleration,
            self.ego_odom.pose.pose.orientation, degraded,
            tracking.arrival_hold)
        bx, by, bz = world_to_body(tracking.world_velocity,
                                   self.ego_odom.pose.pose.orientation)
        bx *= decision.translation_scale
        by *= decision.translation_scale
        self.yaw_mode = decision.mode
        self.translation_scale = decision.translation_scale
        self.velocity_active = True
        self.command_clamped = (tracking.arrival_hold or
                                decision.translation_scale < .999 or
                                tracking.speed_limited)
        self.px4_adapter.velocity_body(
            (bx, by, bz), decision.yaw_rate_flu,
            self.position.heading, tracking.trajectory_id,
            .4 if degraded else 1.)
        return True

    def enter_hold(self, reason, auto_recover=False):
        if self.state == 'HOLD':
            return
        previous_state = self.state
        if self.pose_valid():
            p = self.position
            self.hold_target = (p.x, p.y, p.z)
        self.state = 'HOLD'
        self.hold_auto_recover = bool(auto_recover and
                                      self.p('auto_recover_from_hold'))
        self.hold_resume_state = previous_state if self.hold_auto_recover else None
        self.velocity_active = False
        self.hold_reason = reason
        self.hold_reason_pub.publish(String(data=reason))
        self.get_logger().warn(reason)

    def can_auto_recover(self):
        return self.supervisor.may_auto_recover(
            self.state, self.hold_auto_recover,
            self.reset_map_ready() and self.map_valid(), self.depth_valid(),
            self.hold_resume_state, self.pose_valid(), self.vio_stable(),
            self.status is not None and
            0 <= self.now() - self.status_at <= float(self.p('status_timeout')),
            self.status is not None and self.status.arming_state == ARMED,
            self.status is not None and self.status.nav_state == NAV_OFFBOARD,
            self.status is not None and bool(self.status.failsafe))

    def finish_auto_recovery(self):
        resume_state = self.hold_resume_state
        self.state = resume_state
        self.hold_auto_recover = False
        self.hold_resume_state = None
        self.cmd_at = -math.inf
        self.trajectory_controller.reset()
        self.hold_reason = ''
        self.hold_reason_pub.publish(String(data=''))
        self.get_logger().info(
            f'Estimator stable; automatically resuming {resume_state}')

    def tick(self):
        now = self.now()
        hold_reason = self.supervisor.safety_hold(
            self.state, self.pose_valid(), self.vio_valid() or self.vio_degraded(),
            self.map_valid())
        if hold_reason is not None:
            self.cmd = None
            self.cmd_at = -math.inf
            self.enter_hold(hold_reason, auto_recover=True)
        if self.can_auto_recover():
            self.finish_auto_recovery()
        if self.state == 'PRESTREAM':
            if not self.pose_valid() or not self.vio_valid():
                self.state = 'IDLE'
            else:
                self.send_position(self.hold_target)
                self.prestream_count += 1
                if self.prestream_count >= 20:
                    self.vehicle_command(176, 1., 6.)
                    self.vehicle_command(400, 1.)
                    self.state = 'ARMING'
        elif self.state == 'ARMING':
            if not self.pose_valid() or not self.vio_valid():
                self.enter_hold('Estimator lost while arming')
            else:
                self.send_position(self.hold_target)
                if self.status is not None and self.status.arming_state == ARMED and self.status.nav_state == NAV_OFFBOARD:
                    self.state = 'TAKEOFF'
                elif self.prestream_count % 20 == 0:
                    self.vehicle_command(176, 1., 6.)
                    self.vehicle_command(400, 1.)
                self.prestream_count += 1
        elif self.state == 'TAKEOFF':
            self.send_position(self.takeoff_target)
            p = self.position
            if self.pose_valid() and math.dist((p.x, p.y, p.z), self.takeoff_target) < float(self.p('takeoff_tolerance')):
                if self.takeoff_reached_since is None:
                    self.takeoff_reached_since = now
                elif now - self.takeoff_reached_since >= float(self.p('takeoff_stable_time')):
                    self.hold_target = self.takeoff_target
                    self.state = 'CRUISE'
            else:
                self.takeoff_reached_since = None
        elif self.state == 'CRUISE':
            if self.vio_degraded() and self.pose_valid():
                # With fresh but low-confidence VIO, keep the trajectory at a
                # reduced translation and yaw rate. If odometry itself is
                # stale, fall back to a PX4 local-position hold.
                if (not self.depth_valid() or
                        not self.send_ego_command(degraded=True)):
                    if self.yaw_mode != 'VIO_DEGRADED_HOLD' or self.hold_target is None:
                        p = self.position
                        self.hold_target = (p.x, p.y, p.z)
                    self.yaw_mode = 'VIO_DEGRADED_HOLD'
                    self.translation_scale = 0.0
                    self.send_position(self.hold_target)
            elif not self.depth_valid() and self.pose_valid():
                # Fresh depth is required to follow a path, but not to hold altitude.
                if self.velocity_active:
                    p = self.position
                    self.hold_target = (p.x, p.y, p.z)
                    self.cmd_at = -math.inf
                self.send_position(self.hold_target)
            elif 0 <= now - self.cmd_at <= float(self.p('command_timeout')):
                if not self.send_ego_command() and self.pose_valid():
                    self.send_position(self.hold_target)
            elif self.pose_valid():
                if self.velocity_active or self.hold_target is None:
                    p = self.position
                    self.hold_target = (p.x, p.y, p.z)
                self.send_position(self.hold_target)
        elif self.state == 'HOLD':
            if self.pose_valid():
                if self.hold_target is None:
                    p = self.position
                    self.hold_target = (p.x, p.y, p.z)
                self.send_position(self.hold_target)
            else:
                # Without a valid position, request zero velocity rather than
                # inventing a position hold; PX4's own failsafe may take over.
                self.px4_adapter.velocity_ned((0., 0., 0.))
        self.state_pub.publish(String(data=self.state))
        self.publish_system_status()


def main(args=None):
    rclpy.init(args=args)
    node = FlightBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
