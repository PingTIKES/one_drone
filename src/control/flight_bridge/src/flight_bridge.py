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
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, String, Header
from std_srvs.srv import Trigger
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from flight_bridge.velocity_math import (arrival_deadband_active, body_flu_to_ned,
                                         required_forward_clearance, yaw_policy)


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
                        auto_recover_from_hold=True, status_timeout=1.5)
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
        yaw_policy(
            0.0,
            math.radians(float(self.p('yaw_soft_limit_deg'))),
            math.radians(float(self.p('yaw_hard_limit_deg'))),
            math.radians(float(self.p('yaw_reverse_limit_deg'))),
            float(self.p('yaw_min_speed_scale')),
            float(self.p('max_yaw_rate')),
            float(self.p('yaw_rate_gain')))
        arrival_deadband_active(
            False, 0.0, 0.0,
            float(self.p('arrival_position_deadband')),
            float(self.p('arrival_velocity_deadband')),
            float(self.p('arrival_exit_scale')))
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
        self.hold_auto_recover = False
        self.hold_resume_state = None
        px4_ns = str(self.p('px4_ns')).strip('/')
        px4 = '/' + (px4_ns + '/' if px4_ns else '') + 'fmu/'
        qos = qos_profile_sensor_data
        self.mode_pub = self.create_publisher(OffboardControlMode, px4 + 'in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, px4 + 'in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, px4 + 'in/vehicle_command', qos)
        self.state_pub = self.create_publisher(String, 'flight_state', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hold_reason_pub = self.create_publisher(String, 'flight_hold_reason', latched)
        self.safety_pub = self.create_publisher(String, 'flight_safety_status', latched)
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

    def usec(self):
        return int(self.get_clock().now().nanoseconds / 1000)

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
                 f'depth_valid={self.depth_valid()} depth_age={depth_age:.3f}s'))

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
        self.reset_command_cutoff = time.time()
        self.cmd = None
        self.cmd_at = -math.inf
        self.vio_valid_since = None
        if self.state in ('TAKEOFF', 'CRUISE'):
            self.enter_hold('VIO reset: rebuilding map; old goal discarded', auto_recover=True)
        if self.state == 'HOLD' and self.hold_auto_recover:
            self.hold_resume_state = 'CRUISE'

    def on_map_ready(self, msg):
        self.map_epoch = msg.frame_id

    def on_traj_ready(self, msg):
        self.traj_epoch = msg.frame_id

    def reset_map_ready(self):
        return self.reset_epoch is None or (self.map_epoch == self.reset_epoch and
                                           self.traj_epoch == self.reset_epoch)

    def on_position_command(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.reset_map_ready() or stamp <= self.reset_command_cutoff:
            return
        values = (msg.position.x, msg.position.y, msg.position.z,
                  msg.velocity.x, msg.velocity.y, msg.velocity.z)
        if not all(math.isfinite(v) for v in values):
            return
        if (msg.header.frame_id != 'odom' or
                msg.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY):
            return
        self.cmd = msg
        self.cmd_at = self.now()

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
        missing = []
        if self.state != 'IDLE':
            missing.append(f'flight_state={self.state} (need IDLE)')
        if not self.pose_valid():
            missing.append('fresh PX4 local position')
        if not self.vio_stable():
            missing.append('stable VALID OpenVINS')
        if not self.reset_map_ready():
            missing.append('rebuilt map after VIO reset')
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
        response.success = (self.state == 'HOLD' and self.reset_map_ready() and self.depth_valid() and self.pose_valid() and self.vio_stable() and
                            self.status is not None and
                            self.status.nav_state == NAV_OFFBOARD)
        response.message = ('Flight control resumed; submit a new goal after VIO reset' if response.success else
                            'Waiting for stable VIO, PX4 position and Offboard')
        if response.success:
            self.state = 'CRUISE'
            self.cmd_at = -math.inf
            self.hold_auto_recover = False
            self.hold_resume_state = None
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
        msg = VehicleCommand()
        msg.timestamp = self.usec()
        msg.command = command
        msg.param1, msg.param2 = float(p1), float(p2)
        msg.target_system = int(self.p('target_system'))
        msg.target_component = msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def send_mode(self, velocity):
        msg = OffboardControlMode()
        msg.timestamp = self.usec()
        msg.position = not velocity
        msg.velocity = velocity
        msg.acceleration = msg.attitude = msg.body_rate = False
        self.mode_pub.publish(msg)

    def send_position(self, target):
        self.velocity_active = False
        self.send_mode(False)
        msg = TrajectorySetpoint()
        msg.timestamp = self.usec()
        msg.position = [float(v) for v in target]
        msg.velocity = msg.acceleration = [math.nan] * 3
        msg.yaw = self.position.heading if self.pose_valid() else math.nan
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    @staticmethod
    def world_to_body(vector, orientation):
        q = orientation
        norm = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
        if norm < 1e-6:
            return (0., 0., 0.)
        w, x, y, z = q.w/norm, q.x/norm, q.y/norm, q.z/norm
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        c, s = math.cos(yaw), math.sin(yaw)
        # Planning uses the odom horizontal plane. Roll and pitch must not
        # tilt a horizontal world command into vertical velocity.
        return (c*vector[0] + s*vector[1],
                -s*vector[0] + c*vector[1], vector[2])

    def send_ego_command(self, degraded=False):
        if (not self.reset_map_ready() or
                not 0 <= self.now() - self.cmd_at <= float(self.p("command_timeout")) or
                self.cmd is None or self.ego_odom is None or
                not 0 <= self.now() - self.ego_odom_at <= float(self.p('pose_timeout'))):
            return False
        actual = self.ego_odom.pose.pose.position
        gain = float(self.p('position_gain'))
        position_error = math.hypot(self.cmd.position.x - actual.x,
                                    self.cmd.position.y - actual.y)
        planned_speed = math.hypot(self.cmd.velocity.x, self.cmd.velocity.y)
        self.arrival_hold = arrival_deadband_active(
            self.arrival_hold, position_error, planned_speed,
            float(self.p('arrival_position_deadband')),
            float(self.p('arrival_velocity_deadband')),
            float(self.p('arrival_exit_scale')))
        world = [
            self.cmd.velocity.x + gain * (self.cmd.position.x - actual.x),
            self.cmd.velocity.y + gain * (self.cmd.position.y - actual.y),
            self.cmd.velocity.z + gain * (self.cmd.position.z - actual.z)]
        if self.arrival_hold:
            # Near the endpoint, a millimetre-scale correction has an unstable
            # direction. Do not turn that noise into a yaw-rate command.
            world[0] = world[1] = 0.0
        horizontal = math.hypot(world[0], world[1])
        speed_scale = float(self.p('degraded_speed_scale')) if degraded else 1.0
        limit = float(self.p('max_horizontal_speed')) * speed_scale
        if horizontal > limit:
            world[0] *= limit / horizontal
            world[1] *= limit / horizontal
        world[2] = max(-float(self.p('max_vertical_speed')),
                       min(float(self.p('max_vertical_speed')), world[2]))
        bx, by, bz = self.world_to_body(world, self.ego_odom.pose.pose.orientation)
        if self.arrival_hold:
            scale, yaw_flu, self.yaw_mode = 0.0, 0.0, 'ARRIVAL_HOLD'
        else:
            angle = math.atan2(by, bx)
            scale, yaw_flu, self.yaw_mode = yaw_policy(
                angle,
                math.radians(float(self.p('yaw_soft_limit_deg'))),
                math.radians(float(self.p('yaw_hard_limit_deg'))),
                math.radians(float(self.p('yaw_reverse_limit_deg'))),
                float(self.p('yaw_min_speed_scale')),
                (float(self.p('degraded_max_yaw_rate')) if degraded else
                 float(self.p('max_yaw_rate'))),
                float(self.p('yaw_rate_gain')))
        # XYZ remains the planner's independent trajectory. Yaw is held for
        # small direction changes and blended in only as perception requires.
        bx, by = bx * scale, by * scale
        self.translation_scale = scale
        self.velocity_active = True
        self.command_clamped = self.arrival_hold or scale < .999 or horizontal > limit
        self.send_mode(True)
        north, east = body_flu_to_ned(bx, by, self.position.heading)
        msg = TrajectorySetpoint()
        msg.timestamp = self.usec()
        msg.position = msg.acceleration = [math.nan] * 3
        msg.velocity = [north, east, -bz]
        msg.yaw = math.nan
        msg.yawspeed = -yaw_flu
        self.setpoint_pub.publish(msg)
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
        self.hold_reason_pub.publish(String(data=reason))
        self.get_logger().warn(reason)

    def can_auto_recover(self):
        return (self.state == 'HOLD' and self.hold_auto_recover and
                self.reset_map_ready() and self.depth_valid() and
                self.hold_resume_state in ('TAKEOFF', 'CRUISE') and
                self.pose_valid() and self.vio_stable() and
                self.status is not None and
                0 <= self.now() - self.status_at <= float(self.p('status_timeout')) and
                self.status.arming_state == ARMED and
                self.status.nav_state == NAV_OFFBOARD and
                not bool(self.status.failsafe))

    def finish_auto_recovery(self):
        resume_state = self.hold_resume_state
        self.state = resume_state
        self.hold_auto_recover = False
        self.hold_resume_state = None
        self.cmd_at = -math.inf
        self.hold_reason_pub.publish(String(data=''))
        self.get_logger().info(
            f'Estimator stable; automatically resuming {resume_state}')

    def tick(self):
        now = self.now()
        if self.state in ('TAKEOFF', 'CRUISE') and (not self.pose_valid() or
                                                    (not self.vio_valid() and not self.vio_degraded())):
            self.enter_hold('Position or OpenVINS invalid; waiting for stable automatic recovery',
                            auto_recover=True)
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
                self.send_mode(True)
                msg = TrajectorySetpoint()
                msg.timestamp = self.usec()
                msg.position = msg.acceleration = [math.nan] * 3
                msg.velocity = [0., 0., 0.]
                msg.yaw = msg.yawspeed = math.nan
                self.setpoint_pub.publish(msg)
        self.state_pub.publish(String(data=self.state))


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
