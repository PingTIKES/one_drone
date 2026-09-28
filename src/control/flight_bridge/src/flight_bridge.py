"""Explicit PX4 takeoff/hold/land service and Nav2 body-velocity adapter.

No autonomous land command is sent on VIO or depth loss. On loss, this node
holds PX4's current local position and requires explicit resume after recovery.
PX4's own configured failsafe remains authoritative if its estimator is lost.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from flight_bridge.velocity_math import body_flu_to_ned, limit_horizontal


NAV_OFFBOARD = 14
ARMED = 2


class FlightBridge(Node):
    def __init__(self):
        super().__init__('flight_bridge')
        defaults = dict(px4_ns='px4_1', target_system=2, takeoff_altitude=2.,
                        max_horizontal_speed=.5, max_yaw_rate=.6,
                        command_timeout=.3, pose_timeout=.5, vio_timeout=.5,
                        depth_heartbeat_timeout=.3, depth_grace=.8,
                        require_map_alignment=False,
                        takeoff_tolerance=.2, takeoff_stable_time=1.)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value
        self.state = 'IDLE'
        self.position = None
        self.status = None
        self.pose_at = self.vio_at = self.cmd_at = self.obstacle_at = -math.inf
        self.last_obstacle_good = -math.inf
        self.vio_ok = self.obstacle_ok = self.localized = False
        self.cmd = Twist()
        self.takeoff_target = self.hold_target = None
        self.takeoff_reached_since = None
        self.prestream_count = 0
        self.last_reset = None
        self.velocity_active = False
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
        self.hold_reason_pub.publish(String(data=''))
        self.create_subscription(VehicleLocalPosition, px4 + 'out/vehicle_local_position', self.on_position, qos)
        self.create_subscription(VehicleStatus, px4 + 'out/vehicle_status', self.on_status, qos)
        self.create_subscription(Twist, 'cmd_vel_smoothed', self.on_velocity, 10)
        self.create_subscription(String, 'vio_health', self.on_vio, 10)
        self.create_subscription(Bool, 'obstacle_fresh', self.on_obstacle, 10)
        # Optional legacy readiness input. The default launch disables this gate.
        self.create_subscription(Bool, 'localization_ready', self.on_localized, latched)
        self.create_service(Trigger, 'takeoff', self.takeoff)
        self.create_service(Trigger, 'resume_navigation', self.resume)
        self.create_service(Trigger, 'land', self.land)
        self.create_timer(.05, self.tick)

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

    def depth_valid(self):
        # Software stereo can occasionally skip a frame. Require a recent
        # positive depth heartbeat, with a bounded grace period for one skip.
        return (0 <= self.now() - self.obstacle_at <= float(self.p('depth_heartbeat_timeout')) and
                0 <= self.now() - self.last_obstacle_good <= float(self.p('depth_grace')))

    def on_position(self, msg):
        resets = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        reset_changed = (self.last_reset is not None and resets != self.last_reset and
                         self.state in ('TAKEOFF', 'CRUISE'))
        self.last_reset = resets
        self.position = msg
        self.pose_at = msg.timestamp * 1e-6
        if reset_changed:
            self.enter_hold('PX4 local estimate reset; manual relocalization and resume required')

    def on_status(self, msg):
        self.status = msg

    def on_velocity(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            return
        self.cmd = msg
        self.cmd_at = self.now()

    def on_vio(self, msg):
        self.vio_ok = msg.data == 'VALID'
        self.vio_at = self.now()

    def on_obstacle(self, msg):
        self.obstacle_ok = bool(msg.data)
        self.obstacle_at = self.now()
        if self.obstacle_ok:
            self.last_obstacle_good = self.obstacle_at

    def on_localized(self, msg):
        if not self.p('require_map_alignment'):
            return
        was_localized = self.localized
        self.localized = bool(msg.data)
        if was_localized and not self.localized and self.state == 'CRUISE':
            # A map-frame goal is no longer safe, but PX4 can still hold its
            # current local position using VIO. GoalManager cancels the goal.
            self.cmd_at = -math.inf
            if self.pose_valid():
                p = self.position
                self.hold_target = (p.x, p.y, p.z)

    def takeoff(self, _request, response):
        missing = []
        if self.state != 'IDLE':
            missing.append(f'flight_state={self.state} (need IDLE)')
        if not self.pose_valid():
            missing.append('fresh PX4 local position')
        if not self.vio_valid():
            missing.append('VALID OpenVINS')
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
        response.success = (self.state == 'HOLD' and self.pose_valid() and self.vio_valid() and
                            self.status is not None and
                            self.status.nav_state == NAV_OFFBOARD)
        response.message = ('Flight control resumed; confirm map pose and depth before a new 2D goal' if response.success else
                            'Waiting for stable VIO, PX4 position and Offboard')
        if response.success:
            self.state = 'CRUISE'
            self.cmd_at = -math.inf
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

    def send_velocity(self, body):
        self.velocity_active = True
        self.send_mode(True)
        vx, vy = limit_horizontal(body.linear.x, body.linear.y,
                                  float(self.p('max_horizontal_speed')))
        north, east = body_flu_to_ned(vx, vy, self.position.heading)
        yaw_rate = max(-float(self.p('max_yaw_rate')),
                       min(float(self.p('max_yaw_rate')), -body.angular.z))
        msg = TrajectorySetpoint()
        msg.timestamp = self.usec()
        msg.position = msg.acceleration = [math.nan] * 3
        # PX4 receives velocity-only Offboard commands. A bounded vertical
        # feedback term keeps the aircraft near the selected cruise height.
        vertical = max(-.3, min(.3, self.takeoff_target[2] - self.position.z))
        msg.velocity = [north, east, vertical]
        msg.yaw = math.nan
        msg.yawspeed = yaw_rate
        self.setpoint_pub.publish(msg)

    def enter_hold(self, reason):
        if self.state == 'HOLD':
            return
        if self.pose_valid():
            p = self.position
            self.hold_target = (p.x, p.y, p.z)
        self.state = 'HOLD'
        self.velocity_active = False
        self.hold_reason_pub.publish(String(data=reason))
        self.get_logger().warn(reason)

    def tick(self):
        now = self.now()
        if self.state in ('TAKEOFF', 'CRUISE') and (not self.pose_valid() or not self.vio_valid()):
            self.enter_hold('Position or OpenVINS stale; holding for manual recovery')
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
            if ((self.p('require_map_alignment') and not self.localized) or
                    not self.depth_valid()) and self.pose_valid():
                # Map alignment and depth are required to follow a map-frame
                # path, not to maintain altitude at the current PX4 position.
                if self.velocity_active:
                    p = self.position
                    self.hold_target = (p.x, p.y, p.z)
                    self.cmd_at = -math.inf
                self.send_position(self.hold_target)
            elif 0 <= now - self.cmd_at <= float(self.p('command_timeout')):
                self.send_velocity(self.cmd)
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
    finally:
        node.destroy_node()
        rclpy.shutdown()
