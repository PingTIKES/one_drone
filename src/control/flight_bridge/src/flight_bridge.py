"""PX4 OFFBOARD selects EGO velocity tracking or a latched position hold.

VIO health, map and depth are diagnostics, not navigation interlocks.
The optional takeoff service is separate from manual airborne takeover.
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
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, Range
from mavros_msgs.msg import State, EstimatorStatus

from flight_bridge.mavros_adapter import MavrosAdapter, LocalPosition, wrap
from flight_bridge.trajectory_controller import TrajectoryController
from flight_bridge.trajectory_validator import TrajectoryValidator
from flight_bridge.yaw_manager import YawDecision, YawManager, world_to_body
from flight_bridge.bootstrap_takeoff import BootstrapHeightController



class FlightBridge(Node):
    def __init__(self):
        super().__init__('flight_bridge')
        defaults = dict(mavros_ns='/mavros', takeoff_altitude=2.,
                        localization_source='openvins',
                        bootstrap_takeoff=False, bootstrap_height=.7, bootstrap_height_source='local',
                        bootstrap_hover_thrust=.5, bootstrap_min_thrust=.1,
                        bootstrap_max_thrust=.7, bootstrap_height_gain=1.,
                        bootstrap_velocity_gain=.15, bootstrap_integral_gain=.08,
                        bootstrap_climb_speed=.3, bootstrap_thrust_slew=.3,
                        bootstrap_timeout=20., arming_timeout=15.,
                        bootstrap_height_margin=.5, bootstrap_max_tilt_deg=20.,
                        bootstrap_yaw_reset_deg=15.,
                        bootstrap_localization_stable_time=.5,
                        estimator_timeout=1.,
                        yaw_control=True,
                        max_horizontal_speed=.5, max_vertical_speed=.3,
                        max_yaw_rate=.35, position_gain=.8,
                        yaw_soft_limit_deg=30., yaw_hard_limit_deg=100.,
                        yaw_reverse_limit_deg=150., yaw_min_speed_scale=.25,
                        yaw_rate_gain=.8,
                        arrival_position_deadband=.15,
                        arrival_velocity_deadband=.10,
                        arrival_exit_scale=1.5,
                        command_timeout=.2, pose_timeout=.5,
                        depth_heartbeat_timeout=.5, depth_grace=.8,
                        takeoff_tolerance=.2, takeoff_stable_time=1.,
                        status_timeout=1.5,
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
        if self.p('bootstrap_height_source') not in ('local', 'range'):
            raise ValueError('bootstrap_height_source must be local or range')
        for key in ('takeoff_altitude', 'bootstrap_height', 'bootstrap_timeout', 'arming_timeout',
                    'bootstrap_height_margin', 'bootstrap_max_tilt_deg', 'estimator_timeout'):
            if not math.isfinite(float(self.p(key))) or float(self.p(key)) <= 0:
                raise ValueError(key + ' must be positive and finite')
        stable = float(self.p('bootstrap_localization_stable_time'))
        if not math.isfinite(stable) or stable < 0:
            raise ValueError('bootstrap_localization_stable_time must be nonnegative and finite')
        # Validate before advertising /takeoff, not after an armed request.
        BootstrapHeightController(*[float(self.p(name)) for name in (
            'bootstrap_hover_thrust', 'bootstrap_min_thrust', 'bootstrap_max_thrust',
            'bootstrap_height_gain', 'bootstrap_velocity_gain', 'bootstrap_integral_gain',
            'bootstrap_climb_speed', 'bootstrap_thrust_slew')])
        self.yaw_manager = YawManager(
            float(self.p('yaw_soft_limit_deg')),
            float(self.p('yaw_hard_limit_deg')),
            float(self.p('yaw_reverse_limit_deg')),
            float(self.p('yaw_min_speed_scale')),
            float(self.p('max_yaw_rate')),
            float(self.p('max_yaw_rate')),
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
        self.state = 'IDLE'
        self.bootstrap = None
        self.bootstrap_started = self.takeoff_requested_at = None
        self.bootstrap_ready_since = None
        self.bootstrap_ground_z = self.bootstrap_yaw = None
        self.bootstrap_last_tick = None
        self.takeoff_pending = False
        self.navigation_ready = False
        self.estimator = None
        self.estimator_at = self.vertical_at = self.velocity_at = self.attitude_at = -math.inf
        self.height_z = self.vertical_speed = self.attitude_yaw = self.attitude_tilt = math.nan
        self.range_height = math.nan
        self.range_at = -math.inf
        self.attitude_stamp = None
        self.localization_ready = False
        self.localization_ready_at = -math.inf
        self.position = None
        self.initial_heading = None
        self.status = None
        self.pose_at = self.status_at = self.vio_at = self.cmd_at = self.obstacle_at = -math.inf
        self.last_obstacle_good = -math.inf
        self.obstacle_ok = False
        self.vio_state = 'INVALID'
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
        self.reset_command_cutoff = -math.inf
        self.reset_sensor_stamp = -math.inf
        reset_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Header, '/vio_reset_event', self.on_vio_reset, reset_qos)
        self.velocity_active = False
        self.command_clamped = False
        self.yaw_mode = 'YAW_HOLD'
        self.arrival_hold = False
        self.translation_scale = 1.0
        self.trajectory_status = 'WAITING_FOR_TRAJECTORY'
        self.hold_reason = ''
        self.was_offboard = False
        self.landing_mode_seen = False
        mavros = '/' + str(self.p('mavros_ns')).strip('/')
        qos = qos_profile_sensor_data
        self.px4_adapter = MavrosAdapter(self, mavros)
        self.state_pub = self.create_publisher(String, 'flight_state', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hold_reason_pub = self.create_publisher(String, 'flight_hold_reason', latched)
        self.safety_pub = self.create_publisher(String, 'flight_safety_status', latched)
        self.system_pub = self.create_publisher(SystemStatus, 'system_status', latched)
        self.ready_pub = self.create_publisher(Bool, '/flight_navigation_ready', latched)
        self.ready_pub.publish(Bool(data=False))
        self.hold_reason_pub.publish(String(data=''))
        self.safety_pub.publish(String(
            data=f'OFFBOARD_HOLD_OR_EGO max_speed={float(self.p("max_horizontal_speed")):.2f}m/s'))
        self.create_subscription(PoseStamped, mavros + '/local_position/pose', self.on_mavros_pose, qos)
        self.create_subscription(State, mavros + '/state', self.on_status, qos)
        self.create_subscription(EstimatorStatus, mavros + '/estimator_status', self.on_estimator, qos)
        self.create_subscription(TwistStamped, mavros + '/local_position/velocity_local', self.on_velocity, qos)
        self.create_subscription(Imu, mavros + '/imu/data', self.on_attitude, qos)
        self.create_subscription(Range, '/microvio/range', self.on_range, 10)
        self.create_subscription(Bool, '/microvio/navigation_ready', self.on_localization_ready, 10)
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

    def offboard_active(self):
        return (self.status is not None and self.status.connected and self.status.mode == 'OFFBOARD' and
                0 <= self.now() - self.status_at <= float(self.p('status_timeout')))

    def heading_valid(self):
        return (self.position is not None and math.isfinite(self.position.heading) and
                0 <= self.now() - self.pose_at <= float(self.p('pose_timeout')))

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
                 f'command_clamped={self.command_clamped} '
                 f'yaw_control={self.p("yaw_control")} '
                 f'initial_heading={self.initial_heading} '
                 f'yaw_mode={self.yaw_mode} translation_scale={self.translation_scale:.2f} '
                 f'vio_state={self.vio_state} '
                 f'offboard={self.offboard_active()} vio_age={vio_age:.3f}s '
                 f'depth_valid={self.depth_valid()} depth_age={depth_age:.3f}s '
                 f'trajectory_status={self.trajectory_status} '
                 f'mavros_command={self.px4_adapter.last_command_status}'))

    def publish_system_status(self):
        msg = SystemStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.flight_state = self.state
        msg.vio_state = self.vio_state
        msg.hold_reason = self.hold_reason
        msg.px4_position_valid = self.pose_valid()
        msg.px4_offboard = self.offboard_active()
        msg.depth_valid = self.depth_valid()
        msg.map_valid = self.map_valid()
        msg.trajectory_valid = (self.cmd is not None and
                                0 <= self.now() - self.cmd_at <= float(self.p('command_timeout')) and
                                self.trajectory_status == 'OK')
        self.system_pub.publish(msg)

    def on_mavros_pose(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not -0.05 <= self.now() - stamp <= float(self.p('pose_timeout')):
            return
        p, q = msg.pose.position, msg.pose.orientation
        # Vertical feedback is independent of horizontal initialization.
        if math.isfinite(p.z):
            self.height_z, self.vertical_at = float(p.z), self.now()
        values = (p.x, p.y, p.z, q.w, q.x, q.y, q.z)
        if not all(math.isfinite(v) for v in values):
            return
        norm = math.sqrt(sum(v*v for v in (q.w, q.x, q.y, q.z)))
        if norm < 1e-6:
            return
        w, x, y, z = (v/norm for v in (q.w, q.x, q.y, q.z))
        yaw_enu = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        self.on_position(LocalPosition(p.y, p.x, -p.z, wrap(math.pi/2-yaw_enu)))

    def on_estimator(self, msg):
        self.estimator, self.estimator_at = msg, self.now()

    def on_velocity(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (-.05 <= self.now() - stamp <= float(self.p('pose_timeout')) and
                math.isfinite(msg.twist.linear.z)):
            self.vertical_speed, self.velocity_at = msg.twist.linear.z, self.now()

    def on_attitude(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.orientation
        norm = math.sqrt(sum(v*v for v in (q.w, q.x, q.y, q.z)))
        if not (-.05 <= self.now() - stamp <= float(self.p('pose_timeout')) and
                math.isfinite(norm) and norm > 1e-6 and msg.orientation_covariance[0] >= 0):
            return
        w, x, y, z = (v/norm for v in (q.w, q.x, q.y, q.z))
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        if self.bootstrap is not None and self.attitude_stamp is not None:
            dt = stamp - self.attitude_stamp
            roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
            pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
            rates = msg.angular_velocity
            if (0 < dt <= float(self.p('pose_timeout')) and abs(math.cos(pitch)) > .2 and
                    all(math.isfinite(v) for v in (rates.y, rates.z))):
                predicted = (math.sin(roll)*rates.y + math.cos(roll)*rates.z) / math.cos(pitch) * dt
                jump = wrap(yaw - self.attitude_yaw - predicted)
                if abs(jump) > math.radians(float(self.p('bootstrap_yaw_reset_deg'))):
                    # EV initialization may rename PX4 yaw axes. Shift the
                    # reference with that jump instead of rotating the craft.
                    self.bootstrap_yaw = wrap(self.bootstrap_yaw + jump)
                    self.initial_heading = wrap(math.pi/2 - self.bootstrap_yaw)
                    self.get_logger().warn('Adjusted takeoff yaw reference after estimator frame reset')
        self.attitude_yaw, self.attitude_stamp = yaw, stamp
        self.attitude_tilt = math.acos(max(-1., min(1., 1-2*(x*x+y*y))))
        self.attitude_at = self.now()

    def on_localization_ready(self, msg):
        self.localization_ready, self.localization_ready_at = bool(msg.data), self.now()

    def on_range(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (-.05 <= self.now() - stamp <= float(self.p('pose_timeout')) and
                math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range and
                math.isfinite(self.attitude_tilt)):
            self.range_height = msg.range * math.cos(self.attitude_tilt)
            self.range_at = self.now()

    def bootstrap_height_measurement(self):
        return self.range_height if self.p('bootstrap_height_source') == 'range' else self.height_z

    def bootstrap_feedback_valid(self):
        e = self.estimator
        return (e is not None and e.velocity_vert_status_flag and
                (e.pos_vert_abs_status_flag or e.pos_vert_agl_status_flag) and
                not e.accel_error_status_flag and
                0 <= self.now() - self.estimator_at <= float(self.p('estimator_timeout')) and
                all(0 <= self.now() - t <= float(self.p('pose_timeout'))
                    for t in (self.range_at if self.p('bootstrap_height_source') == 'range' else self.vertical_at,
                              self.velocity_at, self.attitude_at)) and
                self.attitude_tilt <= math.radians(float(self.p('bootstrap_max_tilt_deg'))))

    def navigation_odometry_ready(self):
        if self.p('bootstrap_takeoff') and self.p('localization_source') == 'microvio' and not (
                self.localization_ready and
                0 <= self.now() - self.localization_ready_at <= float(self.p('pose_timeout'))):
            return False
        if self.ego_odom is None or not 0 <= self.now() - self.ego_odom_at <= float(self.p('pose_timeout')):
            return False
        o = self.ego_odom
        stamp = o.header.stamp.sec + o.header.stamp.nanosec * 1e-9
        p, q = o.pose.pose.position, o.pose.pose.orientation
        return (o.header.frame_id == 'odom' and -.02 <= self.now() - stamp <= float(self.p('pose_timeout')) and
                all(math.isfinite(v) for v in (p.x, p.y, p.z, q.w, q.x, q.y, q.z)) and
                sum(v*v for v in (q.w, q.x, q.y, q.z)) > 1e-12)

    def on_position(self, msg):
        resets = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        reset_changed = (self.last_reset is not None and resets != self.last_reset and
                         self.state in ('TAKEOFF', 'CRUISE', 'HOLD'))
        if (self.initial_heading is not None and self.last_reset is not None and
                resets[2] != self.last_reset[2] and math.isfinite(msg.delta_heading)):
            # A PX4 estimator reset changes its local yaw basis, not the intended
            # physical heading. Express the same reference in the new basis.
            heading = self.initial_heading + msg.delta_heading
            self.initial_heading = math.atan2(math.sin(heading), math.cos(heading))
        self.last_reset = resets
        self.position = msg
        # on_mavros_pose has checked the synchronized sample timestamp;
        # track callback arrival separately to detect a stopped pose stream.
        self.pose_at = self.now()
        if self.initial_heading is None and self.pose_valid():
            self.initial_heading = float(msg.heading)
        if reset_changed:
            self.hold_target = (msg.x, msg.y, msg.z)
            self.takeoff_target = None
            self.clear_command('PX4_ESTIMATE_RESET', cutoff=True)

    def on_status(self, msg):
        self.status = msg
        self.status_at = self.now()

    def on_vio_reset(self, msg):
        self.bootstrap_ready_since = None
        self.reset_sensor_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.ego_odom = None
        self.ego_odom_at = -math.inf
        self.hold_target = None
        if self.takeoff_pending and self.bootstrap is None and self.state == 'TAKEOFF':
            self.abort_takeoff('LOCALIZATION_RESET_DURING_TAKEOFF')
        elif not self.takeoff_pending:
            self.takeoff_target = None
        self.vio_state = "INVALID"
        self.vio_at = -math.inf
        self.clear_command('VIO_RESET', cutoff=True)

    def map_valid(self):
        return (self.map_stamp > 0 and -0.02 <= self.now() - self.map_stamp <= float(self.p('map_timeout')) and
                time.monotonic() - self.planner_at <= float(self.p('planner_heartbeat_timeout')))

    def on_map_heartbeat(self, msg):
        changed = self.planner_instance is not None and self.planner_instance != msg.frame_id
        self.planner_instance = msg.frame_id
        self.map_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.planner_at = time.monotonic()
        if changed:
            self.clear_command('PLANNER_RESTARTED', cutoff=True)

    def clear_command(self, reason, cutoff=False):
        self.cmd = None
        self.cmd_at = -math.inf
        self.trajectory_controller.reset()
        self.trajectory_status = reason
        if cutoff:
            self.reset_command_cutoff = self.now()

    def on_position_command(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= self.reset_command_cutoff:
            return
        actual = self.ego_odom.pose.pose.position if self.ego_odom is not None else None
        valid, reason = self.trajectory_validator.validate(msg, self.now(), actual)
        if not valid:
            self.clear_command(reason)
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
        self.vio_state = msg.data
        self.vio_at = self.now()

    def on_obstacle(self, msg):
        self.obstacle_ok = bool(msg.data)
        self.obstacle_at = self.now()
        if self.obstacle_ok:
            self.last_obstacle_good = self.obstacle_at

    def takeoff(self, _request, response):
        missing = []
        if self.state != 'IDLE' or self.offboard_active():
            missing.append('manual OFFBOARD already active or controller busy')
        if self.status is None or not self.status.connected or self.now() - self.status_at > float(self.p('status_timeout')):
            missing.append('fresh connected MAVROS state')
        elif self.status.armed:
            missing.append('vehicle already armed')
        if self.p('bootstrap_takeoff'):
            if not self.bootstrap_feedback_valid():
                missing.append('fresh attitude and vertical height/velocity feedback (horizontal odom not required)')
        elif not self.pose_valid():
            missing.append('fresh PX4 local position')
        response.success = not missing
        response.message = ('Starting PX4 Offboard takeoff' if response.success else
                            'Takeoff blocked: ' + ', '.join(missing))
        if response.success:
            self.takeoff_pending = True
            self.navigation_ready = False
            self.clear_command('AUTOMATIC_TAKEOFF', cutoff=True)
            self.takeoff_requested_at = self.now()
            self.takeoff_reached_since = None
            if self.p('bootstrap_takeoff'):
                self.bootstrap = BootstrapHeightController(*[float(self.p(name)) for name in (
                    'bootstrap_hover_thrust', 'bootstrap_min_thrust', 'bootstrap_max_thrust',
                    'bootstrap_height_gain', 'bootstrap_velocity_gain', 'bootstrap_integral_gain',
                    'bootstrap_climb_speed', 'bootstrap_thrust_slew')])
                self.bootstrap_ground_z = self.bootstrap_height_measurement()
                self.bootstrap_yaw = self.attitude_yaw
                self.initial_heading = wrap(math.pi/2 - self.bootstrap_yaw)
                self.bootstrap_started = None
                self.bootstrap_ready_since = None
                self.bootstrap_last_tick = self.now()
                self.prestream_count = 0
                self.state = 'BOOTSTRAP_PRESTREAM'
                response.message = 'Starting attitude takeoff; navigation waits for airborne odometry'
                return response
            p = self.position
            self.takeoff_target = (p.x, p.y, p.z - float(self.p('takeoff_altitude')))
            self.hold_target = (p.x, p.y, p.z)
            self.prestream_count = 0
            self.state = 'PRESTREAM'
        return response

    def resume(self, _request, response):
        # Compatibility endpoint: recovery no longer requires a service call.
        response.success = self.offboard_active()
        response.message = ('OFFBOARD automatically follows fresh EGO commands' if response.success
                            else 'PX4 is not in OFFBOARD')
        return response

    def land(self, _request, response):
        response.success = self.state in ('TAKEOFF', 'CRUISE', 'HOLD', 'BOOTSTRAP_TAKEOFF')
        response.message = 'Land commanded' if response.success else 'Not airborne in this controller'
        if response.success:
            self.vehicle_command(21)
            self.state = 'LANDING'
            self.landing_mode_seen = False
            self.takeoff_pending = False
            self.bootstrap = None
            self.navigation_ready = False
        return response

    def vehicle_command(self, command, p1=0., p2=0.):
        self.px4_adapter.command(command, p1, p2)

    def send_position(self, target):
        self.velocity_active = False
        if not self.p('yaw_control'):
            if self.initial_heading is None:
                return
            heading = self.initial_heading
        else:
            heading = self.position.heading if self.pose_valid() else math.nan
        self.px4_adapter.position(target, heading)

    def send_ego_command(self):
        if (not self.heading_valid() or
                not 0 <= self.now() - self.cmd_at <= float(self.p("command_timeout")) or
                self.cmd is None or self.ego_odom is None or
                not 0 <= self.now() - self.ego_odom_at <= float(self.p('pose_timeout'))):
            if self.cmd is not None:
                self.trajectory_status = 'COMMAND_OR_ODOMETRY_STALE'
            return False
        odom = self.ego_odom
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        values = (p.x, p.y, p.z, q.w, q.x, q.y, q.z)
        stamp = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        if (odom.header.frame_id != 'odom' or
                not -0.02 <= self.now() - stamp <= float(self.p('pose_timeout')) or
                not all(math.isfinite(v) for v in values) or
                sum(v*v for v in (q.w, q.x, q.y, q.z)) < 1e-12):
            self.trajectory_status = 'INVALID_ODOMETRY'
            return False
        valid, reason = self.trajectory_validator.validate(self.cmd, self.now(), p)
        if not valid:
            self.clear_command(reason)
            return False
        fixed_heading = not self.p('yaw_control')
        if fixed_heading and self.initial_heading is None:
            return False
        tracking = self.trajectory_controller.track(self.cmd, self.ego_odom)
        self.arrival_hold = tracking.arrival_hold
        if fixed_heading:
            decision = YawDecision(0. if tracking.arrival_hold else 1., 0.,
                                   'INITIAL_YAW_HOLD', 0.)
        else:
            decision = self.yaw_manager.decide(
                tracking.world_velocity, tracking.acceleration,
                self.ego_odom.pose.pose.orientation,
                arrival_hold=tracking.arrival_hold)
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
        yaw_reference = {'yaw_ned': self.initial_heading} if fixed_heading else {}
        self.px4_adapter.velocity_body(
            (bx, by, bz), decision.yaw_rate_flu,
            self.position.heading, tracking.trajectory_id,
            1., **yaw_reference)
        self.trajectory_status = 'OK'
        return True

    def hold_position(self, reason):
        self.state = 'HOLD'
        if reason != self.hold_reason:
            self.hold_reason = reason
            self.hold_reason_pub.publish(String(data=reason))
        self.translation_scale = 0.0
        if self.pose_valid():
            if self.velocity_active or self.hold_target is None:
                p = self.position
                self.hold_target = (p.x, p.y, p.z)
            self.yaw_mode = 'POSITION_HOLD'
            self.send_position(self.hold_target)
        else:
            # No valid position to latch: request a stop, never replay an old
            # velocity or claim that position lock can be guaranteed.
            self.hold_target = None
            self.velocity_active = False
            self.yaw_mode = 'ZERO_VELOCITY'
            self.px4_adapter.velocity_ned((0., 0., 0.))

    def standby_setpoint(self):
        # PX4 requires a setpoint stream before the RC can select OFFBOARD.
        # These setpoints do not take control in a manual flight mode.
        self.velocity_active = False
        if self.pose_valid():
            p = self.position
            self.send_position((p.x, p.y, p.z))
        else:
            self.px4_adapter.velocity_ned((0., 0., 0.))

    def tick(self):
        active = self.offboard_active()
        if self.bootstrap is not None:
            self.tick_bootstrap(active)
            self.was_offboard = active
            self.publish_flight_state()
            return
        if (self.status is not None and self.status.connected and self.status.mode == 'OFFBOARD' and
                not active):
            # A missing status update is not evidence of a pilot mode change.
            # Stop tracking and latch a hold instead of chasing each new pose.
            self.clear_command('PX4_STATUS_STALE', cutoff=True)
            self.takeoff_target = None
            self.hold_position('PX4_STATUS_STALE')
            self.publish_flight_state()
            return
        if self.was_offboard and not active:
            # RC mode changes always release EGO control. Never automatically
            # request OFFBOARD or arm again after the pilot has switched out.
            self.clear_command('WAIT_OFFBOARD', cutoff=True)
            self.hold_target = self.takeoff_target = None
            self.takeoff_reached_since = None
            self.state = 'IDLE' if self.state != 'LANDING' else 'LANDING'
            self.takeoff_pending = False
        if active and not self.was_offboard:
            p = self.position
            self.hold_target = (p.x, p.y, p.z) if self.pose_valid() else None
            self.velocity_active = False
            self.trajectory_controller.reset()
            if self.state == 'LANDING' and self.landing_mode_seen and self.status.armed:
                self.state = 'HOLD'
        self.was_offboard = active

        if self.state == 'LANDING':
            if self.status is not None and self.status.mode == 'AUTO.LAND':
                self.landing_mode_seen = True
            if self.status is not None and not self.status.armed:
                self.state = 'IDLE'
            elif active and self.pose_valid():
                # SET_MODE is asynchronous. Keep the previous hold alive until
                # the FC actually leaves OFFBOARD, avoiding a stream-loss race.
                if self.hold_target is None:
                    p = self.position
                    self.hold_target = (p.x, p.y, p.z)
                self.send_position(self.hold_target)
                self.prestream_count += 1
                if self.prestream_count % 20 == 0:
                    self.vehicle_command(21)
        elif not active:
            if self.state in ('PRESTREAM', 'ARMING'):
                # Only an explicit /takeoff request may arm or change mode.
                if not self.pose_valid():
                    self.takeoff_target = None
                    self.takeoff_pending = False
                    self.state = 'IDLE'
                elif self.now() - self.takeoff_requested_at > float(self.p('arming_timeout')):
                    self.takeoff_target = None
                    self.takeoff_pending = False
                    self.state = 'IDLE'
                    self.clear_command('ARMING_TIMEOUT', cutoff=True)
                else:
                    self.send_position(self.hold_target)
                    self.prestream_count += 1
                    if self.prestream_count >= 20 and self.prestream_count % 20 == 0:
                        self.vehicle_command(176, 1., 6.)
                        self.vehicle_command(400, 1.)
                        self.state = 'ARMING'
            else:
                self.state = 'IDLE'
                self.standby_setpoint()
        else:
            if self.state in ('PRESTREAM', 'ARMING') and self.takeoff_target is not None:
                if not self.status.armed:
                    self.state = 'ARMING'
                    self.standby_setpoint()
                    self.prestream_count += 1
                    if self.prestream_count % 20 == 0:
                        self.vehicle_command(400, 1.)
                    self.publish_flight_state()
                    return
                self.state = 'TAKEOFF'
            if self.state == 'TAKEOFF' and self.takeoff_target is not None:
                if not self.pose_valid():
                    self.abort_takeoff('TAKEOFF_POSITION_UNAVAILABLE')
                else:
                    self.send_position(self.takeoff_target)
                    p = self.position
                    error = math.dist((p.x, p.y, p.z), self.takeoff_target)
                    if error < float(self.p('takeoff_tolerance')):
                        if self.takeoff_reached_since is None:
                            self.takeoff_reached_since = self.now()
                        elif self.now() - self.takeoff_reached_since >= float(self.p('takeoff_stable_time')):
                            self.hold_target = self.takeoff_target
                            self.takeoff_target = None
                            self.takeoff_pending = False
                            self.clear_command('TAKEOFF_COMPLETED', cutoff=True)
                            self.hold_position('WAITING_FOR_TRAJECTORY')
                    else:
                        self.takeoff_reached_since = None
            elif self.takeoff_pending:
                self.hold_position('WAITING_FOR_LOCALIZATION')
            elif self.send_ego_command():
                self.state = 'CRUISE'
                if self.hold_reason:
                    self.hold_reason = ''
                    self.hold_reason_pub.publish(String(data=''))
            else:
                self.hold_position(self.trajectory_status)
        self.publish_flight_state()

    def publish_flight_state(self):
        self.navigation_ready = (self.offboard_active() and self.status.armed and
                                 self.state in ('HOLD', 'CRUISE') and not self.takeoff_pending and
                                 self.navigation_odometry_ready())
        self.ready_pub.publish(Bool(data=self.navigation_ready))
        self.state_pub.publish(String(data=self.state))
        self.publish_system_status()

    def abort_takeoff(self, reason):
        self.hold_reason = reason
        self.hold_reason_pub.publish(String(data=reason))
        self.get_logger().error(reason)
        self.clear_command(reason, cutoff=True)
        self.bootstrap = None
        self.takeoff_pending = False
        self.takeoff_target = self.hold_target = None
        # Continue no attitude thrust after abort. PX4 owns the landing/failsafe.
        if self.status is not None and self.status.armed:
            self.vehicle_command(21)
            self.state = 'LANDING'
            self.landing_mode_seen = False
        else:
            self.state = 'IDLE'

    def tick_bootstrap(self, active):
        if self.was_offboard and not active:
            # Respect any FC failsafe or operator mode exit; never rearm/reenter.
            self.bootstrap = None
            self.takeoff_pending = False
            self.clear_command('WAIT_OFFBOARD', cutoff=True)
            self.state = 'IDLE'
            return
        if not self.bootstrap_feedback_valid():
            self.abort_takeoff('BOOTSTRAP_VERTICAL_OR_ATTITUDE_UNAVAILABLE')
            return
        if self.bootstrap_started is None:
            if self.now() - self.takeoff_requested_at > float(self.p('arming_timeout')):
                self.abort_takeoff('BOOTSTRAP_ARMING_TIMEOUT')
                return
            # Zero thrust while waiting for actual mode + arm confirmation.
            self.px4_adapter.attitude(self.bootstrap_yaw, 0.)
            self.prestream_count += 1
            if not active:
                if self.prestream_count >= 25 and self.prestream_count % 20 == 5:
                    self.vehicle_command(176, 1., 6.)
            elif not self.status.armed:
                if self.prestream_count % 20 == 5:
                    self.vehicle_command(400, 1.)
            else:
                self.bootstrap_started = self.now()
                self.bootstrap_last_tick = self.now()
                self.state = 'BOOTSTRAP_TAKEOFF'
            return
        if not self.status.armed:
            self.abort_takeoff('BOOTSTRAP_UNEXPECTED_DISARM')
            return
        height = self.bootstrap_height_measurement() - self.bootstrap_ground_z
        target = min(float(self.p('bootstrap_height')), float(self.p('takeoff_altitude')))
        if self.now() - self.bootstrap_started > float(self.p('bootstrap_timeout')):
            self.abort_takeoff('BOOTSTRAP_LOCALIZATION_TIMEOUT')
            return
        if height > target + float(self.p('bootstrap_height_margin')) or height < -.3:
            self.abort_takeoff('BOOTSTRAP_HEIGHT_OUT_OF_BOUNDS')
            return
        dt = max(.001, self.now() - self.bootstrap_last_tick)
        self.bootstrap_last_tick = self.now()
        thrust = self.bootstrap.update(target - height, self.vertical_speed, dt)
        ready = (height >= target - float(self.p('takeoff_tolerance')) and
                self.navigation_odometry_ready() and self.pose_valid() and
                self.estimator.velocity_horiz_status_flag and
                self.estimator.pos_horiz_rel_status_flag and
                not self.estimator.const_pos_mode_status_flag)
        if not ready:
            self.bootstrap_ready_since = None
        elif self.bootstrap_ready_since is None:
            self.bootstrap_ready_since = self.now()
        if (ready and self.now() - self.bootstrap_ready_since >=
                float(self.p('bootstrap_localization_stable_time'))):
            # Current XY is now meaningful. Keep height relative to the ground,
            # not relative to the first airborne odometry sample.
            p = self.position
            self.takeoff_target = (p.x, p.y, p.z - (float(self.p('takeoff_altitude')) - height))
            self.hold_target = (p.x, p.y, p.z)
            self.takeoff_reached_since = None
            self.bootstrap = None
            self.clear_command('AIRBORNE_LOCALIZATION_INITIALIZED', cutoff=True)
            self.state = 'TAKEOFF'
            self.send_position(self.takeoff_target)
        else:
            self.px4_adapter.attitude(self.bootstrap_yaw, thrust)


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
