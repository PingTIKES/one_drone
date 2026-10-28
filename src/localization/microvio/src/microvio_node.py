"""Read PX4 MAVLink odometry aided by MicoVIO. Never sends flight commands."""
import json
import math
import os
import time
import uuid

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import Mavlink, State, TimesyncStatus
from mavros_msgs.srv import MessageInterval
from vio_bridge.mavlink_codec import decode_frame
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Header, String
from tf2_ros import TransformBroadcaster
from vio_bridge.vio_tf_geometry import InitialYawAlignment

from microvio.geometry import ClockSync, convert, rotation


class Microvio(Node):
    def __init__(self):
        super().__init__('microvio')
        defaults = dict(mavros_ns='/mavros', target_system=1,
                        target_component=1,
                        odom_topic='/microvio/odom', health_topic='/microvio/health',
                        diagnostics_topic='/microvio/diagnostics', reset_topic='/microvio/reset_event',
                        range_topic='/microvio/range', odom_frame='microvio_odom',
                        body_frame='microvio_base_link', range_frame='microvio_range',
                        publish_tf=True,
                        heartbeat_timeout=3., odom_timeout=.5, estimator_timeout=1.,
                        timesync_timeout=5., max_timesync_rtt=.1,
                        future_tolerance=.05, reset_hold_time=1., unknown_variance=1000.,
                        request_streams=True, odometry_rate=30., required_estimator_flags=47,
                        expected_estimator_type=8, external_vision_reference='local_frd',
                        initial_yaw_zero=True,initial_position_zero=True)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ('heartbeat_timeout', 'odom_timeout',
                     'estimator_timeout', 'timesync_timeout',
                     'max_timesync_rtt', 'unknown_variance', 'odometry_rate'):
            if not math.isfinite(float(self.p(name))) or float(self.p(name)) <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if not 1 <= int(self.p('target_system')) <= 255:
            raise ValueError('target_system must be 1..255')
        if self.p('external_vision_reference') not in ('local_frd', 'ned'):
            raise ValueError('external_vision_reference must be local_frd or ned')
        self.connected = False
        self.sync = ClockSync()
        self.initial_yaw_alignment = InitialYawAlignment(
            zero_position=bool(self.p('initial_position_zero')),
            zero_yaw=bool(self.p('initial_yaw_zero')))
        self.provisional_alignment = InitialYawAlignment(
            zero_position=bool(self.p('initial_position_zero')),
            zero_yaw=bool(self.p('initial_yaw_zero')))
        self.ground_z = None
        self.range_distance = self.ground_range = None
        self.range_at = -math.inf
        self.heartbeat_at = self.odom_at = self.estimator_at = -math.inf
        self.reset_at = -math.inf
        self.remote_stamp = self.odom_stamp = self.reset_counter = None
        self.flags = None
        self.quality = None
        self.estimator_type = None
        self.source_pose_frame = self.source_velocity_frame = None
        self.pose_diagonal_known = self.twist_diagonal_known = False
        self.reason = 'WAIT_CONNECTION'
        self.armed = False
        self.stream_requested = False
        self.interval_index = 0
        self.interval_pending = False
        self.received_count = self.published_count = 0
        self.started_at = time.monotonic()
        self.epoch = uuid.uuid4().hex[:12]
        self.reset_index = 0
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom_pub = self.create_publisher(Odometry, str(self.p('odom_topic')), 10)
        self.health_pub = self.create_publisher(String, str(self.p('health_topic')), latched)
        self.diag_pub = self.create_publisher(String, str(self.p('diagnostics_topic')), 10)
        self.reset_pub = self.create_publisher(Header, str(self.p('reset_topic')), latched)
        self.range_pub = self.create_publisher(Range, str(self.p('range_topic')), 10)
        self.ready_pub = self.create_publisher(Bool, '/microvio/navigation_ready', 10)
        self.tf = TransformBroadcaster(self)
        root = '/' + str(self.p('mavros_ns')).strip('/')
        raw_topic = f'/uas{int(self.p("target_system"))}/mavlink_source'
        self.create_subscription(Mavlink, raw_topic, self.on_frame, qos_profile_sensor_data)
        self.create_subscription(State, root + '/state', self.on_mavros_state, latched)
        self.create_subscription(PoseStamped, root + '/local_position/pose', self.on_ground_pose, qos_profile_sensor_data)
        self.create_subscription(TimesyncStatus, root + '/timesync_status', self.on_timesync, qos_profile_sensor_data)
        self.interval_client = self.create_client(MessageInterval, root + '/set_message_interval')
        self.create_timer(1., self.request_intervals)
        self.create_timer(.1, self.publish_health)
        self.create_timer(1., self.publish_diagnostics)
        self.get_logger().info('Telemetry only: no arming, mode changes, setpoints, or external-vision writes')

    def p(self, key):
        return self.get_parameter(key).value

    def reset_event(self, reason):
        self.reset_index += 1
        self.reset_at = time.monotonic()
        self.reset_pub.publish(Header(stamp=self.get_clock().now().to_msg(),
                                      frame_id=f'microvio:{self.epoch}:{self.reset_index}:{reason}'))
        self.get_logger().warn('PX4 odometry reset: ' + reason)

    def on_mavros_state(self, msg):
        if self.connected and not msg.connected:
            self.sync = ClockSync()
            self.odom_at = -math.inf
            self.reset_event('MAVROS_CONNECTION_LOST')
            self.stream_requested = False
        self.connected = msg.connected
        self.armed = msg.armed
        self.flight_mode = msg.mode

    def on_timesync(self, msg):
        if (not math.isfinite(msg.round_trip_time_ms) or
                not 0 <= msg.round_trip_time_ms <= float(self.p('max_timesync_rtt')) * 1000 or
                msg.remote_timestamp_ns == 0 or msg.estimated_offset_ns == 0):
            return
        self.sync.offset_ns = int(msg.estimated_offset_ns)
        self.sync.last_reply_at = time.monotonic()

    def on_ground_pose(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not self.armed and -.05 <= age <= float(self.p('odom_timeout')) and math.isfinite(msg.pose.position.z):
            # ENU and NWU share the same up axis. Preserve the ground datum
            # when XY/yaw become reliable only after liftoff.
            self.ground_z = float(msg.pose.position.z)

    def aiding_ready(self):
        return (self.flags is not None and
                time.monotonic() - self.estimator_at <= float(self.p('estimator_timeout')) and
                not self.flags & (128 | 2048) and
                self.flags & self.required_flags() == self.required_flags())

    def on_frame(self, frame):
        if (frame.sysid != int(self.p('target_system')) or
                frame.compid != int(self.p('target_component'))):
            return
        try:
            self.handle_message(decode_frame(frame))
        except ValueError as error:
            self.get_logger().warn(str(error), throttle_duration_sec=2.)

    def request_intervals(self):
        if (not self.connected or not self.p('request_streams') or
                self.stream_requested or self.interval_pending or
                not self.interval_client.service_is_ready()):
            return
        intervals = ((32, 30.), (31, 30.), (331, float(self.p('odometry_rate'))), (230, 5.), (132, 10.))
        message_id, hz = intervals[self.interval_index]
        self.interval_pending = True
        future = self.interval_client.call_async(MessageInterval.Request(message_id=message_id, message_rate=hz))
        def done(result):
            self.interval_pending = False
            try:
                if result.result().success:
                    self.interval_index += 1
                    if self.interval_index == len(intervals):
                        self.stream_requested = True
                        self.interval_index = 0
            except Exception as error:
                self.get_logger().warn(str(error), throttle_duration_sec=2.)
        future.add_done_callback(done)

    def handle_message(self, msg):
        kind = msg.get_type()
        now = time.monotonic()
        if kind == 'HEARTBEAT':
            if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
                return
            self.heartbeat_at = now
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == 'ESTIMATOR_STATUS':
            self.flags, self.estimator_at = int(msg.flags), now
        elif kind == 'ODOMETRY':
            self.on_odometry(msg)
        elif kind == 'DISTANCE_SENSOR' and msg.orientation == 25:
            distance = msg.current_distance * .01
            if not msg.min_distance <= msg.current_distance <= msg.max_distance:
                return
            try:
                stamp = self.sync.stamp(int(msg.time_boot_ms) * 1000)
            except ValueError:
                return
            out = Range()
            out.header.frame_id = str(self.p('range_frame'))
            out.header.stamp.sec, out.header.stamp.nanosec = divmod(stamp, 1000000000)
            out.radiation_type = Range.INFRARED
            out.min_range, out.max_range = msg.min_distance * .01, msg.max_distance * .01
            out.range = distance
            self.range_distance, self.range_at = distance, now
            if not self.armed:
                self.ground_range = distance
            self.range_pub.publish(out)

    def on_odometry(self, msg):
        self.received_count += 1
        self.quality, self.estimator_type = int(msg.quality), int(msg.estimator_type)
        self.source_pose_frame, self.source_velocity_frame = int(msg.frame_id), int(msg.child_frame_id)
        raw = int(msg.time_usec)
        if self.remote_stamp is not None and raw <= self.remote_stamp:
            if raw < self.remote_stamp - 1_000_000:
                self.sync = ClockSync()
                self.remote_stamp = None
                self.odom_at = -math.inf
                self.reset_event('SOURCE_CLOCK_RESTART')
            self.reason = 'SOURCE_STAMP_NOT_INCREASING'
            return
        self.remote_stamp = raw
        if self.reset_counter is not None and int(msg.reset_counter) != self.reset_counter:
            self.reset_event('PX4_RESET_COUNTER_CHANGED')
        self.reset_counter = int(msg.reset_counter)
        try:
            if self.estimator_type != int(self.p('expected_estimator_type')):
                raise ValueError('UNEXPECTED_ESTIMATOR_TYPE')
            if self.quality < 0:
                raise ValueError('ODOMETRY_FAILED')
            stamp = self.sync.stamp(raw)
            age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
            if not -float(self.p('future_tolerance')) <= age <= float(self.p('odom_timeout')):
                raise ValueError('STALE_OR_FUTURE_SOURCE_STAMP')
            pose = convert(msg, float(self.p('unknown_variance')))
            if not self.armed:
                self.ground_z = float(pose['position'][2])
            if self.p('initial_yaw_zero') or self.p('initial_position_zero'):
                alignment = self.initial_yaw_alignment
                if alignment.rotation is None:
                    if not self.aiding_ready():
                        alignment = self.provisional_alignment
                    else:
                        alignment.apply(pose['position'], rotation(pose['quaternion']), pose['pose_covariance'])
                        if self.p('initial_position_zero') and self.ground_z is not None:
                            alignment.initial_position[2] = self.ground_z
                            if (self.armed and self.ground_range is not None and
                                    time.monotonic() - self.range_at <= float(self.p('odom_timeout'))):
                                tilt_cos = rotation(pose['quaternion'])[2, 2]
                                height = self.range_distance * tilt_cos - self.ground_range
                                alignment.initial_position[2] = pose['position'][2] - height
                        if self.published_count:
                            self.reset_event('AIRBORNE_ORIGIN_INITIALIZED')
                pose['position'], pose['quaternion'], pose['pose_covariance'] = alignment.apply(
                    pose['position'], rotation(pose['quaternion']), pose['pose_covariance'])
        except ValueError as error:
            self.reason = str(error)
            return
        self.pose_diagonal_known = pose['pose_diagonal_known']
        self.twist_diagonal_known = pose['twist_diagonal_known']
        out = Odometry()
        out.header.frame_id = str(self.p('odom_frame'))
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(stamp, 1000000000)
        out.child_frame_id = str(self.p('body_frame'))
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, pose['position'])
        q = pose['quaternion']
        out.pose.pose.orientation.w, out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z = map(float, q)
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, pose['velocity'])
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, pose['omega'])
        out.pose.covariance = pose['pose_covariance'].reshape(-1).tolist()
        out.twist.covariance = pose['twist_covariance'].reshape(-1).tolist()
        self.odom_pub.publish(out)
        if self.p('publish_tf'):
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = out.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, pose['position'])
            tf.transform.rotation = out.pose.pose.orientation
            self.tf.sendTransform(tf)
        self.published_count += 1
        self.odom_at, self.odom_stamp = time.monotonic(), stamp
        self.reason = 'OK'

    def required_flags(self):
        flags = int(self.p('required_estimator_flags'))
        if self.p('external_vision_reference') == 'local_frd':
            # PX4 1.17.0 intentionally clears yaw_align after resetting to FRD
            # vision yaw. ESTIMATOR_ATTITUDE also depends on yaw_align, so that
            # global-alignment bit is not a valid local-FRD failure criterion.
            # The actual quaternion, timestamps, and aiding flags remain checked.
            flags &= ~1
        return flags

    def health(self):
        now = time.monotonic()
        if not self.connected:
            return 'INVALID', 'MAVROS_DISCONNECTED'
        if now - self.heartbeat_at > float(self.p('heartbeat_timeout')):
            return 'INVALID', 'PX4_HEARTBEAT_STALE'
        if now - self.sync.last_reply_at > float(self.p('timesync_timeout')):
            return 'INVALID', 'TIMESYNC_NOT_READY'
        if now - self.odom_at > float(self.p('odom_timeout')):
            return 'INVALID', self.reason if self.reason != 'OK' else 'ODOMETRY_STALE'
        age = (self.get_clock().now().nanoseconds - self.odom_stamp) * 1e-9
        if not -float(self.p('future_tolerance')) <= age <= float(self.p('odom_timeout')):
            return 'INVALID', 'SOURCE_STAMP_AGE'
        if self.reason != 'OK':
            return 'INVALID', self.reason
        if now - self.estimator_at > float(self.p('estimator_timeout')):
            return 'DEGRADED', 'ESTIMATOR_STATUS_STALE'
        required = self.required_flags()
        if self.flags & 2048:
            return 'INVALID', 'ESTIMATOR_ACCEL_ERROR'
        if not self.pose_diagonal_known:
            return 'DEGRADED', 'POSE_COVARIANCE_UNKNOWN'
        if self.flags & 128:
            return 'DEGRADED', 'ESTIMATOR_CONSTANT_POSITION'
        if self.flags & required != required:
            return 'DEGRADED', 'ESTIMATOR_FLAGS_INCOMPLETE'
        if now - self.reset_at < float(self.p('reset_hold_time')):
            return 'DEGRADED', 'ESTIMATOR_RECENT_RESET'
        return 'VALID', 'PX4_ESTIMATE_VALID_MODULE_HEALTH_UNAVAILABLE'

    def publish_health(self):
        state, _ = self.health()
        self.health_pub.publish(String(data=state))
        origin_ready = (self.initial_yaw_alignment.rotation is not None or
                        not (self.p('initial_yaw_zero') or self.p('initial_position_zero')))
        self.ready_pub.publish(Bool(data=origin_ready and self.aiding_ready() and state != 'INVALID'))

    def publish_diagnostics(self):
        state, reason = self.health()
        now = time.monotonic()
        def age(t):
            return round(now - t, 4) if math.isfinite(t) else None
        data = dict(state=state, reason=reason, source='PX4_FUSED_ODOMETRY',
                    module_raw_health_available=False, quality=self.quality,
                    quality_unknown=self.quality == 0, estimator_type=self.estimator_type,
                    source_pose_frame=self.source_pose_frame, source_velocity_frame=self.source_velocity_frame,
                    frame_id=str(self.p('odom_frame')), body_frame=str(self.p('body_frame')),
                    armed=self.armed, flight_mode=getattr(self, 'flight_mode', ''), transport='MAVROS', estimator_flags=self.flags,
                    external_vision_reference=str(self.p('external_vision_reference')),
                    initial_yaw_zero=bool(self.p('initial_yaw_zero')),
                    initial_yaw_rad=self.initial_yaw_alignment.initial_yaw,
                    initial_position_zero=bool(self.p('initial_position_zero')),
                    initial_position=(self.initial_yaw_alignment.initial_position.tolist()
                                      if self.initial_yaw_alignment.initial_position is not None else None),
                    effective_required_estimator_flags=self.required_flags(),
                    global_attitude_flag=None if self.flags is None else bool(self.flags & 1),
                    missing_estimator_flags=None if self.flags is None else self.required_flags() & ~self.flags,
                    odom_age=age(self.odom_at), heartbeat_age=age(self.heartbeat_at),
                    timesync_age=age(self.sync.last_reply_at), offset_ns=self.sync.offset_ns,
                    received_odometry=self.received_count, published_odometry=self.published_count,
                    average_output_hz=round(self.published_count / max(.001, now-self.started_at), 2),
                    reset_counter=self.reset_counter, reset_events=self.reset_index,
                    covariance='diagonal_approximation; unknown entries use configured variance',
                    pose_diagonal_known=self.pose_diagonal_known,
                    twist_diagonal_known=self.twist_diagonal_known)
        self.diag_pub.publish(String(data=json.dumps(data)))



def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Microvio()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
