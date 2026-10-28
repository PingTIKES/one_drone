"""Single MAVROS setpoint publisher. Internal NED is converted to ROS ENU once."""
from dataclasses import dataclass
import math

from mavros_msgs.msg import AttitudeTarget, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, MessageInterval
from flight_bridge.velocity_math import body_flu_to_ned


@dataclass
class LocalPosition:
    x: float = 0.
    y: float = 0.
    z: float = 0.
    heading: float = 0.


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class MavrosAdapter:
    def __init__(self, node, namespace, request_feedback=False):
        self.node = node
        root = '/' + str(namespace).strip('/')
        self.setpoint_pub = node.create_publisher(PositionTarget, root + '/setpoint_raw/local', 10)
        self.attitude_pub = node.create_publisher(AttitudeTarget, root + '/setpoint_raw/attitude', 10)
        self.mode_client = node.create_client(SetMode, root + '/set_mode')
        self.arm_client = node.create_client(CommandBool, root + '/cmd/arming')
        self.pending = {}
        self.last_command_status = 'NO_COMMAND'
        self.interval_client = node.create_client(MessageInterval, root + '/set_message_interval')
        self.interval_index = 0
        self.interval_pending = False
        if request_feedback:
            node.create_timer(1., self.request_feedback)

    def request_feedback(self):
        intervals = ((32, 30.), (31, 30.), (230, 5.), (141, 20.))
        if (self.interval_index >= len(intervals) or
                self.interval_pending or not self.interval_client.service_is_ready()):
            return
        message_id, rate = intervals[self.interval_index]
        self.interval_pending = True
        future = self.interval_client.call_async(MessageInterval.Request(message_id=message_id, message_rate=rate))
        def done(result):
            self.interval_pending = False
            try:
                if result.result().success:
                    self.interval_index += 1
            except Exception as error:
                self.node.get_logger().warn('Bootstrap telemetry request: ' + str(error))
        future.add_done_callback(done)

    def set_mode(self, mode):
        self._request(self.mode_client, SetMode.Request(custom_mode=mode), mode)

    def arm(self):
        self._request(self.arm_client, CommandBool.Request(value=True), 'ARM')

    def _request(self, client, request, key):
        if key in self.pending:
            return
        if not client.service_is_ready():
            self.last_command_status = key + '_SERVICE_UNAVAILABLE'
            self.node.get_logger().warn(self.last_command_status, throttle_duration_sec=2.)
            return
        future = client.call_async(request)
        self.pending[key] = future
        def done(result):
            self.pending.pop(key, None)
            try:
                response = result.result()
                accepted = response.success if key == 'ARM' else response.mode_sent
                self.last_command_status = key + ('_SENT' if accepted else '_REJECTED')
            except Exception as error:
                self.last_command_status = key + '_ERROR:' + str(error)
            self.node.get_logger().info(self.last_command_status)
        future.add_done_callback(done)

    def _local_setpoint(self, values, velocity_mode, yaw=math.nan, yaw_rate=math.nan):
        if not all(math.isfinite(x) for x in values):
            raise ValueError('nonfinite active flight setpoint')
        out = PositionTarget()
        out.header.stamp = self.node.get_clock().now().to_msg()
        out.header.frame_id = 'mavros_local_enu'
        # MAVROS expects ENU even though this field selects MAVLink LOCAL_NED.
        out.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        out.type_mask = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        if velocity_mode:
            out.type_mask |= PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
            vector = out.velocity
        else:
            out.type_mask |= PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
            vector = out.position
        vector.x, vector.y, vector.z = float(values[1]), float(values[0]), -float(values[2])
        if math.isfinite(yaw):
            out.yaw = wrap(math.pi/2 - yaw)
        else:
            out.type_mask |= PositionTarget.IGNORE_YAW
        if math.isfinite(yaw_rate):
            out.yaw_rate = -float(yaw_rate)
        else:
            out.type_mask |= PositionTarget.IGNORE_YAW_RATE
        self.setpoint_pub.publish(out)

    def position(self, target, heading):
        self._local_setpoint(target, False, yaw=heading)

    def attitude(self, yaw_enu, thrust):
        if not math.isfinite(yaw_enu) or not math.isfinite(thrust) or not 0 <= thrust <= 1:
            raise ValueError('invalid attitude/thrust setpoint')
        out = AttitudeTarget()
        out.header.stamp = self.node.get_clock().now().to_msg()
        out.header.frame_id = 'mavros_local_enu'
        out.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE |
                         AttitudeTarget.IGNORE_YAW_RATE)
        out.orientation.w, out.orientation.z = math.cos(yaw_enu / 2), math.sin(yaw_enu / 2)
        out.thrust = float(thrust)
        self.attitude_pub.publish(out)

    def velocity_body(self, body_velocity, yaw_rate_flu, heading_ned, yaw_ned=math.nan):
        north, east = body_flu_to_ned(body_velocity[0], body_velocity[1], heading_ned)
        self.velocity_ned((north, east, -body_velocity[2]), -yaw_rate_flu, yaw_ned)

    def velocity_ned(self, velocity, yaw_rate_ned=math.nan, yaw_ned=math.nan):
        self._local_setpoint(velocity, True, yaw_ned, yaw_rate_ned)
