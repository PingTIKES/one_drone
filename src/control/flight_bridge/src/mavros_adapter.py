"""Single MAVROS setpoint publisher. Internal NED is converted to ROS ENU once."""
from dataclasses import dataclass
import math

from flight_interfaces.msg import FlightSetpoint
from mavros_msgs.msg import AttitudeTarget, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, MessageInterval
from flight_bridge.velocity_math import body_flu_to_ned


@dataclass
class LocalPosition:
    x: float = 0.
    y: float = 0.
    z: float = 0.
    heading: float = 0.
    xy_valid: bool = True
    z_valid: bool = True
    xy_reset_counter: int = 0
    z_reset_counter: int = 0
    heading_reset_counter: int = 0
    delta_heading: float = 0.


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class MavrosAdapter:
    def __init__(self, node, namespace):
        self.node = node
        root = '/' + str(namespace).strip('/')
        self.setpoint_pub = node.create_publisher(PositionTarget, root + '/setpoint_raw/local', 10)
        self.attitude_pub = node.create_publisher(AttitudeTarget, root + '/setpoint_raw/attitude', 10)
        self.audit_pub = node.create_publisher(FlightSetpoint, '/flight_setpoint', 10)
        self.mode_client = node.create_client(SetMode, root + '/set_mode')
        self.arm_client = node.create_client(CommandBool, root + '/cmd/arming')
        self.pending = {}
        self.last_command_status = 'NO_COMMAND'
        self.interval_client = node.create_client(MessageInterval, root + '/set_message_interval')
        self.interval_index = 0
        self.interval_pending = False
        node.create_timer(1., self.request_feedback)

    def request_feedback(self):
        # microvio already owns the complete telemetry interval sequence.
        if (not self.node.p('bootstrap_takeoff') or self.node.p('localization_source') == 'microvio' or
                self.interval_index >= 3 or
                self.interval_pending or not self.interval_client.service_is_ready()):
            return
        message_id, rate = ((32, 30.), (31, 30.), (230, 5.))[self.interval_index]
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

    def command(self, command, p1=0., p2=0.):
        if command == 176 and p1 == 1. and p2 == 6.:
            client, request, key = self.mode_client, SetMode.Request(custom_mode='OFFBOARD'), 'OFFBOARD'
        elif command == 400:
            client, request, key = self.arm_client, CommandBool.Request(value=bool(p1)), 'ARM'
        elif command == 21:
            client, request, key = self.mode_client, SetMode.Request(custom_mode='AUTO.LAND'), 'LAND'
        else:
            raise ValueError('unsupported flight command')
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

    def _setpoint(self, mode, position, velocity, yaw, yaw_rate, trajectory_id=0, confidence=0.):
        msg = FlightSetpoint()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'px4_local_ned'
        msg.mode = mode
        msg.position.x, msg.position.y, msg.position.z = map(float, position)
        msg.velocity.x, msg.velocity.y, msg.velocity.z = map(float, velocity)
        msg.acceleration.x = msg.acceleration.y = msg.acceleration.z = math.nan
        msg.yaw, msg.yaw_rate = float(yaw), float(yaw_rate)
        msg.trajectory_id, msg.confidence = int(trajectory_id), float(confidence)
        return msg

    def apply(self, setpoint):
        if setpoint.header.frame_id != 'px4_local_ned':
            raise ValueError('flight setpoint must be in PX4 local NED')
        velocity_mode = setpoint.mode == FlightSetpoint.MODE_VELOCITY
        if not velocity_mode and setpoint.mode != FlightSetpoint.MODE_POSITION:
            raise ValueError('unsupported flight setpoint mode')
        p, v = setpoint.position, setpoint.velocity
        selected = (v.x, v.y, v.z) if velocity_mode else (p.x, p.y, p.z)
        if not all(math.isfinite(x) for x in selected):
            raise ValueError('nonfinite active flight setpoint')
        out = PositionTarget()
        out.header.stamp = setpoint.header.stamp
        out.header.frame_id = 'mavros_local_enu'
        # FRAME_LOCAL_NED selects the MAVLink frame. MAVROS expects ENU data
        # here and performs ENU -> NED in setpoint_raw.cpp.
        out.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        out.type_mask = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        if velocity_mode:
            out.type_mask |= PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
            out.velocity.x, out.velocity.y, out.velocity.z = v.y, v.x, -v.z
        else:
            out.type_mask |= PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
            out.position.x, out.position.y, out.position.z = p.y, p.x, -p.z
        if math.isfinite(setpoint.yaw):
            out.yaw = wrap(math.pi/2 - setpoint.yaw)
        else:
            out.type_mask |= PositionTarget.IGNORE_YAW
        if math.isfinite(setpoint.yaw_rate):
            out.yaw_rate = -setpoint.yaw_rate
        else:
            out.type_mask |= PositionTarget.IGNORE_YAW_RATE
        self.setpoint_pub.publish(out)
        self.audit_pub.publish(setpoint)

    def position(self, target, heading):
        self.apply(self._setpoint(FlightSetpoint.MODE_POSITION, target,
                                 (math.nan,) * 3, heading, math.nan))

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

    def velocity_body(self, body_velocity, yaw_rate_flu, heading_ned,
                      trajectory_id, confidence, yaw_ned=math.nan):
        north, east = body_flu_to_ned(body_velocity[0], body_velocity[1], heading_ned)
        self.velocity_ned((north, east, -body_velocity[2]), -yaw_rate_flu,
                          trajectory_id, confidence, yaw_ned)

    def velocity_ned(self, velocity, yaw_rate_ned=math.nan,
                     trajectory_id=0, confidence=0., yaw_ned=math.nan):
        self.apply(self._setpoint(FlightSetpoint.MODE_VELOCITY, (math.nan,) * 3,
                                 velocity, yaw_ned, yaw_rate_ned, trajectory_id, confidence))
