"""The only publisher of PX4 Offboard mode, setpoint and vehicle commands."""
import math

from flight_interfaces.msg import FlightSetpoint
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

from flight_bridge.velocity_math import body_flu_to_ned


class Px4Adapter:
    def __init__(self, node, px4_namespace, target_system):
        self.node = node
        self.target_system = int(target_system)
        px4_ns = str(px4_namespace).strip('/')
        prefix = '/' + (px4_ns + '/' if px4_ns else '') + 'fmu/'
        from rclpy.qos import qos_profile_sensor_data
        self.mode_pub = node.create_publisher(
            OffboardControlMode, prefix + 'in/offboard_control_mode', qos_profile_sensor_data)
        self.setpoint_pub = node.create_publisher(
            TrajectorySetpoint, prefix + 'in/trajectory_setpoint', qos_profile_sensor_data)
        self.command_pub = node.create_publisher(
            VehicleCommand, prefix + 'in/vehicle_command', qos_profile_sensor_data)
        self.audit_pub = node.create_publisher(FlightSetpoint, '/flight_setpoint', 10)

    def usec(self):
        return int(self.node.get_clock().now().nanoseconds / 1000)

    def command(self, command, p1=0., p2=0.):
        msg = VehicleCommand()
        msg.timestamp = self.usec()
        msg.command = command
        msg.param1, msg.param2 = float(p1), float(p2)
        msg.target_system = self.target_system
        msg.target_component = msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def mode(self, velocity):
        msg = OffboardControlMode()
        msg.timestamp = self.usec()
        msg.position = not velocity
        msg.velocity = velocity
        msg.acceleration = msg.attitude = msg.body_rate = False
        self.mode_pub.publish(msg)

    def _setpoint(self, mode, position, velocity, yaw, yaw_rate,
                  trajectory_id=0, confidence=0.):
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
        """Translate the typed, supervisor-approved setpoint into PX4 uORB."""
        if setpoint.header.frame_id != 'px4_local_ned':
            raise ValueError('flight setpoint must be in PX4 local NED')
        velocity_mode = setpoint.mode == FlightSetpoint.MODE_VELOCITY
        if not velocity_mode and setpoint.mode != FlightSetpoint.MODE_POSITION:
            raise ValueError('unsupported flight setpoint mode')
        position = (setpoint.position.x, setpoint.position.y, setpoint.position.z)
        velocity = (setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z)
        selected = velocity if velocity_mode else position
        if not all(math.isfinite(v) for v in selected):
            raise ValueError('nonfinite active flight setpoint')
        self.mode(velocity_mode)
        msg = TrajectorySetpoint()
        msg.timestamp = self.usec()
        msg.position = [math.nan] * 3 if velocity_mode else list(position)
        msg.velocity = list(velocity) if velocity_mode else [math.nan] * 3
        msg.acceleration = [math.nan] * 3
        msg.yaw = setpoint.yaw
        msg.yawspeed = setpoint.yaw_rate
        self.setpoint_pub.publish(msg)
        self.audit_pub.publish(setpoint)

    def position(self, target, heading):
        self.apply(self._setpoint(
            FlightSetpoint.MODE_POSITION, target,
            (math.nan, math.nan, math.nan),
            heading if math.isfinite(heading) else math.nan, math.nan))

    def velocity_body(self, body_velocity, yaw_rate_flu, heading_ned,
                      trajectory_id, confidence):
        north, east = body_flu_to_ned(body_velocity[0], body_velocity[1],
                                      heading_ned)
        self.velocity_ned((north, east, -body_velocity[2]),
                          -yaw_rate_flu, trajectory_id, confidence)

    def velocity_ned(self, velocity, yaw_rate_ned=math.nan,
                     trajectory_id=0, confidence=0.):
        self.apply(self._setpoint(
            FlightSetpoint.MODE_VELOCITY,
            (math.nan, math.nan, math.nan), velocity,
            math.nan, yaw_rate_ned, trajectory_id, confidence))
