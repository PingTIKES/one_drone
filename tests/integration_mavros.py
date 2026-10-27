"""Real MAVROS + loopback-only MAVLink FCU. Never connects to hardware.

Run with a dedicated ROS_DOMAIN_ID (195) and installed GeographicLib data.
"""
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pymavlink.dialects.v20 import common as ml
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import Mavlink

from flight_bridge.flight_bridge import FlightBridge
from microvio.microvio_node import Microvio
from vio_bridge.mavlink_codec import encode_frame

ROOT = Path(__file__).resolve().parents[1]
import importlib.util
spec = importlib.util.spec_from_file_location('ego_adapter_test', ROOT/'src/navigation/ego_bridge/src/ego_odom_adapter.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
EgoOdomAdapter = module.EgoOdomAdapter


class FakeFCU:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.setblocking(False)
        self.port = self.sock.getsockname()[1]
        reservation = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        reservation.bind(('127.0.0.1', 0))
        self.mavros_port = reservation.getsockname()[1]
        reservation.close()
        owner = self
        class Writer:
            def write(self, data):
                owner.sock.sendto(data, ('127.0.0.1', owner.mavros_port))
        self.mav = ml.MAVLink(Writer(), srcSystem=1, srcComponent=1)
        self.decoder = ml.MAVLink(None)
        self.start = self.last = time.monotonic()
        self.next_hb = self.next_data = self.last
        self.position = [0., 0., 0.]
        self.velocity = [0., 0., 0.]
        self.mode = 3  # PX4 main mode POSCTL
        self.armed = False
        self.target = None
        self.sp_first = None
        self.sp_last = 0.
        self.control_commands = []
        self.ev_packets = []
        self.reset_counter = 0
        self.wire_velocity = None
        self.cov = [0.] * 21
        for index in (0, 6, 11, 15, 18, 20):
            self.cov[index] = .01

    def ns(self):
        return int((time.monotonic() - self.start + 100.) * 1e9)

    def tick(self):
        now = time.monotonic()
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            for msg in self.decoder.parse_buffer(data) or []:
                if msg.get_type() == 'TIMESYNC' and msg.tc1 == 0:
                    self.mav.timesync_send(self.ns(), msg.ts1)
                elif msg.get_type() == 'SET_POSITION_TARGET_LOCAL_NED':
                    assert msg.coordinate_frame == ml.MAV_FRAME_LOCAL_NED
                    self.sp_first = self.sp_first or now
                    self.sp_last = now
                    self.target = msg
                    if msg.type_mask & 1:
                        self.wire_velocity = (msg.vx, msg.vy, msg.vz)
                elif msg.get_type() == 'SET_MODE':
                    self.control_commands.append(msg)
                    requested = (msg.custom_mode >> 16) & 0xff
                    # Model the protocol precondition instead of accepting an
                    # OFFBOARD command without a live >1 second stream.
                    if requested != 6 or (self.sp_first and now-self.sp_first > 1. and now-self.sp_last < .5):
                        self.mode = requested
                elif msg.get_type() == 'COMMAND_LONG':
                    if msg.command == ml.MAV_CMD_COMPONENT_ARM_DISARM:
                        self.control_commands.append(msg)
                        self.armed = bool(msg.param1)
                    self.mav.command_ack_send(msg.command, ml.MAV_RESULT_ACCEPTED,
                                              0, 0, msg.get_srcSystem(), msg.get_srcComponent())
                elif msg.get_type() == 'ODOMETRY':
                    self.ev_packets.append(msg)
        dt = min(.05, now-self.last)
        self.last = now
        if self.mode == 6 and self.armed and self.target:
            if self.target.type_mask & 1:
                self.velocity = [self.target.vx, self.target.vy, self.target.vz]
            else:
                dest = [self.target.x, self.target.y, self.target.z]
                self.velocity = [max(-.8, min(.8, 2*(v-p))) for v, p in zip(dest, self.position)]
            self.position = [p+v*dt for p, v in zip(self.position, self.velocity)]
        else:
            self.velocity = [0., 0., 0.]
        if now >= self.next_hb:
            self.next_hb = now + .2
            base = ml.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (ml.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
            self.mav.heartbeat_send(ml.MAV_TYPE_QUADROTOR, ml.MAV_AUTOPILOT_PX4,
                                    base, self.mode << 16, ml.MAV_STATE_ACTIVE)
        if now >= self.next_data:
            self.next_data = now + 1/30
            ns = self.ns()
            self.mav.attitude_quaternion_send(ns//1000000, 1., 0., 0., 0., 0., 0., 0.)
            self.mav.local_position_ned_send(ns//1000000, *self.position, *self.velocity)
            self.mav.estimator_status_send(ns//1000, 47, 0., 0., 0., 0., 0., 0., .01, .01)
            self.mav.odometry_send(ns//1000, 1, 1, *self.position, [1., 0., 0., 0.],
                                  *self.velocity, 0., 0., 0., self.cov, self.cov,
                                  self.reset_counter, 8, 0)


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '195', 'Use isolated ROS_DOMAIN_ID=195'
    fcu = FakeFCU()
    with tempfile.TemporaryDirectory(prefix='one_drone_mavros_test_') as folder:
        log = open(Path(folder)/'mavros.log', 'w+')
        process = subprocess.Popen([
            '/opt/ros/humble/lib/mavros/mavros_node', '--ros-args', '-r', '__ns:=/mavros',
            '--params-file', str(ROOT/'src/bringup/params/mavros.yaml'),
            '-p', f'fcu_url:=udp://127.0.0.1:{fcu.mavros_port}@127.0.0.1:{fcu.port}',
            '-p', 'tgt_system:=1', '-p', 'tgt_component:=1'], stdout=log, stderr=subprocess.STDOUT)
        nodes = []
        rclpy.init(args=['--ros-args', '-p', 'mavros_ns:=/mavros', '-p', 'odom_topic:=/odom', '-p', 'odom_frame:=odom', '-p', 'body_frame:=base_link'])
        executor = SingleThreadedExecutor()
        try:
            bridge, micro, adapter, probe = FlightBridge(), Microvio(), EgoOdomAdapter(), Node('mavros_transport_test')
            nodes = [bridge, micro, adapter, probe]
            for n in nodes:
                executor.add_node(n)
            last = {}
            probe.create_subscription(Odometry, '/ego/odom', lambda m:last.__setitem__('odom',m), 10)
            cmd_pub = probe.create_publisher(PositionCommand, '/ego/position_cmd', 10)
            ev_pub = probe.create_publisher(Mavlink, '/uas1/mavlink_sink', qos_profile_sensor_data)

            def pump(seconds, ego=False):
                end = time.monotonic()+seconds
                while time.monotonic() < end:
                    assert process.poll() is None, 'MAVROS exited'
                    fcu.tick()
                    if ego and 'odom' in last:
                        msg = PositionCommand()
                        msg.header.frame_id = 'odom'
                        msg.header.stamp = probe.get_clock().now().to_msg()
                        msg.trajectory_flag = msg.TRAJECTORY_STATUS_READY
                        msg.position = last['odom'].pose.pose.position
                        msg.velocity.x = .3
                        cmd_pub.publish(msg)
                    for _ in range(4):
                        executor.spin_once(timeout_sec=.001)

            def until(predicate, timeout, label):
                end = time.monotonic()+timeout
                while not predicate() and time.monotonic() < end:
                    pump(.05)
                assert predicate(), f'{label}: bridge={bridge.state}, micro={micro.health()}, received={micro.received_count}, published={micro.published_count}'

            until(lambda:bridge.pose_valid() and micro.published_count > 15 and 'odom' in last, 15., 'telemetry connection')
            assert not fcu.control_commands, 'Standby must not arm or switch mode'
            assert abs(bridge.position.heading) < .01
            client = probe.create_client(Trigger, '/takeoff')
            until(client.service_is_ready, 3., 'takeoff service')
            request = client.call_async(Trigger.Request())
            until(request.done, 3., 'takeoff request')
            assert request.result().success, request.result().message
            until(lambda:bridge.state=='HOLD' and fcu.position[2] < -1.7, 15., 'OFFBOARD, arming and takeoff')
            assert fcu.mode == 6 and fcu.armed
            print('PASS actual MAVROS set_mode/arming, PX4-style prestream, 2m takeoff', flush=True)
            x0 = fcu.position[0]
            pump(1., ego=True)
            assert bridge.state == 'CRUISE'
            assert fcu.wire_velocity is not None and abs(fcu.wire_velocity[0]-.3) < .02
            assert abs(fcu.wire_velocity[1]) < .02 and fcu.position[0] > x0+.1
            pump(.8)
            assert bridge.state == 'HOLD'
            print('PASS EGO -> ENU -> MAVLink NED velocity and stale-command hold', flush=True)
            fcu.mode = 3
            pump(.5)
            count = len(fcu.control_commands)
            pump(.5, ego=True)
            assert bridge.state == 'IDLE' and len(fcu.control_commands) == count
            fcu.mode = 6
            pump(.5)
            assert bridge.state == 'HOLD' and len(fcu.control_commands) == count
            print('PASS RC OFFBOARD exit/reentry without automatic rearming', flush=True)
            packet = ml.MAVLink_odometry_message(10000000, 20, 12, 1., 2., 3., [1.,0.,0.,0.],
                0.,0.,0.,0.,0.,0., fcu.cov, fcu.cov, 7, 3, 100)
            ev_pub.publish(encode_frame(packet, probe.get_clock().now().to_msg()))
            until(lambda:bool(fcu.ev_packets), 3., 'EV packet route')
            assert fcu.ev_packets[-1].reset_counter == 7 and fcu.ev_packets[-1].frame_id == 20
            fcu.reset_counter += 1
            until(lambda:micro.reset_index>0, 3., 'VIO reset propagation')
            print('PASS MAVROS raw odometry, timesync, quality, EV reset metadata and FC reset event', flush=True)
        except Exception:
            log.flush();log.seek(0)
            print(log.read()[-6500:])
            raise
        finally:
            for node in nodes:
                executor.remove_node(node);node.destroy_node()
            executor.shutdown()
            if rclpy.ok():rclpy.shutdown()
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
            fcu.sock.close();log.close()


if __name__ == '__main__':
    main()
