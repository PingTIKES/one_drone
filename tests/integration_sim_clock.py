"""Verify trajectory sampling follows /clock, without PX4 or simulator."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from traj_utils.msg import Bspline
from quadrotor_msgs.msg import PositionCommand

assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0')
root = Path(__file__).resolve().parents[1]
rclpy.init()
n = rclpy.create_node('clock_test')
clock_pub = n.create_publisher(Clock, '/clock', 10)
health_pub = n.create_publisher(Header, '/ego/map_heartbeat', 1)
spline_pub = n.create_publisher(Bspline, '/planning/bspline', 10)
commands = []
sub = n.create_subscription(PositionCommand, '/position_cmd', commands.append, 10)
def stamp(t):
    ns = round(t * 1e9)
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)
log = open('/tmp/one_drone_clock_test.log', 'w')
p = subprocess.Popen([str(root/'install/ego_planner/lib/ego_planner/traj_server'),
                      '--ros-args', '-p', 'use_sim_time:=true'], stdout=log, stderr=log)
def spin(seconds, sim_time, heartbeat=True):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        clock_pub.publish(Clock(clock=stamp(sim_time)))
        if heartbeat:
            health_pub.publish(Header(stamp=stamp(sim_time), frame_id='clock-planner'))
        rclpy.spin_once(n, timeout_sec=.01)
        time.sleep(.005)
    assert p.poll() is None
try:
    spin(1.6, 100.)
    trajectory = Bspline(order=3, traj_id=1, start_time=stamp(100.1),
                         pos_pts=[Point(x=float(i), z=1.) for i in range(6)],
                         knots=[i*.5 for i in range(10)])
    spline_pub.publish(trajectory)
    spin(.2, 100.)
    assert not commands, 'future trajectory emitted a command before start'
    spin(.2, 100.3)
    assert commands
    x = commands[-1].position.x
    count = len(commands)
    spin(.5, 100.3)
    assert len(commands) > count
    assert abs(commands[-1].position.x-x) < 1e-9, 'paused /clock advanced trajectory'
    assert abs(commands[-1].header.stamp.sec-100) < 1
    spin(.2, 100.5)
    assert commands[-1].position.x > x, 'advancing /clock did not advance trajectory'
    print('PASS: future start emits nothing; paused /clock freezes sampling; time advance moves trajectory')
    spin(1., 100.5, heartbeat=False)
    count = len(commands)
    spin(.2, 100.5, heartbeat=False)
    assert len(commands) == count, 'watchdog depends on paused ROS clock'
    print('PASS: heartbeat watchdog expires in real time even while ROS clock is paused')
finally:
    p.send_signal(signal.SIGINT)
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        p.kill(); p.wait()
    log.close()
    n.destroy_node(); rclpy.shutdown()
