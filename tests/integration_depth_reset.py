"""Run against built EGO binaries in an isolated ROS_DOMAIN_ID (no PX4).
Usage: ROS_DOMAIN_ID=83 PYTHONNOUSERSITE=1 python3 tests/integration_depth_reset.py
"""
from pathlib import Path
import os
import signal
import subprocess
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import Header
from quadrotor_msgs.msg import PositionCommand
from traj_utils.msg import Bspline
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

ROOT = Path(__file__).resolve().parents[1]
assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated ROS_DOMAIN_ID'
processes = []
logs = []
rclpy.init()
n = rclpy.create_node('depth_reset_regression')
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
reset_pub = n.create_publisher(Header, '/vio_reset_event', latched)
def run(package, executable, args):
    log = open('/tmp/one_drone_test_' + executable + '.log', 'w')
    logs.append(log)
    p = subprocess.Popen([str(ROOT / 'install' / package / 'lib' / package / executable),
                          '--ros-args'] + args, stdout=log, stderr=log)
    processes.append(p)
    return p

def spin(seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(n, timeout_sec=.01)
    assert all(p.poll() is None for p in processes), 'ROS process exited; inspect /tmp/one_drone_test_*.log'

try:
    run('ego_planner', 'ego_planner_node', [
        '--params-file', str(ROOT / 'src/bringup/params/ego_params.yaml'),
        '-p', 'grid_map/cam2body:=[0.0,0.0,1.0,0.0,-1.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,1.0]',
        '-p', 'grid_map/require_camera_info:=false', '-p', 'grid_map/fx:=20.0',
        '-p', 'grid_map/fy:=20.0', '-p', 'grid_map/cx:=8.0', '-p', 'grid_map/cy:=8.0'])
    image_pub = n.create_publisher(Image, '/grid_map/depth', 10)
    odom_pub = n.create_publisher(Odometry, '/grid_map/odom', 10)
    cloud = [None]
    ready = []
    subscriptions = [
        n.create_subscription(PointCloud2, '/grid_map/occupancy', lambda m: cloud.__setitem__(0, m), 10),
        n.create_subscription(Header, '/ego/map_reset_ready', lambda m: ready.append(m.frame_id), latched)]
    spin(1.5)
    bridge = CvBridge()
    def frames(value, count, age=0., sync_error=0.):
        for _ in range(count):
            t = n.get_clock().now().nanoseconds - int(age * 1e9)
            msg = bridge.cv2_to_imgmsg(np.full((16, 16), value, np.float32), encoding='32FC1')
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(t, 10**9)
            odom = Odometry()
            ot = t - int(sync_error * 1e9)
            odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(ot, 10**9)
            odom.pose.pose.orientation.w = 1.
            odom.pose.pose.position.z = 1.
            odom_pub.publish(odom)
            image_pub.publish(msg)
            spin(.085)
        spin(.2)
    frames(2., 12)
    assert cloud[0] is not None and cloud[0].width > 0, 'valid depth did not build occupancy'
    occupied = cloud[0].width
    for bad in (0., float('nan'), float('inf'), 8.):
        frames(bad, 10)
        assert cloud[0].width == occupied, f'invalid depth {bad} cleared occupancy'
    print('PASS: zero/NaN/Inf/out-of-range frames preserve occupied voxels', flush=True)
    def near_hits():
        return sum(1 for p in read_points(cloud[0], field_names=('x', 'y', 'z')) if 1.7 < p[0] < 2.3)
    near_before = near_hits()
    frames(4., 15)
    assert near_before > 0 and near_hits() < near_before, 'fresh valid rays did not clear old obstacle'
    print('PASS: valid rays can clear obsolete occupancy', flush=True)
    reset = Header(stamp=n.get_clock().now().to_msg(), frame_id='integration-reset')
    reset_pub.publish(reset)
    spin(.3)
    assert cloud[0].width == 0, 'reset did not clear occupancy'
    frames(0., 5)
    frames(2., 5, age=2.)
    frames(2., 5, sync_error=.2)
    assert not ready, 'invalid/stale/unsynchronized frames incorrectly acknowledged map readiness'
    frames(2., 12)
    assert ready[-1] == reset.frame_id and cloud[0].width > 0
    print('PASS: reset clears map; only fresh synchronized valid frames rebuild it', flush=True)

    run('ego_planner', 'traj_server', ['-r', 'position_cmd:=/test_position_cmd'])
    bspline_pub = n.create_publisher(Bspline, '/planning/bspline', 10)
    commands = []
    subscriptions.append(n.create_subscription(PositionCommand, '/test_position_cmd',
                                              lambda m: commands.append(m), 10))
    spin(1.5)
    trajectory = Bspline()
    trajectory.order = 3
    trajectory.traj_id = 1
    trajectory.start_time = n.get_clock().now().to_msg()
    trajectory.pos_pts = [Point(x=float(i), y=0., z=1.) for i in range(6)]
    trajectory.knots = [float(i) * .5 for i in range(10)]
    bspline_pub.publish(trajectory)
    spin(.3)
    assert commands, 'trajectory did not produce commands'
    reset_pub.publish(Header(stamp=n.get_clock().now().to_msg(), frame_id='trajectory-reset'))
    spin(.2)
    after_reset = len(commands)
    bspline_pub.publish(trajectory)  # A queued old spline must not reactivate.
    spin(.3)
    assert len(commands) == after_reset, 'old trajectory still produces commands after reset'
    trajectory.start_time = n.get_clock().now().to_msg()
    trajectory.traj_id = 2
    bspline_pub.publish(trajectory)
    spin(.3)
    assert len(commands) > after_reset
    print('PASS: reset stops old PositionCommand and rejects replayed spline; new spline works', flush=True)
finally:
    for p in processes:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    for p in processes:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
    for log in logs:
        log.close()
    n.destroy_node()
    rclpy.shutdown()
