"""Exercise built planner -> trajectory output without depth, health or flight status.
Run with an isolated ROS_DOMAIN_ID; synthetic odometry is confined to that domain.
No PX4, serial devices or flight-control nodes are started.
"""
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Header
from traj_utils.msg import Bspline
from visualization_msgs.msg import Marker

assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated ROS domain'
root = Path(__file__).resolve().parents[1]
rclpy.init()
n = rclpy.create_node('no_depth_planning_test')
processes, logs = [], []
def start(package, executable, args):
    log = open('/tmp/no_depth_' + executable + '.log', 'w')
    logs.append(log)
    processes.append(subprocess.Popen(
        ['ros2', 'run', package, executable, '--ros-args'] + args,
        stdout=log, stderr=log, start_new_session=True))

odom_pub = n.create_publisher(Odometry, '/ego/odom', 10)
goal_pub = n.create_publisher(PoseStamped, '/navigation_goal', 10)
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
reset_pub = n.create_publisher(Header, '/vio_reset_event', latched)
splines, commands, markers, ready = [], [], [], []
subs = [n.create_subscription(Bspline, '/planning/bspline', splines.append, 10),
        n.create_subscription(PositionCommand, '/position_cmd', commands.append, 10),
        n.create_subscription(Marker, '/optimal_list', markers.append, 10),
        n.create_subscription(Header, '/ego/map_reset_ready', ready.append, latched)]

def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = n.get_clock().now().to_msg()
        odom.pose.pose.position.z = 1.
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        rclpy.spin_once(n, timeout_sec=.01)
        time.sleep(.01)
    assert all(p.poll() is None for p in processes), 'A test node crashed; inspect /tmp/no_depth_*.log'

def goal(x):
    msg = PoseStamped()
    msg.header.frame_id = 'odom'
    msg.header.stamp = n.get_clock().now().to_msg()
    msg.pose.orientation.w = 1.
    msg.pose.position.x = x
    msg.pose.position.z = 1.
    goal_pub.publish(msg)

try:
    config = str(root / 'src/bringup/params/ego_params.yaml')
    start('ego_planner', 'ego_planner_node', ['--params-file', config,
          '-r', 'odom_world:=/ego/odom', '-r', 'grid_map/odom:=/ego/odom'])
    start('ego_planner', 'traj_server', ['--params-file', config])
    start('goal_manager', 'goal_manager', ['-p', 'goal_altitude:=1.0'])
    spin(2.)
    goal(2.)
    spin(3.)
    assert splines, 'No B-spline without depth'
    assert markers, 'No RViz optimal-path marker without depth'
    assert commands, 'No sampled trajectory without depth'
    # Depth stays absent for longer than the old 1.5 s timeout.
    count = len(commands)
    spin(2.)
    assert len(commands) > count, 'Trajectory output incorrectly stopped for missing depth'
    event = Header(stamp=n.get_clock().now().to_msg(), frame_id='no-depth-reset')
    reset_pub.publish(event)
    spin(.6)
    assert any(m.frame_id == event.frame_id for m in ready), 'Reset waits for nonexistent depth'
    splines.clear(); commands.clear(); markers.clear()
    spin(.5)
    assert not splines and not commands, 'Old goal/trajectory survived localization reset'
    goal(1.5)
    spin(2.)
    assert splines and commands and markers, 'New target cannot plan after reset without depth'
    print('PASS: no-camera ground goal -> RViz path, B-spline and sampled trajectory; reset clears old task and accepts new target without depth')
finally:
    for p in processes:
        if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
    for p in processes:
        try: p.wait(timeout=5)
        except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGTERM)
    for log in logs: log.close()
    n.destroy_node(); rclpy.shutdown()
