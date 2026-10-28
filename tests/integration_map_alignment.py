"""Check canonical map alignment and remote panel readback in an isolated ROS domain."""
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Pose2D
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy

assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated ROS domain'
root = Path(__file__).resolve().parents[1]
rclpy.init()
node = rclpy.create_node('alignment_test')
changes, transforms = [], []
setpoint = node.create_publisher(Pose2D, '/map_odom/set', 10)
state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscriptions = [node.create_subscription(TFMessage, '/tf', transforms.append, 10)]
process = None


def wait_for(condition, seconds=5.):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.05)
        if process.poll() is not None:
            raise AssertionError('map publisher exited')
        if condition():
            return
    raise AssertionError('timed out waiting for alignment/TF')


def tf_matches(x, y, yaw):
    if not transforms:
        return False
    t = transforms[-1].transforms[0]
    return (t.header.frame_id == 'map' and t.child_frame_id == 'odom' and
            math.isclose(t.transform.translation.x, x) and
            math.isclose(t.transform.translation.y, y) and
            math.isclose(t.transform.rotation.z, math.sin(yaw/2)) and
            math.isclose(t.transform.rotation.w, math.cos(yaw/2)))


try:
    with tempfile.TemporaryFile() as log:
        command = [str(root/'install/modify_map_to_odom/lib/modify_map_to_odom/modify_map_to_odom_node'),
                   '--ros-args', '-p', 'X:=1.25', '-p', 'Y:=-2.5', '-p', 'Rotation:=0.6']
        for restart in range(2):
            changes.clear(); transforms.clear()
            process = subprocess.Popen(command, stdout=log, stderr=log)
            wait_for(lambda: tf_matches(1.25, -2.5, .6))
            # Join after startup: remote RViz receives the actual retained state.
            sub = node.create_subscription(Pose2D, '/map_odom/current', changes.append, state_qos)
            wait_for(lambda: bool(changes))
            assert changes[-1] == Pose2D(x=1.25, y=-2.5, theta=.6)
            setpoint.publish(Pose2D(x=3., y=4., theta=-.2))
            wait_for(lambda: changes[-1].x == 3. and tf_matches(3., 4., -.2))
            setpoint.publish(Pose2D(x=math.nan, y=0., theta=0.))
            until = time.monotonic()+.25
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
            assert changes[-1].x == 3. and tf_matches(3., 4., -.2)
            node.destroy_subscription(sub)
            process.terminate();process.wait(timeout=5)
        print('PASS: config initializes every restart; late readback, ROS updates and invalid-input rejection')
finally:
    if process is not None and process.poll() is None:
        process.terminate();process.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
