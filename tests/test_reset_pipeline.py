"""ROS callback regression tests; no PX4 or flight commands are published."""
import importlib.util
import math
from pathlib import Path
import time
import unittest
from unittest.mock import Mock

import numpy as np
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped

ROOT = Path(__file__).resolve().parents[1]
def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m

Adapter = module('adapter_test', 'src/navigation/ego_bridge/src/ego_odom_adapter.py').EgoOdomAdapter
Goal = module('goal_test', 'src/navigation/goal_manager/src/goal_manager.py').GoalManager
from depth_filter.depth_filter_node import DepthFilterNode
from flight_bridge.flight_bridge import FlightBridge
from vio_bridge.vio_to_px4 import VioBridge
from nav_msgs.msg import Odometry

def stamp(seconds):
    from builtin_interfaces.msg import Time
    ns = round(seconds * 1e9)
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)

class ResetPipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_depth_age_uses_source_and_rejects_replay(self):
        n = Adapter()
        try:
            n.now = lambda: 100.
            image = Image()
            image.header.stamp = stamp(99.)
            n.on_depth(image)
            self.assertEqual(n.last_depth, -math.inf)
            image.header.stamp = stamp(99.9)
            n.on_depth(image)
            self.assertAlmostEqual(n.last_depth, 99.9)
            n.now = lambda: 100.1
            n.on_depth(image)  # Duplicate does not refresh freshness.
            self.assertAlmostEqual(n.last_depth, 99.9)
            image.header.stamp = stamp(99.8)
            n.on_depth(image)
            self.assertAlmostEqual(n.last_depth, 99.9)
            image.header.stamp = stamp(101.)
            n.on_depth(image)
            self.assertAlmostEqual(n.last_depth, 99.9)
        finally:
            n.destroy_node()

    def test_filter_preserves_holes_and_reset_drops_history(self):
        n = DepthFilterNode()
        try:
            n.publisher = Mock()
            depth = np.ones((5, 5), np.float32)
            depth[2, 2] = np.nan
            msg = n.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
            msg.header.stamp = stamp(10.)
            n.callback(msg)
            result = n.bridge.imgmsg_to_cv2(n.publisher.publish.call_args.args[0])
            self.assertTrue(np.isnan(result[2, 2]))
            n.on_reset(Header(stamp=stamp(11.), frame_id='reset1'))
            self.assertIsNone(n.filters.previous)
            n.publisher.reset_mock()
            n.callback(msg)
            n.publisher.publish.assert_not_called()
        finally:
            n.destroy_node()

    def test_vio_jump_publishes_reset_at_detection_and_acceptance(self):
        n = VioBridge()
        try:
            n.reset_pub = Mock()
            msg = Odometry()
            msg.header.frame_id = 'global'
            msg.child_frame_id = 'imu'
            msg.pose.pose.orientation.w = 1.
            for i in range(6):
                msg.pose.covariance[i * 7] = .001
                msg.twist.covariance[i * 7] = .001
            for i in range(65):
                t = 100. + i * .01
                n.now = lambda t=t: t
                n.image_at = [t, t]
                msg.header.stamp = stamp(t)
                msg.pose.pose.position.x = 0. if i == 0 else .6
                n.callback(msg)
            self.assertEqual(n.reset_count, 1)
            epochs = [c.args[0].frame_id for c in n.reset_pub.publish.call_args_list]
            self.assertEqual(len(epochs), 2)
            self.assertIn('POSITION_DISCONTINUITY', epochs[0])
            self.assertIn('RECOVERED_WITH_RESET', epochs[1])
            self.assertNotEqual(epochs[0], epochs[1])
        finally:
            n.destroy_node()

    def test_old_goal_discarded(self):
        n = Goal()
        try:
            n.pending_goal = PoseStamped()
            n.on_reset(Header(stamp=stamp(10.), frame_id='reset1'))
            self.assertIsNone(n.pending_goal)
            old = PoseStamped()
            old.header.frame_id = 'odom'
            old.header.stamp = stamp(9.)
            n.on_goal(old)
            self.assertIsNone(n.pending_goal)
            old.header.stamp = stamp(11.)
            n.on_goal(old)
            self.assertIsNotNone(n.pending_goal)
        finally:
            n.destroy_node()

    def test_flight_reset_requires_matching_map_epoch(self):
        n = FlightBridge()
        try:
            n.state = 'CRUISE'
            n.enter_hold = Mock()
            n.cmd = object()
            n.vio_ok = True
            n.on_vio_reset(Header(stamp=stamp(time.time()), frame_id='reset2'))
            n.on_vio(String(data='VALID'))
            self.assertIsNotNone(n.vio_valid_since)
            n.enter_hold.assert_called_once()
            self.assertIsNone(n.cmd)
            self.assertFalse(n.reset_map_ready())
            n.on_map_ready(Header(frame_id='reset1'))
            self.assertFalse(n.reset_map_ready())
            n.on_map_ready(Header(frame_id='reset2'))
            self.assertFalse(n.reset_map_ready())
            n.on_traj_ready(Header(frame_id='reset2'))
            self.assertTrue(n.reset_map_ready())
        finally:
            n.destroy_node()

if __name__ == '__main__':
    unittest.main()
