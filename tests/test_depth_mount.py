"""Check the real-camera depth mount before it reaches obstacle planning."""
import unittest

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image

from depth_filter.depth_filter_node import DepthFilterNode
from vio_bridge.calibration import hardware_depth_transform


TOPICS = (
    '/camera/camera/infra1/image_rect_raw',
    '/camera/camera/depth/image_rect_raw',
    '/camera/camera/depth/camera_info',
)


class HardwareDepthMountTest(unittest.TestCase):
    def setUp(self):
        body_imu = np.eye(4)
        body_imu[:3, 3] = [.03, 0., -.03]
        imu_cam0 = np.eye(4)
        imu_cam0[:3, 3] = [-.01, .002, .018]
        self.body = {'T_body_imu': body_imu.tolist()}
        self.cameras = {'cam0': {'T_imu_cam': imu_cam0.tolist()}}
        self.expected = body_imu @ imu_cam0

    def test_d435i_depth_uses_measured_body_and_kalibr_cam0(self):
        actual, frame = hardware_depth_transform(
            self.body, self.cameras, *TOPICS)
        np.testing.assert_allclose(actual, self.expected)
        self.assertEqual(frame, 'camera_depth_optical_frame')

    def test_unknown_depth_stream_requires_explicit_mount(self):
        with self.assertRaisesRegex(ValueError, 'provide measured T_body_depth'):
            hardware_depth_transform(
                self.body, self.cameras, TOPICS[0],
                '/camera/camera/aligned_depth_to_color/image_raw', TOPICS[2])

    def test_explicit_mount_overrides_inference(self):
        measured = np.eye(4)
        measured[:3, 3] = [.05, .02, -.03]
        self.body['T_body_depth'] = measured.tolist()
        actual, frame = hardware_depth_transform(
            self.body, self.cameras, 'different_camera', 'different_depth', '')
        np.testing.assert_allclose(actual, measured)
        self.assertEqual(frame, '')

    def test_wrong_runtime_depth_frame_drops_data_and_old_filter_state(self):
        rclpy.init(args=['--ros-args',
                         '-p', 'expected_frame_id:=camera_depth_optical_frame',
                         '-p', 'camera_info_topic:=/test/depth/camera_info'])
        node = DepthFilterNode()
        try:
            info = CameraInfo()
            info.header.frame_id = 'camera_depth_optical_frame'
            info.height, info.width = 480, 640
            node.on_camera_info(info)
            self.assertEqual(node.info_shape, (480, 640))
            node.filters.previous = np.ones((2, 2), np.float32)
            image = Image()
            image.header.frame_id = 'unexpected_optical_frame'
            image.height, image.width = 480, 640
            node.callback(image)
            self.assertIsNone(node.filters.previous)
            info.header.frame_id = 'unexpected_optical_frame'
            node.on_camera_info(info)
            self.assertIsNone(node.info_shape)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
