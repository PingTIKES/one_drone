"""Stereo-only depth relays do not require a camera IMU; OpenVINS still does."""
import pytest
import rclpy
from camera_stream.sensor_relay import SensorRelay


@pytest.mark.parametrize('imu_topic', ['', '/imu'])
def test_stereo_relay_with_optional_imu(imu_topic):
    args = ['--ros-args', '-p', 'cam0:=/left', '-p', 'cam1:=/right']
    if imu_topic:
        args += ['-p', 'imu:='+imu_topic]
    rclpy.init(args=args)
    node = None
    try:
        node = SensorRelay()
        expected = {'/left', '/right'} | ({imu_topic} if imu_topic else set())
        assert {sub.topic_name for sub in node.subscriptions} == expected
        assert len(node.pubs) == len(expected)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
