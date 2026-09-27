"""Keep only depth returns intersecting the fixed cruise-height collision band."""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool


class HeightSlice(Node):
    def __init__(self):
        super().__init__('height_slice')
        for key, value in dict(half_height=.30, max_age=.45, voxel=.08).items():
            self.declare_parameter(key, value)
        self.half_height = float(self.get_parameter('half_height').value)
        self.max_age = float(self.get_parameter('max_age').value)
        self.voxel = float(self.get_parameter('voxel').value)
        self.last_cloud = -math.inf
        self.pub = self.create_publisher(PointCloud2, 'navigation_obstacles', qos_profile_sensor_data)
        self.fresh_pub = self.create_publisher(Bool, 'obstacle_fresh', 1)
        self.create_subscription(PointCloud2, '/uav1/obstacles', self.callback, qos_profile_sensor_data)
        self.create_timer(.1, self.watchdog)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.header.frame_id != 'base_link' or not 0 <= self.now() - stamp <= self.max_age:
            return
        points = np.asarray(point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True), dtype=np.float32).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[np.abs(points[:, 2]) <= self.half_height]
        if self.voxel > 0 and len(points):
            _, indices = np.unique(np.floor(points / self.voxel).astype(np.int32),
                                   axis=0, return_index=True)
            points = points[np.sort(indices)]
        self.pub.publish(point_cloud2.create_cloud_xyz32(msg.header, points.tolist()))
        self.last_cloud = stamp

    def watchdog(self):
        self.fresh_pub.publish(Bool(data=0 <= self.now() - self.last_cloud <= self.max_age))


def main(args=None):
    rclpy.init(args=args)
    node = HeightSlice()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
