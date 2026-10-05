"""Filter metric depth without changing timestamps or camera geometry."""
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from depth_filter.depth_filters import DepthFilters


class DepthFilterNode(Node):
    def __init__(self):
        super().__init__('depth_filter')
        defaults = dict(depth_scale=.001, min_range=.3, max_range=8.,
                        spatial_kernel=3, temporal_alpha=.65,
                        temporal_max_delta=.4, hole_min_neighbors=5, fill_holes=False)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.declare_parameter('expected_frame_id', '')
        self.declare_parameter('camera_info_topic', '')
        p = lambda key: self.get_parameter(key).value
        self.fill_holes = bool(p('fill_holes'))
        self.reset_stamp = -float('inf')
        self.create_subscription(Header, '/vio_reset_event', self.on_reset,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.depth_scale = float(p('depth_scale'))
        self.expected_frame_id = str(p('expected_frame_id'))
        self.camera_info_topic = str(p('camera_info_topic'))
        self.info_shape = None
        self.last_frame_warning = -float('inf')
        if self.expected_frame_id:
            if not self.camera_info_topic:
                raise ValueError('camera_info_topic required when checking depth frame')
            self.create_subscription(CameraInfo, self.camera_info_topic,
                                     self.on_camera_info, qos_profile_sensor_data)
        self.min_range, self.max_range = float(p('min_range')), float(p('max_range'))
        if self.depth_scale <= 0 or not 0 < self.min_range < self.max_range:
            raise ValueError('invalid depth scale or range')
        self.filters = DepthFilters(int(p('spatial_kernel')),
                                    float(p('temporal_alpha')),
                                    float(p('temporal_max_delta')),
                                    int(p('hole_min_neighbors')))
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, 'depth/image_filtered', qos_profile_sensor_data)
        self.create_subscription(
            Image, 'depth/image_raw', self.callback, qos_profile_sensor_data)

    def on_reset(self, msg):
        self.filters.previous = None
        self.reset_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9

    def on_camera_info(self, msg):
        self.info_shape = ((msg.height, msg.width)
                           if msg.header.frame_id == self.expected_frame_id
                           else None)
        if self.info_shape is None:
            self.filters.previous = None

    def warn_frame(self, frame_id):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_frame_warning >= 5:
            self.get_logger().error(
                f'拒绝深度图：frame_id={frame_id!r}，需要 '
                f'{self.expected_frame_id!r} 且 CameraInfo 坐标系/尺寸匹配')
            self.last_frame_warning = now

    def callback(self, msg):
        if self.expected_frame_id and (
                msg.header.frame_id != self.expected_frame_id or
                self.info_shape != (msg.height, msg.width)):
            self.filters.previous = None
            self.warn_frame(msg.header.frame_id)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= self.reset_stamp:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if msg.encoding in ('16UC1', 'mono16'):
                depth = image.astype(np.float32) * self.depth_scale
                depth[image == 0] = np.nan
            elif msg.encoding == '32FC1':
                depth = image.astype(np.float32, copy=True)
            else:
                raise ValueError(f'unsupported depth encoding {msg.encoding}')
            depth[(depth < self.min_range) | (depth > self.max_range)] = np.nan
            filtered = self.filters.apply(depth)
            if not self.fill_holes:
                filtered[~np.isfinite(depth) | (depth <= 0)] = np.nan
                self.filters.previous = filtered.copy()
        except (ValueError, TypeError) as exc:
            self.get_logger().warn(f'depth frame rejected: {exc}')
            return
        out = self.bridge.cv2_to_imgmsg(filtered, encoding='32FC1')
        out.header = msg.header
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
