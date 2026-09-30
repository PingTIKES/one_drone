"""Filter metric depth without changing timestamps or camera geometry."""
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from depth_filter.depth_filters import DepthFilters


class DepthFilterNode(Node):
    def __init__(self):
        super().__init__('depth_filter')
        defaults = dict(depth_scale=.001, min_range=.3, max_range=8.,
                        spatial_kernel=3, temporal_alpha=.65,
                        temporal_max_delta=.4, hole_min_neighbors=5)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.depth_scale = float(p('depth_scale'))
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

    def callback(self, msg):
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
