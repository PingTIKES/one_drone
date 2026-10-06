#!/usr/bin/env python3
"""Provide an isolated navigation TF tree to RViz; leave algorithm TF unchanged."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfDisplayFilter(Node):
    def __init__(self):
        super().__init__('rviz_tf_filter')
        self.declare_parameter('frames', ['map', 'odom', 'base_link'])
        self.frames = set(self.get_parameter('frames').value)
        self.static_transforms = {}
        dynamic_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.dynamic_pub = self.create_publisher(TFMessage, '/rviz/tf', QoSProfile(depth=100))
        self.static_pub = self.create_publisher(TFMessage, '/rviz/tf_static', static_qos)
        self.create_subscription(TFMessage, '/tf', self.dynamic, dynamic_qos)
        self.create_subscription(TFMessage, '/tf_static', self.static, static_qos)

    def selected(self, msg):
        return [t for t in msg.transforms
                if t.header.frame_id.lstrip('/') in self.frames
                and t.child_frame_id.lstrip('/') in self.frames]

    def dynamic(self, msg):
        transforms = self.selected(msg)
        if transforms:
            self.dynamic_pub.publish(TFMessage(transforms=transforms))

    def static(self, msg):
        transforms = self.selected(msg)
        if not transforms:
            return
        for transform in transforms:
            self.static_transforms[transform.child_frame_id] = transform
        # Keep every selected static edge available to late-starting RViz.
        self.static_pub.publish(TFMessage(transforms=list(self.static_transforms.values())))


def main():
    rclpy.init()
    node = TfDisplayFilter()
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
