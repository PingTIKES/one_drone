"""Navigation readiness after the sentry map-to-odom panel confirms alignment."""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, String


class MapOdom(Node):
    def __init__(self):
        super().__init__('map_odom')
        self.transform = (0., 0., 0.)
        self.localized = False
        self.reset_counter = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.current_pub = self.create_publisher(Pose2D, 'map_odom/current', latched)
        self.ready_pub = self.create_publisher(Bool, 'localization_ready', latched)
        self.create_subscription(Pose2D, 'map_odom/applied', self.set_absolute, 10)
        self.create_subscription(String, 'vio_diagnostics', self.on_vio_diagnostics, 10)
        self.publish()

    def publish(self):
        x, y, yaw = self.transform
        self.current_pub.publish(Pose2D(x=x, y=y, theta=yaw))
        self.ready_pub.publish(Bool(data=self.localized))

    def set_absolute(self, msg):
        values = (float(msg.x), float(msg.y), float(msg.theta))
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn('Rejected nonfinite map -> odom')
            return
        self.transform = values
        self.localized = True
        self.publish()
        self.get_logger().info('Map localization manually updated')

    def on_vio_diagnostics(self, msg):
        try:
            counter = int(json.loads(msg.data)['reset_counter'])
        except (ValueError, KeyError, TypeError):
            return
        if self.reset_counter is not None and counter != self.reset_counter and self.localized:
            self.localized = False
            self.ready_pub.publish(Bool(data=False))
            self.get_logger().warn('VIO reset changed its odom origin; reconfirm alignment in MapOdomModify')
        self.reset_counter = counter


def main(args=None):
    rclpy.init(args=args)
    node = MapOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
