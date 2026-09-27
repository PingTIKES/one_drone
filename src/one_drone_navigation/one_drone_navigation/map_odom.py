"""Manual map localization; OpenVINS alone owns odom -> base_link."""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def align_map_to_odom(map_base, odom_base):
    """Return T_map_odom = T_map_base * inverse(T_odom_base), planar SE(2)."""
    mx, my, myaw = map_base
    ox, oy, oyaw = odom_base
    theta = math.atan2(math.sin(myaw - oyaw), math.cos(myaw - oyaw))
    c, s = math.cos(theta), math.sin(theta)
    return (mx - c * ox + s * oy, my - s * ox - c * oy, theta)


class MapOdom(Node):
    def __init__(self):
        super().__init__('map_odom')
        self.transform = (0., 0., 0.)
        self.localized = False
        self.reset_counter = None
        self.declare_parameter('pose_max_age', 0.35)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = StaticTransformBroadcaster(self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.current_pub = self.create_publisher(Pose2D, 'map_odom/current', latched)
        self.ready_pub = self.create_publisher(Bool, 'localization_ready', latched)
        self.create_subscription(Pose2D, 'map_odom/set', self.set_absolute, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose, 10)
        self.create_subscription(String, 'vio_diagnostics', self.on_vio_diagnostics, 10)
        self.publish()

    def publish(self):
        x, y, yaw = self.transform
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z = math.sin(yaw / 2.)
        tf.transform.rotation.w = math.cos(yaw / 2.)
        self.tf_pub.sendTransform(tf)
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

    def initialpose(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warn('2D Pose Estimate must use map frame')
            return
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
        except TransformException:
            self.get_logger().warn('Waiting for OpenVINS odom -> base_link before localization')
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if not 0 <= now - stamp <= float(self.get_parameter('pose_max_age').value):
            self.get_logger().warn('Rejected 2D Pose Estimate because OpenVINS pose is stale')
            return
        desired = msg.pose.pose
        pose = tf.transform
        map_base = (desired.position.x, desired.position.y, yaw_of(desired.orientation))
        odom_base = (pose.translation.x, pose.translation.y, yaw_of(pose.rotation))
        if not all(math.isfinite(v) for v in (*map_base, *odom_base)):
            return
        self.transform = align_map_to_odom(map_base, odom_base)
        self.localized = True
        self.publish()
        self.get_logger().info('2D Pose Estimate aligned map -> odom to current VIO base pose')

    def on_vio_diagnostics(self, msg):
        try:
            counter = int(json.loads(msg.data)['reset_counter'])
        except (ValueError, KeyError, TypeError):
            return
        if self.reset_counter is not None and counter != self.reset_counter and self.localized:
            self.localized = False
            self.ready_pub.publish(Bool(data=False))
            self.get_logger().warn('VIO reset changed its odom origin; relocalize with 2D Pose Estimate')
        self.reset_counter = counter


def main(args=None):
    rclpy.init(args=args)
    node = MapOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
