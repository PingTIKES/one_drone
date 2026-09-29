"""Adapt body-frame OpenVINS twist to EGO-Planner world-frame odometry."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class EgoOdomAdapter(Node):
    def __init__(self):
        super().__init__('ego_odom_adapter')
        defaults = dict(source_odom='/odom', output_odom='/ego/odom',
                        depth_topic='/uav1/d435i/depth/image_raw',
                        depth_fresh_topic='/ego/depth_fresh', depth_timeout=.5,
                        expected_frame='odom')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.expected_frame = str(p('expected_frame'))
        self.depth_timeout = float(p('depth_timeout'))
        self.last_depth = -math.inf
        self.odom_pub = self.create_publisher(Odometry, str(p('output_odom')), 20)
        self.depth_pub = self.create_publisher(Bool, str(p('depth_fresh_topic')), 10)
        self.create_subscription(Odometry, str(p('source_odom')), self.on_odom, 20)
        self.create_subscription(Image, str(p('depth_topic')), self.on_depth,
                                 qos_profile_sensor_data)
        self.create_timer(.1, self.publish_depth_health)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_depth(self, _msg):
        self.last_depth = self.now()

    def publish_depth_health(self):
        age = self.now() - self.last_depth
        self.depth_pub.publish(Bool(data=0.0 <= age <= self.depth_timeout))

    def on_odom(self, msg):
        if msg.header.frame_id != self.expected_frame:
            self.get_logger().error(
                f'expected odometry frame {self.expected_frame}, got {msg.header.frame_id}',
                throttle_duration_sec=2.0)
            return
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
        if norm < 1e-6:
            return
        w, x, y, z = q.w/norm, q.x/norm, q.y/norm, q.z/norm
        r00, r01, r02 = 1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)
        r10, r11, r12 = 2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)
        r20, r21, r22 = 2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)
        # Snapshot before assigning the output fields. ROS Python message
        # assignment can retain the same nested object; mutating out.x first
        # must not change the input used to calculate out.y and out.z.
        v = msg.twist.twist.linear
        vx, vy, vz = v.x, v.y, v.z
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        out.twist.twist.linear.x = r00*vx + r01*vy + r02*vz
        out.twist.twist.linear.y = r10*vx + r11*vy + r12*vz
        out.twist.twist.linear.z = r20*vx + r21*vy + r22*vz
        self.odom_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EgoOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
