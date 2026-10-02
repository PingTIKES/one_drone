"""Validate RViz goals and publish odom-frame targets to EGO-Planner."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, Header


class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        self.declare_parameter('depth_grace', .8)
        self.declare_parameter('goal_frame', 'odom')
        self.declare_parameter('goal_altitude', 2.0)
        self.goal_frame = str(self.get_parameter('goal_frame').value)
        self.vio_valid = False
        self.last_depth_good = -math.inf
        self.flight_state = 'IDLE'
        self.pending_goal = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, 'navigation_state', latched)
        self.goal_pub = self.create_publisher(PoseStamped, '/ego/goal', 10)
        self.create_subscription(PoseStamped, '/navigation_goal', self.on_goal, 10)
        self.create_subscription(Bool, '/ego/depth_fresh', self.on_depth, 10)
        self.create_subscription(String, 'vio_health', self.on_vio, 10)
        self.create_subscription(String, 'flight_state', self.on_flight, 10)
        self.reset_stamp = -math.inf
        self.create_subscription(Header, '/vio_reset_event', self.on_reset, latched)
        self.state('WAITING_FOR_GOAL')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def state(self, text):
        self.state_pub.publish(String(data=text))

    def on_reset(self, msg):
        self.reset_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.pending_goal = None
        self.last_depth_good = -math.inf
        self.state('VIO_RESET_NEW_GOAL_REQUIRED')

    def on_goal(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= self.reset_stamp:
            self.state('REJECTED_PRE_RESET_GOAL')
            return
        p = msg.pose.position
        if msg.header.frame_id != self.goal_frame or not all(math.isfinite(v) for v in (p.x, p.y)):
            self.state('REJECTED_FRAME_OR_POSITION')
            return
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.goal_frame
        goal.pose = msg.pose
        goal.pose.position.z = float(self.get_parameter('goal_altitude').value)
        self.pending_goal = goal
        self.publish_pending_goal('GOAL_SENT_TO_EGO')

    def publish_pending_goal(self, success_state):
        if self.pending_goal is None:
            return False
        if not self.vio_valid:
            self.state('GOAL_QUEUED_VIO_NOT_READY')
            return False
        if self.flight_state != 'CRUISE':
            self.state('GOAL_QUEUED_FLIGHT_NOT_CRUISE')
            return False
        if not 0 <= self.now() - self.last_depth_good <= float(self.get_parameter('depth_grace').value):
            self.state('GOAL_QUEUED_DEPTH_NOT_READY')
            return False
        self.pending_goal.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(self.pending_goal)
        self.state(success_state)
        return True

    def on_depth(self, msg):
        if msg.data:
            self.last_depth_good = self.now()

    def on_vio(self, msg):
        self.vio_valid = msg.data == 'VALID'
        if not self.vio_valid:
            self.state('VIO_NOT_READY')

    def on_flight(self, msg):
        previous = self.flight_state
        self.flight_state = msg.data
        if self.flight_state == 'CRUISE' and previous != 'CRUISE':
            self.publish_pending_goal('GOAL_REPLANNED_AFTER_RECOVERY')


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
