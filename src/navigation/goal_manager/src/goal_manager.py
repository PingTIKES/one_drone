"""Accept RViz map goals and publish odom-frame targets to EGO-Planner."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from std_msgs.msg import Bool, String, Header
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        self.declare_parameter('depth_grace', .8)
        self.declare_parameter('map_timeout', .75)
        self.declare_parameter('planner_heartbeat_timeout', .75)
        self.map_stamp = self.planner_at = -math.inf
        self.declare_parameter('goal_frame', 'odom')
        self.declare_parameter('goal_altitude', 2.0)
        self.goal_frame = str(self.get_parameter('goal_frame').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.vio_valid = False
        self.last_depth_good = -math.inf
        self.flight_state = 'IDLE'
        self.pending_goal = None
        self.goal_unsent = False
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
        self.planner_instance = None
        self.create_subscription(Header, '/ego/map_heartbeat', self.on_planner, 1)
        self.state('WAITING_FOR_GOAL')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def state(self, text):
        self.state_pub.publish(String(data=text))

    def on_reset(self, msg):
        self.reset_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.pending_goal = None
        self.goal_unsent = False
        self.last_depth_good = -math.inf
        self.state('VIO_RESET_NEW_GOAL_REQUIRED')

    def on_goal(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= self.reset_stamp:
            self.state('REJECTED_PRE_RESET_GOAL')
            return
        p = msg.pose.position
        if msg.header.frame_id not in (self.goal_frame, 'map') or not all(
                math.isfinite(v) for v in (p.x, p.y)):
            self.state('REJECTED_FRAME_OR_POSITION')
            return
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = msg.header.frame_id
        goal.pose = msg.pose
        goal.pose.position.z = float(self.get_parameter('goal_altitude').value)
        self.pending_goal = goal
        self.goal_unsent = True
        self.publish_pending_goal('GOAL_SENT_TO_EGO')

    def publish_pending_goal(self, success_state):
        if self.pending_goal is None or not self.goal_unsent:
            return False
        if not (self.map_stamp > 0 and -0.02 <= self.now() - self.map_stamp <= float(self.get_parameter('map_timeout').value)
                and time.monotonic() - self.planner_at <= float(self.get_parameter('planner_heartbeat_timeout').value)):
            self.state('GOAL_QUEUED_MAP_NOT_READY')
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
        goal = PoseStamped()
        goal.header.frame_id = self.goal_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        if self.pending_goal.header.frame_id == self.goal_frame:
            goal.pose = self.pending_goal.pose
        else:
            try:
                # Use the current manual map→odom alignment when dispatching.
                tf = self.tf_buffer.lookup_transform(
                    self.goal_frame, self.pending_goal.header.frame_id, Time())
                goal.pose = do_transform_pose(self.pending_goal.pose, tf)
            except TransformException:
                self.state('GOAL_QUEUED_TF_NOT_READY')
                return False
        self.goal_pub.publish(goal)
        self.goal_unsent = False
        self.state(success_state)
        return True

    def on_depth(self, msg):
        if msg.data:
            self.last_depth_good = self.now()
            self.publish_pending_goal('GOAL_SENT_AFTER_DEPTH_RECOVERY')

    def on_vio(self, msg):
        self.vio_valid = msg.data == 'VALID'
        if self.vio_valid:
            self.publish_pending_goal('GOAL_SENT_AFTER_VIO_RECOVERY')
        if not self.vio_valid:
            self.state('VIO_NOT_READY')

    def on_planner(self, msg):
        if self.planner_instance is not None and self.planner_instance != msg.frame_id:
            self.goal_unsent = self.pending_goal is not None
        self.planner_instance = msg.frame_id
        self.map_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.planner_at = time.monotonic()
        self.publish_pending_goal('GOAL_SENT_AFTER_MAP_RECOVERY')

    def on_flight(self, msg):
        previous = self.flight_state
        self.flight_state = msg.data
        if self.flight_state == 'CRUISE' and previous != 'CRUISE':
            self.goal_unsent = self.pending_goal is not None
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
