"""Accept RViz map goals and publish odom-frame targets to EGO-Planner."""
import math
from copy import deepcopy
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        self.declare_parameter('odom_timeout', .5)
        self.declare_parameter('planner_heartbeat_timeout', .75)
        self.odom_stamp = self.planner_at = -math.inf
        self.declare_parameter('goal_frame', 'odom')
        self.declare_parameter('goal_altitude', 2.0)
        self.goal_frame = str(self.get_parameter('goal_frame').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending_goal = None
        self.goal_unsent = False
        self.decision_request_stamp = None
        self.decision_accepted_goal = None
        self.decision_cutoff = -1
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, 'navigation_state', latched)
        self.goal_pub = self.create_publisher(PoseStamped, '/ego/goal', 10)
        self.decision_ack_pub = self.create_publisher(PoseStamped, '/decision/accepted_goal', 10)
        self.create_subscription(PoseStamped, '/navigation_goal', self.on_goal, 10)
        self.create_subscription(PoseStamped, '/decision/goal', self.on_decision_goal, 10)
        self.create_subscription(Header, '/decision/clear_pending', self.on_clear_decision, 10)
        self.create_subscription(Odometry, '/ego/odom', self.on_odom, 10)
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
        self.decision_request_stamp = None
        self.odom_stamp = -math.inf
        self.state('VIO_RESET_NEW_GOAL_REQUIRED')

    def on_clear_decision(self, msg):
        # Only clear the decision queue. A new manual goal must remain intact.
        cutoff = msg.stamp.sec * 1000000000 + msg.stamp.nanosec
        self.decision_cutoff = max(self.decision_cutoff, cutoff)
        if (self.decision_request_stamp is not None and
                self.decision_request_stamp.sec * 1000000000 +
                self.decision_request_stamp.nanosec <= cutoff):
            self.pending_goal = None
            self.goal_unsent = False
            self.decision_request_stamp = None

    def on_decision_goal(self, msg):
        if msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec <= self.decision_cutoff:
            return
        if self.decision_request_stamp == msg.header.stamp:
            if self.goal_unsent:
                self.publish_pending_goal('GOAL_SENT_TO_EGO')
            elif self.decision_accepted_goal is not None:
                self.decision_ack_pub.publish(self.decision_accepted_goal)
            return
        self.on_goal(msg, decision=True)

    def on_goal(self, msg, decision=False):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= self.reset_stamp:
            self.state('REJECTED_PRE_RESET_GOAL')
            return
        p = msg.pose.position
        if msg.header.frame_id not in (self.goal_frame, 'map') or not all(
                math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.state('REJECTED_FRAME_OR_POSITION')
            return
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = msg.header.frame_id
        goal.pose = deepcopy(msg.pose)
        if not decision:
            self.decision_cutoff = max(self.decision_cutoff, self.get_clock().now().nanoseconds)
            goal.pose.position.z = float(self.get_parameter('goal_altitude').value)
        self.decision_request_stamp = msg.header.stamp if decision else None
        self.decision_accepted_goal = None
        self.pending_goal = goal
        self.goal_unsent = True
        self.publish_pending_goal('GOAL_SENT_TO_EGO')

    def publish_pending_goal(self, success_state):
        # Planning needs a current start pose and a live planner, not depth or flight mode.
        # The control bridge independently decides whether to execute a trajectory.
        if self.pending_goal is None or not self.goal_unsent:
            return False
        if time.monotonic() - self.planner_at > float(self.get_parameter('planner_heartbeat_timeout').value):
            self.state('GOAL_QUEUED_PLANNER_NOT_READY')
            return False
        if not -0.02 <= self.now() - self.odom_stamp <= float(self.get_parameter('odom_timeout').value):
            self.state('GOAL_QUEUED_ODOM_NOT_READY')
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
        if self.decision_request_stamp is not None:
            ack = PoseStamped()
            ack.header.frame_id = goal.header.frame_id
            # Echo the request stamp as its identifier; the pose is the actual
            # transformed target sent to EGO, not the untransformed map point.
            ack.header.stamp = self.decision_request_stamp
            ack.pose = goal.pose
            self.decision_accepted_goal = ack
            self.decision_ack_pub.publish(ack)
        self.goal_unsent = False
        self.state(success_state)
        return True

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        if (msg.header.frame_id != self.goal_frame or stamp <= self.reset_stamp or
                not all(math.isfinite(x) for x in (p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z)) or
                q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z < 1e-6):
            return
        self.odom_stamp = stamp
        self.publish_pending_goal('GOAL_SENT_AFTER_ODOM_RECOVERY')

    def on_planner(self, msg):
        if self.planner_instance is not None and self.planner_instance != msg.frame_id:
            self.goal_unsent = self.pending_goal is not None
        self.planner_instance = msg.frame_id
        self.planner_at = time.monotonic()
        self.publish_pending_goal('GOAL_SENT_AFTER_PLANNER_RECOVERY')


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
