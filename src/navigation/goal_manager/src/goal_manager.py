"""RViz 2D Goal Pose -> Nav2 NavigateToPose with localization/freshness gates."""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String


class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        self.declare_parameter('depth_grace', 0.8)
        self.ready = self.obstacle_fresh = self.vio_valid = False
        self.last_obstacle_good = -math.inf
        self.flight_state = 'IDLE'
        self.pending = self.goal_handle = None
        self.sending = False
        self.request_epoch = 0
        self.alignment = None
        self.action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, 'navigation_state', latched)
        self.create_subscription(PoseStamped, '/navigation_goal', self.on_goal, 10)
        self.create_subscription(Bool, 'localization_ready', self.on_ready, latched)
        self.create_subscription(Pose2D, 'map_odom/current', self.on_alignment, latched)
        self.create_subscription(Bool, 'obstacle_fresh', self.on_obstacle, 10)
        self.create_subscription(String, 'vio_health', self.on_vio, 10)
        self.create_subscription(String, 'flight_state', self.on_flight, 10)
        self.create_timer(.1, self.tick)
        self.state('WAITING_FOR_GOAL')

    def state(self, text):
        self.pub.publish(String(data=text))

    def cancel(self):
        self.request_epoch += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def on_goal(self, msg):
        p = msg.pose.position
        if msg.header.frame_id != 'map' or not all(math.isfinite(v) for v in (p.x, p.y)):
            self.state('REJECTED_FRAME_OR_POSITION')
            return
        if not self.ready:
            self.state('RELOCALIZATION_REQUIRED')
            return
        if not self.vio_valid:
            self.state('VIO_NOT_READY')
            return
        if self.flight_state != 'CRUISE':
            self.state('FLIGHT_NOT_CRUISE')
            return
        depth_age = self.get_clock().now().nanoseconds * 1e-9 - self.last_obstacle_good
        if not 0 <= depth_age <= float(self.get_parameter('depth_grace').value):
            self.state('DEPTH_NOT_READY')
            return
        self.cancel()
        self.pending = msg
        self.state('GOAL_QUEUED')

    def on_ready(self, msg):
        self.ready = bool(msg.data)
        if not self.ready:
            self.pending = None
            self.cancel()
            self.state('RELOCALIZATION_REQUIRED')

    def on_alignment(self, msg):
        new = (msg.x, msg.y, msg.theta)
        if self.alignment is not None and any(abs(a-b) > 1e-6 for a,b in zip(new,self.alignment)):
            had_goal = self.pending is not None or self.goal_handle is not None or self.sending
            self.pending = None
            self.cancel()
            if had_goal:
                self.state('MAP_ALIGNMENT_CHANGED_REISSUE_GOAL')
        self.alignment = new

    def on_obstacle(self, msg):
        self.obstacle_fresh = bool(msg.data)
        if self.obstacle_fresh:
            self.last_obstacle_good = self.get_clock().now().nanoseconds * 1e-9

    def on_vio(self, msg):
        self.vio_valid = msg.data == 'VALID'
        if not self.vio_valid:
            self.cancel()

    def on_flight(self, msg):
        self.flight_state = msg.data
        if self.flight_state != 'CRUISE':
            self.cancel()

    def tick(self):
        depth_age = self.get_clock().now().nanoseconds * 1e-9 - self.last_obstacle_good
        depth_valid = 0 <= depth_age <= float(self.get_parameter('depth_grace').value)
        if not depth_valid:
            self.cancel()
        if not self.ready or not self.vio_valid or not depth_valid:
            return
        if self.flight_state != 'CRUISE' or self.pending is None or self.sending:
            return
        if not self.action.server_is_ready():
            self.state('WAITING_FOR_NAV2')
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.pending
        self.pending = None
        self.sending = True
        epoch = self.request_epoch
        future = self.action.send_goal_async(goal)
        future.add_done_callback(lambda result, e=epoch: self.on_goal_response(result, e))
        self.state('PLANNING')

    def on_goal_response(self, future, epoch):
        self.sending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Nav2 goal request failed: {exc}')
            self.state('NAV2_UNAVAILABLE')
            return
        if epoch != self.request_epoch or not self.ready or not self.vio_valid or self.flight_state != 'CRUISE':
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.state('GOAL_REJECTED_BY_NAV2')
            return
        self.goal_handle = handle
        self.state('FOLLOWING_PATH')
        handle.get_result_async().add_done_callback(
            lambda result, h=handle: self.on_result(result, h))

    def on_result(self, future, handle):
        if self.goal_handle is not handle:
            return
        self.goal_handle = None
        try:
            result = future.result()
            self.state('GOAL_REACHED' if result.status == 4 else f'NAVIGATION_STOPPED_{result.status}')
        except Exception as exc:
            self.get_logger().error(f'Nav2 result failed: {exc}')
            self.state('NAVIGATION_RESULT_ERROR')


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
