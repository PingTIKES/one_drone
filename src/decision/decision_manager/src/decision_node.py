"""Select decision_1 on OFFBOARD entry and execute its waypoints sequentially."""
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Header, String

from decision_manager.mission import Mission, load_decisions


def stamp_key(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


class DecisionManager(Node):
    def __init__(self):
        super().__init__('decision_manager')
        defaults = dict(decisions_file='', initial_decision='decision_1', mavros_ns='/mavros',
                        enabled=True, check_rate=10.0, xy_tolerance=0.25,
                        z_tolerance=0.20, speed_tolerance=0.20, dwell_time=0.5,
                        odom_timeout=0.5, status_timeout=3.0,
                        goal_retry_interval=1.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ('check_rate', 'xy_tolerance', 'z_tolerance', 'speed_tolerance',
                     'dwell_time', 'odom_timeout', 'status_timeout', 'goal_retry_interval'):
            if not math.isfinite(float(self.p(name))) or float(self.p(name)) <= 0:
                raise ValueError(f'{name} must be positive and finite')
        self.decisions = load_decisions(yaml.safe_load(
            Path(str(self.p('decisions_file'))).expanduser().read_text()))
        if self.select_decision() not in self.decisions:
            raise ValueError('initial_decision is not defined in decisions_file')
        self.mission = None
        self.offboard = False
        self.odom = None
        self.status_at = self.odom_at = -math.inf
        self.request = None
        self.accepted = None
        self.last_request_key = 0
        self.sent_at = -math.inf
        self.last_state = None
        self.last_reset = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/decision/state', latched)
        self.goal_pub = self.create_publisher(PoseStamped, '/decision/goal', 10)
        self.cancel_pub = self.create_publisher(Header, '/decision/clear_pending', 10)
        prefix = str(self.p('mavros_ns')).strip('/')
        status_topic = '/' + prefix + '/state'
        self.create_subscription(State, status_topic, self.on_status, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ego/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/decision/accepted_goal', self.on_accepted, 10)
        self.create_subscription(PoseStamped, '/navigation_goal', self.on_manual_goal, 10)
        self.create_subscription(Header, '/vio_reset_event', self.on_reset, latched)
        self.create_timer(1.0 / float(self.p('check_rate')), self.tick)
        self.report('WAIT_OFFBOARD' if self.p('enabled') else 'DISABLED')

    def p(self, name):
        return self.get_parameter(name).value

    def select_decision(self):
        # Future behavior selection belongs here. For now always choose decision_1
        # (or the configured initial_decision), without resetting on every tick.
        return str(self.p('initial_decision'))

    def report(self, state):
        data = dict(state=state, decision=self.select_decision(),
                    waypoint_index=self.mission.index if self.mission else None,
                    completed_laps=self.mission.laps if self.mission else 0)
        text = json.dumps(data, ensure_ascii=False)
        if text != self.last_state:
            self.last_state = text
            self.state_pub.publish(String(data=text))
            self.get_logger().info(text)

    def clear_pending(self):
        if self.request is not None:
            self.cancel_pub.publish(self.request.header)

    def on_status(self, msg):
        self.status_at = time.monotonic()
        offboard = msg.connected and msg.mode == 'OFFBOARD'
        entered = offboard and not self.offboard
        self.offboard = offboard
        if not self.p('enabled'):
            return
        if not offboard:
            if self.mission is not None:
                self.clear_pending()
                self.mission = None
                self.request = self.accepted = None
            self.report('WAIT_OFFBOARD')
        elif entered:
            definition = self.decisions[self.select_decision()]
            if not definition['waypoints']:
                self.report('NO_WAYPOINTS')
                return
            self.mission = Mission(definition)
            self.send_waypoint()

    def send_waypoint(self):
        goal = PoseStamped()
        key = max(self.get_clock().now().nanoseconds, self.last_request_key + 1)
        self.last_request_key = key
        goal.header.stamp.sec, goal.header.stamp.nanosec = divmod(key, 1000000000)
        goal.header.frame_id = self.mission.definition['frame_id']
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = map(float, self.mission.waypoint)
        goal.pose.orientation.w = 1.0
        self.request = goal
        self.accepted = None
        self.goal_pub.publish(goal)
        self.sent_at = time.monotonic()
        self.report('WAIT_NAVIGATION_ACCEPTANCE')

    def on_accepted(self, msg):
        if (self.request is not None and self.mission is not None and
                stamp_key(msg.header.stamp) == stamp_key(self.request.header.stamp) and
                msg.header.frame_id == 'odom'):
            self.accepted = msg
            self.mission.arrival_since = None

    def on_odom(self, msg):
        self.odom = msg
        self.odom_at = time.monotonic()

    def on_manual_goal(self, _msg):
        if self.mission is not None and not self.mission.complete:
            self.clear_pending()
            self.mission = None
            self.request = self.accepted = None
            self.report('CANCELED_MANUAL_GOAL')

    def on_reset(self, msg):
        key = (stamp_key(msg.stamp), msg.frame_id)
        if key == self.last_reset:
            return
        self.last_reset = key
        if self.mission is not None:
            self.clear_pending()
            self.mission = None
            self.request = self.accepted = None
            self.report('VIO_RESET_NEW_OFFBOARD_REQUIRED')

    def ready(self, now):
        if not self.offboard or now - self.status_at > float(self.p('status_timeout')):
            return 'PAUSED_PX4_STATUS'
        if self.odom is None or now - self.odom_at > float(self.p('odom_timeout')):
            return 'PAUSED_ODOMETRY'
        age = (self.get_clock().now().nanoseconds - stamp_key(self.odom.header.stamp)) * 1e-9
        if self.odom.header.frame_id != 'odom' or not -0.02 <= age <= float(self.p('odom_timeout')):
            return 'PAUSED_ODOMETRY'
        return None

    def tick(self):
        if self.mission is None or self.mission.complete:
            return
        now = time.monotonic()
        reason = self.ready(now)
        if reason:
            self.mission.arrival_since = None
            self.report(reason)
            return
        if self.accepted is None:
            if now - self.sent_at >= float(self.p('goal_retry_interval')):
                self.goal_pub.publish(self.request)
                self.sent_at = now
            self.report('WAIT_NAVIGATION_ACCEPTANCE')
            return
        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear
        t = self.accepted.pose.position
        values = (p.x, p.y, p.z, v.x, v.y, v.z, t.x, t.y, t.z)
        if not all(math.isfinite(x) for x in values):
            self.mission.arrival_since = None
            self.report('PAUSED_INVALID_ODOMETRY')
            return
        speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        self.report('RUNNING')
        if self.mission.observe((p.x, p.y, p.z), (t.x, t.y, t.z), speed, now,
                                float(self.p('xy_tolerance')), float(self.p('z_tolerance')),
                                float(self.p('speed_tolerance')), float(self.p('dwell_time'))):
            if self.mission.advance():
                self.send_waypoint()
            else:
                self.request = self.accepted = None
                self.report('SUCCEEDED')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DecisionManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
