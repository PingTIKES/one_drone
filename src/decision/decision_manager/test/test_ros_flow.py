"""ROS integration with synthetic sensors only; never sends PX4 control commands."""
import json
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Header, String

from decision_manager.decision_node import DecisionManager
from goal_manager.goal_manager import GoalManager


def test_offboard_route_recovery_and_manual_override(tmp_path):
    config = tmp_path / 'decisions.yaml'
    config.write_text('''decisions:
  decision_1:
    mode: loop
    frame_id: odom
    waypoints: [[1.0, 0.0, 2.0], [2.0, 0.0, 3.0]]
''')
    rclpy.init(args=['--ros-args', '-p', f'decisions_file:={config}',
                    '-p', 'dwell_time:=0.05', '-p', 'check_rate:=40.0'])
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        decision, manager, fake = DecisionManager(), GoalManager(), Node('synthetic_navigation')
        nodes = [decision, manager, fake]
        for node in nodes:
            executor.add_node(node)
        pubs = {
            'px4': fake.create_publisher(VehicleStatus, '/fmu/out/vehicle_status', qos_profile_sensor_data),
            'odom': fake.create_publisher(Odometry, '/ego/odom', 10),
            'health': fake.create_publisher(String, '/vio_health', 10),
            'flight': fake.create_publisher(String, '/flight_state', 10),
            'depth': fake.create_publisher(Bool, '/ego/depth_fresh', 10),
            'map': fake.create_publisher(Header, '/ego/map_heartbeat', 10),
            'manual': fake.create_publisher(PoseStamped, '/navigation_goal', 10),
        }
        goals = []
        fake.create_subscription(PoseStamped, '/ego/goal', goals.append, 10)
        nav = VehicleStatus.NAVIGATION_STATE_POSCTL
        flight, health, x, z, speed, stale = 'CRUISE', 'VALID', 0.0, 2.0, 0.0, False

        def pump(duration=.3):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                stamp = fake.get_clock().now().to_msg()
                pubs['px4'].publish(VehicleStatus(nav_state=nav))
                odom = Odometry()
                odom.header.frame_id = 'odom'
                odom.header.stamp = stamp if not stale else type(stamp)(sec=1)
                odom.pose.pose.position.x, odom.pose.pose.position.z = x, z
                odom.twist.twist.linear.x = speed
                pubs['odom'].publish(odom)
                pubs['health'].publish(String(data=health))
                pubs['flight'].publish(String(data=flight))
                pubs['depth'].publish(Bool(data=True))
                pubs['map'].publish(Header(stamp=stamp, frame_id='synthetic_planner'))
                # Drain callbacks promptly while still letting timer time advance.
                for _ in range(15):
                    executor.spin_once(timeout_sec=.001)

        pump(.5)
        assert not goals
        nav, flight = VehicleStatus.NAVIGATION_STATE_OFFBOARD, 'TAKEOFF'
        pump()
        assert decision.mission.index == 0 and not goals
        flight = 'CRUISE'
        pump()
        assert goals[-1].pose.position.z == 2.0
        assert decision.mission.index == 0
        # Remaining in OFFBOARD does not reset the active waypoint.
        x, stale = 1.0, True
        pump()
        assert decision.mission.index == 0
        stale, speed = False, 1.0
        pump()
        assert decision.mission.index == 0
        speed, health = 0.0, 'INVALID'
        pump()
        assert decision.mission.index == 0
        health = 'VALID'
        pump(.5)
        assert decision.mission.index == 1
        assert goals[-1].pose.position.z == 3.0  # RViz altitude must not overwrite decision z.
        # Late acknowledgement for the old waypoint cannot change the active target.
        accepted = decision.accepted
        old = PoseStamped()
        old.header.frame_id = 'odom'
        decision.on_accepted(old)
        assert decision.accepted is accepted
        x, z = 2.0, 3.0
        pump(.5)
        assert decision.mission.index == 0 and decision.mission.laps == 1
        assert goals[-1].pose.position.x == 1.0
        # A repeated request is acknowledged without republishing /ego/goal.
        count = len(goals)
        decision.goal_pub.publish(decision.request)
        pump()
        assert len(goals) == count
        # Map-frame 3D targets use the actual transformed EGO point for acknowledgement.
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'map'
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = 10.0
        transform.transform.translation.z = 1.0
        manager.tf_buffer.set_transform_static(transform, 'test')
        mapped = PoseStamped()
        mapped.header.frame_id = 'map'
        mapped.header.stamp = fake.get_clock().now().to_msg()
        mapped.pose.position.x = 1.0
        mapped.pose.position.z = 4.0
        mapped.pose.orientation.w = 1.0
        manager.on_decision_goal(mapped)
        assert manager.decision_accepted_goal.pose.position.x == 11.0
        assert manager.decision_accepted_goal.pose.position.z == 5.0
        # Restore the real decision request before testing OFFBOARD cancellation.
        manager.on_decision_goal(decision.request)
        # Exiting OFFBOARD clears its queue; reentry selects a fresh mission once.
        nav = VehicleStatus.NAVIGATION_STATE_POSCTL
        pump()
        assert decision.mission is None and manager.pending_goal is None
        nav = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        pump()
        assert decision.mission.index == 0
        # Reset invalidates the task. Health recovery alone cannot revive old targets.
        manager.on_reset(Header(stamp=fake.get_clock().now().to_msg()))
        decision.on_reset(Header(stamp=fake.get_clock().now().to_msg()))
        pump()
        assert decision.mission is None and manager.pending_goal is None
        nav = VehicleStatus.NAVIGATION_STATE_POSCTL
        pump()
        nav = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        pump()
        assert decision.mission is not None
        manual = PoseStamped()
        manual.header.frame_id = 'odom'
        manual.header.stamp = fake.get_clock().now().to_msg()
        manual.pose.position.x = 9.0
        manual.pose.orientation.w = 1.0
        old_request = decision.request
        pubs['manual'].publish(manual)
        pump()
        assert decision.mission is None
        assert goals[-1].pose.position.x == 9.0
        assert manager.pending_goal.pose.position.x == 9.0
        manager.on_decision_goal(old_request)
        assert manager.pending_goal.pose.position.x == 9.0
        assert json.loads(decision.last_state)['state'] == 'CANCELED_MANUAL_GOAL'
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
