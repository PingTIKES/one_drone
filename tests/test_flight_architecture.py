"""Safety contracts for the refactored single-writer flight chain."""
import math
import unittest
import numpy as np
import rclpy

from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import Quaternion

from flight_bridge.flight_supervisor import FlightSupervisor
from flight_bridge.px4_adapter import Px4Adapter
from flight_bridge.trajectory_controller import TrajectoryController
from flight_bridge.trajectory_validator import TrajectoryValidator
from flight_bridge.yaw_manager import YawManager


def sample(stamp=99.95):
    command = PositionCommand()
    command.header.frame_id = 'odom'
    command.header.stamp.sec = int(stamp)
    command.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
    command.trajectory_id = 17
    command.position.x, command.position.y, command.position.z = 1., 0., 2.
    command.velocity.x = .5
    return command


class FlightArchitectureTest(unittest.TestCase):
    def test_px4_audit_accepts_numpy_coordinates_from_sensor_messages(self):
        rclpy.init()
        node = rclpy.create_node('flight_adapter_type_test')
        try:
            adapter = Px4Adapter(node, 'px4_1', 2)
            adapter.position((np.float32(1.), np.float32(2.), np.float32(-2.)),
                             np.float32(0.))
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_validator_rejects_stale_nonfinite_and_tracking_jump(self):
        validator = TrajectoryValidator(.5, .1, 8., 12., 5.)
        command = sample()
        actual = Odometry().pose.pose.position
        self.assertEqual(validator.validate(command, 100., actual), (True, 'OK'))
        self.assertEqual(validator.validate(sample(99.), 100., actual)[1],
                         'TRAJECTORY_TIMESTAMP')
        command.velocity.x = math.nan
        self.assertEqual(validator.validate(command, 100., actual)[1],
                         'NONFINITE_TRAJECTORY')
        command = sample()
        command.position.x = 9.
        self.assertEqual(validator.validate(command, 100., actual)[1],
                         'TRAJECTORY_TRACKING_ERROR')

    def test_yaw_looks_ahead_and_limits_translation_outside_camera_fov(self):
        yaw = YawManager(30., 100., 150., .25, .35, .10, .8,
                         87., .3, .25)
        orientation = Quaternion(w=1.)
        forward = yaw.decide((.5, 0., 0.), (0., 0., 0.), orientation)
        self.assertEqual(forward.mode, 'YAW_HOLD')
        self.assertAlmostEqual(forward.translation_scale, 1.)
        turn = yaw.decide((.5, 0., 0.), (0., 2., 0.), orientation)
        self.assertEqual(turn.mode, 'YAW_FOV_LIMIT')
        self.assertLessEqual(turn.translation_scale, .25)
        self.assertGreater(turn.yaw_rate_flu, 0.)
        stopped = yaw.decide((.5, 0., 0.), (0., 2., 0.), orientation,
                             arrival_hold=True)
        self.assertEqual(stopped.translation_scale, 0.)
        self.assertEqual(stopped.yaw_rate_flu, 0.)

    def test_controller_preserves_arrival_hold_and_speed_cap(self):
        controller = TrajectoryController(.8, .5, .3, .15, .1, 1.5)
        odom = Odometry()
        command = sample()
        result = controller.track(command, odom)
        self.assertTrue(result.speed_limited)
        self.assertAlmostEqual(math.hypot(*result.world_velocity[:2]), .5)
        command.position.x = .1
        command.velocity.x = .05
        result = controller.track(command, odom)
        self.assertTrue(result.arrival_hold)
        self.assertEqual(result.world_velocity[:2], (0., 0.))

    def test_supervisor_holds_and_recovers_only_with_fresh_offboard(self):
        supervisor = FlightSupervisor()
        self.assertIsNone(supervisor.safety_hold('CRUISE', True, True, True))
        self.assertIn('Planner', supervisor.safety_hold('CRUISE', True, True, False))
        args = ('HOLD', True, True, True, 'CRUISE', True, True,
                True, True, True, False)
        self.assertTrue(supervisor.may_auto_recover(*args))
        self.assertFalse(supervisor.may_auto_recover(*args[:-1], True))


if __name__ == '__main__':
    unittest.main()
