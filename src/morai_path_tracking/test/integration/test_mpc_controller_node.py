#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PointStamped, PoseStamped
from morai_path_tracking.msg import ControllerStatus
from morai_udp_bridge.msg import ActuatorCommand, CompetitionVehicleStatus
from nav_msgs.msg import Odometry, Path


class MpcControllerNodeTest(unittest.TestCase):
    WAIT_TIMEOUT_SEC = 8.0

    def setUp(self):
        self._lock = threading.Lock()
        self._commands = []
        self._statuses = []
        self._projections = []
        self._command_subscriber = rospy.Subscriber(
            "/mpc_test/actuator_command", ActuatorCommand,
            self._on_command, queue_size=20
        )
        self._status_subscriber = rospy.Subscriber(
            "/mpc_test/controller_status", ControllerStatus,
            self._on_status, queue_size=20
        )
        self._projection_subscriber = rospy.Subscriber(
            "/mpc_test/mpc_projection_point", PointStamped,
            self._on_projection, queue_size=20
        )
        self._path_publisher = rospy.Publisher(
            "/mpc_test/local_path", Path, queue_size=1
        )
        self._odometry_publisher = rospy.Publisher(
            "/mpc_test/odometry", Odometry, queue_size=1
        )
        self._status_publisher = rospy.Publisher(
            "/mpc_test/competition_status", CompetitionVehicleStatus,
            queue_size=1
        )

    def _on_status(self, message):
        with self._lock:
            self._statuses.append(message)

    def _on_command(self, message):
        with self._lock:
            self._commands.append(message)

    def _on_projection(self, message):
        with self._lock:
            self._projections.append(message)

    def _connected(self):
        return (
            self._path_publisher.get_num_connections() > 0
            and self._odometry_publisher.get_num_connections() > 0
            and self._status_publisher.get_num_connections() > 0
        )

    def _publish_inputs(self):
        stamp = rospy.Time.now()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = "map"
        for x in (0.0, 5.0, 10.0, 20.0, 35.0):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = 0.5
            path.poses.append(pose)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "map"
        odometry.pose.pose.orientation.w = 1.0

        status = CompetitionVehicleStatus()
        status.header.stamp = stamp
        status.header.frame_id = "base_link"
        status.control_mode = 2
        status.gear = 4
        status.velocity_x_mps = 5.0

        self._path_publisher.publish(path)
        self._odometry_publisher.publish(odometry)
        self._status_publisher.publish(status)

    def test_mpc_publishes_solver_state_and_bounded_command(self):
        deadline = time.monotonic() + self.WAIT_TIMEOUT_SEC
        while time.monotonic() < deadline and not self._connected():
            rospy.sleep(0.01)
        self.assertTrue(self._connected(), "MPC subscribers did not connect")

        while time.monotonic() < deadline and not rospy.is_shutdown():
            self._publish_inputs()
            rospy.sleep(0.04)
            with self._lock:
                statuses = [s for s in self._statuses if s.active]
                commands = list(self._commands)
                projections = list(self._projections)
            if not statuses or not commands or not projections:
                continue
            status = statuses[-1]
            command = commands[-1]
            self.assertEqual(status.lateral_controller, "mpc")
            self.assertEqual(status.lateral_mpc_solver, "osqp")
            self.assertEqual(status.longitudinal_controller, "mpc")
            self.assertEqual(
                status.longitudinal_mpc_solver, "osqp"
            )
            self.assertTrue(status.mpc_solver_success)
            self.assertGreater(status.mpc_solver_iterations, 0)
            self.assertTrue(math.isfinite(status.mpc_solver_time_ms))
            self.assertLess(
                status.mpc_solver_time_ms,
                1000.0 / 30.0,
                "MPC solve time exceeded the 30 Hz control period",
            )
            self.assertTrue(math.isfinite(status.mpc_solver_cost))
            self.assertTrue(math.isfinite(status.mpc_steering_rate_rad_per_sec))
            self.assertTrue(status.longitudinal_mpc_solver_success)
            self.assertFalse(status.longitudinal_mpc_fallback_active)
            self.assertGreater(status.longitudinal_mpc_solver_iterations, 0)
            self.assertTrue(
                math.isfinite(status.longitudinal_mpc_solver_time_ms)
            )
            self.assertTrue(math.isfinite(status.longitudinal_mpc_solver_cost))
            self.assertLessEqual(status.target_speed_mps, 5.0 + 1e-9)
            self.assertTrue(math.isfinite(command.steering_angle_rad))
            self.assertLessEqual(abs(command.steering_angle_rad), math.radians(40.0))
            self.assertFalse(command.accel > 0.0 and command.brake > 0.0)
            self.assertEqual(projections[-1].header.frame_id, "base_link")
            return
        self.fail("MPC mode did not publish an active solver result")


if __name__ == "__main__":
    rospy.init_node("mpc_controller_node_test")
    rostest.rosrun(
        "morai_path_tracking", "mpc_controller", MpcControllerNodeTest
    )
