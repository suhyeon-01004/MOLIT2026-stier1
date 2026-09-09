#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PointStamped, PoseStamped
from morai_path_tracking.msg import ControllerStatus
from morai_udp_bridge.msg import ActuatorCommand, CompetitionVehicleStatus
from nav_msgs.msg import Odometry, Path


class StanleyControllerNodeTest(unittest.TestCase):
    WAIT_TIMEOUT_SEC = 4.0

    def setUp(self):
        self._lock = threading.Lock()
        self._commands = []
        self._statuses = []
        self._tracking_targets = []
        self._command_subscriber = rospy.Subscriber(
            "/stanley_test/actuator_command",
            ActuatorCommand,
            self._on_command,
            queue_size=20,
        )
        self._status_subscriber = rospy.Subscriber(
            "/stanley_test/controller_status",
            ControllerStatus,
            self._on_status,
            queue_size=20,
        )
        self._tracking_target_subscriber = rospy.Subscriber(
            "/stanley_test/lookahead_point",
            PointStamped,
            self._on_tracking_target,
            queue_size=20,
        )
        self._path_publisher = rospy.Publisher(
            "/stanley_test/local_path", Path, queue_size=1
        )
        self._odometry_publisher = rospy.Publisher(
            "/stanley_test/odometry", Odometry, queue_size=1
        )
        self._vehicle_status_publisher = rospy.Publisher(
            "/stanley_test/competition_status",
            CompetitionVehicleStatus,
            queue_size=1,
        )

    def _on_command(self, message):
        with self._lock:
            self._commands.append(message)

    def _on_status(self, message):
        with self._lock:
            self._statuses.append(message)

    def _on_tracking_target(self, message):
        with self._lock:
            self._tracking_targets.append(message)

    def _connected(self):
        return (
            self._path_publisher.get_num_connections() > 0
            and self._odometry_publisher.get_num_connections() > 0
            and self._vehicle_status_publisher.get_num_connections() > 0
        )

    def _publish_path(self, offset_y=1.0, yaw_rate_radps=0.0):
        stamp = rospy.Time.now()

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = "map"
        for x in (0.0, 5.0, 10.0, 20.0):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = offset_y
            path.poses.append(pose)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "map"
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.angular.z = yaw_rate_radps

        vehicle_status = CompetitionVehicleStatus()
        vehicle_status.header.stamp = stamp
        vehicle_status.header.frame_id = "base_link"
        vehicle_status.control_mode = 2
        vehicle_status.gear = 4
        vehicle_status.velocity_x_mps = 5.0

        self._path_publisher.publish(path)
        self._odometry_publisher.publish(odometry)
        self._vehicle_status_publisher.publish(vehicle_status)

    def _publish_left_offset_path(self):
        self._publish_path(offset_y=1.0)

    def test_left_cross_track_error_produces_positive_stanley_steering(self):
        deadline = time.monotonic() + self.WAIT_TIMEOUT_SEC
        while (
            time.monotonic() < deadline
            and not rospy.is_shutdown()
            and not self._connected()
        ):
            rospy.sleep(0.01)
        self.assertTrue(self._connected(), "controller subscribers did not connect")

        while time.monotonic() < deadline and not rospy.is_shutdown():
            self._publish_left_offset_path()
            rospy.sleep(0.04)
            with self._lock:
                valid_statuses = [
                    status
                    for status in self._statuses
                    if status.active
                    and status.state == "ACTIVE"
                    and status.lateral_controller == "stanley"
                    and status.cross_track_error_m > 0.9
                    and abs(status.heading_error_rad) < 1.0e-6
                    and status.steering_angle_rad > 0.0
                ]
                valid_commands = [
                    command
                    for command in self._commands
                    if command.steering_angle_rad > 0.0
                    and command.accel == 0.0
                    and command.brake > 0.0
                ]
            if valid_statuses and valid_commands:
                return

        self.fail("Stanley mode did not publish positive correction for left error")

    def test_yaw_rate_error_damps_curve_exit_rotation(self):
        deadline = time.monotonic() + self.WAIT_TIMEOUT_SEC
        while (
            time.monotonic() < deadline
            and not rospy.is_shutdown()
            and not self._connected()
        ):
            rospy.sleep(0.01)
        self.assertTrue(self._connected(), "controller subscribers did not connect")

        while time.monotonic() < deadline and not rospy.is_shutdown():
            self._publish_path(offset_y=0.0, yaw_rate_radps=0.4)
            rospy.sleep(0.04)
            with self._lock:
                valid_statuses = [
                    status
                    for status in self._statuses
                    if status.active
                    and abs(status.cross_track_error_m) < 1.0e-6
                    and abs(status.heading_error_rad) < 1.0e-6
                    and abs(status.reference_curvature_m_inv) < 1.0e-6
                    and abs(status.reference_yaw_rate_radps) < 1.0e-6
                    and abs(status.measured_yaw_rate_radps - 0.4) < 1.0e-6
                    and abs(status.yaw_rate_error_radps - 0.4) < 1.0e-6
                    and abs(
                        status.applied_yaw_rate_damping_gain_sec - 0.3
                    ) < 1.0e-6
                    and abs(
                        status.yaw_rate_damping_steering_rad + 0.12
                    ) < 1.0e-6
                    and abs(
                        status.requested_steering_angle_rad + 0.12
                    ) < 1.0e-6
                    and status.steering_angle_rad < 0.0
                ]
            if valid_statuses:
                return

        self.fail("Stanley mode did not damp positive yaw rate on a straight path")

    def test_stanley_publishes_front_axle_projection_as_tracking_target(self):
        deadline = time.monotonic() + self.WAIT_TIMEOUT_SEC
        while (
            time.monotonic() < deadline
            and not rospy.is_shutdown()
            and not self._connected()
        ):
            rospy.sleep(0.01)
        self.assertTrue(self._connected(), "controller subscribers did not connect")

        while time.monotonic() < deadline and not rospy.is_shutdown():
            self._publish_left_offset_path()
            rospy.sleep(0.04)
            with self._lock:
                targets = list(self._tracking_targets)
            if targets:
                target = targets[-1]
                self.assertEqual(target.header.frame_id, "base_link")
                self.assertAlmostEqual(target.point.x, 3.0, places=6)
                self.assertAlmostEqual(target.point.y, 1.0, places=6)
                return

        self.fail("Stanley mode did not publish its front-axle path projection")


if __name__ == "__main__":
    rospy.init_node("stanley_controller_node_test")
    rostest.rosrun(
        "morai_path_tracking",
        "stanley_controller",
        StanleyControllerNodeTest,
    )
