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


class HybridControllerNodeTest(unittest.TestCase):
    WAIT_TIMEOUT_SEC = 6.0

    def setUp(self):
        self._lock = threading.Lock()
        self._commands = []
        self._statuses = []
        self._targets = []
        self._command_subscriber = rospy.Subscriber(
            "/hybrid_test/actuator_command",
            ActuatorCommand,
            self._on_command,
            queue_size=20,
        )
        self._status_subscriber = rospy.Subscriber(
            "/hybrid_test/controller_status",
            ControllerStatus,
            self._on_status,
            queue_size=20,
        )
        self._target_subscriber = rospy.Subscriber(
            "/hybrid_test/lookahead_point",
            PointStamped,
            self._on_target,
            queue_size=20,
        )
        self._path_publisher = rospy.Publisher(
            "/hybrid_test/local_path", Path, queue_size=1
        )
        self._odometry_publisher = rospy.Publisher(
            "/hybrid_test/odometry", Odometry, queue_size=1
        )
        self._vehicle_status_publisher = rospy.Publisher(
            "/hybrid_test/competition_status",
            CompetitionVehicleStatus,
            queue_size=1,
        )

    def _on_command(self, message):
        with self._lock:
            self._commands.append(message)

    def _on_status(self, message):
        with self._lock:
            self._statuses.append(message)

    def _on_target(self, message):
        with self._lock:
            self._targets.append(message)

    def _connected(self):
        return (
            self._path_publisher.get_num_connections() > 0
            and self._odometry_publisher.get_num_connections() > 0
            and self._vehicle_status_publisher.get_num_connections() > 0
        )

    def _publish_inputs(self):
        stamp = rospy.Time.now()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = "map"
        for x in (0.0, 5.0, 10.0, 20.0):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = 0.5
            path.poses.append(pose)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "map"
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.linear.y = 0.2
        odometry.twist.twist.angular.z = 0.1

        vehicle_status = CompetitionVehicleStatus()
        vehicle_status.header.stamp = stamp
        vehicle_status.header.frame_id = "base_link"
        vehicle_status.control_mode = 2
        vehicle_status.gear = 4
        vehicle_status.velocity_x_mps = 5.0

        self._path_publisher.publish(path)
        self._odometry_publisher.publish(odometry)
        self._vehicle_status_publisher.publish(vehicle_status)

    def test_hybrid_publishes_normalized_model_blend_and_both_references(self):
        deadline = time.monotonic() + self.WAIT_TIMEOUT_SEC
        while (
            time.monotonic() < deadline
            and not rospy.is_shutdown()
            and not self._connected()
        ):
            rospy.sleep(0.01)
        self.assertTrue(self._connected(), "hybrid subscribers did not connect")

        while time.monotonic() < deadline and not rospy.is_shutdown():
            self._publish_inputs()
            rospy.sleep(0.04)
            with self._lock:
                statuses = [
                    status
                    for status in self._statuses
                    if status.active
                    and status.state == "ACTIVE"
                    and status.lateral_controller == "hybrid"
                ]
                commands = list(self._commands)
                targets = list(self._targets)
            if not statuses or not commands or not targets:
                continue

            status = statuses[-1]
            probability_sum = (
                status.hybrid_pure_pursuit_probability
                + status.hybrid_stanley_probability
            )
            self.assertAlmostEqual(probability_sum, 1.0, places=9)
            self.assertGreaterEqual(status.hybrid_stanley_probability, 0.15)
            self.assertLessEqual(status.hybrid_stanley_probability, 0.90)
            self.assertAlmostEqual(
                status.hybrid_effective_pure_pursuit_weight
                + status.hybrid_effective_stanley_weight,
                1.0,
                places=9,
            )
            self.assertAlmostEqual(status.lane_half_width_m, 1.30, places=9)
            self.assertAlmostEqual(status.vehicle_width_m, 1.892, places=9)
            self.assertTrue(math.isfinite(status.wheel_outer_offset_m))
            self.assertTrue(math.isfinite(status.wheel_minimum_clearance_m))
            self.assertAlmostEqual(
                status.wheel_minimum_clearance_m,
                status.lane_half_width_m - status.wheel_outer_offset_m,
                places=9,
            )
            self.assertGreaterEqual(
                status.lane_clearance_recovery_urgency, 0.0
            )
            self.assertLessEqual(
                status.lane_clearance_recovery_urgency, 1.0
            )
            self.assertGreater(status.lane_clearance_speed_limit_mps, 0.0)
            self.assertTrue(
                math.isfinite(status.pure_pursuit_steering_angle_rad)
            )
            self.assertTrue(
                math.isfinite(
                    status.hybrid_corrected_pure_pursuit_steering_angle_rad
                )
            )
            self.assertTrue(math.isfinite(status.stanley_steering_angle_rad))
            expected_request = (
                status.hybrid_effective_pure_pursuit_weight
                * status.hybrid_corrected_pure_pursuit_steering_angle_rad
                + status.hybrid_effective_stanley_weight
                * status.stanley_steering_angle_rad
            )
            self.assertAlmostEqual(
                status.requested_steering_angle_rad,
                expected_request,
                places=8,
            )
            self.assertAlmostEqual(
                status.measured_sideslip_angle_rad,
                math.atan2(0.2 + 1.5 * 0.1, 5.0),
                places=8,
            )
            self.assertAlmostEqual(
                status.stanley_projection_point_base.x, 3.0, places=6
            )
            self.assertAlmostEqual(
                status.stanley_projection_point_base.y, 0.5, places=6
            )
            target = targets[-1]
            self.assertEqual(target.header.frame_id, "base_link")
            self.assertGreater(target.point.x, 5.0)
            self.assertAlmostEqual(target.point.y, 0.5, places=6)
            command = commands[-1]
            self.assertTrue(math.isfinite(command.steering_angle_rad))
            self.assertFalse(command.accel > 0.0 and command.brake > 0.0)
            return

        self.fail("hybrid mode did not publish a valid model-probability blend")


if __name__ == "__main__":
    rospy.init_node("hybrid_controller_node_test")
    rostest.rosrun(
        "morai_path_tracking",
        "hybrid_controller",
        HybridControllerNodeTest,
    )
