#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PointStamped, PoseStamped
from morai_udp_bridge.msg import ActuatorCommand, CompetitionVehicleStatus
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu


class LocalizationGenerationSafetyTest(unittest.TestCase):
    SAFE_BRAKE = 0.5
    WAIT_TIMEOUT_SEC = 3.0
    INPUT_TIMEOUT_SEC = 0.25
    MINIMUM_GENERATION_DT_SEC = 0.02
    PRE_TIMEOUT_GUARD_SEC = 0.02

    def setUp(self):
        self._lock = threading.Lock()
        self._commands = []
        self._poses = []
        self._paths = []
        self._odometry = []
        self._command_sub = rospy.Subscriber(
            "/generation_test/command", ActuatorCommand, self._on_command, queue_size=50
        )
        self._pose_sub = rospy.Subscriber(
            "/generation_test/pose", PoseStamped, self._on_pose, queue_size=50
        )
        self._path_sub = rospy.Subscriber(
            "/generation_test/local_path", Path, self._on_path, queue_size=50
        )
        self._odometry_sub = rospy.Subscriber(
            "/generation_test/odometry", Odometry, self._on_odometry, queue_size=50
        )
        self._gps_pub = rospy.Publisher("/generation_test/gps", PointStamped, queue_size=10)
        self._imu_pub = rospy.Publisher("/generation_test/imu", Imu, queue_size=10)
        self._status_pub = rospy.Publisher(
            "/generation_test/competition_status",
            CompetitionVehicleStatus,
            queue_size=10,
        )

    def _on_command(self, message):
        with self._lock:
            self._commands.append((time.monotonic(), message))

    def _on_pose(self, message):
        with self._lock:
            self._poses.append((time.monotonic(), message))

    def _on_path(self, message):
        with self._lock:
            self._paths.append((time.monotonic(), message))

    def _on_odometry(self, message):
        with self._lock:
            self._odometry.append((time.monotonic(), message))

    def _snapshot_after(self, entries, mark):
        with self._lock:
            return [entry for entry in entries if entry[0] >= mark]

    def _wait_for(self, predicate, description, timeout_sec=WAIT_TIMEOUT_SEC):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.005)
        self.fail("timed out waiting for " + description)

    def _publish_generation(self, stamp, x):
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w = 1.0
        self._imu_pub.publish(imu)
        gps = PointStamped()
        gps.header.stamp = stamp
        gps.header.frame_id = "map"
        gps.point.x = x
        self._gps_pub.publish(gps)

    def _publish_status(self):
        status = CompetitionVehicleStatus()
        status.header.stamp = rospy.Time.now()
        status.header.frame_id = "base_link"
        status.control_mode = 2
        status.gear = 4
        status.velocity_x_mps = 0.0
        self._status_pub.publish(status)

    @staticmethod
    def _is_safe(command):
        return (
            command.accel == 0.0
            and command.brake == LocalizationGenerationSafetyTest.SAFE_BRAKE
            and command.steering_angle_rad == 0.0
        )

    def test_rejected_velocity_advances_path_without_matching_odometry(self):
        self._wait_for(
            lambda: self._gps_pub.get_num_connections() > 0
            and self._imu_pub.get_num_connections() > 0
            and self._status_pub.get_num_connections() > 0,
            "localization subscriptions",
        )
        baseline_mark = time.monotonic()
        source_zero = rospy.Time.now()
        self._publish_generation(source_zero, 0.0)
        self._wait_for(
            lambda: any(
                pose.header.stamp == source_zero
                for _, pose in self._snapshot_after(self._poses, baseline_mark)
            ),
            "baseline pose at its published source stamp",
        )

        minimum_valid_stamp = source_zero + rospy.Duration(
            self.MINIMUM_GENERATION_DT_SEC
        )
        self._wait_for(
            lambda: rospy.Time.now() >= minimum_valid_stamp,
            "ROS clock to reach the minimum valid-generation interval",
        )
        source_valid = rospy.Time.now()
        valid_dt_sec = (source_valid - source_zero).to_sec()
        self.assertGreaterEqual(
            valid_dt_sec,
            self.MINIMUM_GENERATION_DT_SEC,
            "valid generation must be separated from the baseline by a valid dt",
        )
        self.assertLess(
            valid_dt_sec,
            self.INPUT_TIMEOUT_SEC,
            "valid generation must remain within the velocity estimator dt limit",
        )
        valid_mark = time.monotonic()
        self._publish_generation(source_valid, 0.04)
        self._wait_for(
            lambda: any(
                pose.header.stamp == source_valid
                for _, pose in self._snapshot_after(self._poses, valid_mark)
            ),
            "valid pose at its published source stamp",
        )
        self._wait_for(
            lambda: any(
                path.header.stamp == source_valid
                for _, path in self._snapshot_after(self._paths, valid_mark)
            ),
            "local path at the valid generation stamp",
        )
        self._wait_for(
            lambda: any(
                odometry.header.stamp == source_valid
                for _, odometry in self._snapshot_after(self._odometry, valid_mark)
            ),
            "odometry at the valid generation stamp",
        )
        valid_command_mark = time.monotonic()
        valid_command_phase_stamp = rospy.Time.now()
        self._publish_status()
        self._wait_for(
            lambda: any(
                command.accel > 0.0
                and command.brake == 0.0
                and command.header.stamp > valid_command_phase_stamp
                for _, command in self._snapshot_after(
                    self._commands, valid_command_mark
                )
            ),
            "valid controller command generated after the valid phase",
        )

        minimum_rejected_stamp = source_valid + rospy.Duration(
            self.MINIMUM_GENERATION_DT_SEC
        )
        self._wait_for(
            lambda: rospy.Time.now() >= minimum_rejected_stamp,
            "ROS clock to reach the rejected-generation interval",
        )
        source_rejected = rospy.Time.now()
        rejected_dt_sec = (source_rejected - source_valid).to_sec()
        self.assertGreaterEqual(
            rejected_dt_sec,
            self.MINIMUM_GENERATION_DT_SEC,
            "rejected generation must use a fresh source stamp after the valid sample",
        )
        self.assertLess(
            rejected_dt_sec,
            self.INPUT_TIMEOUT_SEC,
            "rejected generation must arrive before the valid velocity sample expires",
        )
        rejected_mark = time.monotonic()
        self._publish_generation(source_rejected, 100.0)
        self._wait_for(
            lambda: any(
                pose.header.stamp == source_rejected
                for _, pose in self._snapshot_after(self._poses, rejected_mark)
            ),
            "pose for rejected-velocity generation",
        )
        self._wait_for(
            lambda: any(
                path.header.stamp == source_rejected
                for _, path in self._snapshot_after(self._paths, rejected_mark)
            ),
            "local path copied from the rejected-velocity pose stamp",
        )

        rejected_path_command_mark = time.monotonic()
        rejected_path_phase_stamp = rospy.Time.now()
        input_timeout_cutoff = source_valid + rospy.Duration(self.INPUT_TIMEOUT_SEC)
        observation_cutoff = input_timeout_cutoff - rospy.Duration(
            self.PRE_TIMEOUT_GUARD_SEC
        )
        remaining_observation_sec = (observation_cutoff - rospy.Time.now()).to_sec()
        self.assertGreater(
            remaining_observation_sec,
            0.0,
            "rejected path phase must complete before the valid input timeout guard",
        )
        observation_deadline = time.monotonic() + remaining_observation_sec
        saw_safe_command = False
        while time.monotonic() < observation_deadline and not rospy.is_shutdown():
            with self._lock:
                rejected_odometry_seen = any(
                    odometry.header.stamp == source_rejected
                    for _, odometry in self._odometry
                )
            self.assertFalse(
                rejected_odometry_seen,
                "rejected velocity must never publish odometry at its source stamp",
            )
            saw_safe_command = saw_safe_command or any(
                self._is_safe(command)
                and command.header.stamp > rejected_path_phase_stamp
                and command.header.stamp < input_timeout_cutoff
                for _, command in self._snapshot_after(
                    self._commands, rejected_path_command_mark
                )
            )
            time.sleep(0.005)

        with self._lock:
            rejected_odometry_seen = any(
                odometry.header.stamp == source_rejected
                for _, odometry in self._odometry
            )
        self.assertFalse(
            rejected_odometry_seen,
            "rejected velocity must not publish matching odometry during the full observation window",
        )
        self.assertFalse(
            saw_safe_command,
            "an unmatched path callback must not replace the retained synchronized pair",
        )

        self._wait_for(
            lambda: any(
                self._is_safe(command)
                and command.header.stamp >= input_timeout_cutoff
                for _, command in self._snapshot_after(
                    self._commands, rejected_path_command_mark
                )
            ),
            "safe command after the retained synchronized pair timeout",
        )


if __name__ == "__main__":
    rospy.init_node("localization_generation_safety_test")
    rostest.rosrun(
        "morai_path_tracking",
        "localization_generation_safety",
        LocalizationGenerationSafetyTest,
    )
