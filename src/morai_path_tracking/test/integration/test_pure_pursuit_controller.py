#!/usr/bin/env python3

import subprocess
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PointStamped, PoseStamped
from morai_path_tracking.msg import ControllerStatus
from morai_udp_bridge.msg import ActuatorCommand, CompetitionVehicleStatus
from nav_msgs.msg import Odometry, Path


class PurePursuitControllerTest(unittest.TestCase):
    SAFE_BRAKE = 0.5
    INPUT_TIMEOUT_SEC = 0.25
    INVALID_REACTION_TIMEOUT_SEC = 0.12
    TIMEOUT_REACTION_GRACE_SEC = 0.20
    ODOMETRY_REFRESH_PERIOD_SEC = 0.04
    WAIT_TIMEOUT_SEC = 3.0

    def setUp(self):
        self._commands = []
        self._controller_statuses = []
        self._lookahead_points = []
        self._command_lock = threading.Lock()
        self._command_subscriber = rospy.Subscriber(
            "/control/actuator_command", ActuatorCommand, self._on_command, queue_size=50
        )
        self._odometry_publisher = rospy.Publisher(
            "/localization/odometry", Odometry, queue_size=1
        )
        self._path_publisher = rospy.Publisher("/local_path", Path, queue_size=1)
        self._vehicle_status_publisher = rospy.Publisher(
            "/vehicle/competition_status", CompetitionVehicleStatus, queue_size=1
        )
        self._controller_status_subscriber = rospy.Subscriber(
            "/control/controller_status",
            ControllerStatus,
            self._on_controller_status,
            queue_size=50,
        )
        self._lookahead_subscriber = rospy.Subscriber(
            "/control/lookahead_point",
            PointStamped,
            self._on_lookahead_point,
            queue_size=50,
        )

    def _on_command(self, message):
        with self._command_lock:
            self._commands.append((time.monotonic(), message))

    def _on_controller_status(self, message):
        with self._command_lock:
            self._controller_statuses.append((time.monotonic(), message))

    def _on_lookahead_point(self, message):
        with self._command_lock:
            self._lookahead_points.append((time.monotonic(), message))

    def _commands_after(self, mark):
        with self._command_lock:
            return [entry for entry in self._commands if entry[0] >= mark]

    @staticmethod
    def _is_safe(command):
        return (
            command.accel == 0.0
            and command.brake == PurePursuitControllerTest.SAFE_BRAKE
            and command.steering_angle_rad == 0.0
        )

    @staticmethod
    def _safe_at_or_after_cutoff(receipt_time, command, cutoff):
        del receipt_time
        return (
            PurePursuitControllerTest._is_safe(command)
            and command.header.stamp >= cutoff
        )

    def _wait_for(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail("timed out waiting for " + description)

    def _wait_for_until(self, predicate, deadline, description):
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail("timed out waiting for " + description)

    def _safe_commands_before_output_cutoff(self, start, cutoff):
        return [
            command
            for receipt_time, command in self._commands_after(start)
            if self._is_safe(command) and command.header.stamp < cutoff
        ]

    def _wait_for_safe_output_window(
        self, start, not_before, output_deadline, monotonic_deadline, description
    ):
        self._wait_for_until(
            lambda: any(
                self._is_safe(command)
                and command.header.stamp >= not_before
                and command.header.stamp < output_deadline
                for _, command in self._commands_after(start)
            ),
            monotonic_deadline,
            description,
        )

    def _assert_no_safe_output_before(self, start, cutoff, description):
        early_safe_commands = self._safe_commands_before_output_cutoff(start, cutoff)
        self.assertFalse(
            early_safe_commands,
            "{} (cutoff={} now={})".format(
                description, cutoff.to_sec(), rospy.Time.now().to_sec()
            ),
        )

    def _wait_for_safe_after_cutoff_without_early(
        self, start, cutoff, monotonic_deadline, description
    ):
        while time.monotonic() < monotonic_deadline and not rospy.is_shutdown():
            self._assert_no_safe_output_before(
                start, cutoff, "controller published a safe command before " + description
            )
            if any(
                self._safe_at_or_after_cutoff(receipt_time, command, cutoff)
                for receipt_time, command in self._commands_after(start)
            ):
                return
            rospy.sleep(0.01)
        self._assert_no_safe_output_before(
            start, cutoff, "controller published a safe command before " + description
        )
        self.fail("timed out waiting for " + description)

    def _publish_inputs(
        self,
        points=None,
        odometry_frame="map",
        path_frame="map",
        odometry_stamp=None,
        path_stamp=None,
        position=(0.0, 0.0, 0.0),
        speed=0.0,
        orientation=(0.0, 0.0, 0.0, 1.0),
        status_velocity_x_mps=0.0,
        status_frame="base_link",
        publish_odometry=True,
        publish_path=True,
        publish_status=True,
    ):
        if points is None:
            points = [(float(x), 0.0) for x in range(10)]
        odometry = Odometry()
        odometry.header.frame_id = odometry_frame
        if odometry_stamp is not None:
            odometry.header.stamp = odometry_stamp
        odometry.pose.pose.position.x = position[0]
        odometry.pose.pose.position.y = position[1]
        odometry.pose.pose.position.z = position[2]
        odometry.pose.pose.orientation.x = orientation[0]
        odometry.pose.pose.orientation.y = orientation[1]
        odometry.pose.pose.orientation.z = orientation[2]
        odometry.pose.pose.orientation.w = orientation[3]
        odometry.twist.twist.linear.x = speed

        path = Path()
        path.header.frame_id = path_frame
        if path_stamp is not None:
            path.header.stamp = path_stamp
        for x, y in points:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)

        publish_mark = time.monotonic()
        publication_stamp = rospy.Time.now()
        if odometry_stamp is None:
            odometry.header.stamp = publication_stamp
        if path_stamp is None:
            path.header.stamp = publication_stamp
        if publish_odometry:
            self._odometry_publisher.publish(odometry)
        if publish_path:
            self._path_publisher.publish(path)
        if publish_status:
            status = CompetitionVehicleStatus()
            status.header.stamp = publication_stamp
            status.header.frame_id = status_frame
            status.control_mode = 2
            status.gear = 4
            status.velocity_x_mps = status_velocity_x_mps
            self._vehicle_status_publisher.publish(status)
        return {
            "monotonic_mark": publish_mark,
            "publication_stamp": publication_stamp,
            "odometry_stamp": odometry.header.stamp,
            "path_stamp": path.header.stamp,
        }

    def _wait_for_subscriptions(self):
        self._wait_for(
            lambda: self._odometry_publisher.get_num_connections() > 0
            and self._path_publisher.get_num_connections() > 0
            and self._vehicle_status_publisher.get_num_connections() > 0,
            self.WAIT_TIMEOUT_SEC,
            "controller subscriptions to test odometry and path publishers",
        )

    def _establish_valid_baseline(self):
        inputs = self._publish_inputs()
        mark = inputs["monotonic_mark"]
        cutoff = inputs["path_stamp"] + rospy.Duration(self.INPUT_TIMEOUT_SEC)
        self._wait_for(
            lambda: any(
                command.accel > 0.0
                and command.brake == 0.0
                and abs(command.steering_angle_rad) < 1.0e-4
                and command.header.stamp >= inputs["publication_stamp"]
                for _, command in self._commands_after(mark)
            ),
            self.WAIT_TIMEOUT_SEC,
            "fresh valid straight-path command",
        )
        valid_receipts = [
            receipt_time
            for receipt_time, command in self._commands_after(mark)
            if command.accel > 0.0
            and command.brake == 0.0
            and abs(command.steering_angle_rad) < 1.0e-4
            and command.header.stamp >= inputs["publication_stamp"]
        ]
        return {
            "monotonic_mark": mark,
            "valid_receipt": min(valid_receipts),
            "publication_stamp": inputs["publication_stamp"],
            "retained_path_stamp": inputs["path_stamp"],
            "cutoff": cutoff,
        }

    def _assert_invalid_inputs_brake(self, description, **kwargs):
        baseline = self._establish_valid_baseline()
        invalid_inputs = self._publish_inputs(**kwargs)
        remaining_sec = (baseline["cutoff"] - invalid_inputs["publication_stamp"]).to_sec()
        if remaining_sec < self.INVALID_REACTION_TIMEOUT_SEC:
            self.fail(
                "fresh baseline left only {:.3f}s before its natural expiry; "
                "need at least {:.3f}s for invalid-input reaction".format(
                    remaining_sec, self.INVALID_REACTION_TIMEOUT_SEC
                )
            )
        self._wait_for_safe_output_window(
            invalid_inputs["monotonic_mark"],
            invalid_inputs["publication_stamp"],
            baseline["cutoff"],
            time.monotonic() + self.INVALID_REACTION_TIMEOUT_SEC,
            description,
        )

    @staticmethod
    def _controller_parameters(
        safe_brake_command, lateral_controller="pure_pursuit"
    ):
        return {
            "local_path_topic": "/local_path",
            "odometry_topic": "/localization/odometry",
            "command_topic": "/invalid_config_command",
            "vehicle_status_topic": "/vehicle/competition_status",
            "controller_status_topic": "/invalid_controller_status",
            "lookahead_point_topic": "/invalid_lookahead_point",
            "stanley_projection_point_topic": (
                "/invalid_stanley_projection_point"
            ),
            "expected_frame_id": "map",
            "expected_velocity_frame_id": "base_link",
            "control_rate_hz": 30.0,
            "path_timeout_sec": 0.25,
            "odometry_timeout_sec": 0.25,
            "vehicle_status_timeout_sec": 0.25,
            "maximum_input_skew_sec": 0.0,
            "input_sync_queue_size": 10,
            "minimum_control_dt_sec": 0.005,
            "maximum_control_dt_sec": 0.10,
            "safe_brake_command": safe_brake_command,
            "wheelbase_m": 3.0,
            "lookahead_base_m": 3.0,
            "lookahead_speed_gain_sec": 0.5,
            "lookahead_curvature_gain_m": 5.0,
            "lookahead_min_m": 3.0,
            "lookahead_max_m": 6.0,
            "minimum_target_distance_m": 0.5,
            "maximum_steering_angle_deg": 40.0,
            "lateral_controller": lateral_controller,
            "stanley_gain": 2.0,
            "stanley_softening_speed_mps": 1.0,
            "stanley_minimum_control_speed_mps": 1.0,
            "stanley_heading_window_m": 4.0,
            "stanley_heading_error_gain": 0.5,
            "stanley_curvature_feedforward_gain": 1.0,
            "stanley_curvature_preview_distance_m": 8.0,
            "stanley_yaw_rate_damping_gain_sec": 0.1,
            "stanley_yaw_rate_damping_nonlinear_gain_sec2": 0.5,
            "stanley_maximum_steering_rate_deg_per_sec": 180.0,
            "hybrid_mass_kg": 2000.0,
            "hybrid_yaw_inertia_kgm2": 4000.0,
            "hybrid_front_cornering_stiffness_n_per_rad": 60000.0,
            "hybrid_rear_cornering_stiffness_n_per_rad": 60000.0,
            "hybrid_front_axle_to_cg_m": 1.5,
            "hybrid_rear_axle_to_cg_m": 1.5,
            "hybrid_process_noise_sideslip": 0.1,
            "hybrid_process_noise_yaw_rate": 0.01,
            "hybrid_measurement_noise_sideslip": 0.001,
            "hybrid_measurement_noise_yaw_rate": 0.001,
            "hybrid_initial_covariance_sideslip": 0.1,
            "hybrid_initial_covariance_yaw_rate": 0.01,
            "hybrid_initial_pure_pursuit_probability": 0.8,
            "hybrid_initial_stanley_probability": 0.2,
            "hybrid_stanley_probability_min": 0.15,
            "hybrid_stanley_probability_max": 0.90,
            "hybrid_transition_pure_pursuit_to_pure_pursuit": 0.9,
            "hybrid_transition_pure_pursuit_to_stanley": 0.1,
            "hybrid_transition_stanley_to_pure_pursuit": 0.95,
            "hybrid_transition_stanley_to_stanley": 0.05,
            "hybrid_transition_speed_gain": 0.1,
            "hybrid_transition_reference_speed_kph": 60.0,
            "hybrid_minimum_model_speed_mps": 1.0,
            "hybrid_pure_pursuit_cross_track_correction_gain": 0.5,
            "hybrid_candidate_conflict_curvature_threshold_m_inv": 0.015,
            "hybrid_candidate_conflict_cross_track_threshold_m": 0.45,
            "hybrid_cross_track_recovery_full_scale_m": 0.50,
            "hybrid_cross_track_recovery_heading_error_suppression_start_deg": 15.0,
            "hybrid_cross_track_recovery_heading_error_suppression_full_deg": 17.5,
            "hybrid_cross_track_recovery_heading_error_maximum_suppression_ratio": 0.30,
            "hybrid_maximum_steering_rate_deg_per_sec": 180.0,
            "hybrid_steering_return_rate_multiplier": 2.0,
            "target_speed_kph": 10.8,
            "minimum_curve_speed_kph": 3.0,
            "maximum_lateral_acceleration_mps2": 1.5,
            "curvature_speed_reduction_gain_m": 0.0,
            "curvature_preview_distance_m": 20.0,
            "lookahead_curvature_preview_distance_m": 20.0,
            "curvature_sample_spacing_m": 0.5,
            "curve_approach_deceleration_mps2": 0.001,
            "curvature_epsilon_m_inv": 0.001,
            "target_speed_acceleration_limit_mps2": 1.0,
            "curve_target_speed_acceleration_limit_mps2": 0.5,
            "target_speed_deceleration_limit_mps2": 2.0,
            "target_speed_filter_time_constant_sec": 0.0,
            "speed_filter_time_constant_sec": 0.0,
            "speed_kp": 0.35,
            "speed_ki": 0.08,
            "speed_kd": 0.02,
            "speed_integral_limit": 2.0,
            "speed_integral_unwind_rate_per_sec": 0.5,
            "speed_error_deadband_mps": 0.05,
            "speed_accel_feedforward_gain_per_mps": 0.0,
            "speed_coast_overspeed_kph": 0.2,
            "speed_brake_overspeed_kph": 1.8,
            "hard_brake_activation_speed_kph": 59.0,
            "minimum_hard_brake_command": 0.25,
            "maximum_accel_command": 0.40,
            "maximum_brake_command": 0.60,
            "longitudinal_command_rate_limit_per_sec": 0.0,
        }

    def test_rejects_nan_safe_brake_configuration(self):
        node_name = "invalid_config_controller"
        rospy.set_param("/" + node_name, self._controller_parameters(float("nan")))
        process = subprocess.Popen(
            [
                "rosrun",
                "morai_path_tracking",
                "path_tracking_controller_node",
                "__name:=" + node_name,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            self._wait_for(
                lambda: process.poll() is not None,
                self.WAIT_TIMEOUT_SEC,
                "controller rejection of NaN safe_brake_command",
            )
            self.assertNotEqual(0, process.returncode)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=1.0)
            rospy.delete_param("/" + node_name)

    def test_rejects_unknown_lateral_controller(self):
        node_name = "invalid_lateral_controller"
        rospy.set_param(
            "/" + node_name,
            self._controller_parameters(0.5, lateral_controller="unknown"),
        )
        process = subprocess.Popen(
            [
                "rosrun",
                "morai_path_tracking",
                "path_tracking_controller_node",
                "__name:=" + node_name,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            self._wait_for(
                lambda: process.poll() is not None,
                self.WAIT_TIMEOUT_SEC,
                "controller rejection of unknown lateral_controller",
            )
            self.assertNotEqual(0, process.returncode)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=1.0)
            rospy.delete_param("/" + node_name)

    def test_output_stamp_classification_rejects_late_delivery_of_early_safe(self):
        cutoff = rospy.Time(10, 250000000)
        command = ActuatorCommand()
        command.brake = self.SAFE_BRAKE
        command.header.stamp = rospy.Time(10, 249000000)
        self.assertFalse(
            self._safe_at_or_after_cutoff(11.0, command, cutoff),
            "an early controller output must remain early when delivery is delayed",
        )

    def test_safe_state_for_missing_and_invalid_inputs(self):
        initial_mark = time.monotonic()
        self._wait_for(
            lambda: any(
                self._is_safe(command) for _, command in self._commands_after(initial_mark)
            ),
            self.WAIT_TIMEOUT_SEC,
            "initial safe braking command before controller inputs",
        )
        self._wait_for_subscriptions()

        baseline = self._establish_valid_baseline()
        path_timeout_cutoff = baseline["cutoff"]
        pre_cutoff_deadline = (
            baseline["monotonic_mark"]
            + self.INPUT_TIMEOUT_SEC
            + self.TIMEOUT_REACTION_GRACE_SEC
        )
        while (
            rospy.Time.now() < path_timeout_cutoff
            and time.monotonic() < pre_cutoff_deadline
            and not rospy.is_shutdown()
        ):
            self._publish_inputs(
                publish_path=False,
                odometry_stamp=baseline["retained_path_stamp"],
            )
            self._assert_no_safe_output_before(
                baseline["valid_receipt"],
                path_timeout_cutoff,
                "controller braked before the retained path receipt timeout",
            )
            refresh_deadline = min(
                pre_cutoff_deadline,
                time.monotonic() + self.ODOMETRY_REFRESH_PERIOD_SEC,
            )
            while time.monotonic() < refresh_deadline and not rospy.is_shutdown():
                self._assert_no_safe_output_before(
                    baseline["valid_receipt"],
                    path_timeout_cutoff,
                    "controller braked before the retained path receipt timeout",
                )
                rospy.sleep(0.01)
        if rospy.Time.now() < path_timeout_cutoff:
            self.fail("ROS time did not reach the retained path timeout before deadline")
        self._wait_for_safe_after_cutoff_without_early(
            baseline["valid_receipt"],
            path_timeout_cutoff,
            time.monotonic() + self.TIMEOUT_REACTION_GRACE_SEC,
            "safe brake after the retained path receipt timeout",
        )

        self._assert_invalid_inputs_brake(
            "safe brake for wrong input frame", odometry_frame="odom", path_frame="odom"
        )
        self._assert_invalid_inputs_brake(
            "safe brake for zero input stamps",
            odometry_stamp=rospy.Time(),
            path_stamp=rospy.Time(),
        )

        stale_stamp = rospy.Time.now() - rospy.Duration(1.0)
        self._assert_invalid_inputs_brake(
            "safe brake for stale input stamps",
            odometry_stamp=stale_stamp,
            path_stamp=stale_stamp,
        )
        future_stamp = rospy.Time.now() + rospy.Duration(1.0)
        self._assert_invalid_inputs_brake(
            "safe brake for future input stamps",
            odometry_stamp=future_stamp,
            path_stamp=future_stamp,
        )

        self._assert_invalid_inputs_brake(
            "safe brake for non-finite odometry pose", position=(float("nan"), 0.0, 0.0)
        )
        self._assert_invalid_inputs_brake(
            "safe brake for non-finite path point", points=[(0.0, 0.0), (float("nan"), 1.0)]
        )
        self._assert_invalid_inputs_brake(
            "safe brake for invalid odometry quaternion", orientation=(0.0, 0.0, 0.0, 0.0)
        )

    def test_rejects_small_nonzero_path_odometry_stamp_mismatch(self):
        self._wait_for_subscriptions()
        baseline = self._establish_valid_baseline()
        mixed_stamp = rospy.Time.now()
        mismatch = self._publish_inputs(
            odometry_stamp=mixed_stamp - rospy.Duration(0.01),
            path_stamp=mixed_stamp,
        )
        observation_deadline = min(
            time.monotonic() + self.INVALID_REACTION_TIMEOUT_SEC,
            baseline["valid_receipt"] + self.INPUT_TIMEOUT_SEC - 0.02,
        )
        while time.monotonic() < observation_deadline and not rospy.is_shutdown():
            self._assert_no_safe_output_before(
                mismatch["monotonic_mark"],
                baseline["cutoff"],
                "unpaired callbacks must not replace the retained synchronized pair",
            )
            rospy.sleep(0.01)

    def test_competition_velocity_drives_pid_and_odometry_twist_is_ignored(self):
        self._wait_for_subscriptions()
        inputs = self._publish_inputs(
            speed=float("inf"),
            status_velocity_x_mps=4.0,
        )
        mark = inputs["monotonic_mark"]
        self._wait_for(
            lambda: any(
                status.active
                and status.state == "ACTIVE"
                and abs(status.configured_target_speed_mps - 3.0) < 1.0e-6
                and abs(status.raw_target_speed_mps - 3.0) < 1.0e-6
                and abs(status.filtered_target_speed_mps - 3.0) < 1.0e-6
                and abs(status.target_speed_mps - 3.0) < 1.0e-6
                and abs(status.preview_curvature_m_inv) < 1.0e-9
                and abs(status.speed_limiting_curve_distance_m + 1.0) < 1.0e-9
                and abs(status.lookahead_curvature_m_inv) < 1.0e-9
                and abs(status.curvature_speed_limit_mps - 3.0) < 1.0e-6
                and abs(status.measured_velocity_x_mps - 4.0) < 1.0e-6
                and abs(status.speed_overshoot_mps - 1.0) < 1.0e-6
                and status.longitudinal_state == "BRAKE"
                and status.brake > 0.0
                for receipt, status in self._controller_statuses
                if receipt >= mark
            ),
            self.WAIT_TIMEOUT_SEC,
            "active braking PID command from Competition velocity",
        )

    def test_active_cycle_publishes_lookahead_point_in_base_link(self):
        self._wait_for_subscriptions()
        inputs = self._publish_inputs()
        mark = inputs["monotonic_mark"]
        self._wait_for(
            lambda: any(
                point.header.frame_id == "base_link"
                and point.point.x > 0.0
                and abs(point.point.y) < 1.0e-6
                for receipt, point in self._lookahead_points
                if receipt >= mark
            ),
            self.WAIT_TIMEOUT_SEC,
            "lookahead point in base_link",
        )

    def test_curvature_and_lane_clearance_lower_pid_target_and_lookahead(self):
        self._wait_for_subscriptions()
        radius_m = 2.0
        diagonal_m = radius_m / (2.0 ** 0.5)
        inputs = self._publish_inputs(
            points=[
                (0.0, 0.0),
                (diagonal_m, radius_m - diagonal_m),
                (radius_m, radius_m),
            ],
            status_velocity_x_mps=2.0,
        )
        mark = inputs["monotonic_mark"]
        self._wait_for(
            lambda: any(
                status.active
                and status.state == "ACTIVE"
                and status.preview_curvature_m_inv > 0.0
                and status.speed_limiting_curve_distance_m >= 0.0
                and status.lookahead_curvature_m_inv > 0.0
                and status.curvature_speed_limit_mps
                < status.configured_target_speed_mps
                and status.raw_target_speed_mps
                < status.configured_target_speed_mps
                and abs(
                    status.filtered_target_speed_mps
                    - status.raw_target_speed_mps
                )
                < 1.0e-9
                and status.wheel_minimum_clearance_m < 0.0
                and abs(status.lane_clearance_speed_limit_mps - (3.0 / 3.6))
                < 1.0e-6
                and abs(
                    status.target_speed_mps
                    - status.lane_clearance_speed_limit_mps
                )
                < 1.0e-6
                and status.target_speed_mps < status.filtered_target_speed_mps
                and status.target_speed_mps < status.configured_target_speed_mps
                and abs(status.lookahead_distance_m - 3.0) < 1.0e-6
                for receipt, status in self._controller_statuses
                if receipt >= mark
            ),
            self.WAIT_TIMEOUT_SEC,
            "curvature-limited PID target and lookahead",
        )

    def test_timeout_does_not_brake_immediately_after_valid_left_command(self):
        self._wait_for_subscriptions()
        self._establish_valid_baseline()
        inputs = self._publish_inputs(
            points=[(0.0, 0.0), (3.0, 1.5), (6.0, 3.0), (9.0, 4.5)]
        )
        mark = inputs["monotonic_mark"]
        timeout_cutoff = inputs["path_stamp"] + rospy.Duration(self.INPUT_TIMEOUT_SEC)
        self._wait_for_until(
            lambda: any(
                command.header.stamp < timeout_cutoff
                and command.header.stamp >= inputs["publication_stamp"]
                and command.steering_angle_rad > 0.0
                and command.accel > 0.0
                and command.brake == 0.0
                for _, command in self._commands_after(mark)
            ),
            mark + self.INVALID_REACTION_TIMEOUT_SEC,
            "positive steering command for fresh left-curving path",
        )
        self._wait_for_safe_after_cutoff_without_early(
            mark,
            timeout_cutoff,
            mark + self.INPUT_TIMEOUT_SEC + self.TIMEOUT_REACTION_GRACE_SEC,
            "safe braking command after the input timeout",
        )


if __name__ == "__main__":
    rospy.init_node("pure_pursuit_controller_test")
    rostest.rosrun("morai_path_tracking", "pure_pursuit_controller", PurePursuitControllerTest)
