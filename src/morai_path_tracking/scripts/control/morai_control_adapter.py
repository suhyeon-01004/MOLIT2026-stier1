#!/usr/bin/python3
"""MORAI adapter for the locally adapted Autoware AI lateral MPC.

Steering state is estimated, not measured. sent_control reports the final bounded
command handed to the UDP sender, not tire feedback or simulator acknowledgement.
"""
import copy
import math
import threading
import time
import numpy as np
import rospy
import message_filters
from steering_prediction import SteeringPrediction
from autoware_msgs.msg import Lane, Waypoint, VehicleStatus, ControlCommandStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32
from morai_udp_bridge.msg import ActuatorCommand
from morai_path_tracking.msg import ControllerStatus


def fresh(stamp, received, now_ros, now_wall):
    return 0.0 <= now_ros - stamp <= .25 and 0.0 <= now_wall - received <= .25


def bounded_angle(requested, previous, dt):
    if not all(math.isfinite(v) for v in (requested, previous, dt)) or not .005 <= dt <= .10:
        raise ValueError('Invalid steering or control period')
    requested = max(-math.radians(40), min(math.radians(40), requested))
    step = math.radians(60) * dt
    return max(previous - step, min(previous + step, requested))


class Adapter:
    def __init__(self):
        # ponytail: one lock suffices for a 30 Hz adapter; no worker pipeline.
        self.lock = threading.RLock()
        self.values = {}
        self.route = None
        self.route_xy = None
        self.angle = 0.
        self.last_tick = time.monotonic()
        self.align_state = rospy.get_param('/autoware_mpc/morai_state_time_alignment', False)
        self.maximum_input_speed = float(rospy.get_param('~maximum_input_speed_kph', 61.2)) / 3.6
        if not math.isfinite(self.maximum_input_speed) or not 60. / 3.6 <= self.maximum_input_speed <= 102. / 3.6:
            raise ValueError('maximum_input_speed_kph must be in [60, 102]')
        if self.maximum_input_speed > 17. and not rospy.get_param('/path_tracking_controller_node/use_map_speed_limits', False):
            raise ValueError('Above-60 input range requires map speed limits')
        model = rospy.get_param('/autoware_mpc/vehicle_model_type')
        self.steer_gain = rospy.get_param('/autoware_mpc/vehicle_model_steer_gain', 1.0)
        if not math.isfinite(self.steer_gain) or not .5 <= self.steer_gain <= 1.5:
            raise ValueError('Invalid effective steering gain')
        self.predictor = SteeringPrediction(
            rospy.get_param('/autoware_mpc/delay_compensation_time'),
            rospy.get_param('/autoware_mpc/vehicle_model_steer_tau'), self.last_tick) if model == 'kinematics' else None
        self.last_input = None
        self.input_error = 'WAIT_INPUT'
        self.pub_lane = rospy.Publisher('/control/internal/waypoints', Lane, queue_size=1)
        self.pub_pose = rospy.Publisher('/control/internal/pose', PoseStamped, queue_size=1)
        self.pub_vehicle = rospy.Publisher('/control/internal/vehicle_status_estimated', VehicleStatus, queue_size=1)
        self.pub_twist = rospy.Publisher('/control/internal/twist', TwistStamped, queue_size=1)
        self.pub_command = rospy.Publisher('/control/actuator_command', ActuatorCommand, queue_size=1)
        self.pub_sent = rospy.Publisher('/control/internal/sent_control', ControlCommandStamped, queue_size=1)
        self.pub_status = rospy.Publisher('/control/controller_status', ControllerStatus, queue_size=1)
        self.subs = [rospy.Subscriber('/global_path', Path, self.global_path, queue_size=1)]
        for name, topic, kind in [
            ('long', '/control/internal/longitudinal_status', ControllerStatus),
            ('mpc', '/control/internal/mpc_raw', ControlCommandStamped),
            ('time', '/autoware_mpc/debug/mpc_calc_time', Float32)]:
            self.subs.append(rospy.Subscriber(topic, kind, lambda msg, n=name: self.store(n, msg), queue_size=1, tcp_nodelay=self.align_state))
        self.path_sub = message_filters.Subscriber('/local_path', Path, queue_size=5, tcp_nodelay=self.align_state)
        self.odom_sub = message_filters.Subscriber('/localization/odometry', Odometry, queue_size=5, tcp_nodelay=self.align_state)
        self.sync = message_filters.TimeSynchronizer([self.path_sub, self.odom_sub], 10)
        self.sync.registerCallback(self.inputs)
        self.timer = rospy.Timer(rospy.Duration(1. / 30.), self.tick)

    def store(self, name, msg):
        with self.lock:
            self.values[name] = (msg, time.monotonic())

    def global_path(self, msg):
        with self.lock:
            if len(msg.poses) < 20 or msg.header.frame_id != 'map':
                self.input_error = 'INVALID_GLOBAL_PATH'
                self.route = None
                return
            self.route = msg.poses[:-1]  # Existing route explicitly repeats point 0.
            self.route_xy = np.array([(p.pose.position.x, p.pose.position.y) for p in self.route])
            last = msg.poses[-1].pose.position
            if not np.all(np.isfinite(self.route_xy)) or np.linalg.norm(self.route_xy[0] - [last.x, last.y]) > .01:
                self.route = None
                self.input_error = 'INVALID_CLOSED_ROUTE'

    def inputs(self, path, odom):
        with self.lock:
            try:
                if self.route is None or len(path.poses) < 20:
                    raise ValueError('WAIT_PATH')
                if path.header.frame_id != 'map' or odom.header.frame_id != 'map' or odom.child_frame_id != 'base_link':
                    raise ValueError('INPUT_FRAME')
                pos, q, twist = odom.pose.pose.position, odom.pose.pose.orientation, odom.twist.twist
                if not all(math.isfinite(v) for v in (pos.x, pos.y, q.x, q.y, q.z, q.w, twist.linear.x, twist.angular.z)):
                    raise ValueError('NONFINITE_INPUT')
                if abs(sum(v*v for v in (q.x, q.y, q.z, q.w)) - 1) > .01 or abs(twist.linear.x) > self.maximum_input_speed:
                    raise ValueError('INPUT_RANGE')
                p0 = path.poses[0].pose.position
                nearest = int(np.argmin(np.sum((self.route_xy - [p0.x, p0.y]) ** 2, axis=1)))
                if np.linalg.norm(self.route_xy[nearest] - [p0.x, p0.y]) > .05:
                    raise ValueError('ROUTE_MISMATCH')
                # Add 4 m behind from the SAME route, without shifting XY.
                poses = [self.route[(nearest - i) % len(self.route)] for i in range(8, 0, -1)] + path.poses
                lane = Lane(header=path.header)
                # AI MPC predicts using waypoint speeds. Use measured speed,
                # not the distant target speed, as its constant-speed model.
                speed = max(.5, abs(twist.linear.x))
                for pose in poses:
                    point = pose.pose.position
                    if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
                        raise ValueError('NONFINITE_PATH')
                    waypoint = Waypoint()
                    waypoint.pose = pose
                    waypoint.twist.twist.linear.x = speed
                    lane.waypoints.append(waypoint)
                vehicle = VehicleStatus(header=odom.header, speed=twist.linear.x * 3.6)
                vehicle.angle = math.atan(3. * twist.angular.z / twist.linear.x) if abs(twist.linear.x) > 1. else 0.
                if self.predictor:
                    age = (rospy.Time.now()-odom.header.stamp).to_sec()
                    wall = time.monotonic()
                    vehicle.angle = self.predictor.advance(wall)
                    if self.align_state:
                        if not 0.<=age<=.15:raise ValueError('OBSERVATION_AGE')
                        vehicle.angle = self.predictor.at(wall-age)
                twist_message = TwistStamped(header=odom.header, twist=twist)
                if self.align_state:
                    vehicle.header=copy.deepcopy(odom.header)
                    twist_message.header=copy.deepcopy(odom.header)
                    vehicle.header.frame_id=twist_message.header.frame_id='base_link'
                self.pub_pose.publish(PoseStamped(header=odom.header, pose=odom.pose.pose))
                self.pub_vehicle.publish(vehicle)
                self.pub_twist.publish(twist_message)
                self.pub_lane.publish(lane)
                self.last_input = (odom.header.stamp.to_sec(), time.monotonic())
                self.input_error = ''
            except ValueError as exc:
                self.input_error = str(exc)

    def tick(self, event):
        with self.lock:
            wall, stamp = time.monotonic(), rospy.Time.now()
            dt = wall - self.last_tick
            self.last_tick = wall
            status = copy.deepcopy(self.values['long'][0]) if 'long' in self.values else ControllerStatus()
            status.header.stamp = stamp
            status.lateral_controller = 'autoware_ai_mpc'
            status.lateral_mpc_solver = 'qpoases_hotstart'
            status.mpc_solver_iterations = -1  # Not exposed by upstream node.
            status.mpc_solver_cost = float('nan')
            status.mpc_modeled_steering_angle_rad = float('nan')  # Two-state model.
            if self.predictor:
                status.mpc_modeled_steering_angle_rad = self.predictor.advance(wall)
            status.mpc_yaw_rate_steering_estimate_blend = 0.
            status.mpc_solver_time_ms = self.values['time'][0].data if 'time' in self.values else float('nan')
            reason = self.input_error
            valid = not reason and self.last_input and fresh(*self.last_input, stamp.to_sec(), wall)
            for name in ('long', 'mpc'):
                item = self.values.get(name)
                valid = valid and item is not None and fresh(item[0].header.stamp.to_sec(), item[1], stamp.to_sec(), wall)
            requested = self.angle
            try:
                if not valid:
                    raise ValueError(reason or 'STALE_INPUT')
                command = self.values['mpc'][0].cmd
                if not status.active or not math.isfinite(command.linear_velocity) or command.linear_velocity <= 0.:
                    raise ValueError('CONTROLLER_REJECTED')
                if not all(math.isfinite(v) for v in (status.accel, status.brake)) or not (0 <= status.accel <= .7 and 0 <= status.brake <= 1) or (status.accel > 0 and status.brake > 0):
                    raise ValueError('INVALID_LONGITUDINAL')
                requested = command.steering_angle
                angle = bounded_angle(requested, self.angle, dt)
                status.mpc_raw_steering_rate_rad_per_sec = (requested - self.angle) / dt
                status.mpc_steering_rate_rad_per_sec = (angle - self.angle) / dt
                self.angle = angle
                status.active, status.state, status.mpc_solver_success = True, 'ACTIVE', True
            except ValueError as exc:
                status.active, status.state, status.mpc_solver_success = False, str(exc), False
                status.accel, status.brake = 0., .5
                status.mpc_steering_rate_rad_per_sec = 0.
            status.requested_steering_angle_rad = requested
            status.steering_angle_rad = self.angle
            if self.predictor:
                self.predictor.sent(wall, self.angle * self.steer_gain)
            self.pub_command.publish(ActuatorCommand(header=status.header, accel=status.accel,
                brake=status.brake, steering_angle_rad=self.angle))
            sent = ControlCommandStamped(header=status.header)
            sent.cmd.steering_angle = self.angle
            self.pub_sent.publish(sent)
            self.pub_status.publish(status)


if __name__ == '__main__':
    rospy.init_node('morai_control_adapter')
    Adapter()
    rospy.spin()
