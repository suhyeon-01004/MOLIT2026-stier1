#!/usr/bin/env python3
"""One simulator lap, verified Manual/I/P reset, bounded drive and cleanup."""
import json
import hashlib
import math
import os
import pathlib
import signal
import socket
import struct
import subprocess
import sys
import time
import yaml

def is_parked(samples):
    return len(samples)>=10 and all(s['mode']==1 and s['gear']==1 and abs(s['speed_kph'])<.05
                                   for s in samples[-10:])

if sys.argv[1:]==['--check-parking']:
    good={'mode':1,'gear':1,'speed_kph':0.}
    assert is_parked([good]*10)
    assert not is_parked([])
    for bad in [dict(good,mode=0),dict(good,gear=4),dict(good,speed_kph=.4),dict(good,speed_kph=float('nan'))]:
        assert not is_parked([good]*9+[bad])
    print('PASS: Manual/P/standstill requires 10 consecutive valid samples')
    sys.exit(0)

ROOT = pathlib.Path(__file__).resolve().parent
high_speed_kph = float(os.environ['TRIAL_HIGH_SPEED_KPH']) if os.environ.get('TRIAL_HIGH_SPEED_KPH') else None
if high_speed_kph is not None:
    assert math.isfinite(high_speed_kph) and 60. <= high_speed_kph <= 100., 'High-speed target must be 60..100 km/h'
    assert os.environ.get('TRIAL_AUTOWARE_CONFIG'), 'Map-speed trial requires the Autoware comparison'
    assert not os.environ.get('TRIAL_EXCITATION'), 'No excitation during a high-speed trial'
maximum_trial_speed_mps = (high_speed_kph + 2.) / 3.6 if high_speed_kph is not None else 17.
label, config = sys.argv[1:3]
out = ROOT / label
out.mkdir(exist_ok=False)
processes = []
metadata = {'label': label, 'config': config, 'reset_sequence': 'Manual -> I -> P'}
metadata['high_speed_target_kph'] = high_speed_kph
metadata['maximum_trial_speed_mps'] = maximum_trial_speed_mps
workspace = pathlib.Path('/home/stier/molit-2026-stier1-suhyeon')
snapshot_files = [
    'src/ioniq5_description/config/vehicle/vehicle_specs.yaml',
    'autoware_mpc_compare/install/lib/mpc_follower/mpc_follower',
    'autoware_mpc_compare/src/mpc_follower/src/mpc_follower_core.cpp',
    'autoware_mpc_compare/src/mpc_follower/src/mpc_utils.cpp',
    'autoware_mpc_compare/src/mpc_follower/include/mpc_follower/mpc_utils.h',
    'autoware_mpc_compare/src/mpc_follower/include/mpc_follower/mpc_follower_core.h',
    'autoware_mpc_compare/src/mpc_follower/src/vehicle_model/vehicle_model_bicycle_kinematics.cpp',
    'autoware_mpc_compare/src/mpc_follower/include/mpc_follower/curvature_weight_blend.h',
    'autoware_mpc_compare/src/mpc_follower/include/mpc_follower/prediction_length.h',
    'autoware_mpc_compare/src/mpc_follower/src/qp_solver/qp_solver_qpoases.cpp',
    'autoware_mpc_compare/src/mpc_follower/include/mpc_follower/qp_solver/qp_solver_qpoases.h',
    'autoware_mpc_compare/install/lib/morai_autoware_compare/adapter.py',
    'autoware_mpc_compare/install/lib/morai_autoware_compare/steering_prediction.py',
    'install/lib/morai_path_tracking/path_tracking_controller_node',
    'install/lib/libmorai_longitudinal_mpc.so',
    'install/lib/libmorai_curvature_speed_planner.so',
    'install/lib/morai_kcity_hd_map/rddf_lanelet_route_node.py',
    'src/morai_kcity_hd_map/map/lanelet2_map.osm',
]
metadata['candidate_code_sha256'] = {
    name: hashlib.sha256((workspace/name).read_bytes()).hexdigest()
    for name in snapshot_files if (workspace/name).is_file()
}
trial_seconds = float(os.environ.get('TRIAL_SECONDS', '260'))
assert 10. <= trial_seconds <= 260.
metadata['planned_seconds'] = trial_seconds
metadata['rviz_enabled'] = os.environ.get('TRIAL_RVIZ') == '1'

def launch(args, name):
    handle = open(str(out / (name + '.log')), 'w')
    proc = subprocess.Popen(args, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append((proc, handle))
    return proc

def stop(proc):
    if proc is not None and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)

def manual_packet():
    packet = bytearray(55)
    packet[:14] = b'#MoraiCtrlCmd$'
    struct.pack_into('<I', packet, 14, 23)
    packet[30:33] = bytes((1, 1, 1))
    struct.pack_into('<fffff', packet, 33, 0., 0., 0., 1., 0.)
    packet[53:] = b'\r\n'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for _ in range(5):
            sock.sendto(packet, ('127.0.0.1', 9093))
            time.sleep(.04)

def read_raw_status(seconds=1.0):
    samples = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('0.0.0.0', 9094))
        sock.settimeout(2)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            packet = sock.recv(2048)
            if len(packet) == 181 and packet.startswith(b'#MoraiInfo$'):
                samples.append({'mode': packet[35], 'gear': packet[36],
                                'heading_deg': struct.unpack_from('<f', packet, 97)[0],
                                'speed_kph': struct.unpack_from('<f', packet, 101)[0]})
    if not samples:
        raise RuntimeError('No Competition Vehicle Status packets')
    return samples

def key(key_name):
    subprocess.run(['xdotool', 'windowactivate', '--sync', '104857611'], check=True)
    subprocess.run(['xdotool', 'key', '--clearmodifiers', key_name], check=True)

stack = sender = recorder = None
try:
    if high_speed_kph is not None:
        running = subprocess.check_output(['rosnode', 'list'], text=True).splitlines()
        conflicting = {'/control_sender_node', '/path_tracking_controller_node', '/autoware_adapter', '/autoware_mpc'} & set(running)
        assert not conflicting, 'Stop the existing controller/sender before this trial: ' + repr(conflicting)
    for candidate_config in (config, os.environ.get('TRIAL_AUTOWARE_CONFIG'), os.environ.get('TRIAL_LOCALIZATION_CONFIG')):
        if candidate_config:
            parsed = yaml.safe_load(pathlib.Path(candidate_config).read_text())
            assert isinstance(parsed, dict), 'Configuration must be a YAML mapping'
    manual_packet()
    mode = read_raw_status(.4)[-1]
    assert mode['mode'] == 1, mode
    key('i')
    time.sleep(.5)
    key('p')
    reset = read_raw_status(1)
    assert is_parked(reset), reset[-10:]
    metadata['reset_status'] = reset[-1]
    print('RESET VERIFIED ' + json.dumps(reset[-1]), flush=True)
    # Trials start from configuration files, never stale preceding ROS params.
    for namespace in ('/path_tracking_controller_node', '/autoware_mpc', '/autoware_adapter', '/rddf_lanelet_route'):
        subprocess.run(['rosparam', 'delete', namespace], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    import rospy
    from nav_msgs.msg import Odometry
    from morai_path_tracking.msg import ControllerStatus
    from morai_path_tracking.msg import RouteSpeedLimit
    from morai_udp_bridge.msg import CompetitionVehicleStatus
    from morai_udp_bridge.msg import ActuatorCommand
    rospy.init_node('offset_free_trial_monitor', anonymous=True, disable_signals=True)
    latest = {}
    def receive(name, msg):
        latest[name] = (time.monotonic(), msg)
    subscriptions = [rospy.Subscriber(topic, kind, lambda msg, n=name: receive(n, msg), queue_size=1)
                     for name, topic, kind in [('odom', '/localization/odometry', Odometry),
                                              ('status', '/control/controller_status', ControllerStatus),
                                              ('vehicle', '/vehicle/competition_status', CompetitionVehicleStatus)]]
    if high_speed_kph is not None:
        subscriptions.append(rospy.Subscriber('/route_speed_limit', RouteSpeedLimit,
                             lambda msg: receive('map_speed', msg), queue_size=1))
    stack_args = ['roslaunch', 'morai_kcity_hd_map', 'kcity_autonomous.launch',
                  'send_control:=false', 'rviz:=' + str(metadata['rviz_enabled']).lower(), 'controller_config:=' + config]
    if os.environ.get('TRIAL_AUTOWARE_CONFIG'):
        stack_args[1:3] = ['morai_autoware_compare', 'compare.launch']
        metadata['autoware_config'] = os.environ['TRIAL_AUTOWARE_CONFIG']
        stack_args.append('autoware_config:=' + metadata['autoware_config'])
        if high_speed_kph is not None:
            stack_args[2] = 'map_speed.launch'
            stack_args.append('high_speed_target_kph:=' + str(high_speed_kph))
        metadata['use_map_guard'] = os.environ.get('TRIAL_MAP_GUARD') == '1'
        if metadata['use_map_guard']:
            stack_args.append('use_map_guard:=true')
    if os.environ.get('TRIAL_LOCALIZATION_CONFIG'):
        metadata['localization_config'] = os.environ['TRIAL_LOCALIZATION_CONFIG']
        stack_args.append('localization_config:=' + metadata['localization_config'])
    stack = launch(stack_args, 'stack')
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        if stack.poll() is not None:
            raise RuntimeError('Stack exited')
        if all(n in latest and time.monotonic() - latest[n][0] < .3 for n in ('odom', 'status', 'vehicle')) and latest['status'][1].active:
            break
        time.sleep(.1)
    else:
        raise RuntimeError('Stack not ready')
    vehicle = latest['vehicle'][1]
    point = latest['odom'][1].pose.pose.position
    initial = [point.x, point.y]
    metadata['initial'] = {'x': point.x, 'y': point.y, 'mode': vehicle.control_mode,
                           'gear': vehicle.gear, 'speed_mps': vehicle.velocity_x_mps,
                           'heading_error_rad': latest['status'][1].heading_error_rad}
    metadata['resolved_position_filter_time_constant_sec'] = rospy.get_param('/localization_fusion/position_filter_time_constant_sec', 0.0)
    subprocess.run(['rosparam','dump',str(out/'resolved_controller.yaml'),'/path_tracking_controller_node'],check=True)
    subprocess.run(['rosparam','dump',str(out/'resolved_localization.yaml'),'/localization_fusion'],check=True)
    if high_speed_kph is not None:
        assert rospy.get_param('/path_tracking_controller_node/use_map_speed_limits') is True
        assert abs(rospy.get_param('/path_tracking_controller_node/curve_approach_deceleration_mps2') - 1.7) < 1e-9
        assert abs(rospy.get_param('/path_tracking_controller_node/maximum_speed_kph') - high_speed_kph) < 1e-9
        assert abs(rospy.get_param('/rddf_lanelet_route/high_speed_target_kph') - high_speed_kph) < 1e-9
        subprocess.run(['rosparam','dump',str(out/'resolved_route.yaml'),'/rddf_lanelet_route'],check=True)
        subprocess.run(['rosparam','dump',str(out/'resolved_adapter.yaml'),'/autoware_adapter'],check=True)
    if metadata.get('autoware_config'):
        subprocess.run(['rosparam','dump',str(out/'resolved_autoware.yaml'),'/autoware_mpc'],check=True)
        assert rospy.get_param('/autoware_mpc/vehicle_model_type') in ('kinematics_no_delay', 'kinematics')
        assert rospy.get_param('/autoware_mpc/enable_path_smoothing') is False
        if rospy.get_param('/autoware_mpc/morai_weight_front_point_error',0.)>0.:
            dims=yaml.safe_load((workspace/'src/ioniq5_description/config/vehicle/vehicle_specs.yaml').read_text())['dimensions']
            assert abs(rospy.get_param('/autoware_mpc/vehicle_model_wheelbase')-dims['wheelbase_m'])<1e-9
            assert abs(rospy.get_param('/autoware_mpc/morai_front_overhang_m')-dims['front_overhang_m'])<1e-9
            metadata['front_body_tracking_distance_m']=dims['wheelbase_m']+dims['front_overhang_m']
    assert vehicle.control_mode == 1 and vehicle.gear == 1 and abs(vehicle.velocity_x_mps) < .02, metadata['initial']
    assert math.hypot(point.x + 131.626047045, point.y + 428.279280173) < .15, metadata['initial']
    assert abs(latest['status'][1].heading_error_rad) < math.radians(3), metadata['initial']
    assert rospy.get_param('/path_tracking_controller_node/mpc_swept_path_compensation_enabled', None) is False, 'Offset parameter not false'
    topics = ['/control/controller_status', '/localization/odometry', '/local_path', '/global_path',
              '/control/actuator_command', '/vehicle/competition_status', '/localization/status_text',
              '/diagnostics', '/sensors/gps/fix', '/sensors/imu/data', '/control/mpc_compensated_path',
              '/localization/gps/local_point', '/localization/imu/data', '/validation/actuator_command']
    topics += ['/comparison/custom_status', '/comparison/custom_command', '/comparison/ctrl_raw', '/comparison/sent_control',
               '/route_speed_limit',
               '/comparison/map_clearance',
               '/comparison/waypoints', '/comparison/vehicle_status_estimated', '/comparison/pose',
               '/autoware_mpc/debug/debug_values', '/autoware_mpc/debug/mpc_calc_time']
    recorder = launch(['rosbag', 'record', '--buffsize=256', '-O', str(out / 'raw.bag')] + topics, 'recorder')
    time.sleep(2)
    metadata['drive_start_ros'] = rospy.Time.now().to_sec()
    excitation = bool(os.environ.get('TRIAL_EXCITATION'))
    sender_args = ['roslaunch', 'morai_udp_bridge', 'control_sender.launch']
    if excitation:
        sender_args.append('config:=' + str(ROOT / 'sysid_sender.yaml'))
        excitation_publisher = rospy.Publisher('/validation/actuator_command', ActuatorCommand, queue_size=1)
        metadata['excitation_sequence_deg'] = [[0,4,0],[4,5.2,3],[5.2,7.6,-3],[7.6,8.8,3],[8.8,11,0]]
    sender = launch(sender_args, 'sender')
    started = time.monotonic()
    previous = initial
    distance = 0.
    next_print = started
    while time.monotonic() - started < trial_seconds:
        now = time.monotonic()
        if any(now - latest[n][0] > 1 for n in ('odom', 'status', 'vehicle')):
            raise RuntimeError('Stale monitor input')
        if any(p.poll() is not None for p in (stack, sender, recorder)):
            raise RuntimeError('A trial process exited')
        status, vehicle = latest['status'][1], latest['vehicle'][1]
        if excitation:
            elapsed=now-started
            angle=0. if elapsed<4 else 3. if elapsed<5.2 else -3. if elapsed<7.6 else 3. if elapsed<8.8 else 0.
            command=ActuatorCommand()
            command.header.stamp=rospy.Time.now()
            command.accel=status.accel
            command.brake=status.brake
            command.steering_angle_rad=math.radians(angle)
            excitation_publisher.publish(command)
            if elapsed>=11:
                metadata['excitation_complete']=True
                break
        point = latest['odom'][1].pose.pose.position
        distance += math.hypot(point.x - previous[0], point.y - previous[1])
        previous = [point.x, point.y]
        if high_speed_kph is not None:
            if 'map_speed' not in latest or now - latest['map_speed'][0] > .3:
                raise RuntimeError('Stale map-speed monitor input')
            if abs(vehicle.velocity_x_mps) > latest['map_speed'][1].current_limit_mps + 2. / 3.6:
                raise RuntimeError('Map-speed overshoot >2 km/h (abort margin, not a permitted speed)')
        if now - started > 4 and (not status.active or abs(status.cross_track_error_m) > .8 or abs(vehicle.velocity_x_mps) > maximum_trial_speed_mps):
            raise RuntimeError('Trial guard: active={} cte={} speed={}'.format(status.active, status.cross_track_error_m, vehicle.velocity_x_mps))
        if now >= next_print:
            print('DRIVE {:6.1f}s {:7.1f}m v={:4.1f}kph cte={:+.3f} yaw={:+.2f}deg accel={:.3f} brake={:.3f}'.format(
                now-started, distance, vehicle.velocity_x_mps*3.6, status.cross_track_error_m,
                math.degrees(status.heading_error_rad), status.accel, status.brake), flush=True)
            next_print += 10
        if distance > 1900 and math.hypot(point.x - initial[0], point.y - initial[1]) < 5:
            metadata['lap_complete'] = True
            break
        time.sleep(.05)
    metadata['drive_end_ros'] = rospy.Time.now().to_sec()
    metadata['drive_duration_sec'] = time.monotonic() - started
    metadata['distance_m'] = distance
    metadata['segment_complete'] = trial_seconds < 260 and time.monotonic()-started >= trial_seconds
    if not (metadata.get('lap_complete') or metadata.get('excitation_complete') or metadata['segment_complete']):
        raise RuntimeError('Lap timeout')
    print('TRIAL COMPLETE ' + json.dumps(metadata), flush=True)
except BaseException as exc:
    metadata['error'] = repr(exc)
    if 'drive_start_ros' in metadata and 'drive_end_ros' not in metadata:
        metadata['drive_end_ros'] = rospy.Time.now().to_sec()
        metadata['drive_duration_sec'] = time.monotonic() - started
        metadata['distance_m'] = distance
    print('TRIAL ERROR ' + repr(exc), flush=True)
    raise
finally:
    # Remove the controller first: the existing sender watchdog then brakes.
    stop(stack)
    if sender is not None:
        try:
            for _ in range(12):
                samples = read_raw_status(.5)
                if abs(samples[-1]['speed_kph']) < .05:
                    break
            else:
                raise RuntimeError('Vehicle did not stop')
            metadata['stopped_status'] = samples[-1]
            stop(sender)
            manual_packet()
            key('p')
            parked=read_raw_status(.5)
            metadata['parked_status'] = parked[-1]
            assert is_parked(parked), 'Final Manual/P/standstill not verified: '+repr(parked[-10:])
            metadata['parking_sequence_verified']=True
        except Exception as exc:
            metadata['cleanup_error'] = repr(exc)
            print('CLEANUP ERROR ' + repr(exc), flush=True)
            # If parking failed after the sender was stopped, restore its
            # existing watchdog braking; never silently claim a parked finish.
            if sender is not None and sender.poll() is not None:
                try:
                    sender=launch(['roslaunch','morai_udp_bridge','control_sender.launch'],'cleanup_watchdog')
                    metadata['cleanup_watchdog_restarted']=True
                except Exception as restart_error:
                    metadata['cleanup_watchdog_restart_error']=repr(restart_error)
            # Preserve watchdog braking if stationary/parked state is unverified.
            sender = None
    stop(recorder)
    for proc, handle in processes:
        handle.close()
    (out / 'run.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print('SAVED ' + str(out / 'run.json'), flush=True)
