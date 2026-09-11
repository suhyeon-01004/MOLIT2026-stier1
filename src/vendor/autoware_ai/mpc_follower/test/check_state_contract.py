#!/usr/bin/python3
"""Production MPC with isolated synthetic topics; NEVER starts UDP control sender."""
import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rospy
import yaml
from autoware_msgs.msg import Lane,Waypoint,VehicleStatus,ControlCommandStamped
from geometry_msgs.msg import PoseStamped,TwistStamped
from std_msgs.msg import Float64MultiArray

ws=Path(__file__).resolve().parents[5]
log_path=ws/'build/state_alignment_contract.log'
ns='/state_alignment_check_'+str(os.getpid())
rospy.init_node('state_alignment_contract',anonymous=True,disable_signals=True)
cfg=yaml.safe_load((ws/'src/morai_path_tracking/config/controllers/autoware_mpc.yaml').read_text())
cfg['morai_state_time_alignment']=True
rospy.set_param(ns+'/mpc',cfg)
publishers={name:rospy.Publisher(ns+'/'+name,kind,queue_size=5,tcp_nodelay=True)
    for name,kind in [('current_pose',PoseStamped),('vehicle_status',VehicleStatus),
                      ('estimate_twist',TwistStamped),('base_waypoints',Lane),('sent_control',ControlCommandStamped)]}
debug=[];commands=[]
subs=[rospy.Subscriber(ns+'/mpc/debug/debug_values',Float64MultiArray,
                      lambda m:debug.append((time.monotonic(),list(m.data))),queue_size=20),
      rospy.Subscriber(ns+'/ctrl_raw',ControlCommandStamped,
                      lambda m:commands.append((time.monotonic(),m.cmd.linear_velocity)),queue_size=20)]
lane=Lane();lane.header.frame_id='map'
for i in range(150):
    w=Waypoint();w.pose.header.frame_id='map';w.pose.pose.position.x=-5+i*.5
    w.pose.pose.orientation.w=1.;w.twist.twist.linear.x=4.;lane.waypoints.append(w)
proc=None
def publish_for(seconds,mode='valid'):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        now=rospy.Time.now();source=now-rospy.Duration(.06)
        pose=PoseStamped();pose.header.stamp=source;pose.header.frame_id='map';pose.pose.orientation.w=1.
        vehicle=VehicleStatus();vehicle.header.stamp=source;vehicle.header.frame_id='base_link';vehicle.speed=14.4
        twist=TwistStamped();twist.header=copy.deepcopy(vehicle.header)
        twist.twist.linear.x=4.;twist.twist.angular.z=.2
        lane.header.stamp=source
        if mode=='mismatch':vehicle.header.stamp=source-rospy.Duration(.02)
        if mode=='stale':
            old=now-rospy.Duration(.3)
            pose.header.stamp=vehicle.header.stamp=twist.header.stamp=lane.header.stamp=old
        if mode=='wrong_frame':twist.header.frame_id='map'
        sent=ControlCommandStamped();sent.header.stamp=now
        for name,msg in [('sent_control',sent),('current_pose',pose),('vehicle_status',vehicle),
                         ('estimate_twist',twist),('base_waypoints',lane)]:publishers[name].publish(msg)
        time.sleep(.04)
try:
    with log_path.open('w') as log:
        proc=subprocess.Popen([str(ws/'install/lib/mpc_follower/mpc_follower'),
            '__ns:='+ns,'__name:=mpc'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        publish_for(1.8)
        assert proc.poll() is None and len(debug)>10,'MPC did not accept matching generation'
        for _,d in debug[-10:]:
            age=d[23]
            assert len(d)==28 and .05<age<.15 and d[24]==age and d[25]==1.
            assert abs(d[26]-20*math.sin(.2*age))<1e-7
            assert abs(d[27]-20*(1-math.cos(.2*age)))<1e-7
            assert abs(d[6]-.2*age)<1e-7
        for mode in ('mismatch','stale','wrong_frame'):
            publish_for(.55,mode)
            recent=[v for t,v in commands if t>time.monotonic()-.15]
            assert recent and all(v==0. for v in recent),'Unsafe acceptance: '+mode
            publish_for(.6)
            assert commands[-1][1]>0.,'Did not recover matching state after '+mode
        print('PASS: same-generation pose/velocity/yaw/steer/path; exact control-time arc; mismatch/stale/frame rejection and recovery')
finally:
    if proc and proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=4)
        except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=4)
    rospy.delete_param(ns)
