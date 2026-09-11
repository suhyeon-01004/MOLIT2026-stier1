#!/usr/bin/python3
"""Standalone geometric/fallback/input contract; no simulator commands."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np
from lanelet2.core import Lanelet,LineString3d,Point3d
from map_clearance import MapGeometry,projections,line_box_distance,valid_inputs
from nav_msgs.msg import Path as PathMessage,Odometry
from geometry_msgs.msg import PoseStamped
import rospy

for window in [(10.,0.),(-1.,2.),(0.,101.),(0.,float('nan'))]:
 try:MapGeometry(None,None,qualification_window=window)
 except ValueError:pass
 else:raise AssertionError('Invalid qualification window accepted')

low=np.array([-1.,-1.]);high=-low
assert line_box_distance(np.array([[-2.,0.],[2.,0.]]),low,high)==0.
assert abs(line_box_distance(np.array([[-2.,2.],[2.,2.]]),low,high)-1)<1e-9
assert abs(line_box_distance(np.array([[2.,2.],[3.,3.]]),low,high)-np.sqrt(2))<1e-9
g=MapGeometry.__new__(MapGeometry)
g.xy=np.array([[-5.,0.],[0.,0.],[10.,0.]])
g.s=np.array([905.,910.,920.]);g.sources=['lane']*3;g.half_width=1.35
g.low=np.array([-.790,-.946]);g.high=np.array([3.845,.946])
g.wheels=np.array([[0.,.946],[0.,-.946],[3.,.946],[3.,-.946]])
left=LineString3d(1,[Point3d(10,-5,1.75,0),Point3d(11,10,1.75,0)])
right=LineString3d(2,[Point3d(12,-5,-1.75,0),Point3d(13,10,-1.75,0)])
g.lanes={'lane':Lanelet(3,left,right)};g.known={'lane':True}
g.bounds={1:(np.array([[-5.,1.75],[10.,1.75]]),.075),2:(np.array([[-5.,-1.75],[10.,-1.75]]),.075)}
r=g.calculate(g.xy,np.array([0.,0.]),0.)
assert np.allclose(r[:3],[.729,.404,.729]) and r[3]==4,r
g.known['lane']=False
r=g.calculate(g.xy,np.array([0.,0.]),0.)
assert np.allclose(r[:3],[.404,.404,.729]) and r[3]==0,r
g.bounds[1]=(np.array([[-5.,.9],[10.,.9]]),.075)
assert g.calculate(g.xy,np.array([0.,0.]),0.)[0]==-.075
g.s[:]=0;assert g.calculate(g.xy,np.array([0.,0.]),0.) is None
try:projections(np.zeros(2),np.zeros((2,2)))
except ValueError:pass
else:raise AssertionError('Degenerate path accepted')
path,odom=PathMessage(),Odometry()
path.header.frame_id=odom.header.frame_id='map';odom.child_frame_id='base_link'
path.header.stamp=odom.header.stamp=rospy.Time(10)
odom.pose.pose.orientation.w=1.
for x in [0.,1.]:
 p=PoseStamped();p.pose.position.x=x;path.poses.append(p)
valid_inputs(path,odom)
for kind in ['stamp','frame','quaternion','nan']:
 import copy
 bad=copy.deepcopy(odom)
 if kind=='stamp':bad.header.stamp=rospy.Time(11)
 if kind=='frame':bad.child_frame_id='gps_link'
 if kind=='quaternion':bad.pose.pose.orientation.w=0.
 if kind=='nan':bad.pose.pose.position.x=float('nan')
 try:valid_inputs(path,bad)
 except ValueError:pass
 else:raise AssertionError('Invalid input accepted: '+kind)
print('PASS: real-bound wheels; unknown fallback; body overlap; out-of-region fallback; generation/frame/quaternion/finite checks')
