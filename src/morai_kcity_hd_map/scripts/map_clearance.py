#!/usr/bin/python3
"""Experimental S-curve guard: source-backed wheels, virtual fallback, full body.

PointStamped: x=hybrid clearance [m], y=1 valid / 0 original fallback, z=compute ms.
The timestamp is the EXACT local-path/odometry generation, not publication time.
This sampled pose check is not a predictive swept-volume safety guarantee.
"""
import json
import math
from pathlib import Path
import time

import numpy as np
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import inside
import rddf_lanelet_route_node as builder


def projections(point, line):
    delta=line[1:]-line[:-1]
    norm=np.sum(delta*delta,axis=1)
    good=norm>1e-12
    if not np.any(good):raise ValueError('Degenerate boundary/path')
    delta,start,norm=delta[good],line[:-1][good],norm[good]
    ratio=np.clip(np.sum((point-start)*delta,axis=1)/norm,0.,1.)
    residual=point-start-ratio[:,None]*delta
    return np.linalg.norm(residual,axis=1), (delta[:,0]*residual[:,1]-delta[:,1]*residual[:,0])/np.sqrt(norm)


def distance_to_line(point,line):
    return float(projections(point,line)[0].min())


def line_box_distance(line,low,high):
    # Same segment/rectangle check used in the recorded full-body audit.
    corners=np.array([[low[0],low[1]],[high[0],low[1]],[high[0],high[1]],[low[0],high[1]]])
    best=float(np.min(np.linalg.norm(np.maximum(np.maximum(low-line,line-high),0.),axis=1)))
    for a,b in zip(line[:-1],line[1:]):
        d=b-a;enter,leave=0.,1.
        for j in (0,1):
            if abs(d[j])<1e-12:
                if not low[j]<=a[j]<=high[j]:enter,leave=1.,0.;break
            else:
                u,v=sorted(((low[j]-a[j])/d[j],(high[j]-a[j])/d[j]))
                enter,leave=max(enter,u),min(leave,v)
        if enter<=leave:return 0.
        if np.dot(d,d)>1e-12:
            t=np.clip((corners-a).dot(d)/np.dot(d,d),0.,1.)
            best=min(best,float(np.min(np.linalg.norm(corners-a-t[:,None]*d,axis=1))))
    return best


class MapGeometry:
    def __init__(self,package,specs,candidate='',half_width=1.35,qualification_window=(865.,955.)):
        import yaml
        lo,hi=qualification_window
        if not all(math.isfinite(v) for v in (lo,hi)) or not 0<=lo<hi or hi-lo>100:
            raise ValueError('Invalid boundary qualification window')
        dims=yaml.safe_load(Path(specs).read_text())['dimensions']
        self.half_width=float(half_width)
        width,wheelbase=float(dims['width_m']),float(dims['wheelbase_m'])
        self.low=np.array([-float(dims['rear_overhang_m']),-width/2])
        self.high=np.array([wheelbase+float(dims['front_overhang_m']),width/2])
        self.wheels=np.array([[0.,width/2],[0.,-width/2],[wheelbase,width/2],[wheelbase,-width/2]])
        if not np.all(np.isfinite(np.r_[self.low,self.high,self.half_width])) or not .5<width<3 or not 1<wheelbase<5 or not width/2<self.half_width<3:
            raise ValueError('Invalid vehicle/corridor dimensions')
        links=builder.load_links(package/'raw_mgeo')
        _,self.lanes=builder.load_lanelet2_map(package/'map/lanelet2_map.osm',links)
        route,_,_,stats=builder.build_route(package/'map/2026_molit_comp_global_path.txt',package/'map/lanelet2_map.osm',package/'raw_mgeo')
        self.xy=np.array(builder.apply_local_candidate(route,candidate))[:,:2]
        self.sources=stats['source_by_point']
        self.s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(self.xy,axis=0),axis=1))]
        self.bounds={};self.known={};self.errors={}
        raw={v['idx']:np.array(v['points'])[:,:2] for v in json.loads((package/'raw_mgeo/lane_boundary_set.json').read_text())}
        # Offline audits may select another local region; runtime guard remains S-only.
        for key in {self.sources[i] for i in np.flatnonzero((self.s>=lo)&(self.s<=hi))}:
            self.known[key]=True
            for b in (self.lanes[key].leftBound,self.lanes[key].rightBound):
                attr=dict(b.attributes)
                if attr.get('type') in ('virtual','',None):
                    self.known[key]=False;continue
                line=np.array([(p.x,p.y) for p in b]);paint=float(attr.get('width','0.15'))/2
                if len(line)<2 or not np.all(np.isfinite(line)):raise ValueError('Invalid boundary coordinates')
                originals=[raw[k] for k in attr.get('source_boundary_ids',attr.get('source_id','')).split(';')]
                error=max(min(distance_to_line(p,old) for old in originals) for p in line)
                if error>1e-4 or not 0<paint<.25:raise ValueError('Unverified map boundary '+str(b.id))
                self.bounds[b.id]=(line,paint);self.errors[b.id]=error

    def calculate(self,local_xy,pos,yaw):
        nearest=int(np.argmin(np.sum((self.xy-local_xy[0])**2,axis=1)))
        if np.linalg.norm(self.xy[nearest]-local_xy[0])>.05:raise ValueError('Local/global route mismatch')
        if not 885<=self.s[nearest]<=935:return None
        ids={self.sources[i%len(self.sources)] for i in range(nearest-12,nearest+13)}
        c,s=math.cos(yaw),math.sin(yaw);rot=np.array([[c,-s],[s,c]])
        wheel_map=self.wheels.dot(rot.T)+pos
        margins=[];virtual=[];mapped=0
        for point in wheel_map:
            distance,offset=projections(point,local_xy)
            original=self.half_width-abs(offset[np.argmin(distance)])
            virtual.append(original)
            candidates=[]
            for key in ids:
                if self.known.get(key,False) and inside(self.lanes[key],BasicPoint2d(*point)):
                    candidates.append(min(distance_to_line(point,self.bounds[b.id][0])-self.bounds[b.id][1]
                        for b in (self.lanes[key].leftBound,self.lanes[key].rightBound)))
            margins.append(max(candidates) if candidates else original)
            mapped+=bool(candidates)
        bounds={b.id for key in ids for b in (self.lanes[key].leftBound,self.lanes[key].rightBound) if b.id in self.bounds}
        body=min((line_box_distance((self.bounds[b][0]-pos).dot(rot),self.low,self.high)-self.bounds[b][1] for b in bounds),default=float('inf'))
        # Actual paint can only tighten the per-wheel mixed guard; overhang is not omitted.
        return min(min(margins),body),min(virtual),body,mapped


def valid_inputs(path,odom):
    p,q=odom.pose.pose.position,odom.pose.pose.orientation
    if path.header.stamp!=odom.header.stamp or not path.header.stamp.to_nsec():raise ValueError('Generation mismatch')
    if path.header.frame_id!='map' or odom.header.frame_id!='map' or odom.child_frame_id!='base_link':raise ValueError('Frame mismatch')
    xy=np.array([(v.pose.position.x,v.pose.position.y) for v in path.poses])
    if len(xy)<2 or not np.all(np.isfinite(xy)) or not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)):
        raise ValueError('Nonfinite/short inputs')
    if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01:raise ValueError('Invalid quaternion')
    return xy,np.array([p.x,p.y]),math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def main():
    import rospy,rospkg,message_filters
    from nav_msgs.msg import Odometry,Path as PathMessage
    from geometry_msgs.msg import PointStamped
    rospy.init_node('hd_map_clearance')
    packages=rospkg.RosPack()
    geometry=MapGeometry(Path(packages.get_path('morai_kcity_hd_map')),
        Path(packages.get_path('ioniq5_description'))/'config/vehicle/vehicle_specs.yaml',
        rospy.get_param('~route_candidate_file',''),rospy.get_param('/path_tracking_controller_node/lane_half_width_m'))
    if abs(geometry.high[1]*2-rospy.get_param('/path_tracking_controller_node/vehicle_width_m'))>1e-6 or abs(geometry.wheels[2,0]-rospy.get_param('/path_tracking_controller_node/wheelbase_m'))>1e-6:
        raise ValueError('Controller/vehicle geometry mismatch')
    pub=rospy.Publisher('/control/internal/map_clearance',PointStamped,queue_size=1)
    debug=rospy.Publisher('/control/internal/map_clearance_details',PointStamped,queue_size=1)
    route_matches=[False]
    def route(msg):
        xy=np.array([(p.pose.position.x,p.pose.position.y) for p in msg.poses])
        route_matches[0]=msg.header.frame_id=='map' and xy.shape==geometry.xy.shape and np.allclose(xy,geometry.xy,atol=1e-5,rtol=0)
    def inputs(path,odom):
        start=time.monotonic();output=PointStamped(header=odom.header)
        try:
            if not route_matches[0]:raise ValueError('Unmatched global route')
            result=geometry.calculate(*valid_inputs(path,odom))
            if result is not None:
                margin,virtual,body,mapped=result
                if not math.isfinite(margin) or abs(margin)>=10:raise ValueError('Invalid margin')
                output.point.x,output.point.y=margin,1.
                detail=PointStamped(header=odom.header)
                detail.point.x,detail.point.y,detail.point.z=virtual,body,mapped
                debug.publish(detail)
        except (ValueError,KeyError) as exc:
            rospy.logwarn_throttle(10.,'Map guard uses original fallback: %s',str(exc))
        output.point.z=(time.monotonic()-start)*1000
        pub.publish(output)
    global_sub=rospy.Subscriber('/global_path',PathMessage,route,queue_size=1)
    path_sub=message_filters.Subscriber('/local_path',PathMessage,queue_size=5)
    odom_sub=message_filters.Subscriber('/localization/odometry',Odometry,queue_size=5)
    sync=message_filters.TimeSynchronizer([path_sub,odom_sub],10)
    sync.registerCallback(inputs)
    rospy.spin()


if __name__=='__main__':main()
