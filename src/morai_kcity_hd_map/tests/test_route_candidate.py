#!/usr/bin/python3
"""Runnable opt-in route replacement contract; no ROS node/vehicle control."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
from copy import deepcopy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from rddf_lanelet_route_node import apply_local_candidate

route = [(0.,0.,1.), (1.,0.,2.), (2.,0.,3.), (0.,0.,1.)]
assert apply_local_candidate(route, '') is route
payload = {'baseline_xy_sha256':hashlib.sha256(b''.join(struct.pack('<dd',*p[:2]) for p in route)).hexdigest(),
           'route_window_m':[.5,1.5], 'maximum_displacement_m':.2,
           'changes':[{'index':1,'x':1.,'y':.1}]}
with tempfile.TemporaryDirectory() as folder:
    path=Path(folder)/'candidate.json'
    path.write_text(json.dumps(payload))
    changed=apply_local_candidate(route,path)
    assert changed == [route[0],(1.,.1,2.),route[2],route[3]]
    assert route[1] == (1.,0.,2.)
    multi=deepcopy(payload)
    del multi['route_window_m']
    multi['route_windows_m']=[[.5,1.5],[1.6,2.5]]
    multi['changes'].append({'index':2,'x':2.,'y':.1})
    path.write_text(json.dumps(multi))
    assert apply_local_candidate(route,path)==[route[0],(1.,.1,2.),(2.,.1,3.),route[3]]
    triple=deepcopy(multi)
    triple['route_windows_m'].append([2.6,3.5])
    triple['changes'].append({'index':3,'x':3.,'y':.1})
    longer=route[:3]+[(3.,0.,4.),route[-1]]
    triple['baseline_xy_sha256']=hashlib.sha256(b''.join(struct.pack('<dd',*p[:2]) for p in longer)).hexdigest()
    path.write_text(json.dumps(triple))
    assert apply_local_candidate(longer,path)==[longer[0],(1.,.1,2.),(2.,.1,3.),(3.,.1,4.),longer[-1]]
    six=deepcopy(triple)
    six_route=[(float(i),0.,float(i+1)) for i in range(8)]+[route[0]]
    six['baseline_xy_sha256']=hashlib.sha256(b''.join(struct.pack('<dd',*p[:2]) for p in six_route)).hexdigest()
    six['route_windows_m']=[[i-.4,i+.4] for i in range(1,7)]
    six['changes']=[{'index':i,'x':float(i),'y':.1} for i in range(1,7)]
    path.write_text(json.dumps(six))
    expected=list(six_route)
    for i in range(1,7):expected[i]=(float(i),.1,float(i+1))
    assert apply_local_candidate(six_route,path)==expected
    for kind in ('hash','outside','large','nan','duplicate','endpoint','empty','bool_index','both','overlap','gap','too_many','long_window','empty_windows','bad_window'):
        bad=deepcopy(payload)
        if kind=='hash':bad['baseline_xy_sha256']='invalid'
        elif kind=='outside':bad['route_window_m']=[2.,3.]
        elif kind=='large':bad['changes'][0]['y']=.3
        elif kind=='nan':bad['changes'][0]['x']=float('nan')
        elif kind=='duplicate':bad['changes']*=2
        elif kind=='endpoint':bad['changes'][0]['index']=0
        elif kind=='empty':bad['changes']=[]
        elif kind=='bool_index':bad['changes'][0]['index']=True
        elif kind=='both':bad['route_windows_m']=[[.5,1.5]]
        elif kind in ('overlap','gap','too_many','long_window','empty_windows','bad_window'):
            bad=deepcopy(multi)
            bad['route_windows_m']={'overlap':[[.5,1.5],[1.4,2.5]],'gap':[[.1,.5],[1.5,2.5]],
                'too_many':[[i,i+.1] for i in range(7)],'long_window':[[0.,51.]],
                'empty_windows':[],'bad_window':[[.5]]}[kind]
        path.write_text(json.dumps(bad))
        try:apply_local_candidate(route,path)
        except ValueError:pass
        else:raise AssertionError('Accepted invalid candidate: '+kind)
print('PASS: default/legacy/two/three/six disjoint windows; Z preserved; 15 invalid cases rejected')
