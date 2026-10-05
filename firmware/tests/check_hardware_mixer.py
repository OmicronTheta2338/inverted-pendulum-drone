#!/usr/bin/env python3
"""Check emitted Lua against force/torque geometry, including one-way motor clipping.

This validates level, low-output allocation signs. It is not a flight dynamics test.
"""
import json
import math
from pathlib import Path
import re

root=Path(__file__).resolve().parents[1]
layout=json.loads((root/'hardware-layout.json').read_text())
rows=[]
for match in re.finditer(r'Motors_6DoF:add_motor\((.*?)\)',(root/'scripts/base_drone.lua').read_text()):
    args=[x.strip() for x in match[1].split(',')]
    assert int(args[0])==len(rows) and int(args[-1])==len(rows)+1
    assert args[-2]=='false'
    rows.append([float(v) for v in args[1:7]])
assert len(rows)==8

wrenches=[]
for motor in layout['motors']:
    x,y,z=motor['r_frd_m']; nx,ny,nz=motor['thrust_unit_frd']
    if y is None:
        assert nx == 0 and nz == 0
        y = 0  # y drops out of r cross F for purely lateral force.
    spin=1 if motor['rotation_viewed_from_shaft_end']=='CCW' else -1
    sx,sy,sz=motor['shaft_outward_unit_frd']; k=layout['torque_per_thrust_m_assumed']
    wrenches.append([nx,ny,nz,y*nz-z*ny-spin*k*sx,
                     z*nx-x*nz-spin*k*sy,x*ny-y*nx-spin*k*sz])

# At level, upstream performs rotational allocation, then horizontal allocation.
# These small demands do not trigger its global upper/lower [-1,1] scaling.
# Non-reversible motors finally clamp negative thrust to zero.
def allocate(command):
    rot_roll,rot_pitch,rot_yaw,up,forward,right=command
    # Match level upstream scaling of horizontal demand by collective throttle.
    scaled=[rot_roll,rot_pitch,rot_yaw,up,forward*up,right*up]
    raw=[sum(a*b for a,b in zip(row,scaled)) for row in rows]
    assert all(-1 <= t <= 1 for t in raw), 'Test exceeded the scope of this low-output model'
    return [max(0,t) for t in raw]
def wrench(outputs):
    return [sum(t*w[j] for t,w in zip(outputs,wrenches)) for j in range(6)]
neutral=wrench(allocate([0,0,0,.2,0,0]))
assert neutral[2]<0 and all(abs(neutral[j])<1e-8 for j in (0,1,3,4,5))
for axis,component in [(0,3),(1,4),(2,5),(5,1)]:
    for sign in (-1,1):
        cmd=[0,0,0,.2,0,0]; cmd[axis]=sign*.03
        out=allocate(cmd); actual=wrench(out)
        assert sign*(actual[component]-neutral[component])>0,(axis,sign,out,actual)
        for j in (0,1,3,4,5):
            if j!=component:
                assert abs(actual[j]-neutral[j])<1e-7,(axis,sign,'cross-axis torque/force',j,actual)
        if axis==5:
            active=[i+1 for i,t in enumerate(out) if i%2==1 and t>0]
            assert active==([4,6] if sign>0 else [2,8]),active
# Lift request must not drive lateral motors.
assert [i+1 for i,t in enumerate(allocate([0,0,0,.2,0,0])) if t>0]==[1,3,5,7]
# Collective must balance the reported forward CoM offset, independent of demand.
for collective in (.1,.2,.5,.8):
    out=allocate([0,0,0,collective,0,0]); actual=wrench(out)
    assert abs(actual[4])<1e-10
    assert abs((out[0]+out[2])/sum(out) - .38/.74)<1e-10
# Combinations of yaw and translation retain yaw/lateral authority, but one-way
# motor clipping creates a bounded prop-reaction pitch residual. Do not call this decoupled.
for lateral in (-.1,-.02,.02,.1):
    for yaw in (-.03,.03):
        actual=wrench(allocate([0,0,yaw,.3,0,lateral]))
        assert abs(actual[1] - 2*.3*lateral)<1e-10
        expected_yaw=(.255 + 4*.01*(.01/.1275))*yaw
        assert abs(actual[5]-expected_yaw)<1e-10
        assert abs(actual[4]) <= 2*.01*min(abs(.3*lateral),abs(yaw))+1e-10
        assert abs(actual[3])<1e-10
print('PASS: measured geometry, CoM-balanced collective, pure-axis signs/cancellation, mixed lateral/yaw bound')
print('Scope: approximate measured geometry and assumed prop torque k; level low-output checks, no flight validation')
