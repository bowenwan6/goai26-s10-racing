"""python -s -B tools/s10/_heightmap/_replay/selfcheck.py"""
import struct
from collections import deque
from types import SimpleNamespace as Obj

import numpy as np
from replay import (
    CFG,
    DEFAULT_REPO,
    GRID,
    NS,
    HeightMap,
    eligible,
    pure_functions,
    surfaces,
    translation_icp,
    voxel,
    xyz_points,
)
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def main():
    expected=np.arange(12,dtype=float).reshape(4,3)
    for endian in ('>','<'):
        raw=bytearray(128)
        for i,p in enumerate(expected):struct.pack_into(endian+'ddd',raw,(i//2)*64+(i%2)*28+4,*p)
        m=Obj(width=2,height=2,point_step=28,row_step=64,data=raw,is_bigendian=endian=='>',
              fields=[Obj(name=k,offset=4+j*8,datatype=8,count=1) for j,k in enumerate('xyz')])
        assert np.allclose(xyz_points(m),expected)
        m.row_step=20
        try:xyz_points(m)
        except ValueError:pass
        else:raise AssertionError('bad row padding accepted')
    assert np.allclose(GRID[0],[-.6,-.6]) and np.allclose(GRID[9],[-.45,-.6])
    assert np.allclose(GRID[-1],[1.2,.6])
    pts=np.array([(x,y,-.4+(.3 if x>.5 else 0)) for x in np.arange(-1,2,.015) for y in np.arange(-1,1,.015)])
    surf,conf,_=surfaces(pts);hm=HeightMap();hm.update(surf,conf,NS,NS,0,0,0)
    q=hm.query(np.zeros(3),0,NS)
    assert q['V'].mean()>.8
    assert np.allclose(q['H'][q['V']&(GRID[:,0]<.4)],-.4,atol=1e-5)
    assert np.allclose(q['H'][q['V']&(GRID[:,0]>.65)],-.1,atol=1e-5)
    # Same static world points, re-expressed after base translation, yaw and height change.
    pose=np.array([.3,-.2,.1]);yaw=.4
    query=(GRID-pose[:2])@Rotation.from_euler('z',yaw).as_matrix()[:2,:2]
    moved=hm.query(pose,yaw,NS,grid=query)
    assert np.array_equal(q['V'],moved['V'])
    assert np.allclose(moved['H'][q['V']],q['H'][q['V']]-.1,atol=1e-5)
    expired=hm.query(np.zeros(3),0,NS+round((CFG['ttl']+.01)*NS))
    assert not expired['V'].any() and np.all(expired['H']==0) and np.isinf(expired['age']).all()
    future=HeightMap();future.update(surf,conf,NS,2*NS,0,0,0)
    assert not future.query(np.zeros(3),0,NS)['V'].any()
    assert not hm.query(np.zeros(3),0,NS-1)['V'].any()
    history=deque([(NS,NS,'old'),(2*NS,2*NS,'future')])
    assert eligible(history,NS,3*NS,.1)[0]=='old'
    assert eligible(history,2*NS,NS,.1)[0] is None
    assert eligible(history,NS+NS//2,2*NS,.1)[0] is None
    # A vertical/multi-layer stack cannot become the mean height of a ramp.
    stack=np.vstack([pts[abs(pts[:,0])<.02],pts[abs(pts[:,0])<.02]+[0,0,.25]])
    layered,_,_=surfaces(stack)
    assert len(layered)==0
    target,=pure_functions(DEFAULT_REPO/'artifacts/evidence/runs/s10-recording-review-20260909/reconstruct_stairs.py',
                          ['target'],dict(np=np,cKDTree=cKDTree,voxel=voxel))
    rng=np.random.default_rng(19)
    corners=np.vstack([np.c_[rng.uniform(-2,2,(1500,2)),np.zeros(1500)],
                       np.c_[np.zeros(1500),rng.uniform(-2,2,(1500,2))],
                       np.c_[rng.uniform(-2,2,1500),np.full(1500,2),rng.uniform(-2,2,1500)]])
    shift=np.array([.08,-.04,.03]);found,quality=translation_icp(corners-shift,np.zeros(3),target(corners))
    assert np.max(abs(found-shift))<.005 and quality['condition']<CFG['icp_condition']
    print('PASS: decoder endian/offset/padding; signs/order; translation/yaw; unknown/expiry; future exclusion; multilayer rejection; ICP translation')


if __name__=='__main__':main()
