"""Build bounded terrain proxies and held-out scan checks for soft motion references.
Run python -s build_batch_terrain.py --job NAME. Mesh is not a certified contact model.
"""
import argparse
import json
import numpy as np
from scipy.spatial import cKDTree
from batch_match_stairs import BATCH, CONFIG

def grid(points,size=.08):
    origin=np.floor(points[:,:2].min(axis=0)/size)*size
    ids=np.floor((points[:,:2]-origin)/size).astype(int)
    shape=tuple(ids.max(axis=0)+1);heights=np.full(shape,np.nan);count=np.zeros(shape,dtype=np.int32)
    linear=np.ravel_multi_index(ids.T,shape);order=np.argsort(linear);keys,starts=np.unique(linear[order],return_index=True)
    for key,a,b in zip(keys,starts,np.r_[starts[1:],len(order)]):
        zz=points[order[a:b],2];ij=np.unravel_index(key,shape)
        if len(zz)>=3 and np.percentile(zz,90)-np.percentile(zz,10)<.20:
            heights[ij]=np.percentile(zz,25);count[ij]=len(zz)
    observed=np.isfinite(heights);known=np.argwhere(observed)
    if len(known)<100:raise ValueError('Insufficient observed terrain cells')
    missing=np.argwhere(~observed);dist,idx=cKDTree(known*size).query(missing*size)
    fill=missing[dist<=.17];heights[tuple(fill.T)]=heights[tuple(known[idx[dist<=.17]].T)]
    valid=np.isfinite(heights)
    return origin,heights,observed,valid,count

def mesh(origin,h,valid,size=.08):
    ii,jj=np.indices(h.shape);verts=np.c_[origin[0]+(ii.ravel()+.5)*size,origin[1]+(jj.ravel()+.5)*size,np.nan_to_num(h).ravel()]
    faces=[];ny=h.shape[1]
    for i,j in np.argwhere(valid[:-1,:-1]&valid[1:,:-1]&valid[:-1,1:]&valid[1:,1:]):
        if np.ptp(h[i:i+2,j:j+2])>.25:continue
        a=i*ny+j;faces.extend([[a,a+ny,a+ny+1],[a,a+ny+1,a+1]])
    return verts,np.array(faces,dtype=np.int32)

def run(job):
    out=BATCH/job['name'];data=np.load(out/'registered_terrain.npz');a=np.load(out/'reference_motion.npz');xyz=a['root_position_m'][::10]
    tree=cKDTree(xyz[:,:2])
    def corridor(p):
        d,idx=tree.query(p[:,:2]);dz=p[:,2]-xyz[idx,2]
        return p[(d<2)&(dz>-.85)&(dz<.20)]
    train=corridor(data['train']);test=corridor(data['test'])
    origin,h,observed,valid,count=grid(train);vertices,faces=mesh(origin,h,valid)
    np.savez_compressed(out/'terrain_proxy.npz',origin_xy_m=origin,resolution_m=.08,height_m=h,
                        observed=observed,valid=valid,point_count=count,vertices=vertices,faces=faces)
    used=np.unique(faces);dist,_=cKDTree(vertices[used]).query(test)
    # Report coverage as well as residual; do not score distant points against invented fill.
    xy_tree=cKDTree(vertices[used,:2]);xy,_=xy_tree.query(test[:,:2]);covered=xy<.12
    dd=dist[covered]
    result=dict(recording_id=job['recording_id'],job=job['name'],resolution_m=.08,
                observed_cells=int(observed.sum()),interpolated_cells=int((valid&~observed).sum()),triangles=len(faces),
                validation_points=len(test),covered_fraction=float(np.mean(covered)),
                median_surface_distance_m=float(np.median(dd)),p90_surface_distance_m=float(np.percentile(dd,90)),
                ground_truth=False,physics_contact_validated=False,
                note='Near-path lower-surface proxy from even scans; odd scans validate. Fill radius <=17cm. Mesh bridges stair edges over 8cm cells.')
    (out/'terrain_quality.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    # Human-readable USD is validated and packaged with the robot by the replay exporter.
    text=['#usda 1.0','(\n metersPerUnit = 1\n upAxis = "Z"\n)','def Mesh "TerrainProxy" {',
          ' uniform token subdivisionScheme = "none"',' bool doubleSided = true',
          ' color3f[] primvars:displayColor = [(0.42, 0.57, 0.62)]',
          ' point3f[] points = ['+','.join('('+','.join(f'{x:.5f}' for x in p)+')' for p in vertices)+']',
          ' int[] faceVertexCounts = ['+','.join(['3']*len(faces))+']',
          ' int[] faceVertexIndices = ['+','.join(map(str,faces.ravel()))+']','}']
    (out/'terrain_proxy.usda').write_text('\n'.join(text))
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    im=axes[0].imshow(h.T,origin='lower',extent=[origin[0],origin[0]+h.shape[0]*.08,origin[1],origin[1]+h.shape[1]*.08],cmap='terrain')
    axes[0].plot(xyz[:,0],xyz[:,1],'r-',lw=1);fig.colorbar(im,ax=axes[0],label='Terrain Z (m)');axes[0].set(title=job['name'],xlabel='X (m)',ylabel='Y (m)')
    axes[1].hist(dd,bins=60,range=(0,.3));axes[1].set(xlabel='Held-out point to mesh-vertex distance (m)',ylabel='Count');fig.tight_layout();fig.savefig(out/'terrain_validation.png',dpi=130);plt.close(fig)

def selfcheck():
    rng=np.random.default_rng(4);xy=rng.uniform(0,2,(16000,2));p=np.c_[xy,np.where(xy[:,0]<1,0,.15)]
    o,h,obs,valid,c=grid(p);v,f=mesh(o,h,valid)
    assert len(f)>500 and np.nanmax(h)==.15 and np.nanmin(h)==0 and np.all(f>=0)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--job',choices=[j['name'] for j in CONFIG['jobs']]);parser.add_argument('--selfcheck',action='store_true');a=parser.parse_args()
    if a.selfcheck:selfcheck();print('Terrain proxy selfcheck passed')
    elif a.job:run(next(j for j in CONFIG['jobs'] if j['name']==a.job))
    else:
        for job in CONFIG['jobs']:run(job)
