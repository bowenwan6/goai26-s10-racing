"""Release-level source preservation, imported render/collision and void rays."""
from pathlib import Path
import json,hashlib,sys
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1]
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main():
    out=ROOT/sys.argv[1];m=mujoco.MjModel.from_xml_path(str(out/'scene_contact_v1.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    gg=np.load(out/'geometry.npz');v=gg['vertices'];f=gg['faces'];centers=v[f].mean(1);gid=np.array([-1],np.int32)
    def ray(p,which):
        group=np.zeros(6,np.uint8);group[which]=1
        return mujoco.mj_ray(m,d,np.asarray(p,float),np.array([0.,0.,-1.]),group,True,-1,gid)
    errors=[];misses=[]
    for i,c in enumerate(centers):
        a=ray(c+[0,0,.5],2);b=ray(c+[0,0,.5],3)
        if a<0 or b<0:misses.append(i)
        else:errors.append(abs(a-b))
    holes=[]
    for name in ['Start','Post_B']:
        support=np.load(out/f'data/{name}_support.npz');points=support['xy'][support['nearest_xy_m']>.05]
        if name=='Start':points=points[~((points[:,0]>=3)&(points[:,0]<=8)&(points[:,1]>=-1.1)&(points[:,1]<=1.4))]
        hits=[i for i,p in enumerate(points) if ray(np.r_[p,8.],3)>=0]
        holes.append(dict(name=name,unknown_support_centers_tested=len(points),collision_hits=len(hits),passed=not hits))
    records=json.loads((out/'input_manifest.json').read_text());unique={r['path']:r['sha256'] for r in records};changed=[p for p,h in unique.items() if sha(p)!=h]
    original_B=ROOT.parent/'B_structured_repair_v1/candidate_05'
    copies={name:sha(original_B/a)==sha(out/b) for name,a,b in [('B_geometry','geometry.npz','data/B_geometry_preserved.npz'),('B_visual','assets/B_structured_map.obj','visual/B_preserved.obj'),('B_original_collision','B_structured_collision.xml','collision/B_original.xml')]}
    report=dict(imported_visual_collision_centroid_rays=len(centers),misses=misses,maximum_imported_visual_collision_difference_m=max(errors),
        imported_visual_collision_pass=not misses and max(errors)<.0002,unknown_mask_rays=holes,
        manifest_records=len(records),unique_source_files=len(unique),changed_source_files=changed,original_B_byte_copies=copies,
        all_passed=not misses and max(errors)<.0002 and all(h['passed'] for h in holes) and not changed and all(copies.values()),
        limitations='These are bounded export/implementation checks, not surveyed geometry truth, full obstacle coverage or robot policy acceptance.')
    with (out/'release_checks.json').open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
