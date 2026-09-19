"""Actual isolated local MuJoCo tests; no ROS/network/policy or robot API."""
from pathlib import Path
import argparse,json,sys,time,copy,subprocess
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1];MAP=ROOT.parents[1]
def dump(p,v):
    with p.open('x') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
def xml(p,r):
    ET.indent(r);ET.ElementTree(r).write(p,encoding='utf-8',xml_declaration=True)
def warns(d):return {str(mujoco.mjtWarning(i)):int(w.number) for i,w in enumerate(d.warning) if w.number}
def main():
    ap=argparse.ArgumentParser();ap.add_argument('name');a=ap.parse_args();out=ROOT/a.name
    g=np.load(out/'geometry.npz');v=g['vertices'];f=g['faces'];sid=g['surface_id']
    centers=v[f].mean(1);normals=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]);areas=np.linalg.norm(normals,axis=1)/2;normals/=np.linalg.norm(normals,axis=1)[:,None]
    fixtures=[]
    for s,xy in [(2,[0,.5]),(0,[5,.5]),(2,[10,.5]),(3,[28,24])]:
        candidates=np.flatnonzero((sid==s)&(areas>.005))
        j=candidates[np.argmin(np.linalg.norm(centers[candidates,:2]-xy,axis=1))]
        fixtures.append(dict(name=f'new_{len(fixtures)}_surface{s}',xyz=centers[j].tolist(),normal=normals[j].tolist()))
    B=MAP/'reconstruction/B_structured_repair_v1/candidate_05';bp=[p for p in json.loads((B/'repair_report.json').read_text())['surface_candidates'] if 'footprint_map_xy' in p]
    for p in bp:
        xy=np.array(p['footprint_map_xy']).mean(0);c=np.array(p['plane']);n=np.r_[-c[:2],1.];n/=np.linalg.norm(n)
        fixtures.append(dict(name=f'B{p["id"]}',xyz=[*xy,float(xy@c[:2]+c[2])],normal=n.tolist()))
    expected=np.array([x['xyz'] for x in fixtures]);normal=np.array([x['normal'] for x in fixtures]);radius=.06
    allcases=[];loaded={};replay={}
    for variant in ['baseline','contact_v1']:
        start=time.monotonic();base=mujoco.MjModel.from_xml_path(str(out/f'scene_{variant}.xml'));bd=mujoco.MjData(base);mujoco.mj_forward(base,bd)
        group=np.array([0,0,0,1,0,0],np.uint8);gid=np.array([-1],np.int32)
        def ray(p,vec):return mujoco.mj_ray(base,bd,np.array(p,float),np.array(vec,float),group,True,-1,gid)
        errors=[]
        # New triangles, all rendered surfaces at centroids. Compare to actual collision.
        for p in centers:
            distance=ray(p+[0,0,.5],[0,0,-1]);errors.append(abs(.5-distance) if distance>=0 else None)
        brays=[]
        for lower,upper in zip(bp[:-1],bp[1:]):
            lo=np.array(lower['footprint_map_xy']).mean(0);hi=np.array(upper['footprint_map_xy']).mean(0);vec=hi-lo;vec/=np.linalg.norm(vec)
            zl=lo@np.array(lower['plane'])[:2]+lower['plane'][2];zh=hi@np.array(upper['plane'])[:2]+upper['plane'][2]
            origin=np.r_[lo,.5*(zl+zh)];hit=ray(origin,np.r_[vec,0]);brays.append(dict(from_patch=lower['id'],to_patch=upper['id'],hit=bool(hit>=0),distance_m=float(hit)))
        voids=[]
        for p in [[-2,-2,8],[0,5,8],[35,25,8],[-10,0,8]]:
            hit=ray(p,[0,0,-1]);voids.append(dict(origin=p,hit=bool(hit>=0),note='No hit at a sampled unmodeled coordinate is not proof of a real field hole.'))
        loaded[variant]=dict(compile_s=time.monotonic()-start,ngeom=int(base.ngeom),ray_error_max_m=max(x for x in errors if x is not None),
            ray_misses=sum(x is None for x in errors),new_surface_samples=len(errors),B_riser_rays=brays,void_rays=voids,
            finite_geom=bool(np.isfinite(base.geom_pos).all()),infinite_planes=int(np.sum(base.geom_type==mujoco.mjtGeom.mjGEOM_PLANE)))
        tree=ET.parse(out/f'scene_{variant}.xml').getroot();world=ET.SubElement(tree,'worldbody')
        for i,p in enumerate(expected):
            body=ET.SubElement(world,'body',name=f'probe_{i}',pos=' '.join(map(str,p+[0,0,.35])))
            ET.SubElement(body,'joint',type='slide',axis='0 0 1');ET.SubElement(body,'geom',name=f'probe_geom_{i}',type='sphere',size=str(radius),mass='.4',group='1',rgba='1 .2 .06 1')
        file=out/f'probes_{variant}.xml';xml(file,tree);m=mujoco.MjModel.from_xml_path(str(file))
        terrain=np.flatnonzero(m.geom_group==3);bodies=np.array([mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,f'probe_{i}') for i in range(len(fixtures))]);probe_geoms={mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,f'probe_geom_{i}') for i in range(len(fixtures))}
        qs=[];tt=[];phases=[]
        for dt,collisions,duration in [(.001,True,2.),(.002,True,2.),(.001,False,1.)]:
            m.opt.timestep=dt;m.geom_contype[terrain]=m.geom_conaffinity[terrain]=int(collisions);d=mujoco.MjData(m);mujoco.mj_forward(m,d);initial=d.xpos[bodies,2].copy();depth=0.;nextframe=0
            for step in range(round(duration/dt)):
                if dt==.001 and d.time>=nextframe-1e-8:
                    qs.append(d.qpos.copy());tt.append(float(d.time));phases.append('on' if collisions else 'off');nextframe+=1/24
                mujoco.mj_step(m,d);depth=min(depth,min((c.dist for c in d.contact),default=0.))
            # Vertical slide sphere tangent height on an inclined infinite plane.
            ideal=expected[:,2]+radius/normal[:,2];err=abs(d.xpos[bodies,2]-ideal);contacted={int(z) for c in d.contact for z in c.geom};supported=len(contacted&probe_geoms)
            finite=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all());ww=warns(d);drop=float((initial-d.xpos[bodies,2]).min())
            passed=finite and not ww and (supported==len(fixtures) and err.max()<.005 and depth>=-.015 if collisions else supported==0 and drop>1.)
            allcases.append(dict(variant=variant,dt=dt,collisions=collisions,duration_s=duration,fixtures=len(fixtures),supported=supported,
                maximum_height_error_m=float(err.max()),deepest_contact_m=float(depth),minimum_drop_m=drop,finite=finite,warnings=ww,passed=bool(passed)))
        np.savez_compressed(out/f'probe_replay_{variant}.npz',qpos=np.array(qs),time=np.array(tt),phase=np.array(phases));replay[variant]=(qs,tt,phases)
    # Exact new mesh shared vertices prohibit the old independently-valued same-cell layers.
    keys=np.round(v[:,:2],7);_,inv=np.unique(keys,axis=0,return_inverse=True);lo=np.full(inv.max()+1,np.inf);hi=np.full(inv.max()+1,-np.inf)
    np.minimum.at(lo,inv,v[:,2]);np.maximum.at(hi,inv,v[:,2]);duplicate_height=float((hi-lo).max())
    report=dict(engine=mujoco.__version__,load=loaded,tests=allcases,fixtures=fixtures,new_mesh_same_xy_vertex_height_difference_m=duplicate_height,duplicate_xy_note='This diagnostic can include distinct elevations of an intentional raised feature; continuous seam tests are separate.',
        new_mesh_export_checks_pass=bool(loaded['contact_v1']['ray_misses']==0 and loaded['contact_v1']['ray_error_max_m']<=.0002),
        contact_v1_passed=all(x['passed'] for x in allcases if x['variant']=='contact_v1'),baseline_passed=all(x['passed'] for x in allcases if x['variant']=='baseline'),
        contact_change='candidate solref=.01 1 versus MuJoCo default; same 0.35m spawn height/radius/duration. Not measured material calibration.',
        field_accuracy_accepted=False,policy_run=False,network_interfaces_used=False,scope='New mesh and preserved B export/rays plus slide sphere contact, not full robot-route acceptance.')
    dump(out/'mujoco_checks.json',report);print(json.dumps({k:v for k,v in report.items() if k not in ['fixtures','load']},indent=2),flush=True)

if __name__=='__main__':main()

