"""Portable, offline reproduction of the 36 constrained-probe tests.
Requires numpy + mujoco==3.13.0. Does not access ROS, the network or a robot.
"""
from pathlib import Path
import json
import numpy as np
import mujoco

def main():
    root=Path(__file__).resolve().parent
    prior=json.loads((root/'mujoco_checks.json').read_text())
    fixtures=prior['fixtures'];expected=np.array([x['xyz'] for x in fixtures]);normals=np.array([x['normal'] for x in fixtures])
    model=mujoco.MjModel.from_xml_path(str(root/'probes_contact_v1.xml'))
    terrain=np.flatnonzero(model.geom_group==3)
    bodies=np.array([mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'probe_{i}') for i in range(len(fixtures))])
    geoms={mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,f'probe_geom_{i}') for i in range(len(fixtures))}
    results=[]
    for dt,on,duration in [(.001,True,2.),(.002,True,2.),(.001,False,1.)]:
        model.opt.timestep=dt;model.geom_contype[terrain]=int(on);model.geom_conaffinity[terrain]=int(on)
        data=mujoco.MjData(model);mujoco.mj_forward(model,data);start=data.xpos[bodies,2].copy();depth=0.
        for _ in range(round(duration/dt)):
            mujoco.mj_step(model,data);depth=min(depth,min((c.dist for c in data.contact),default=0.))
        contacted={int(z) for c in data.contact for z in c.geom};n=len(contacted&geoms)
        err=float(abs(data.xpos[bodies,2]-(expected[:,2]+.06/normals[:,2])).max());drop=float((start-data.xpos[bodies,2]).min())
        finite=bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all());warnings=sum(int(w.number) for w in data.warning)
        passed=finite and warnings==0 and (n==len(fixtures) and err<.005 and depth>=-.015 if on else n==0 and drop>1.)
        results.append(dict(dt=dt,collision=on,passed=passed,contacts=n,height_error_m=err,deepest_contact_m=depth,warnings=warnings))
    print(json.dumps(dict(engine=mujoco.__version__,tests=results,all_passed=all(r['passed'] for r in results),field_accuracy=False,policy_run=False),indent=2))
    raise SystemExit(0 if all(r['passed'] for r in results) else 1)
if __name__=='__main__':main()
