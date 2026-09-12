"""MuJoCo kinematic review of the matched trajectory, not a dynamics success test.
Run with .venv-win/Scripts/python.exe; reads the export made with python -s.
"""
import json
from pathlib import Path
import sys
import subprocess
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'stairs_reconstruction'
sys.path.insert(0,str(ROOT.parent/'s10-expert-analysis'))
import validate_motion as replay

def scene(g):
    tree=ET.parse(replay.OUT/'free_base.xml');world=tree.getroot().find('worldbody')
    world.find("geom[@name='floor']").set('pos',f"0 0 {g['floor_z_m']}")
    usd=['#usda 1.0','(\n metersPerUnit = 1\n upAxis = "Z"\n)','def Xform "MatchedStairs" {']
    for k in range(g['count']):
        z=g['floor_z_m']+(k+1)*g['height_m'];bottom=g['floor_z_m']-.1
        for j,y in enumerate(np.arange(-2.5,2.5,.2)+.1):
            start=g['edge_m']+k*g['depth_m']+g['lateral_slope']*y+g['curvature_per_m']*y*y
            length=g['depth_m'] if k<g['count']-1 else 3
            center=[start+length/2,y,(z+bottom)/2];size=[length,.2,z-bottom]
            ET.SubElement(world,'geom',name=f'step_{k}_{j}',type='box',pos=' '.join(map(str,center)),
                          size=' '.join(str(s/2) for s in size),rgba='0.56 0.66 0.71 1' if k%2 else '0.72 0.77 0.78 1')
            usd.extend([f' def Cube "Step{k}_{j}" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {{',
                        '  double size = 1','  bool physics:collisionEnabled = true',
                        '  double3 xformOp:translate = ('+', '.join(map(str,center))+')',
                        '  double3 xformOp:scale = ('+', '.join(map(str,size))+')',
                        '  uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',' }'])
    usd.append('}');(OUT/'matched_stairs.usda').write_text('\n'.join(usd))
    path=OUT/'matched_stairs.xml';tree.write(path,encoding='utf-8');return mujoco.MjModel.from_xml_path(str(path))

def main():
    g=json.loads((OUT/'scene_geometry.json').read_text());a=np.load(OUT/('contact_matched_motion.npz' if '--contact' in sys.argv else 'matched_motion.npz'));model=scene(g);md=mujoco.MjData(model)
    qa=model.jnt_qposadr[model.actuator_trnid[:,0]]
    wheels=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n+'_wheel') for n in ('fl','fr','hl','hr')]
    logs=[];penetrations=[];deep=[];render='--check-only' not in sys.argv
    if render:
        renderer=mujoco.Renderer(model,height=640,width=960);camera=mujoco.MjvCamera();camera.distance=3.4;camera.azimuth=115;camera.elevation=-20
        proc=subprocess.Popen([shutil.which('ffmpeg'),'-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x640','-r','20','-i','-','-an','-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p',str(OUT/'matched_kinematic_replay.mp4')],stdin=subprocess.PIPE)
    for t in np.arange(a['time_s'][0],a['time_s'][-1],.05):
        i=np.argmin(abs(a['time_s']-t));md.qpos[:3]=a['root_position_m'][i];md.qpos[3:7]=a['root_quaternion_wxyz'][i];md.qpos[qa]=a['joint_position_rad'][i]
        mujoco.mj_forward(model,md)
        contacts=[]
        for contact in md.contact[:md.ncon]:
            ids=model.geom_bodyid[[contact.geom1,contact.geom2]]
            if 0 in ids and ids.sum()>0:
                contacts.append(float(contact.dist))
                if contact.dist<-.04:deep.append(dict(t_s=float(t),depth_m=float(-contact.dist),body=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(ids.sum()))))
        penetrations.append(min(contacts+[0.]))
        wp=md.xpos[wheels].copy();x=wp[:,0]-g['lateral_slope']*wp[:,1]-g['curvature_per_m']*wp[:,1]**2
        level=np.clip(np.floor((x-g['edge_m'])/g['depth_m'])+1,0,g['count'])
        gap=wp[:,2]-.081-(g['floor_z_m']+level*g['height_m'])
        logs.append([t,*wp.ravel(),*gap,*level])
        if render:
            camera.lookat[:]=md.qpos[:3];camera.lookat[2]-=.15
            renderer.update_scene(md,camera=camera,scene_option=replay.VIEW_OPTIONS);rgb=renderer.render();proc.stdin.write(rgb.tobytes())
            for mark in (1,14,18,24,27):
                if abs(t-mark)<.026:
                    out=OUT/f'pose_{mark}s.ppm';out.write_bytes(f'P6\n960 640\n255\n'.encode()+rgb.tobytes())
    if render:renderer.close();proc.stdin.close();assert proc.wait(timeout=30)==0
    logs=np.array(logs);np.savez_compressed(OUT/'wheel_geometry_check.npz',rows=logs)
    summary=dict(frames=len(logs),minimum_wheel_bottom_gap_m=float(logs[:,13:17].min()),
                 max_robot_terrain_penetration_m=float(-min(penetrations)),
                 frames_penetrating_over_2cm=int(np.sum(np.array(penetrations)<-.02)),
                 frames_penetrating_over_5cm=int(np.sum(np.array(penetrations)<-.05)),
                 fraction_wheels_below_surface_by_5cm=float(np.mean(logs[:,13:17]<-.05)),
                 last_wheel_levels=logs[-1,17:21].tolist(),last_wheel_bottom_gap_m=logs[-1,13:17].tolist(),
                 note='Wheel-bottom vertical gap is a rough geometry check; ignores wheel/riser side contact and is not measured contact state')
    (OUT/'wheel_geometry_check.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
    (OUT/'deep_contacts.json').write_text(json.dumps(sorted(deep,key=lambda r:-r['depth_m']),indent=2))

if __name__=='__main__':main()
