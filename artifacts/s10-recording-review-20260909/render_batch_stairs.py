"""Export batch robot/terrain review movies and self-contained USD animations.
Run with .venv-win/Scripts/python.exe --job NAME. No dynamics or forced foot snapping.
"""
import argparse
import json
import sys
import subprocess
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from export_stair_usd import ROOT, Usd, UsdGeom, Gf, robot_visuals, set_robot_frame
from render_stair_motion import replay

CONFIG=json.loads((ROOT/'stair_batch.json').read_text());BATCH=ROOT/'stairs_batch'

def run(job):
    out=BATCH/job['name'];a=np.load(out/'reference_motion.npz');fitted=(out/'terrain_fit.npz').exists();terrain=np.load(out/('terrain_fit.npz' if fitted else 'terrain_proxy.npz'))
    tree=ET.parse(replay.OUT/'free_base.xml');world=tree.getroot().find('worldbody');world.remove(world.find("geom[@name='floor']"))
    if fitted:
        for box in json.loads((out/'terrain_boxes.json').read_text()):
            ET.SubElement(world,'geom',name=box['name'],type='box',pos=' '.join(map(str,box['pos'])),size=' '.join(str(x/2) for x in box['size']),
                          quat=f"{np.cos(box['yaw']/2)} 0 0 {np.sin(box['yaw']/2)}",rgba='.54 .65 .69 1',contype='0',conaffinity='0',group='0')
    else:
        verts=terrain['vertices'];faces=terrain['faces'];used,inv=np.unique(faces.ravel(),return_inverse=True)
        ET.SubElement(tree.getroot().find('asset'),'mesh',name='terrain_proxy',vertex=' '.join(map(str,verts[used].ravel())),face=' '.join(map(str,inv)))
        ET.SubElement(world,'geom',name='terrain_proxy',type='mesh',mesh='terrain_proxy',rgba='.42 .57 .62 1',contype='0',conaffinity='0',group='0')
    path=out/'review_scene.xml';tree.write(path,encoding='utf-8');model=mujoco.MjModel.from_xml_path(str(path));md=mujoco.MjData(model)
    qa=model.jnt_qposadr[model.actuator_trnid[:,0]];wheels=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n+'_wheel') for n in ('fl','fr','hl','hr')]
    stage=Usd.Stage.Open(Usd.Stage.Open(str(out/('terrain_fit.usda' if fitted else 'terrain_proxy.usda'))).Flatten());ops=robot_visuals(stage,model)
    times=np.arange(a['time_s'][0],a['time_s'][-1],.05);stage.SetTimeCodesPerSecond(20);stage.SetStartTimeCode(0);stage.SetEndTimeCode(len(times)-1)
    stage.GetRootLayer().customLayerData={'purpose':'Soft reference review; terrain proxy and inferred root; no contact truth or dynamics',
                                        'recordingSourceStartSeconds':float(times[0]),'recordingId':job['recording_id']}
    usd_cam=UsdGeom.Camera.Define(stage,'/ReviewCamera');usd_cam.CreateFocalLengthAttr(28);cam_op=usd_cam.AddTransformOp()
    renderer=mujoco.Renderer(model,height=640,width=960);camera=mujoco.MjvCamera();camera.distance=3.3;camera.elevation=-25
    proc=subprocess.Popen([shutil.which('ffmpeg'),'-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x640','-r','20','-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(out/'matched_replay.mp4')],stdin=subprocess.PIPE)
    gaps=[];wheel_positions=[];flags=[];h=terrain['height_m'];valid=terrain['valid'];origin=terrain['origin_xy_m'];size=float(terrain['resolution_m'])
    marks=[(x[0]+x[1])/2 for x in job['phases']]
    try:
        for frame,t in enumerate(times):
            i=np.argmin(abs(a['time_s']-t));md.qpos[:3]=a['root_position_m'][i];md.qpos[3:7]=a['root_quaternion_wxyz'][i];md.qpos[qa]=a['joint_position_rad'][i];mujoco.mj_forward(model,md)
            set_robot_frame(model,md,ops,frame)
            w,x,y,z=md.qpos[3:7];yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z));camera.azimuth=np.degrees(yaw)+110
            camera.lookat[:]=md.qpos[:3];camera.lookat[2]-=.2;renderer.update_scene(md,camera=camera,scene_option=replay.VIEW_OPTIONS);rgb=renderer.render();proc.stdin.write(rgb.tobytes())
            eye=md.qpos[:3]+np.array([2*np.sin(yaw),-2*np.cos(yaw),1.3]);cam_op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*md.qpos[:3]),Gf.Vec3d(0,0,1)).GetInverse(),frame)
            wp=md.xpos[wheels].copy();wheel_positions.append(wp);row=[]
            for p in wp:
                ij=np.floor((p[:2]-origin)/size).astype(int)
                row.append(float(p[2]-.081-h[tuple(ij)]) if (ij>=0).all() and (ij<np.array(h.shape)).all() and valid[tuple(ij)] else np.nan)
            gaps.append(row);flags.append(bool(a['reference_valid'][i]))
            if any(abs(t-m)<.026 for m in marks):(out/f'preview_{t:.1f}s.ppm').write_bytes(b'P6\n960 640\n255\n'+rgb.tobytes())
    finally:
        renderer.close();proc.stdin.close()
    assert proc.wait(timeout=30)==0
    assert stage.Flatten().Export(str(out/'matched_replay.usdc'))
    check=Usd.Stage.Open(str(out/'matched_replay.usdc'));assert check.GetPrimAtPath('/Robot').IsValid() and not check.GetRootLayer().subLayerPaths
    gaps=np.array(gaps);np.savez_compressed(out/'wheel_proxy_check.npz',time_s=times,wheel_position_m=wheel_positions,gap_to_proxy_m=gaps,reference_valid=flags)
    result=dict(video_frames=len(times),usd_visual_meshes=len(ops),known_wheel_terrain_fraction=float(np.mean(np.isfinite(gaps))),
                median_minimum_wheel_gap_m=float(np.nanmedian(np.nanmin(gaps,axis=1))),
                wheel_samples_below_proxy_5cm_fraction=float(np.nanmean(np.where(np.isfinite(gaps),gaps<-.05,np.nan))),
                note='Vertical wheel-bottom distance to approximate height grid; unknown cells stay NaN. No snapping, no exact contact claim.')
    (out/'replay_check.json').write_text(json.dumps(result,indent=2));print('PASS',job['name'],json.dumps(result),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--job',choices=[j['name'] for j in CONFIG['jobs']]);a=parser.parse_args()
    for j in CONFIG['jobs']:
        if a.job is None or j['name']==a.job:run(j)
