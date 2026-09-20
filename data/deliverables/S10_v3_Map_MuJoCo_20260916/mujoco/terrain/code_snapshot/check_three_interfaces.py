"""Sample B interfaces and push an original-size wheel-shaped probe across them.
This is a constrained mechanics fixture, never a navigation/robot command.
"""
from pathlib import Path
import sys,json,argparse,subprocess
import xml.etree.ElementTree as ET
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1];MAP=ROOT.parents[1]
def dump(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,ensure_ascii=False,allow_nan=False)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('name');a=ap.parse_args();out=ROOT/a.name
    B=MAP/'reconstruction/B_structured_repair_v1/candidate_05';bp=[p for p in json.loads((B/'repair_report.json').read_text())['surface_candidates'] if 'footprint_map_xy' in p]
    model=mujoco.MjModel.from_xml_path(str(out/'scene_contact_v1.xml'));d=mujoco.MjData(model);mujoco.mj_forward(model,d);group=np.array([0,0,0,1,0,0],np.uint8);gid=np.array([-1],np.int32)
    def height(m,d,xy):
        ray=mujoco.mj_ray(m,d,np.r_[xy,10.],np.array([0.,0.,-1.]),group,True,-1,gid)
        return 10-ray if ray>=0 else None
    seams=[];fixtures=[('Start_mesh_seam',np.array([2.,.5]),np.array([1.,0.]))]
    for name,p,forward in [('Start_B',bp[0],np.array([1.,.4])),('B_Post',bp[-1],np.array([.3,1.]))]:
        aa,bb=np.array(p['footprint_map_xy'])[:2];e=(bb-aa)/np.linalg.norm(bb-aa);n=np.array([-e[1],e[0]])
        if n@forward<0:n=-n
        rows=[]
        for t in np.linspace(.1,.9,41):
            xy=(1-t)*aa+t*bb;left=height(model,d,xy-n*.002);right=height(model,d,xy+n*.002)
            rows.append(dict(t=float(t),left_z=left,right_z=right,jump_m=abs(left-right) if left is not None and right is not None else None))
        both=[r['jump_m'] for r in rows if r['jump_m'] is not None]
        seams.append(dict(name=name,samples=rows,covered_samples=len(both),max_adjacent_difference_m=max(both) if both else None,passed=bool(len(both)==len(rows) and max(both)<=.002)))
        fixtures.append((name,(aa+bb)/2,n))
    results=[];replays=[]
    for name,center,heading in fixtures:
        origin=center-heading*.30;hh=height(model,d,origin)
        if hh is None:
            results.append(dict(name=name,passed=False,status='UNMODELED_START'));continue
        tree=ET.parse(out/'scene_contact_v1.xml').getroot();world=ET.SubElement(tree,'worldbody');body=ET.SubElement(world,'body',name='wheel_probe',pos=' '.join(map(str,np.r_[origin,hh+.25])))
        ET.SubElement(body,'joint',name='travel',type='slide',axis=f'{heading[0]} {heading[1]} 0');ET.SubElement(body,'joint',name='vertical',type='slide',axis='0 0 1')
        ET.SubElement(body,'joint',name='spin',type='hinge',axis=f'{-heading[1]} {heading[0]} 0')
        # S10 collision-wheel dimensions from the unchanged source model.
        quat=[2**-.5,-heading[0]*2**-.5,-heading[1]*2**-.5,0]
        ET.SubElement(body,'geom',name='wheel_probe_geom',type='cylinder',size='.081 .0185',quat=' '.join(map(str,quat)),mass='.4',group='1',rgba='.9 .2 .05 1')
        act=ET.SubElement(tree,'actuator');ET.SubElement(act,'velocity',joint='travel',kv='20',ctrllimited='true',ctrlrange='-.3 .3',forcelimited='true',forcerange='-5 5')
        file=out/f'wheel_{name}.xml';ET.indent(tree);ET.ElementTree(tree).write(file,encoding='utf-8',xml_declaration=True)
        m=mujoco.MjModel.from_xml_path(str(file));dd=mujoco.MjData(m);bi=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'wheel_probe');gi=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'wheel_probe_geom')
        deepest=0.;qs=[];tt=[];nextframe=0
        for i in range(4000):
            if dd.time>=nextframe-1e-9:qs.append(dd.qpos.copy());tt.append(float(dd.time));nextframe+=1/24
            dd.ctrl[0]=0 if dd.time<1 else .2;mujoco.mj_step(m,dd);deepest=min(deepest,min((c.dist for c in dd.contact),default=0.))
        contacts={int(x) for c in dd.contact for x in c.geom};warnings={str(mujoco.mjtWarning(i)):int(w.number) for i,w in enumerate(dd.warning) if w.number}
        finite=bool(np.isfinite(dd.qpos).all());passed=finite and not warnings and gi in contacts and .50<float(dd.qpos[0])<.70 and deepest>=-.015
        results.append(dict(name=name,passed=bool(passed),finite=finite,warnings=warnings,travel_m=float(dd.qpos[0]),final_contact=bool(gi in contacts),deepest_contact_m=float(deepest),duration_s=4))
        np.savez_compressed(out/f'wheel_{name}_replay.npz',qpos=np.array(qs),time=np.array(tt));replays.append((name,file,qs,tt))
    dump(out/'interface_checks.json',dict(B_interfaces=seams,wheel_checks=results,all_passed=all(x['passed'] for x in seams+results),
        field_accuracy=False,scope='Shared-boundary vertical rays + guided cylinder with original S10 wheel dimensions; imposed velocity, NOT robot/follower'))
    print(json.dumps(dict(interfaces=[{k:v for k,v in s.items() if k!='samples'} for s in seams],wheels=results),indent=2),flush=True)
    sys.path.append('/opt/anaconda3/lib/python3.12/site-packages');from PIL import Image,ImageDraw,ImageFont
    font=ImageFont.truetype('/System/Library/Fonts/STHeiti Medium.ttc',21);opt=mujoco.MjvOption();opt.geomgroup[3]=0
    cmd=['/opt/homebrew/bin/ffmpeg','-loglevel','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','24','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(out/'media/08_wheel_interfaces_1x.mp4')];proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    for name,file,qs,tt in replays:
        m=mujoco.MjModel.from_xml_path(str(file));dd=mujoco.MjData(m);bi=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'wheel_probe')
        with mujoco.Renderer(m,height=720,width=1280) as renderer:
            for q,t in zip(qs,tt):
                dd.qpos[:]=q;mujoco.mj_forward(m,dd);cam=mujoco.MjvCamera();cam.lookat[:]=dd.xpos[bi];cam.distance=2.;cam.azimuth=120;cam.elevation=-27
                renderer.update_scene(dd,camera=cam,scene_option=opt);renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
                im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,1280,76),fill='#13202a');draw.text((15,9),f'轮形探针接口测试｜{name}｜t={t:.2f}s（1×）',font=font,fill='white');draw.text((15,42),'每段独立初始化；S10轮尺寸，导轨约束/外加推进，不是机器狗自主控制。',font=font,fill='#ffda8c');proc.stdin.write(np.asarray(im).tobytes())
    proc.stdin.close();assert proc.wait()==0
if __name__=='__main__':main()

