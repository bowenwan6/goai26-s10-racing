"""Export a compact body-centred measured/simulated animation and mesh snapshots."""
import json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import match as m

OUT=m.OUT/'distance_search'
VIS=m.Path('C:/Users/Lenovo/.codex/visualizations/2026/09/09/01a0866b-8e1f-7b92-a609-67ff3511337f/ledge-motion.html')
ref=dict(np.load(m.OUT/'expert_reference.npz'));fk=dict(np.load(OUT/'recorded_kinematics.npz'))
cloud=dict(np.load(m.OUT/'full_clouds.npz'));a=json.loads((m.OUT/'alignment.json').read_text())
r=[x for x in json.loads((m.OUT/'fits.json').read_text()) if x['accepted_track']]
model=mujoco.MjModel.from_xml_path(str(m.OUT/'matched_scene.xml'));data=mujoco.MjData(model)
ids=[model.body(str(n)).id for n in fk['body_names']]
cases=[json.loads((OUT/(name+'.json')).read_text()) for name in ['baseline','selected']]
states=[dict(np.load(OUT/(name+'.npz'))) for name in ['baseline','selected']]
normal=np.r_[a['edge_normal_xy'],0]
frames=[]
for t in np.arange(9,15.5001,.05):
    k=int(np.argmin(abs(ref['time_s']-t)));quat=ref['base_quaternion_wxyz'][k]
    matrix=np.empty(9);mujoco.mju_quat2Mat(matrix,quat);matrix=matrix.reshape(3,3)
    points=[]
    for topic in ['/rslidar_front/points','/rslidar_rear/points']:
        ix=int(np.argmin(abs(cloud['time_s']-t)+np.where(cloud['topic']==topic,0,100)))
        p=cloud['xyz'][cloud['offsets'][ix]:cloud['offsets'][ix+1]]@matrix.T
        p=p[(p[:,0]>-1)&(p[:,0]<1.5)&(abs(p[:,1])<.6)&(p[:,2]>-.9)&(p[:,2]<.65)]
        points.extend(p[::max(1,int(np.ceil(len(p)/80)))].tolist())
    recorded_q=ref['joint_position'][k]*180/np.pi
    recorded_q[m.replay.WHEELS]=ref['joint_velocity'][k,m.replay.WHEELS]
    simulations=[]
    for s in states:
        ix=int(np.argmin(abs(s['log'][:,0]-t)));data.qpos[:]=s['qpos'][ix];mujoco.mj_forward(model,data)
        simulations.append(dict(base=np.round(data.qpos[:3],4).tolist(),
            bones=np.round(data.xpos[ids]-data.qpos[:3],4).flatten().tolist(),tilt=round(float(s['log'][ix,1]),2)))
    frames.append(dict(t=round(float(t),2),bones=np.round(fk['positions'][k],4).flatten().tolist(),
        cloud=np.round(points,3).flatten().tolist(),sim=simulations,joints=np.round(recorded_q,2).tolist(),
        distance=float(np.interp(t,[x['time_s'] for x in r],[x['edge_horizontal_distance_m'] for x in r])) if t<=r[-1]['time_s'] else None))
payload=dict(normal=normal.tolist(),edge=a['edge_horizontal_distance_m'],
    cases=[dict(offset=c['offset_m'],delay=c['delay_s']) for c in cases],frames=frames)
(OUT/'motion_data.json').write_text(json.dumps(payload,separators=(',',':')),encoding='utf-8')
html=VIS.read_text(encoding='utf-8')
start=html.index('<script id="lm-data" type="application/json">')+len('<script id="lm-data" type="application/json">')
end=html.index('</script>',start)
html=html[:start]+json.dumps(payload,separators=(',',':'))+html[end:]
assert len(html.encode())<1_000_000
VIS.write_text(html,encoding='utf-8')
renderer=mujoco.Renderer(model,height=420,width=640);camera=mujoco.MjvCamera()
camera.distance=2.2;camera.azimuth=90;camera.elevation=-12;camera.lookat[:]=[.5,0,.38]
original=model.geom_pos[model.geom('matched_ledge').id].copy()
try:
    for ci,s in enumerate(states):
        model.geom_pos[model.geom('matched_ledge').id]=original+cases[ci]['offset_m']*normal
        for i,t in enumerate([12.2,12.85,13.5,14.5]):
            ix=int(np.argmin(abs(s['log'][:,0]-t)));data.qpos[:]=s['qpos'][ix];mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera=camera,scene_option=m.replay.VIEW_OPTIONS)
            (OUT/f'mesh_{ci}_{i}.ppm').write_bytes(b'P6\n640 420\n255\n'+renderer.render().tobytes())
finally:renderer.close()
tree=ET.parse(m.OUT/'matched_scene.xml')
tree.getroot().find("worldbody/geom[@name='matched_ledge']").set('pos',' '.join(map(str,original+cases[1]['offset_m']*normal)))
tree.write(OUT/'selected_scene.xml',encoding='utf-8')
new_centre=original+cases[1]['offset_m']*normal
usd=(m.OUT/'matched_terrain.usda').read_text(encoding='utf-8')
old_translate='double3 xformOp:translate = ('+', '.join(map(str,original))+')'
new_translate='double3 xformOp:translate = ('+', '.join(map(str,new_centre))+')'
assert old_translate in usd
(OUT/'selected_terrain.usda').write_text(usd.replace(old_translate,new_translate,1),encoding='utf-8')
selected_alignment={**a,'nominal_lidar_edge_distance_m':a['edge_horizontal_distance_m'],
    'edge_horizontal_distance_m':a['edge_horizontal_distance_m']+cases[1]['offset_m'],
    'front_wheel_outer_gap_m':a['front_wheel_outer_gap_m']+cases[1]['offset_m'],
    'simulation_distance_correction_m':cases[1]['offset_m'],'action_delay_s':cases[1]['delay_s'],
    'correction_status':'Fitted simulation offset, NOT a verified real sensor calibration'}
m.review.dump(OUT/'selected_alignment.json',selected_alignment)
print('animation bytes',len(html.encode()),'frames',len(frames))
