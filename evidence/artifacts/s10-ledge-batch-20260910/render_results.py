"""Export measured FK, representative mesh frames, and terrain for every selected clip."""
import argparse
import json
import numpy as np
import mujoco
import batch_match as b


def export(c):
    folder=b.OUT/c['id']
    if not (folder/'selected.npz').exists():return
    ref=dict(np.load(folder/'reference.npz'));sim=dict(np.load(folder/'selected.npz'))
    result=json.loads((folder/'selected.json').read_text());geometry=json.loads((folder/'geometry.json').read_text())
    model=mujoco.MjModel.from_xml_path(str(folder/'selected_scene.xml'));data=mujoco.MjData(model)
    names=['base_link']+[leg+'_'+joint for leg in ['fl','fr','hl','hr'] for joint in ['hipx','hipy','knee','wheel']]
    ids=[model.body(n).id for n in names];joints=model.actuator_trnid[:,0];qa=model.jnt_qposadr[joints]
    positions=[]
    for q,quat in zip(ref['joint_position'][::4],ref['base_quaternion_wxyz'][::4]):
        data.qpos[:3]=0;data.qpos[3:7]=quat;data.qpos[qa]=q;mujoco.mj_forward(model,data)
        positions.append(data.xpos[ids].copy())
    np.savez_compressed(folder/'recorded_kinematics.npz',time_s=ref['time_s'][::4],body_names=names,positions=positions)
    third=c['reverse_peak'] if c['mode']=='cycle' else c['peak']+.7
    stages=[c['start'],c['peak'],min(third,c['end']-.1),c['end']-.1]
    assert all(c['start']<=t<=c['end'] for t in stages)
    renderer=mujoco.Renderer(model,height=400,width=640);camera=mujoco.MjvCamera()
    camera.distance=2.1;camera.azimuth=90+geometry['yaw_deg'];camera.elevation=-12
    try:
        for i,t in enumerate(stages):
            k=int(np.argmin(abs(sim['log'][:,0]-t)));data.qpos[:]=sim['qpos'][k];mujoco.mj_forward(model,data)
            camera.lookat[:]=[data.qpos[0]+.12,data.qpos[1],.35]
            renderer.update_scene(data,camera=camera,scene_option=b.m.replay.VIEW_OPTIONS)
            (folder/f'mesh_{i}.ppm').write_bytes(b'P6\n640 400\n255\n'+renderer.render().tobytes())
    finally:renderer.close()
    b.m.review.dump(folder/'render_times.json',stages)
    n=np.array([np.cos(np.deg2rad(geometry['yaw_deg'])),np.sin(np.deg2rad(geometry['yaw_deg'])),0])
    data.qpos[:]=sim['initial_qpos'];mujoco.mj_forward(model,data)
    feet=data.xpos[[model.body(x+'_wheel').id for x in ['fl','fr','hl','hr']]]
    limits=model.jnt_range[joints];over=np.maximum(limits[:,0]-ref['joint_position'],ref['joint_position']-limits[:,1])
    over[:,~model.jnt_limited[joints].astype(bool)]=0
    alignment=dict(source_clip=c['id'],initial_base_position_m=sim['initial_qpos'][:3].tolist(),
        initial_base_quaternion_wxyz=sim['initial_qpos'][3:7].tolist(),
        nominal_lidar_edge_distance_m=geometry['initial_distance_m'],edge_horizontal_distance_m=result['initial_distance_m'],
        simulation_distance_correction_m=result['offset_m'],action_delay_s=result['delay_s'],edge_normal_xy=n[:2].tolist(),
        front_wheel_outer_gap_m=float(result['initial_distance_m']-np.max(feet@n)-.081),
        gap_definition='Signed distance to forward wheel rim; negative at a descending edge means wheels already protrude',
        height_m=c['height'],height_source='recorded manifest.parameters.height_cm',
        platform_depth_m=3,platform_width_m=2,dimensions_source='Sufficient-size test geometry, not measured extents',
        start_support='platform' if c['mode']=='down' else 'lower ground',
        joint_names=[model.joint(int(j)).name for j in joints],isaac_sim_executed=False,
        max_reference_joint_limit_excess_rad=float(max(0,over.max())),
        controller={k:result[k] for k in ['kp_leg','kd_leg','kd_wheel']},
        correction_status='Simulation fit, not verified sensor calibration',
        coordinates='Initial base XY origin; initial yaw removed; Z-up; floor Z=0',
        initial_height_method='Lowest SDK wheel centre at support height + 0.083 m')
    b.m.review.dump(folder/'selected_alignment.json',alignment)
    # Independently inspect selected landing geometry; no extra dynamics or tuning.
    contact_counts=[]
    for pose in sim['qpos'][-10:]:
        data.qpos[:]=pose;mujoco.mj_forward(model,data);touching=set()
        for contact in data.contact[:data.ncon]:
            g1,g2=model.geom(contact.geom1),model.geom(contact.geom2)
            if contact.dist>.003:continue
            if g1.name in ('floor','matched_ledge'):other=g2
            elif g2.name in ('floor','matched_ledge'):other=g1
            else:continue
            name=model.body(int(other.bodyid[0])).name
            if name.endswith('_wheel'):touching.add(name)
        contact_counts.append(len(touching))
    b.m.review.dump(folder/'contact_review.json',dict(final_10_frame_wheel_contact_counts=contact_counts,
        check='Recomputed contacts from selected qpos, distance <= 3 mm; supports are floor or matched_ledge',
        all_final_frames_have_at_least_two_wheels_in_contact=bool(min(contact_counts)>=2)))
    def cube(name,pos,size,angle=0):
        return f'''    def Cube "{name}" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {{
        double size = 1
        double3 xformOp:translate = ({', '.join(map(str,pos))})
        float xformOp:rotateZ = {angle}
        double3 xformOp:scale = ({', '.join(map(str,size))})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ", "xformOp:scale"]
        color3f[] primvars:displayColor = [(0.48, 0.63, 0.75)]
    }}
'''
    usd='#usda 1.0\n(defaultPrim = "World"\n metersPerUnit = 1\n upAxis = "Z")\ndef Xform "World" {\n    def PhysicsScene "PhysicsScene" {}\n'
    usd+=cube('Ground',[0,0,-.05],[20,20,.1])+cube('MatchedLedge',model.geom('matched_ledge').pos,[3,2,c['height']],geometry['yaw_deg'])+'}\n'
    (folder/'selected_terrain.usda').write_text(usd,encoding='utf-8')
    print('EXPORTED',c['id'],flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--only');args=parser.parse_args()
    for c in b.CLIPS:
        if not args.only or args.only in (c['id'],c['recording']):export(c)
