"""Self-contained USD animation for Isaac Sim/OpenUSD inspection, not a trained controller.
Run with .venv-win/Scripts/python.exe (usd-core in existing analysis-deps).
"""
import sys
import json
import numpy as np
import mujoco
from render_stair_motion import ROOT, OUT, scene
sys.path.append(str(ROOT.parents[1]/'tmp/s10-analysis-deps'))
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Vt

def robot_visuals(stage,model):
    UsdGeom.Xform.Define(stage,'/Robot');ops=[]
    for i in range(model.ngeom):
        if model.geom_group[i]!=2 or model.geom_type[i]!=mujoco.mjtGeom.mjGEOM_MESH:continue
        meshid=model.geom_dataid[i];start=model.mesh_vertadr[meshid];count=model.mesh_vertnum[meshid]
        fs=model.mesh_faceadr[meshid];fc=model.mesh_facenum[meshid]
        mesh=UsdGeom.Mesh.Define(stage,f'/Robot/Visual{i}')
        vertices=np.asarray(model.mesh_vert[start:start+count],dtype=np.float32)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices))
        mesh.CreateFaceVertexCountsAttr([3]*fc);mesh.CreateFaceVertexIndicesAttr(model.mesh_face[fs:fs+fc].ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True);mesh.CreateDisplayColorAttr([Gf.Vec3f(.48,.52,.55)])
        ops.append((i,mesh.AddTranslateOp(),mesh.AddOrientOp()))
    assert len(ops)>10
    return ops

def set_robot_frame(model,md,ops,frame):
    q=np.empty(4)
    for i,pos,orient in ops:
        mujoco.mju_mat2Quat(q,md.geom_xmat[i]);pos.Set(Gf.Vec3d(*md.geom_xpos[i]),frame);orient.Set(Gf.Quatf(float(q[0]),Gf.Vec3f(*q[1:])),frame)

def main():
    g=json.loads((OUT/'scene_geometry.json').read_text());model=scene(g);md=mujoco.MjData(model)
    a=np.load(OUT/'contact_matched_motion.npz');qa=model.jnt_qposadr[model.actuator_trnid[:,0]]
    stage=Usd.Stage.Open(str(OUT/'matched_stairs.usda'))
    # Work in memory; keep the standalone terrain file unchanged.
    stage=Usd.Stage.Open(stage.Flatten())
    stage.SetTimeCodesPerSecond(20);stage.SetFramesPerSecond(20);stage.SetStartTimeCode(0)
    ts=np.arange(a['time_s'][0],a['time_s'][-1],.05);stage.SetEndTimeCode(len(ts)-1)
    stage.GetRootLayer().customLayerData={'purpose':'Measured joints + estimated/contact-constrained root; kinematic replay only',
                                        'recordingSourceStartSeconds':float(ts[0]),'terrainWidthIsAssumed':True}
    floor=UsdGeom.Cube.Define(stage,'/Ground');floor.CreateSizeAttr(1)
    floor.AddTranslateOp().Set(Gf.Vec3d(3,0,g['floor_z_m']-.05));floor.AddScaleOp().Set(Gf.Vec3f(25,20,.1));UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    floor.CreateDisplayColorAttr([Gf.Vec3f(.16,.20,.24)])
    ops=robot_visuals(stage,model)
    camera=UsdGeom.Camera.Define(stage,'/ReviewCamera');camera.CreateFocalLengthAttr(28)
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(5,-9,5),Gf.Vec3d(3,.5,.7),Gf.Vec3d(0,0,1)).GetInverse())
    for frame,t in enumerate(ts):
        idx=np.argmin(abs(a['time_s']-t));md.qpos[:3]=a['root_position_m'][idx];md.qpos[3:7]=a['root_quaternion_wxyz'][idx];md.qpos[qa]=a['joint_position_rad'][idx]
        mujoco.mj_kinematics(model,md)
        set_robot_frame(model,md,ops,frame)
    path=OUT/'matched_replay.usdc';assert stage.Flatten().Export(str(path))
    check=Usd.Stage.Open(str(path));assert check.GetEndTimeCode()==len(ts)-1
    assert not check.GetRootLayer().subLayerPaths and check.GetPrimAtPath('/Robot').IsValid()
    print('PASS self-contained USD:',path,'visual meshes',len(ops),'animation frames',len(ts),'bytes',path.stat().st_size)

if __name__=='__main__':main()
