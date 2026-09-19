"""Static, local-only viewer. No physics stepping, ROS, policy, or robot transport."""
from pathlib import Path
import argparse,time
import mujoco
import mujoco.viewer
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',choices=['terrain','robot'],default='terrain')
    p.add_argument('--collision',action='store_true')
    a=p.parse_args()
    xml=ROOT/('mujoco/terrain/scene_contact_v1.xml' if a.scene=='terrain' else 'mujoco/robot_scene/scene.xml')
    model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model)
    if a.scene=='robot':
        key=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,'start_pose')
        mujoco.mj_resetDataKeyframe(model,data,key)
    mujoco.mj_forward(model,data)
    print('STATIC VIEW ONLY: no physics, no policy, no hardware connection. Close window to exit.')
    with mujoco.viewer.launch_passive(model,data) as viewer:
        viewer.cam.lookat[:]=[14,11,2];viewer.cam.distance=48;viewer.cam.azimuth=130;viewer.cam.elevation=-35
        viewer.opt.geomgroup[:]=[0,1 if a.collision else 0,0 if a.collision else 1,1 if a.collision else 0,0,0]
        while viewer.is_running():
            viewer.sync();time.sleep(.03)
if __name__=='__main__':main()
