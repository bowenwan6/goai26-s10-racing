"""Read-only package checks. Prints JSON; does not write into the package."""
from pathlib import Path
import hashlib,json,subprocess,sys
import xml.etree.ElementTree as ET
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1]
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def cloud_check():
    p=ROOT/'maps/v3/full_cloud.pcd';header={}
    with p.open('rb') as f:
        while True:
            line=f.readline().decode('ascii').strip()
            if line and not line.startswith('#'):
                key,*value=line.split();header[key]=value
                if key=='DATA':break
        offset=f.tell()
    assert header['DATA']==['binary']
    assert header['FIELDS']==['x','y','z','intensity']
    assert header['SIZE']==['4']*4 and header['TYPE']==['F']*4 and header['COUNT']==['1']*4
    count=int(header['POINTS'][0]);assert p.stat().st_size-offset==count*16
    points=np.memmap(p,dtype='<f4',offset=offset,mode='r',shape=(count,4))
    assert np.isfinite(points).all()
    return dict(points=count,fields=header['FIELDS'],all_finite=True,min_xyz=points[:,:3].min(axis=0).tolist(),max_xyz=points[:,:3].max(axis=0).tolist())
def resources(xml,seen=None):
    seen=set() if seen is None else seen
    xml=xml.resolve()
    if xml in seen:return seen
    seen.add(xml)
    for node in ET.parse(xml).getroot().iter():
        if 'file' not in node.attrib:continue
        ref=Path(node.attrib['file']);assert not ref.is_absolute(),str(ref)
        target=(xml.parent/ref).resolve();assert target.is_relative_to(ROOT.resolve()) and target.is_file(),str(target)
        if node.tag=='include':resources(target,seen)
    return seen
def main():
    manifest=ROOT/'MANIFEST_SHA256.json'
    hashes_checked=0
    if manifest.exists():
        for row in json.loads(manifest.read_text()):
            assert sha(ROOT/row['path'])==row['sha256'],row['path'];hashes_checked+=1
    scenes=[]
    for rel in ['mujoco/terrain/scene_contact_v1.xml','mujoco/robot_scene/scene.xml']:
        xml=ROOT/rel;refs=resources(xml);m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m)
        if 'robot_scene' in rel:
            assert (m.nq,m.nv,m.nu)==(23,22,16)
            mujoco.mj_resetDataKeyframe(m,d,mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_KEY,'start_pose'))
        mujoco.mj_forward(m,d);assert np.isfinite(d.qpos).all()
        scenes.append(dict(path=rel,nq=m.nq,nv=m.nv,nu=m.nu,ngeom=m.ngeom,nmesh=m.nmesh,xml_files=len(refs),loaded=True))
    probe=subprocess.run([sys.executable,str(ROOT/'mujoco/terrain/check_scene.py')],capture_output=True,text=True)
    assert probe.returncode==0,probe.stdout+probe.stderr
    results=dict(engine=mujoco.__version__,numpy=np.__version__,manifest_files_checked=hashes_checked,
                 cloud=cloud_check(),scenes=scenes,probes=json.loads(probe.stdout),passed=True,
                 coordinate_transform='identity / v3 map metres Z-up',field_accuracy_validated=False,robot_policy_test=False)
    print(json.dumps(results,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
