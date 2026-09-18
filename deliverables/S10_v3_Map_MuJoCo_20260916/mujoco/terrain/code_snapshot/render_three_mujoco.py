"""Actual MuJoCo pixels and saved state playback, no policy or ROS/network."""
from pathlib import Path
import sys,argparse,subprocess,json
import numpy as np,mujoco
sys.path.append('/opt/anaconda3/lib/python3.12/site-packages')
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1]
FONT='/System/Library/Fonts/STHeiti Medium.ttc'
def annotate(rgb,lines):
    im=Image.fromarray(rgb);draw=ImageDraw.Draw(im);font=ImageFont.truetype(FONT,24)
    draw.rectangle((0,0,im.width,84),fill='#152638')
    for i,line in enumerate(lines):draw.text((22,12+32*i),line,font=font,fill='white' if i==0 else '#f2cb86')
    return im
def camera(look,distance,az,el):
    c=mujoco.MjvCamera();c.lookat[:]=look;c.distance=distance;c.azimuth=az;c.elevation=el;return c
def encoder(path):
    return subprocess.Popen(['/opt/homebrew/bin/ffmpeg','-loglevel','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','24','-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('name');ap.add_argument('--stills-only',action='store_true');args=ap.parse_args();out=ROOT/args.name
    m=mujoco.MjModel.from_xml_path(str(out/'scene_contact_v1.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d);opt=mujoco.MjvOption();opt.geomgroup[3]=0
    # Scene cameras use actual meter scale. No vertical exaggeration.
    views=[('10_mujoco_overview',[14.5,12,2.1],43,130,-53,'全段'),('11_mujoco_Start',[5,1,-.3],17,120,-45,'Start：局部微坡与曲边'),('12_mujoco_B',[20,12,2.1],30,145,-34,'原 B：保留踏面/立面'),('13_mujoco_Post',[28.1,23.6,4.6],9,130,-40,'B 后短平台：边侧障碍尚未建模')]
    with mujoco.Renderer(m,height=900,width=1600) as renderer:
        for name,look,distance,az,el,title in views:
            renderer.update_scene(d,camera=camera(look,distance,az,el),scene_option=opt);renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
            annotate(renderer.render(),['MuJoCo 实际导入｜'+title,'有限碰撞体 + 独立可视层；结构假设模型，非现场尺寸验收。']).save(out/f'media/{name}.png')
    print('MuJoCo stills complete',flush=True)
    if args.stills_only:return
    proc=encoder(out/'media/07_scene_tour.mp4')
    chapters=[('总览：Start → 原 B → B 后约 4.9 m',[14.5,12,2.1],43,115,-53,5),('Start：只对同类路面分面，曲边保持细节',[5,1,-.3],17,110,-42,6),('原 B：这轮没有重新平滑台阶',[20,12,2.1],30,128,-34,7),('B 后短平台：保留微坡，未知侧边不补成地板',[28.1,23.6,4.6],9,115,-40,6)]
    with mujoco.Renderer(m,height=720,width=1280) as renderer:
        for title,look,distance,az,el,seconds in chapters:
            for i in range(seconds*24):
                c=camera(look,distance,az+20*i/(seconds*24-1),el);renderer.update_scene(d,camera=c,scene_option=opt);renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
                im=annotate(renderer.render(),[title,'24 秒实际 MuJoCo 镜头浏览｜不是机器狗行走，也不是 policy/follower。']);proc.stdin.write(np.asarray(im).tobytes())
    proc.stdin.close();assert proc.wait()==0
    print('24s scene tour complete',flush=True)
    # Replays preserve every recorded frame at 24 fps: 2s on + reset + 1s off.
    m=mujoco.MjModel.from_xml_path(str(out/'probes_contact_v1.xml'));d=mujoco.MjData(m);replay=np.load(out/'probe_replay_contact_v1.npz');proc=encoder(out/'media/09_contact_counterexample_1x.mp4')
    with mujoco.Renderer(m,height=720,width=1280) as renderer:
        # Two independent camera views of exactly the same saved physics.
        for region,look,distance,az,el in [('Start 近景',[4.6,.5,-.2],14,110,-32),('B 台阶近景',[20,12,2.1],24,140,-35)]:
            for q,t,phase in zip(replay['qpos'],replay['time'],replay['phase']):
                d.qpos[:]=q;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=camera(look,distance,az,el),scene_option=opt);renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
                state='碰撞开启' if phase=='on' else '关闭碰撞反例（独立重置）'
                im=annotate(renderer.render(),[f'真实落球探针｜{region}｜{state}｜t={t:.2f}s','1×物理时间；同一实验换镜头完整回放；共 36 个探针，不是机器人策略。']);proc.stdin.write(np.asarray(im).tobytes())
    proc.stdin.close();assert proc.wait()==0
    print('6s 1x probe positive/counterexample replay complete',flush=True)
if __name__=='__main__':main()
