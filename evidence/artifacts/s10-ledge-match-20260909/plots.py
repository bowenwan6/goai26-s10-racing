"""Render results with the existing Anaconda plotting runtime: python -s -B .../plots.py."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy.spatial.transform import Rotation
from PIL import Image, ImageDraw

OUT=Path(__file__).resolve().parent
a=json.loads((OUT/'alignment.json').read_text())
rows=json.loads((OUT/'fits.json').read_text())
rows=[r for r in rows if r['accepted_track']]
ref=np.load(OUT/'expert_reference.npz');sim=np.load(OUT/'tracking.npz');cloud=np.load(OUT/'full_clouds.npz')
fig,axes=plt.subplots(2,2,figsize=(14,9),layout='constrained')
fig.suptitle('38 cm ledge: lidar alignment and measured-state tracking trial',fontsize=17)
index=np.argmin(abs(cloud['time_s']-9)+np.where(cloud['topic']=='/rslidar_front/points',0,100))
xyz=cloud['xyz'][cloud['offsets'][index]:cloud['offsets'][index+1]]
p=Rotation.from_quat(ref['base_quaternion_wxyz'][0,[1,2,3,0]]).apply(xyz)
n=np.r_[a['edge_normal_xy'],0]; tangent=np.array([-n[1],n[0],0])
along=p@n; side=p@tangent; z=p[:,2]+a['initial_base_position_m'][2]
select=(abs(side)<.5)&(along>.1)&(along<2.4)&(z>-.15)&(z<.8)
ax=axes[0,0];ax.scatter(along[select],z[select],s=1,alpha=.25,label='Raw front lidar (near centreline)')
ax.add_patch(Rectangle((a['edge_horizontal_distance_m'],0),2.5,.38,color='#dc963d',alpha=.2,label='38 cm solid-block model'))
ax.axhline(0,color='gray');ax.scatter([0],[a['initial_base_position_m'][2]],marker='+',s=100,color='black',label='Initial base origin')
ax.set(xlim=(-.1,2.4),ylim=(-.15,.8),xlabel='Distance along edge normal (m)',ylabel='Height above model floor (m)',title='Source 9.0 s: matched initial geometry')
ax.legend(fontsize=8,loc='upper right')
ax=axes[0,1]; select=(p[:,0]>.1)&(p[:,0]<1)&(abs(p[:,1])<.7)&(p[:,2]>-.5)&(p[:,2]<.1)
ax.scatter(p[select,0],p[select,1],s=1,alpha=.3)
yy=np.linspace(-.7,.7,100);xx=(a['edge_horizontal_distance_m']-n[1]*yy)/n[0]
ax.plot(xx,yy,color='#b86600',label=f"Fitted normal yaw {a['edge_yaw_deg']:.2f} deg")
ax.scatter([0],[0],marker='+',s=90,color='black');ax.set(xlim=(-.05,1),ylim=(-.7,.7),xlabel='Forward X (m)',ylabel='Left Y (m)',title='Top view: wall fit and base origin');ax.legend(fontsize=8)
ax=axes[1,0];t=np.array([r['time_s'] for r in rows]);distance=np.array([r['edge_horizontal_distance_m'] for r in rows])
ax.plot(t,distance,label='Lidar wall distance (accepted track)',color='#168985')
log=sim['log'];ax.plot(log[:,0],a['edge_horizontal_distance_m']-sim['qpos'][:,:3]@n,label='Free-dynamics model distance',color='#d25732')
ax.axvspan(12.2,15.5,color='gray',alpha=.12,label='Original wall no longer tracked')
ax.set(xlim=(8.8,15.5),ylim=(0,.85),xlabel='Recording source time (s)',ylabel='Base-to-wall distance (m)',title='Motion diverges despite matched initial geometry');ax.legend(fontsize=8)
ax=axes[1,1]
rp=Rotation.from_quat(ref['base_quaternion_wxyz'][:,[1,2,3,0]]).as_euler('xyz',degrees=True)
sp=Rotation.from_quat(sim['qpos'][:,[4,5,6,3]]).as_euler('xyz',degrees=True)
ax.plot(ref['time_s'],rp[:,1],label='Recorded IMU pitch',color='#168985');ax.plot(log[:,0],sp[:,1],label='Simulated pitch',color='#d25732')
ax.set(xlabel='Recording source time (s)',ylabel='Pitch (deg)',title='State-reference playback did not reproduce successful ascent');ax.legend(fontsize=8)
for ax in axes.flat:ax.grid(alpha=.2)
fig.savefig(OUT/'alignment_comparison.png',dpi=155);plt.close(fig)
out=Image.new('RGB',(1440,700),'white');draw=ImageDraw.Draw(out)
for i,shot_time in enumerate([9,11,12,12.5,13,14]):
    im=Image.open(OUT/f'tracking_{i}.ppm');im.thumbnail((480,320));x=i%3*480;y=i//3*350
    out.paste(im,(x,y+25));draw.text((x+15,y+7),f'MuJoCo dynamics | source {shot_time:.1f}s',fill='black')
out.save(OUT/'tracking_preview.png')
# One useful comparison at the end of the measured-wall track.
k=np.argmin(abs(log[:,0]-12.2));measured=min(rows,key=lambda r:abs(r['time_s']-12.2))
metrics=dict(source_time_s=float(log[k,0]),lidar_distance_m=measured['edge_horizontal_distance_m'],
             simulated_distance_m=float(a['edge_horizontal_distance_m']-sim['qpos'][k,:3]@n))
select=(t>=9)&(t<=12.2)
pred=a['edge_horizontal_distance_m']-sim['qpos'][:,:3]@n
error=np.interp(t[select],log[:,0],pred)-distance[select]
metrics['approach_distance_rmse_m']=float(np.sqrt(np.mean(error**2)))
metrics['approach_distance_abs_error_p95_m']=float(np.percentile(abs(error),95))
(OUT/'comparison_metrics.json').write_text(json.dumps(metrics,indent=2))
print(json.dumps(metrics))

search=OUT/'distance_search'
if (search/'mesh_1_3.ppm').exists():
    output=Image.new('RGB',(1280,490),'white');draw=ImageDraw.Draw(output)
    for case in range(2):
        for j,stamp in enumerate([12.2,12.85,13.5,14.5]):
            im=Image.open(search/f'mesh_{case}_{j}.ppm');im.thumbnail((320,210));x=j*320;y=case*245
            output.paste(im,(x,y+30));draw.text((x+8,y+8),f"{'Original gap' if case==0 else '6 cm closer'} | {stamp:.2f}s",fill='black')
    output.save(search/'before_after.png')
    fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True,layout='constrained')
    for leg,ax in enumerate(axes):
        for joint,label in enumerate(['Hip roll','Hip pitch','Knee']):
            ax.plot(ref['time_s'],np.degrees(ref['joint_position'][:,leg*4+joint]),label=label)
        ax.axvspan(12.5,13.19,color='gray',alpha=.12)
        ax.set(ylabel=['FL','FR','HL','HR'][leg]+' angle (deg)',xlim=(10.5,14.5));ax.grid(alpha=.2)
    axes[0].set_title('Recorded leg joints; shaded = IMU nose-up phase');axes[0].legend(ncol=3)
    axes[-1].set_xlabel('Recording source time (s)');fig.savefig(search/'joint_phases.png',dpi=140);plt.close(fig)
