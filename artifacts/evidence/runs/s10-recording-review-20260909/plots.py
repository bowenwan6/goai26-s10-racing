"""Static signal charts and local point-cloud contact sheets. Run with python -s -B."""
import base64
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
plt.rcParams['font.sans-serif']=['Microsoft YaHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus']=False

def angles(q):
    x,y,z,w=q.T
    return np.degrees(np.array([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x),-1,1)), np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))])).T

def main():
    (OUT/'previews').mkdir(exist_ok=True)
    rs=json.loads((OUT/'inventory.json').read_text(encoding='utf-8'))
    summary=[]
    for r in rs:
        sid=r['id'];d=np.load(OUT/'decoded'/f'{sid}.npz');anchor=int(r['anchor_ns']);dur=r['duration_s']
        t=lambda k:(d[k+'_src']-anchor)/1e9
        rp=angles(d['IMU_v'][:,:4]);mv=d['MOTION_INFO_v'];jv=d['JOINTS_DATA_v']
        fig,ax=plt.subplots(5,1,figsize=(15,10),sharex=True)
        ax[0].plot(t('IMU'),rp[:,0],label='roll');ax[0].plot(t('IMU'),rp[:,1],label='pitch');ax[0].set_ylabel('IMU deg');ax[0].legend(loc='upper right')
        for i,n in enumerate(['vx','vy','yaw rate']):ax[1].plot(t('MOTION_INFO'),mv[:,i],label=n)
        ax[1].set_ylabel('MOTION_INFO');ax[1].legend(loc='upper right')
        leg=[i for i in range(16) if i%4!=3]
        ax[2].plot(t('JOINTS_DATA'),np.sqrt(np.mean(jv[:,16:32][:,leg]**2,axis=1)),label='leg dq RMS')
        ax[2].plot(t('JOINTS_DATA'),np.mean(abs(jv[:,16:32][:,[3,7,11,15]]),axis=1),label='wheel |dq| mean',alpha=.6)
        ax[2].set_ylabel('rad/s');ax[2].legend(loc='upper right')
        ax[3].step(t('MOTION_INFO'),np.where(mv[:,5]==0,0,mv[:,5]-4096),label='gait: 0=none,1=1001,2=1002,3=1003',where='post');ax[3].step(t('MOTION_INFO'),mv[:,4],label='state',where='post')
        for c in r['gait_commands']:ax[3].axvline(c['receive_s'],color='red',alpha=.5,lw=.7)
        ax[3].set_ylabel('control');ax[3].legend(loc='upper right')
        for k in ['IMU','JOINTS_DATA','MOTION_INFO','rslidar_front_points','rslidar_rear_points']:
            ax[4].plot(t(k)[1:],np.diff(d[k+'_src'])/1e9,label=k,alpha=.7)
        ax[4].set_yscale('log');ax[4].set_ylim(.001,10);ax[4].set_ylabel('source Δt / s');ax[4].legend(loc='upper right',ncol=3,fontsize=7)
        for a in ax:a.grid(alpha=.2);a.set_xlim(0,dur)
        ax[-1].set_xlabel('原录制秒：source stamp − manifest.started_wall_ns；红线为 GAIT 接收时刻')
        fig.suptitle(sid+' | manifest: '+r['manifest']['terrain']+' / '+r['manifest']['outcome']+' (仅整段标签)')
        fig.tight_layout();fig.savefig(OUT/'previews'/f'{sid}_signals.png',dpi=110);plt.close(fig)
        clouds=json.loads((OUT/'decoded'/f'{sid}.clouds.json').read_text(encoding='utf-8'))
        times=np.linspace(0,dur,12,endpoint=False)+dur/24
        fig,axes=plt.subplots(4,3,figsize=(15,12))
        for tt,a in zip(times,axes.ravel()):
            for topic,color in [('/rslidar_front/points','#1864ab'),('/rslidar_rear/points','#d9480f')]:
                cc=min(clouds[topic],key=lambda c:abs(c['src']-tt))
                if abs(cc['src']-tt)>.65:
                    a.text(.04,.85 if 'front' in topic else .7,topic.split('/')[1]+' 无邻近帧',transform=a.transAxes,color=color)
                    continue
                xyz=np.frombuffer(base64.b64decode(cc['xyz_mm_b64']),dtype='<i2').reshape(-1,3)/1000
                xyz=xyz[abs(xyz[:,1])<1.3]
                a.scatter(xyz[:,0],xyz[:,2],s=.6,c=color,alpha=.55,rasterized=True)
            a.set_xlim(-5,5);a.set_ylim(-2.5,2.5);a.set_aspect('equal');a.grid(alpha=.2);a.set_title(f'{tt:.1f}s | x-z 本地截面，|y|<1.3m');a.set_xlabel('x / m');a.set_ylabel('z / m')
        fig.suptitle(sid+' | 前蓝 / 后橙；仅原点云声明坐标，无累积、无外参校正')
        fig.tight_layout();fig.savefig(OUT/'previews'/f'{sid}_clouds.jpg',dpi=110);plt.close(fig)
        summary.append({'id':sid,'pitch_percentiles':np.percentile(rp[:,1],[0,10,50,90,100]).tolist(),
            'bins':[{'t':float(a),'pitch':round(float(np.median(rp[(t('IMU')>=a)&(t('IMU')<a+2),1])),1),
                     'vx':round(float(np.median(mv[(t('MOTION_INFO')>=a)&(t('MOTION_INFO')<a+2),0])),2)}
                    for a in np.arange(0,min(dur,t('MOTION_INFO')[-1],t('IMU')[-1])-2,2)]})
        print('PLOTS '+sid,flush=True)
    (OUT/'signal_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__':main()
