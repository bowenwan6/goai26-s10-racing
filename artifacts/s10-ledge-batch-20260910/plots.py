"""Render readable comparisons and rebuild the batch report (Anaconda matplotlib/Pillow)."""
import base64
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

OUT=Path(__file__).resolve().parent
REVIEW=OUT.parent/'s10-recording-review-20260909'
plt.rcParams.update({'font.sans-serif':['Microsoft YaHei','SimHei','DejaVu Sans'],'axes.unicode_minus':False})
CLIPS=json.loads((OUT/'clips.json').read_text(encoding='utf-8'))
COLORS=['#a52336','#e88775','#205ea8','#66a6d7']


def angles(q):
    w,x,y,z=q.T
    return np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1))),np.degrees(np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)))


def heading(q):
    w,x,y,z=q.T
    return np.degrees(np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))))


def matrix(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def label(r,mode):
    if r['completed']:return '整段通过'
    if mode=='cycle' and r.get('up_completed'):return '上台通过，后续未通过'
    if mode=='cycle' and r.get('down_completed'):return '已退回，完整上台未通过'
    return '未通过'


def plot_clip(c):
    folder=OUT/c['id'];s=json.loads((folder/'summary.json').read_text());r=s['selected']
    ref=np.load(folder/'reference.npz');fk=np.load(folder/'recorded_kinematics.npz')
    baseline=np.load(folder/'baseline.npz');sim=np.load(folder/'selected.npz')
    g=json.loads((folder/'geometry.json').read_text());config=json.loads((folder/'configuration.json').read_text())
    clouds=json.loads((REVIEW/'decoded'/(config['source_recording_id']+'.clouds.json')).read_text())
    times=json.loads((folder/'render_times.json').read_text())
    fig,axs=plt.subplots(2,4,figsize=(17,8),gridspec_kw={'height_ratios':[1,1.2]})
    for k,t in enumerate(times):
        ax=axs[0,k];i=int(np.argmin(abs(ref['time_s']-t)));rot=matrix(ref['base_quaternion_wxyz'][i])
        for topic,color in [('/rslidar_front/points','#c58824'),('/rslidar_rear/points','#80aabe')]:
            cloud=min(clouds[topic],key=lambda x:abs(x['src']-t))
            if abs(cloud['src']-t)>.3:continue
            p=np.frombuffer(base64.b64decode(cloud['xyz_mm_b64']),dtype='<i2').reshape(-1,3)/1000@rot.T
            p=p[(abs(p[:,1])<.65)&(p[:,0]>-1)&(p[:,0]<1.5)&(p[:,2]>-1)&(p[:,2]<.5)]
            ax.scatter(p[:,0],p[:,2],s=1,color=color,alpha=.4)
        bones=fk['positions'][np.argmin(abs(fk['time_s']-t))]
        ax.plot(bones[[1,5,13,9,1],0],bones[[1,5,13,9,1],2],c='#485467',lw=2)
        for leg,col in enumerate(COLORS):
            ix=np.arange(1+leg*4,5+leg*4);ax.plot(bones[ix,0],bones[ix,2],'-o',c=col,lw=2,ms=3)
            ax.add_patch(plt.Circle(bones[ix[-1],[0,2]],.081,fill=False,color=col,lw=1.5))
        ax.set(xlim=(-1,1.5),ylim=(-1,.55),title=f'实录 {t:.2f} s',xlabel='相对机身 X / m')
        ax.set_aspect('equal');ax.grid(alpha=.15)
        if k==0:ax.set_ylabel('IMU 姿态 + 关节 FK + 前后点云\n相对机身 Z / m')
        ax=axs[1,k]
        with Image.open(folder/f'mesh_{k}.ppm') as im:ax.imshow(np.asarray(im))
        ax.axis('off');ax.set_title(f'自由动力学回放 {t:.2f} s')
    scope='上台阶段通过' if 'parent_mode' in c and r['completed'] else label(r,c['mode'])
    fig.suptitle(f"{c['id']} · {scope} · 台高 {c['height']*100:.0f} cm · 距离 {r['offset_m']*100:+.0f} cm · 延迟 {r['delay_s']:+.1f} s · 腿 kp/kd={r['kp_leg']:g}/{r['kd_leg']:g}",fontsize=15)
    fig.text(.5,.02,'上排以实录机身居中；下排为 MuJoCo 仿真。点云为约 2 Hz 预览，关节插值为 200 Hz；没有重建实录世界位移。',ha='center',fontsize=10)
    fig.tight_layout(rect=[0,.04,1,.95]);fig.savefig(folder/'comparison.png',dpi=140);plt.close(fig)
    fig,axs=plt.subplots(5,1,figsize=(13,13),sharex=True)
    pitch,roll=angles(ref['base_quaternion_wxyz'])
    axs[0].plot(ref['time_s'],pitch,c='#1d3349',lw=2,label='实录 IMU')
    axs[1].plot(ref['time_s'],roll,c='#1d3349',lw=2,label='实录 IMU')
    axs[2].plot(ref['time_s'],heading(ref['base_quaternion_wxyz']),c='#1d3349',lw=2,label='实录 IMU')
    for data,name,color,style in [(baseline,'零修正','#ad8174','--'),(sim,'选中修正','#247f99','-')]:
        pp,rr=angles(data['qpos'][:,3:7]);t=data['log'][:,0]
        axs[0].plot(t,pp,c=color,ls=style,label=name);axs[1].plot(t,rr,c=color,ls=style,label=name)
        axs[2].plot(t,heading(data['qpos'][:,3:7]),c=color,ls=style,label=name)
        yaw=np.deg2rad(g['yaw_deg']);n=np.array([np.cos(yaw),np.sin(yaw),0])
        edge=g['initial_distance_m']+(r['offset_m'] if name=='选中修正' else 0)
        axs[3].plot(t,edge-data['qpos'][:,:3]@n,c=color,ls=style,label=name+'机身到台沿')
    if g['fits']:axs[3].scatter([x['time_s'] for x in g['fits']],[x['edge_horizontal_distance_m'] for x in g['fits']],s=12,c='#be8c33',label='原始 10 Hz 点云估计')
    for j,col in zip([3,7,11,15],COLORS):axs[4].plot(ref['time_s'],ref['joint_velocity'][:,j],c=col,label=['左前','右前','左后','右后'][j//4])
    for ax in axs:
        ax.axvline(c['peak'],c='#999',ls=':',lw=1)
        if c.get('reverse_start'):ax.axvline(c['reverse_start'],c='#b38555',ls=':',lw=1)
        ax.grid(alpha=.2);ax.legend(loc='upper left',ncol=3,fontsize=9)
    for ax,y in zip(axs,['俯仰 / °（负值抬头）','侧倾 / °','朝向 yaw / °','机身到沿 / m','映射后的轮速 / rad/s']):ax.set_ylabel(y)
    axs[-1].set_xlabel('原始记录相对时间 / s（不是裁剪后从零计时）')
    fig.suptitle(c['id']+' · 姿态、前后距离和真实轮速对照',fontsize=15);fig.tight_layout(rect=[0,0,1,.96])
    fig.savefig(folder/'diagnostics.png',dpi=130);plt.close(fig)
    return s


def report(summaries):
    rows=[];phase_rows=[];totals={'units':len(summaries),'completed':0,'up_completed':0,'down_completed':0,'trials':0}
    fig,axs=plt.subplots(6,2,figsize=(16,24))
    for s,ax in zip(summaries,axs.flat):
        c=s['clip'];r=s['selected'];g=s['geometry'];totals['completed']+=bool(r['completed'])
        totals['up_completed']+=bool(r.get('up_completed'));totals['down_completed']+=bool(r.get('down_completed'));totals['trials']+=s['trial_count']
        with Image.open(OUT/c['id']/'mesh_1.ppm') as a, Image.open(OUT/c['id']/'mesh_3.ppm') as z:
            ax.imshow(np.concatenate([np.asarray(a),np.asarray(z)],axis=1))
        ax.axis('off');ax.set_title(f"{c['id']}  {c['start']}–{c['end']} s  {label(r,c['mode'])}\n左：上/下台峰值附近；右：末帧 | 修正 {r['offset_m']*100:+.0f} cm / {r['delay_s']:+.1f} s",fontsize=11)
        mode={'cycle':'上台→倒退下台','up':'上台','down':'前进下台'}[c['mode']]
        rmse=r.get('edge_distance_rmse_m');rmse_text=f'{rmse*100:.1f}' if rmse is not None else '—'
        rows.append(f"| [{c['id']}]({c['id']}/comparison.png) | {c['start']}–{c['end']} | {mode} | {c['height']*100:.0f} | {g['initial_distance_m']*100:.1f} | {r['offset_m']*100:+.0f} / {r['delay_s']:+.1f} | {label(r,c['mode'])} | {rmse_text} |")
        fall=r.get('first_tilt_over_60_source_s');fall_text=f'{fall:.3f}' if fall is not None else '无'
        up_time=r.get('up_completed_source_s');up_text=f'{up_time:.3f}' if up_time is not None else '未达到/不适用'
        phase_rows.append(f"| [{c['id']}]({c['id']}/diagnostics.png) | {s['successful_trials']}/{s['trial_count']} | {up_text} | {fall_text} | {r['final_tilt_deg']:.1f}° | {s['baseline']['reference_orientation_rmse_deg']:.1f}° → {r['reference_orientation_rmse_deg']:.1f}° |")
    fig.suptitle('剩余四条高台记录 · 每段选中方案的关键帧',fontsize=18);fig.tight_layout(rect=[0,0,1,.98]);fig.savefig(OUT/'results_overview.png',dpi=130);plt.close(fig)
    (OUT/'batch_summary.json').write_text(json.dumps(dict(totals=totals,clips=summaries),ensure_ascii=False,indent=2),encoding='utf-8')
    text=f'''# 剩余高台数据匹配记录与经验（2026-09-10）

**短台信息修正：** 用户补充实物台深只够机器狗，前方是草地。最新参考见 [裁剪与实录证据](cropped_ascent/REPORT.md)。下文的 3 m 深场景、整段统计和 12.95/30.15 s“回摆”解释均为历史实验/判断，不能作为真实短台结论；后两处动作需结合远侧台沿重新复核。

后续已按用户要求单独处理上台，见 [上台专项结果与判据修正](ascent/REPORT.md)。下述为此前完整循环及旧越沿余量判据的历史记录。

本次处理剩余 4 条高台录制，整理成 12 段连续回放，覆盖 10 次上台候选、2 次前进下台和 6 次倒退/退回阶段。其中 150921 含控制中断，151220 结束时是否完成上台尚未确认。**共搜索 {totals['trials']} 组参数，当前几何和倾角判据下整段通过 {totals['completed']}/12；选中方案中，上台阶段通过 {totals['up_completed']}/10，下台或退回阶段通过 {totals['down_completed']}/8。** 阶段通过不等于整段通过。

本次运行后端为 MuJoCo 自由动力学，未在 Isaac Sim 中运行。输出包含可供 Isaac 导入的高台 USDA、关节参考、初始姿态和匹配参数。实录数据是关节反馈，不是原控制器动作；本次未新增人工确认或训练合格标签。

## 结果与可视化

表中距离为片段起点的机身原点到台沿法向距离；修正正值把前方台沿移远，负值移近。点云误差单位 cm，按仍能连续辨认同一台沿的接近阶段计算。点击片段编号查看真实关节/点云与仿真关键帧。各目录的 `diagnostics.png` 展示实录与仿真的俯仰、侧倾、距离、轮速曲线。

| 连续片段 | 原始时间 / s | 动作 | 高 / cm | 点云初始距离 / cm | 距离 cm / 延迟 s | 仿真结果 | 距离 RMSE / cm |
|---|---|---|---:|---:|---|---|---:|
'''+ '\n'.join(rows)+'''

![选中方案关键帧](results_overview.png)

## 阶段检查

姿态误差是同一原始时间上的整段四元数角误差；上台完成时刻是本次持续判据首次满足时刻。点击编号查看连续曲线；“无超过 60°”不代表已到达目标支持面。

| 片段 | 完整通过参数数/总数 | 上台完成 / s | 首次倾角 >60° / s | 末帧倾角 | 零修正→选中姿态 RMSE |
|---|---:|---|---|---|---|
'''+ '\n'.join(phase_rows)+'''

150146_D1 的可行方案处于距离与时序搜索边界，关键帧/曲线仍能看到下台姿态较实录滞后；它通过落地判据，并非精确复现实录。`contact_review.json` 另外重新检查了选中方案末段的轮子与支持面接触。完整通过方案均需结合这些曲线、接触与实录人工标注使用。

150146_U2 的零修正基线也能上台，13 个距离参数中有 10 个完整通过；−3 cm 是综合误差较小的选择。两个完整通过方案在末段 10 帧都重算出了四轮支持面接触。

失败形态也不同：150146_U3/U4、151220_U1 的选中方案最后仍在低地；150146_D2 和 150705_C1/C2/C4 出现明显翻倒；150705_C3 虽退回低地，但中途超过 60°倾角；150921_C1 在截断末尾仍是两后轮着地、前轮抬起。150146_C1 返回低地，但没有满足完整上台判据。因此只看最后机身回平或没有翻倒，都会漏掉未上台的情况。

## 这次必须保留的区别

1. **倒退下台也会抬头。** 150146 第一组、150705 多组都出现“先上台、再倒退”的连续过程。结合映射后的轮速变号、前方立面重新出现、后雷达所见低地回升来判断。俯仰为负不能直接标成又一次上台。
2. **上台后的回摆不一定是前进下台。** 150705 约 12.95、30.15 秒的正俯仰保留在原连续轨迹中。150705_C4 保留了台上的长等待，没有删除后拼接。
3. **下台的初始地面必须高一层。** 两段前进下台将机器人初始化在台面，台体位于台沿后侧，目标是四轮落到前方低地。上台→倒退下台则保持同一场景和连续状态，目标是退回原来低地。
4. **点云有遮挡和多平面歧义。** 150705_C1 在 9 秒附近近台沿不可见，台上物体可能被误选成台沿；起点改为 5 秒。拟合要求合适的地面、可见高度、法向，并在连续跟踪断开后停止。30 cm 台前缘点云并非严格竖直，可能包含真实外形和传感器误差；本次仍以方块碰撞体作近似。
5. **前进下台没有足够的立面回波时，只能给距离区间。** 使用高、低两个支持面可见点之间的区间估沿，`geometry.json` 保存各横向条带的上下界；区间中点不是毫米级实测真值。
6. **整段录制的 success 标签不能传给每个动作。** 150921 在 16.8127 秒由状态 17 退出，连续受控参考截到 16.8 秒；其后的失稳/恢复不包装成专家上台。151220 的结尾仍需人工确认。

## 参数、证据与使用方式

距离粗搜索为 −18…+18 cm，步长 3 cm，先保持时序不变；未完成的普通片段继续搜索延迟 −0.3、−0.2、−0.1、+0.1、+0.2、+0.3 秒。正延迟表示动作更晚。150921 中断案例只按姿态和点云一致性选取，不按成功强行修饰原记录。

普通片段优先选整段通过的参数，再最小化“整段姿态 RMSE（度）+ 200 × 接近距离 RMSE（米）”；没有通过方案则保存误差最小的失败方案。所有试验使用同一关节参考、模型、PD 和台高。边界方案表示本次范围内最佳，不证明范围外不存在可行参数。`search.json` 保存所有尝试，`baseline.*` 是零修正对照，`selected.*` 是选中方案。

上台通过要求四个轮心越过台沿至少一个轮半径，处于台体宽度/深度内，轮心高度接近台面加轮半径，倾角低于 25°，持续约 0.2 秒；完整上台片段末段仍须满足。下台/退回要求末段四轮回到相应低地，且该阶段没有超过 60°倾角。整段通过还要求整个片段没有超过 60°倾角。这些是本次匹配筛选判据，不是实机安全验收或训练合格标准。

每段的 `selected_alignment.json` 给出实测名义距离、仿真修正、初态、关节顺序和沿方向；`reference.npz` 为 200 Hz 的实录关节/IMU参考；`selected.npz` 为 50 Hz 仿真姿态和轮心；`recorded_kinematics.npz` 为 50 Hz、以机身为原点的实录 FK。不要把后者当成世界轨迹。

平台高度取记录值；宽 2 m、深 3 m 是测试假设。外参、碰撞几何、摩擦、反馈替代原动作以及初始线速度的误差仍会改变接触时机；仿真成功的距离修正不能回写为传感器标定。上一条 151135 移近 6 cm 能完成上台，证明距离会影响结果，但不是所有录制通用的修正量。

完整复现步骤与 Isaac 交接约定见 [使用方法](../../docs/S10_LEDGE_MATCHING_GUIDE_ZH.md)。上一条已人工确认的示例见 [151135 的距离复核](../s10-ledge-match-20260909/distance_search/REPORT.md)。

## 已执行的检查

PointCloud2 行填充/大小端、平面拟合以及上下台分支自检通过；全部片段关节/IMU参考有限、四元数归一，所选区间原始关节/IMU相邻时间间隔均小于 30 ms。参考关节超出 SDK 角度范围的最大值为 0.01256 rad（150146_U4，约 0.72°），保留实录值并由物理关节限位约束，未静默裁平。

共享动力学函数修改后，151135 原零修正基线逐帧重放一致（325 帧，`atol=1e-10`）。本次原始 bag 只读，现有人工标注未改动。新增方法和小型结果可以进入 Git；NPZ、模型网格、生成图和原始数据仍作为本机生成物，换机需按使用方法重建。
'''
    (OUT/'REPORT.md').write_text(text,encoding='utf-8')
    print(json.dumps(totals),flush=True)


if __name__=='__main__':
    summaries=[plot_clip(c) for c in CLIPS]
    report(summaries)
