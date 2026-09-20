"""Reuse the existing comparisons and add an ascent-only result/contact report."""
import json
import numpy as np
from PIL import Image
import plots as p

p.OUT=p.OUT/'ascent'
p.CLIPS=json.loads((p.OUT/'clips.json').read_text(encoding='utf-8'))


def main():
    summaries=[p.plot_clip(c) for c in p.CLIPS]
    rows=[];fig,axes=p.plt.subplots(5,2,figsize=(16,20))
    for s,ax in zip(summaries,axes.flat):
        c=s['clip'];r=s['selected'];folder=p.OUT/c['id'];sim=np.load(folder/'selected.npz');ref=np.load(folder/'reference.npz')
        g=s['geometry'];n=np.array([np.cos(np.deg2rad(g['yaw_deg'])),np.sin(np.deg2rad(g['yaw_deg'])),0])
        margin=float((sim['wheel_positions'][-1]@n-r['initial_distance_m']).min())
        status='上台且末段保持支撑' if r['completed'] else '曾上台，末段未保持' if r['up_completed'] else '未完成上台'
        rmse=r['edge_distance_rmse_m'];rmsetext=f'{rmse*100:.1f}' if rmse is not None else '—'
        rows.append(f"| [{c['id']}]({c['id']}/comparison.png) | {c['start']}–{c['end']} | {status} | {r['offset_m']*100:+.0f}/{r['delay_s']:+.1f} | {r['kp_leg']:.0f}/{r['kd_leg']:g}/{r['kd_wheel']:g} | {r['reference_orientation_rmse_deg']:.1f}° / {rmsetext} cm | {margin*100:.1f} |")
        with Image.open(folder/'mesh_1.ppm') as im1,Image.open(folder/'mesh_3.ppm') as im2:
            ax.imshow(np.concatenate([np.asarray(im1),np.asarray(im2)],axis=1))
        ax.axis('off');ax.set_title(c['id']+' · '+status+'\n左：抬升附近；右：上台片段末尾',fontsize=11)
        chart,axs=p.plt.subplots(3,1,figsize=(11,8),sharex=True)
        rt=ref['time_s'];t=sim['log'][:,0]
        axs[0].plot(rt,p.angles(ref['base_quaternion_wxyz'])[0],c='#172e40',label='实录俯仰')
        axs[0].plot(t,p.angles(sim['qpos'][:,3:7])[0],c='#277e93',label='仿真俯仰');axs[0].set_ylabel('俯仰 / °');axs[0].legend()
        for j,col in enumerate(p.COLORS):
            axs[1].plot(t,sim['wheel_positions'][:,j,2]-.081-c['height'],c=col,label=['左前','右前','左后','右后'][j])
        axs[1].axhline(0,c='#777',ls='--');axs[1].set_ylabel('轮心减半径后\n相对台面高度 / m');axs[1].legend(ncol=4)
        axs[2].imshow(sim['wheel_top_contact'].T,origin='lower',aspect='auto',interpolation='nearest',
                      extent=[t[0],t[-1],-.5,3.5],cmap='Blues',vmin=0,vmax=1)
        axs[2].set_yticks(range(4),['左前','右前','左后','右后']);axs[2].set_ylabel('台面接触\n蓝色=是');axs[2].set_xlabel('原始录制时间 / s')
        for axis in axs[:2]:axis.grid(alpha=.2)
        chart.suptitle(c['id']+' · 台面高度与接触证据');chart.tight_layout();chart.savefig(folder/'support.png',dpi=130);p.plt.close(chart)
    fig.suptitle('仅上台阶段 · 10 个候选逐段匹配',fontsize=18);fig.tight_layout(rect=[0,0,1,.98]);fig.savefig(p.OUT/'overview.png',dpi=130);p.plt.close(fig)
    up=sum(s['selected']['up_completed'] for s in summaries);held=sum(s['selected']['completed'] for s in summaries)
    total=sum(s['trial_count'] for s in summaries)
    (p.OUT/'summary.json').write_text(json.dumps(dict(ascent_reached=up,held_at_end=held,trials=total,clips=summaries),ensure_ascii=False,indent=2),encoding='utf-8')
    report=f'''# 只处理上台：复核结果与失败原因

**后续修正：** 用户确认实物台深只够机器狗，前方是草地。下文为深 3 m 测试场景的历史结果，10/10 不能作为真实短台或新裁剪的通过率。当前参考及实录边界证据见 [短高台裁剪](../cropped_ascent/REPORT.md)，新参考尚未重跑动力学。

用户要求先集中处理上台。本轮对剩余四条录制的 10 个上台候选分别匹配，保留原始 source 时间及接近动作。**选中方案中 {up}/10 达到上台支撑判据，{held}/10 在上台片段末段保持；保存 {total} 组参数试验。** 这些是 MuJoCo 自由动力学结果，实录是否合格仍需人工确认，Isaac Sim 尚未运行。

## 为什么上一轮看起来失败很多

1. **目标混在一起了。** 之前用“上台→台上动作→倒退”整段结果和整段误差选参数。上台完成后倒退失稳也被计入失败，后半段的大姿态误差还会改变参数排名。原保存的搜索中有 7/10 个候选出现过旧上台判据通过的参数，原最终选中方案只显示 4/10。上台专项现在只按上台区间选参。
2. **台沿余量被当成了上台必要条件。** 旧几何判据要求每个轮心都超过前缘至少 8.1 cm。150705_C4 的原零修正轨迹在 66 秒已经四轮接触台面，最后侧轮轮心却只越沿约 0.5 cm，因而被误判。这一条无需移动平台即可完成上台；余量不足单独记录。
3. **也有真正的动力学偏差。** 150146_U3 的零修正、原增益轨迹在 70.3 秒俯仰/侧倾合成倾角约 45°，实录俯仰已回到约 −4°，同时左前腿出现约 22°的单关节误差。只把反馈角度作为 PD 目标，不能保证重载接触时复制原动作。本轮加入小范围增益对照；模型质量、台高、力矩限幅和实录参考均保留。

## 上台结果

“末段保持”指该裁剪片段末尾约 0.2 秒有支撑，不代表长时间站稳、倒退成功或实机安全验收。距离修正单位 cm，延迟单位 s；正距离为移远，正延迟为更晚执行。余量是末帧最靠近台沿的轮心到沿的有符号距离，不是轮外缘余量。点击编号查看实录点云/关节和仿真关键帧。

| 上台候选 | 原始区间 / s | 结果 | 距离/延迟 | 腿 kp/kd/轮 kd | 姿态 / 点云距离 RMSE | 最小轮心余量 / cm |
|---|---|---|---|---|---|---:|
'''+ '\n'.join(rows)+'''

![仅上台的关键帧](overview.png)

每个目录还有 `diagnostics.png`（实录/仿真姿态、距离、轮速）、`support.png`（四轮高度与台面接触）、`search.json`（全部参数，包括失败）、`selected_alignment.json`（场景初态、时移、增益）、`selected_scene.xml`、`selected_terrain.usda` 和原始 `reference.npz`。

## 判据与裁剪约定

上台支撑要求：每个轮子都出现过台面接触；四轮保持在台体范围和台面附近；当前至少三轮接触台面，机身倾角小于 25°，持续约 0.2 秒。接触须发生在台面高度附近，法向接近竖直；碰到立面不计。曾翻倒超过 60°后重新搭到台上不算正常上台。移动和调整姿态时允许一个轮子短暂离地，悬空但未接触过台面的轮子不能因此通过。

上台后的长期等待、明显再次调整和倒退不纳入本次短阶段评估。循环片段的结束时间根据实录首次回平及后续动作变化预先固定；独立上台记录保留原整个区间，151220 的末尾不为了通过而提前截掉。这里的裁剪不修改上一轮完整轨迹，也不表示原整段已经成功。

先试原 `kp=80, kd=2` 的距离/时序对照，若没有末段保持方案，再试腿 `120/3`、`160/4`，轮增益维持 `0.6`。提高增益只能证明当前模型的跟踪能力影响结果，不等于恢复了厂家原控制器。优先选末段保持方案，其次选曾达到上台支撑的方案，再按上台区间姿态和接近距离误差排序。

对已上台但姿态误差仍大的方案，另做一轮局部距离/时序和腿部增益复核（最高 `200/5`），以及轮部速度增益 `0.9/1.2/1.8/2.4` 的单因素对照。所有试验保持原力矩限幅；实际采用值以表格和每段配置为准。增加增益不是统一解法，试验中的失败方案也保留在 `search.json`。

**仍有匹配偏差：** 150146_C1 的接近距离误差仍较大；150146_U4 的主要问题是朝向漂移，仿真末尾 yaw 约 −44°、实录约 +6°；151220_U1 的末段姿态也未精确跟随。它们虽然能建立台面支撑，尚不作为高精度匹配或合格专家样本。其余片段也仅验证当前模型下的短时回放；`training_ready=false` 保留。

## 复现

从仓库根目录运行：

```powershell
.venv-win/Scripts/python.exe -B artifacts/s10-ledge-batch-20260910/ascent.py
python -s -B artifacts/s10-ledge-batch-20260910/ascent_report.py
```

单段可加 `--only 150146_U3`。模型和输入缓存沿用原批次；原始 bag 与人工标注未修改。先读 [完整使用方法](../../../docs/S10_LEDGE_MATCHING_GUIDE_ZH.md)。[上一轮整段报告](../REPORT.md)保留为旧判据和完整循环的历史对照，不能用它的 2/12 代表本轮上台成功率。
'''
    (p.OUT/'REPORT.md').write_text(report,encoding='utf-8')
    print(dict(ascent_reached=up,held_at_end=held,trials=total),flush=True)


if __name__=='__main__':main()
