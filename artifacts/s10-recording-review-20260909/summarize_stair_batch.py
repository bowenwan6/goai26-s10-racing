"""Write batch review index, phase ledger, and per-channel provisional training masks."""
import csv
import html
import json
import numpy as np
from batch_match_stairs import BATCH, CONFIG
from fit_stairs import load

def main():
    rows=[];ledger=[];cards=[]
    for job in CONFIG['jobs']:
        out=BATCH/job['name'];s=json.loads((out/'summary.json').read_text());fits=json.loads((out/'fitted_flights.json').read_text());replay=json.loads((out/'replay_check.json').read_text());terrain=json.loads((out/'terrain_quality.json').read_text())
        a=np.load(out/'reference_motion.npz');check=np.load(out/'wheel_proxy_check.npz');mask=a['reference_valid']
        _,original,_=load(job['recording_id'])
        native=np.searchsorted(original['JOINTS_DATA_src'],a['joint_source_ns'])
        assert np.array_equal(original['JOINTS_DATA_src'][native],a['joint_source_ns'])
        assert np.array_equal(original['JOINTS_DATA_v'][native,:16],a['raw_joint_position_rad'])
        assert np.all(np.diff(a['time_s'])>0) and np.isfinite(a['root_position_m']).all()
        assert np.max(abs(np.linalg.norm(a['root_quaternion_wxyz'],axis=1)-1))<1e-6
        assert not a['contact_label_valid'].any()
        assert (out/'matched_replay.mp4').stat().st_size>1000 and (out/'matched_replay.usdc').stat().st_size>1000
        idx=np.searchsorted(check['time_s'],a['time_s']).clip(0,len(check['time_s'])-1)
        gap=check['gap_to_proxy_m'][idx]
        geometry=np.isfinite(gap).all(axis=1)&(np.min(gap,axis=1)>-.04)&(np.min(abs(gap),axis=1)<.06)
        values={k:a[k] for k in a.files if k!='root_reference_weight'};values.update(root_xy_weight=mask.astype('f4')*.25,
                   root_z_weight=(mask&geometry).astype('f4')*.05,joint_reference_weight=mask.astype('f4')*.5,
                   orientation_reference_weight=mask.astype('f4')*.25,terrain_contact_consistency_candidate=geometry)
        # Weights are documented starting heuristics, not training-tested reward coefficients.
        np.savez_compressed(out/'training_reference.npz',**values)
        for begin,end,label in job['phases']:
            use=(a['time_s']>=begin)&(a['time_s']<end)
            ledger.append(dict(job=job['name'],recording_id=job['recording_id'],start_s=begin,end_s=end,
                               phase_candidate=label,samples=int(use.sum()),reference_valid_fraction=float(mask[use].mean()) if use.any() else 0.,
                               root_z_enabled_fraction=float((mask&geometry)[use].mean()) if use.any() else 0.,contact_truth=False,outcome='unconfirmed'))
        s.update(fitted_flights=fits,replay=replay,terrain=terrain,root_z_enabled_fraction=float(np.mean(mask&geometry)))
        rows.append(s)
        ftext=' / '.join(f"{f['count']}级（{'上' if f['direction']=='stairs_up' else '下'}），中位误差 {100*f['median_m']:.1f} cm" for f in fits)
        name=html.escape(job['name']);cards.append(f'<section><h2>{name}</h2><p>源时间 {job["start_s"]}–{job["end_s"]} 秒；{html.escape(ftext)}</p><video controls preload="metadata" width="800" src="{name}/matched_replay.mp4"></video><p><a href="{name}/matched_replay.usdc">完整 USD 动画</a> · <a href="{name}/training_reference.npz">带权重与掩码的参考数据</a> · <a href="{name}/trajectory.png">轨迹图</a> · <a href="{name}/fitted_flights.json">几何参数与误差</a></p></section>')
    result=dict(jobs=rows,excluded=CONFIG['excluded'],duration_s=sum(r['duration_s'] for r in rows),samples=sum(r['samples'] for r in rows),
                flight_phases=sum(len(r['fitted_flights']) for r in rows),training_status='soft_reference_candidates_only_no_RL_run',
                existing_first_flight='../stairs_reconstruction/REPORT.md')
    (BATCH/'index.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    with (BATCH/'phase_ledger.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(ledger[0]));writer.writeheader();writer.writerows(ledger)
    (BATCH/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>S10 楼梯匹配复核</title><style>body{font:17px system-ui;max-width:1000px;margin:40px auto;line-height:1.7;padding:0 20px;background:#f4f5f2;color:#172025}section{padding:24px 0;border-top:1px solid #bcc6c5}video{width:100%;max-width:800px}a{color:#14606b}</style><h1>S10 剩余楼梯参考匹配</h1><p>这是运动学参考候选，不是控制策略通过证明。关节来自实录，机身平移来自点云配准；轮端接触未知。低质量区间和地形不一致区间已给出掩码。所有级数与方向均保留为拟合/候选，不替代人工确认。</p><p><a href="../stairs_reconstruction/REPORT.md">原第一梯段结果</a> · <a href="phase_ledger.csv">逐阶段索引</a> · <a href="index.json">完整批次指标</a></p>'+''.join(cards),encoding='utf-8')
    text=['# 剩余楼梯匹配结果','',f"新增 {len(rows)} 个参考包，{result['duration_s']:.2f} 秒，{result['samples']:,} 个原关节样本，覆盖 {result['flight_phases']} 个上/下楼候选阶段。",'',
          '## 结果总览','','| 参考包 | 源时间（秒） | 样本 | 轨迹质量掩码通过 | 楼梯拟合中位误差 |','|---|---:|---:|---:|---:|']
    for r in rows:text.append(f"| {r['name']} | {r['start_s']}–{r['end_s']} | {r['samples']} | {100*r['reference_valid_fraction']:.1f}% | "+' / '.join(f"{100*f['median_m']:.1f} cm" for f in r['fitted_flights'])+' |')
    text.extend(['','[集中看视频和下载 USD](index.html)。各目录保留原始点云缓存、配准轨迹、原关节参考、拟合地形、几何残差和复核视频；NPZ/视频/USD 为本地产物。',
                 '', '轨迹质量通过率不是成功通行率；配准中位残差约 1 cm 不代表楼梯边缘或轮端有同样精度。拟合楼梯的 P90 误差约 10–17 cm，轮端尚有偏差。本批不强制抬高机身消除穿插，不宣称碰撞或动力学通过。',
                 '', '## 排除区间',''])
    for r in CONFIG['excluded']:text.append(f"- `{r['recording_id']}` {r['start_s']}–{r['end_s']} s：{r['reason']}")
    text.extend(['','## 使用入口','','完整操作和训练用法见 [使用方法与经验文档](../../../docs/S10_STAIRS_MATCHING_GUIDE_ZH.md)。先看逐阶段 CSV 与回放，再使用 training_reference.npz 的逐通道权重；contact_label_valid 始终为 false。'])
    (BATCH/'REPORT.md').write_text('\n'.join(text),encoding='utf-8');print(json.dumps({k:v for k,v in result.items() if k not in ('jobs','excluded')},indent=2))

if __name__=='__main__':main()
