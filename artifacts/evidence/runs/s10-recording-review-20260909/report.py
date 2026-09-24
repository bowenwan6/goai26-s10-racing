"""Publish the inventory, review queue and per-recording notes as static files."""
from collections import Counter
import html
import json
from pathlib import Path
from build import OUT, csvwrite

def read(name):return json.loads((OUT/name).read_text(encoding='utf-8'))
def esc(s):return html.escape(str(s),quote=True)
def link(r,a=0,b=None):return f'viewer.html?id={r}&t={a:.3f}'+(f'&end={b:.3f}' if b is not None else '')
def head(title):return '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><link rel="icon" href="data:,"><link rel="stylesheet" href="viewer.css"><title>'+esc(title)+'</title><main><h1>'+esc(title)+'</h1>'

def main():
    inv=read('inventory.json');cats=read('recordings.json');ps=read('passages.json');ss=read('segments.json');qq=read('review_queue.json');notes=read('observations.json')
    count=sum(c['candidate_passages'] for c in cats);contexts=sum(c['context_blocks'] for c in cats)
    queue=head('S10 待确认区间 · 56 个问题')+'<p><a href="index.html">← 总览</a> · <a href="review_queue.csv">下载 CSV</a></p><p>各按钮按源秒打开对应区间并在区间末尾停止播放。全部人工待复核；你可按录制 ID、起止秒集中回复。</p><table><tr><th>录制 ID</th><th>区间（源秒）</th><th>具体问题</th><th>静态证据</th></tr>'
    qmd=['# 待确认区间','', '全部人工待复核。时间为 source_stamp − manifest.started_wall_ns；原点纳秒见 recordings/inventory。','']
    for q in qq:
        a,b=q['start_s'],q['end_s'];sid=q['recording_id']
        queue+=f'<tr><td>{esc(sid)}</td><td><a href="{esc(q["preview"])}">{a:.2f}–{b:.2f}s</a></td><td>{esc(q["question"])}</td><td><a href="{q["signals"]}">信号</a> / <a href="{q["cloud_sheet"]}">点云</a></td></tr>'
        qmd.append(f'- `{sid}` [{a:.2f}–{b:.2f}s]({q["preview"]})：{q["question"]}')
    (OUT/'review_queue.html').write_text(queue+'</table></main></html>',encoding='utf-8')
    (OUT/'REVIEW_QUEUE.md').write_text('\n'.join(qmd)+'\n',encoding='utf-8')
    intro=f'17 段 · 1251.903 秒 · {count} 个通行候选 + {contexts} 个静止/中断后上下文组 · {len(ss)} 个内部片段 · {len(qq)} 个复核区间'
    page=head('S10 · 2026-09-09 录制整理')+f'<p>{intro}</p><p class="notice"><b>已检查事实</b>：文件、数据库计数、话题、时间戳和实际状态值。<b>自动候选</b>：片段边界与动作分类。<b>模型初览</b>：信号图及点云抽样支持的地形/方向候选。<b>人工确认：0</b>。结果没有从整段标签传给子片段；没有认定专家样本。</p>'
    page+='<p><a href="review_queue.html">集中复核清单</a> · <a href="REPORT.md">整理报告</a> · <a href="recordings.csv">完整录制清单</a> · <a href="passages.csv">通行 CSV</a> · <a href="segments.csv">片段 CSV</a> · <a href="inventory.json">完整审计 JSON</a> · <a href="topics.csv">话题/时钟统计</a> · <a href="sampling_windows.csv">5秒采样窗</a></p>'
    page+='<p>这是 ROS 数据预览；未发现相机话题或视频文件。打开录制即可同步检查前后点云、固定机身关节示意、IMU 姿态和实际控制。所有页面也可直接用本机浏览器打开，无需安装依赖。</p><table><tr><th>原录制 ID / 预览</th><th>时长</th><th>整段标签 / 参数记录</th><th>通行候选 + 上下文</th><th>内部片段</th><th>需要注意</th></tr>'
    report=['# S10 录制整理 · 2026-09-09','',intro,'',
        '入口：[全部录制与同步预览](index.html)；[集中复核问题](review_queue.html)；[完整录制清单](recordings.csv)；[通行索引](passages.csv)；[内部片段索引](segments.csv)。','',
        '## 已检查事实','',
        '- 原始目录下实际为 17 个录制目录、17 个 SQLite bag，合计 519,678 条消息、1251.902941 秒。标签：基础 7、楼梯 5、高台 5；整段 success 7、failure 4、unlabeled 6；人工 events 均为空。',
        '- 全部包含 JOINTS_DATA、JOINTS_DATA_10HZ、IMU、MOTION_INFO、前后点云；部分含 GAIT、STEER、HANDLE_STEER。实际消息类型共六种，无相机图像/压缩图像话题；原始目录未发现视频文件。没有 SLAM_ODOM、JOINTS_CMD 或 TF。',
        '- 所有消息已实际解码，解码异常 0；数据库各话题计数与 manifest 一致。17 份 metadata.yaml 的总消息数正确，但 files 内消息数均为实际值的两倍。清单按数据库实际计数，未改写源 metadata。',
        '- [原有转存校验记录](D:/S10Data/050/2026-09-09/verified.json)记载 15,148,185,579 字节，远端/本机 SHA256 一致及 SQLite quick_check=ok。本轮引用该已有结果，没有重新下载、复制原始 bag 或计算全量哈希。',
        '- height_cm/depth_cm 仅作为 manifest 原值保存；例如楼梯 15/55 cm、高台 30/32/38 cm，均未认定为测量真值。','',
        '## 时间、同步和数据问题','',
        '- 索引时间基准是 `source_stamp_ns − manifest.started_wall_ns`，原点和区间纳秒字符串保留在 JSON/CSV。区间采用 `[start,end)`，末端包含录制终点；少数首条源消息早于开始记录，负时间原样保存在话题统计中，索引从 0 开始。控制指令 GAIT 单独按接收秒标注。',
        '- IMU 源间隔通常约 5 ms；MOTION_INFO 约 50 ms；点云与 JOINTS_DATA_10HZ 约 100 ms。主 JOINTS_DATA 有 200/50 Hz 两档，各段范围见下表及 [5 秒窗口](sampling_windows.csv)。`150921`、`152557` 在退出控制附近降到 50 Hz，`154654`、`154856` 主关节流为约 50 Hz。',
        '- IMU 的 receive−source 中位数约 10–12 ms，主关节约 1.5–3.4 ms，MOTION_INFO 约 1.3–4.6 ms，前点云约 119–129 ms；这些包含时钟偏差、处理与缓冲，不能直接解释为纯传输延迟。未对齐/消除各话题时钟偏移。',
        '- GAIT 的源戳为 0，不能与传感器按源时间拼接；实际反馈用 MOTION_INFO。STEER/HANDLE_STEER 源间隔呈约 80 微秒短间隔与约 1 秒跳跃混合，且相对接收时钟有数秒偏移；中位间隔的倒数不是实际发布速率。这些是事件/指令流，不能按连续传感器“掉帧”解释。',
        '- 机身三路在各自有效覆盖内没有超过 `max(0.1s, 5×中位源间隔)` 的长源时间间隔；存在接收卡顿但源序列连续的情况，单列在 inventory.json。此结论不包含提前结束后的空白区间。',
        '- **150921：** 约 16–19s 大倾角，16.813s 状态17→2，18.814s→0；IMU/JOINTS_DATA/MOTION_INFO 接收分别在约63.748/63.743/63.716s结束。前点云源63.723→87.923s间隔24.200s、后点云源63.723→88.623s间隔24.900s；前者旧帧直到接收85.468s才到达（约21.744s延迟）。前后流的延迟接收与源空档分别保留。',
        '- **152557：** 约40s大roll倾斜，40.396s状态17→2，42.398s→0；随后姿态多次变化。**154654：** 52.891s→4、54.491s→0、106.005s→1、107.606s恢复17。姿态变化不自动等于人工搬动。',
        '- 控制退出后，MOTION_INFO 部分速度字段仍保持旧常值，不能用这些值推断机身继续移动。全部原始数值仍保留。','',
        '## 候选结构与结果边界','',
        '- 三层保持为原始录制 → 候选连续通行/上下文 → 内部片段。通行候选不是已核实的通行数量：基础段中的往返和长等待是否分成独立测试仍待确认；151538全段近静止，作为上下文保留。',
        '- 自动候选由0.5秒窗的IMU姿态、角速度和关节运动提出，结合状态退出/数据终止；步态切换只作为独立控制事件。之后模型检查全部17段信号图与每段12帧点云速览，细化有证据的活动区。此检查不等于人工逐帧确认。',
        '- 145331的多个上楼活动区以及两处平台/重新接近候选暂保留在同一连续组；154856在转向、姿态方向改变附近提出上/下行两组，其48.5–55s平台候选保留在下行组内；150146/150705重复高台动作簇提出候选分组，允许人工合并。没有按每次步态码变化拆通行。',
        '- `terrain`/`action_phase`/`direction` 与 control_timeline 分开；方向只填写 up_candidate/down_candidate 或 unknown。障碍、梯段、平台的正式ID均为null，没有证据确认是同一障碍的记录不做关联。',
        '- 子片段结果为 uncertain、interrupted_control 或 interrupted_data；控制退出/缺流是可检查事实，地形通行是否失败/完成仍需人工判断。没有继承 manifest success/failure，也没有判为合格专家样本。所有 human_review_status=pending。',
        '- 自动边界一般约±0.5–1s；精确状态转换保留原采样时刻。预览抽样上限2Hz点云、10Hz姿态/关节，不能精确判定逐轮接触。原始bag路径与时间范围保留，未切出新bag。','',
        '## 逐录制概览','',
        '表中时间为源秒，简称取自完整ID的时分秒；点击原录制ID进入同步预览。所有参数均为未验证记录值。','',
        '| 原录制 ID | 秒 | 整段标签/结果 | 主关节源Hz（5s窗） | 通行候选+上下文 / 片段 |',
        '|---|---:|---|---|---|']
    for r,c in zip(inv,cats):
        sid=r['id'];short=sid[14:20];jhz=[w['source_median_hz'] for w in r['streams']['/JOINTS_DATA']['rate_windows']]
        important=[s for s in r['issues'] if not any(w in s for w in ('receive-source','metadata.files','/GAIT 源时间'))]
        page+=f'<tr><td><a href="{esc(c["preview"])}">{sid}</a><br><a href="previews/{sid}_signals.png">信号图</a> · <a href="previews/{sid}_clouds.jpg">点云速览</a></td><td>{c["duration_s"]:.3f}s</td><td>{esc(c["terrain_label"])} / {esc(c["recording_outcome_label"])}<br>{esc(c["parameters_recorded_unverified"])}</td><td>{c["candidate_passages"]} + {c["context_blocks"]}</td><td>{c["segments"]}</td><td>{esc("；".join(important) or "无新增核心流异常；地形、方向和结果见复核问题")}</td></tr>'
        report.append(f'| [{sid}]({c["preview"]}) | {c["duration_s"]:.3f} | {c["terrain_label"]}/{c["recording_outcome_label"]} | {min(jhz):.0f}–{max(jhz):.0f} | {c["candidate_passages"]}+{c["context_blocks"]} / {c["segments"]} |')
    report+=['','逐段的通行范围、全部片段编号、观察依据和具体问题见 [逐段记录](RECORDING_NOTES.md)；机器可读格式见 [passages.json](passages.json)、[segments.json](segments.json)、[review_queue.json](review_queue.json)。','',
        '## 浏览与复现','',
        '双击 `index.html` 用本机浏览器打开；选择录制后可播放、跳转、慢放，切换源/接收时间、点云侧视/俯视/立体投影及截面宽度。关节采用SDK模型运动学骨架，机身固定原点、移除初始yaw；关节数字通道映射及IMU外参仍未实物验证，不是世界轨迹，不做物理仿真或地形重建。',
        '', '如需HTTP本地预览，在项目根目录运行（仅绑定本机）：','',
        '```powershell', r'.venv-win\Scripts\python.exe -B -m http.server 8766 --bind 127.0.0.1 --directory artifacts/s10-recording-review-20260909','```','',
        '脚本：review.py 复用既有消息定义与解码帮助函数、server.py已测试的点云抽样；build.py复用validate_motion.py中的关节校准/模型和运动学。match_stairs.py已检查，其地形拟合/物理搜索不适用于本轮，因此未执行。plots.py用本机Anaconda的 `python -s -B` 绘制科学图表。其余脚本用项目 `.venv-win`。解码缓存存在时review.py直接复用；build.py的 `--reuse-replay` 用于仅更新索引/报告时保留已生成运动学采样。',
        '', '保留一个可运行自检 `build.py`：索引对全部录制覆盖完整且连续不重叠、时间范围有效、最近样本选择边界、四元数基本运算、无整段结果传播、全部人工状态仍待复核。结果见 [validation.json](validation.json)。原始文件只读；无远端/机器人连接、训练或与本轮无关的源代码改动。']
    page+='</table><p>已有哈希/SQLite转存验证复用；本轮未改写原始录制。全部片段保留，包括等待、失败标签录制、控制中断和不确定部分。</p></main></html>'
    (OUT/'index.html').write_text(page,encoding='utf-8');(OUT/'REPORT.md').write_text('\n'.join(report)+'\n',encoding='utf-8')
    detail=['# 逐段清点与候选片段','', '以下“观察”均指模型对信号图/点云速览的初览，人工状态全部待复核；片段原始文件与精确纳秒边界见 segments.json。','']
    for r in inv:
        sid=r['id'];pp=[p for p in ps if p['recording_id']==sid];seg=[s for s in ss if s['recording_id']==sid]
        detail += [f'## {sid}','',f'[同步预览]({link(sid)}) · [信号](previews/{sid}_signals.png) · [点云](previews/{sid}_clouds.jpg)','',
                   f'原始：`{r["raw_path"]}`；时长 {r["duration_s"]:.6f}s；标签 `{r["manifest"]["terrain"]}/{r["manifest"]["outcome"]}`，参数 `{r["manifest"]["parameters"]}`；人工events为空。','',
                   '候选通行/上下文：'+'；'.join(f'{p["passage_id"].split("-P")[1]}：{p["start_s"]:.3f}–{p["end_s"]:.3f}s，{p["kind"]}，{p["outcome"]}' for p in pp),'',
                   '实际控制（源秒）：'+'；'.join(f'{x["start_s"]:.3f}–{x["end_s"]:.3f}：state {x["values"][0]} / gait 0x{x["values"][1]:04X}' for x in r['control_timeline']),'']
        detail += ['观察：'+o['evidence'] for o in notes.get(sid[14:20],{}).get('observations',[])]
        detail += ['', '数据问题：'+'；'.join(r['issues']), '', '| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |','|---|---|---|---|---|---|']
        for s in seg:detail.append(f'| [{s["segment_id"].split("-S")[1]}]({s["preview"]}) | {s["passage_id"].split("-P")[1]} | {s["start_s"]:.3f}–{s["end_s"]:.3f} | {s["terrain"]} / {s["action_phase"]} | {s["direction"]} | {s["outcome"]} |')
        detail+=['','待确认：']+[f'- [{q["start_s"]:.2f}–{q["end_s"]:.2f}s]({q["preview"]})：{q["question"]}' for q in qq if q['recording_id']==sid]+['']
    (OUT/'RECORDING_NOTES.md').write_text('\n'.join(detail)+'\n',encoding='utf-8')
    csvwrite(OUT/'gait_commands.csv',[{'recording_id':r['id'],**c} for r in inv for c in r['gait_commands']])
    print(intro)

if __name__=='__main__':main()
