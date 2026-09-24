# 本地副本与数据格式

部署历史、自启配置和清理结果见 [DEPLOYMENT_050.md](DEPLOYMENT_050.md)。2026-09-10 已移除 50 号 AGX 独立采集站及 103 临时中转，本地源码和录制保留。

历史 9 轮录制、关节/姿态验证、地图和 RL 参考结论见 [研究交接] `git show docs-archive-20260920:docs/S10_DATA_RESEARCH_ZH.md`；106 的建图调用与算法证据见 [SLAM 调查] `git show docs-archive-20260920:docs/S10_SLAM_106_RESEARCH_ZH.md`。历史数据已复制到本地 `artifacts/`，不会随 Git 仓库分发。

2026-09-08 从 50 号机器人 AGX `10.21.33.102:/home/xwy/s10_gait_capture` 只读复制。
保存了 Python 源码、网页、启动脚本、服务文件和 vendor_ws/src/drdds 消息定义。
初次复制没有包含虚拟环境、编译产物、认证配置或完整 ROS bag。此后本地副本已改版；机器人原作者目录没有被覆盖。
`run-demo-local.py` 是本次新增的本地演示入口，使用已有 Flask，完全不连接 ROS 或机器人。

## 本地演示

在仓库根目录运行：

```powershell
python tools/s10_gait_capture/run-demo-local.py
```

打开 http://127.0.0.1:8090 。访问口令在 `artifacts/s10-capture-demo/.access-token`。
若页面已在运行，无需重复启动。关闭运行终端可结束前台实例。
不要复用历史 PID；停止演示应关闭其启动终端，或先核对当前进程命令行。

选择地形及参数 → 开始录制 → 标注事件 → 停止并保存 → 下载 ZIP。
本次已通过页面完成上述流程，保存一轮 synthetic 演示。
页面中的 RL 状态、关节命令与接收率都是合成演示，不代表机器人现场状态。
原 `start-local.ps1` 假定 `.venv/Scripts/python.exe` 和 waitress 已安装，本机不满足；本地使用上面的入口。

## 真实数据格式

```text
gait_<日期>_<时间>_<随机编号>/
  manifest.json
  bag/
    metadata.yaml
    <bag文件名>.db3
  SHA256SUMS.json          # 点击 ZIP 导出时生成
```

- manifest：schema_version、id、terrain、parameters、teacher、notes、mode、status、outcome、started_wall_ns、stopped_wall_ns、duration_s、topics、events、motion_feedback_at_start、motion_feedback_events、missing_topics、training_ready。
- topics 统计：count、type、source_stamp_missing、source_nonincreasing、max_receive_gap_s 等。
- bag：ROS bag2 SQLite3；messages 表为 id、topic_id、timestamp、data。timestamp 是采集端 ROS 接收时间（纳秒）；data 是 CDR 序列化二进制，不是 JSON 或 CSV。
- topics 表关联 topic 名称、消息类型及 serialization_format；源时间戳保留在 CDR 消息 header.stamp 内。
- 事件 receive_wall_ns/elapsed_s 表示服务端接收标注时刻，不是传感器触发时刻。
- 所有原始记录 training_ready=false；需要后续审查，不代表数据损坏。

## 本次查看的真实样本

来源：机器人 `/home/xwy/s10_gait_data/gait_20260906_134245_67a76c09d415`。
对应本地文件在 `artifacts/s10-readonly-probe/`：

- `real-manifest.json`：原始会话说明。
- `real-bag-metadata.yaml`：原始 bag 元数据。
- `data-format-example.json`：真实 bag 每个 topic 第一条消息的解码样例、SQLite 表结构。点云只保留布局及少量原始字节预览，未复制整帧数据。

该样本包含 IMU、16 关节反馈、运动状态、前后点云、HANDLE_STEER、STEER；没有 JOINTS_CMD。
IMU：orientation、angular_velocity、linear_acceleration 与各 covariance。
关节：data.joints_data[16]；每项 name、data_id、status_word、position、torque、velocity、motion_temp、driver_temp。
运动：data.vel_x/vel_y/vel_yaw/height、motion_state.state、gait_state.gait、payload、remain_mile。
轴输入：data.x/y/z/roll/pitch/yaw。
点云：PointCloud2，base_link，x/y/z/intensity 均 float32，偏移 0/4/8/12，小端，每点 16 字节；该样例无逐点时间字段。

演示模式生成 `synthetic.jsonl` 替代 bag：每行仅 topic、synthetic=true、receive_ns，用于验证录制流程，不能当作真实传感器数据。

实际模式只订阅 server.py 的 TOPICS 白名单，匹配发布者 QoS，直接把完整 CDR 写入 rosbag2。
JOINTS_CMD 缺失时，页面需要选择“关键话题缺失时，仅录制诊断数据”；这只影响采集准入，不启用 SDK、不改变机器人控制。
