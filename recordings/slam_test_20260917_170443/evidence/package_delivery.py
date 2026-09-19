"""Verify downloaded files against robot SHA256, build and test a self-contained ZIP."""
import datetime as dt
import hashlib
import json
import zipfile
from pathlib import Path

root = Path(__file__).resolve().parent.parent
audit = json.loads((root/'evidence/bag_audit.json').read_text())


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


assert audit['all_messages_deserialized']
assert audit['source_sizes_and_mtimes_unchanged']
assert audit['message_count'] == audit['metadata_message_count']
assert set(audit['topics']) == {'/IMU', '/LIDAR/POINTS', '/ODOM'}
assert all(t['count_matches_metadata'] for t in audit['topics'].values())
for name, spec in audit['source_files'].items():
    p = root/'raw_bag'/name
    assert p.stat().st_size == spec['bytes'], f'Incomplete download: {name}'
    assert sha(p) == spec['sha256'], f'SHA256 mismatch: {name}'
    print('SOURCE_MATCH', name, flush=True)
assert {p.name for p in (root/'raw_bag').iterdir()} == set(audit['source_files'])

rows = []
for name in ('/LIDAR/POINTS', '/IMU', '/ODOM'):
    s = audit['topics'][name]
    rows.append(f"| `{name}` | {s['count']:,} | {s['source_hz']:.3f} Hz | {s['source_interval_s']['max']*1000:.3f} ms | {s['source_backward_timestamps']} / {s['source_duplicate_timestamps']} |")
points = audit['lidar']
readme = f'''# S10-048 雷达 + IMU 接口测试录制

录制目录：`slam_test_20260917_170443`。ROS 2 Jazzy，MCAP/CDR。
来源：106 定位板 `/var/opt/robot/data/slam_test_20260917_170443`。
接收时间：2026-09-17 17:04:45.984 ～ 17:07:43.808（UTC+8）。
实际包时长：177.824 秒；180 秒录制命令包含启动/发现时间，不应简单将差额判作中途丢包。

## 文件

- `raw_bag/`：原样下载的 `metadata.yaml` 和 8 个 MCAP 分卷。它们是**同一次录制**，请保留在一起。
- `evidence/bag_audit.json`：全部 {audit['message_count']:,} 条消息的原生 ROS 反序列化、时间、字段和哈希检查。
- `evidence/configuration_snapshot_after_recording/`：录完后只读采集的驱动、IMU、SLAM、DDS 配置参考；不是录制开始时冻结的配置，也不是独立标定证明。
- `MANIFEST.json`：交付文件的大小与 SHA256。

## 实际数据

| 话题 | 条数 | 按源时间计算频率 | 最大源时间间隔 | 倒退 / 重复时间戳 |
|---|---:|---:|---:|---:|
{chr(10).join(rows)}

LiDAR 总点数：{points['total_points']:,}。字段为 `x,y,z,intensity,ring,timestamp`。
逐点时间扫描跨度中位数：{points['scan_span_s']['median']:.6f} 秒。
源文件与 Mac 副本逐文件 SHA256 一致；全部消息可读取，计数与元数据一致。

## 交给新 SLAM 开发者时必须说明

1. 这是 **ROS 2 rosbag2 / MCAP**，不是 ROS 1 的单个 `.bag`。如对方仅支持 ROS 1，应另行转换并保留本包，不直接改扩展名。
2. `/LIDAR/POINTS` 是厂商驱动的输入点云话题；不是 `full_cloud.pcd`、网页预览或 AGX 的 XYZI 合并话题。保留了实际 `ring` 和逐点 `timestamp`，但不包含原始 UDP/PCAP。
3. 点内 `timestamp` 实测为秒级 epoch 时间；应结合样本和驱动配置确认相对帧首/帧尾关系，不将其直接当毫秒偏移。`bag_audit.json` 保留具体字段类型、偏移、样本、范围和与 header 的关系。
4. 点云 frame 为 `lidar_link`；`/IMU` 的 frame_id 为空。不能因此认定两者同一物理坐标系，也不能仅凭话题名认定雷达完全未经坐标变换或去畸变。外参方向、双雷达合成及处理语义需对接确认；不要重复变换/去畸变。
5. `/IMU` 保存了连续三轴角速度、加速度和姿态等原消息，和 `/ODOM`（已有融合定位输出）不同。单位仍应按驱动契约核对，不只依据量级判断。
6. 本次没有录 `/tf`、`/tf_static`，也没有分别录两台 AIRY 自带 IMU；不能把本包称作完整标定资料。
7. 点时间超出已录 IMU 首尾覆盖的扫描数：{points['scans_outside_imu_time_coverage']}。若重跑 LIO，应保留初始化上下文并审查边界扫描，不把缺前文的首帧直接当作有效去畸变结果。
8. 保存了传感器 header/逐点时间与 bag 接收时间。它们不是同一种时钟含义；不能用接收时间覆盖源时间。
9. 通过文件/连续性检查，不等于标定、绝对几何或新 SLAM 效果已验收；不承诺零 UDP 丢包。
10. 当前驱动配置为 `send_separately: false`，并配置了 CD1 双雷达坐标变换；本包 ring 实测覆盖 0–191。请按厂商接口确认通道/雷达来源，不能未经适配就按单台 16/32 线雷达读入。
11. 本包 PointCloud2 的 `point_step=26`；`ring` 为 uint16、offset=16，`timestamp` 为 float64、offset=18。请按消息中的字段偏移/步长读取，不硬套 C++ 结构体默认对齐。

## 使用

在已安装 ROS 2 Jazzy / MCAP 插件的离线环境，进入本目录：

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag info raw_bag
```

本次审查只是读文件，没有回放、重启 SLAM、切换地图或发送运动命令。
**若开发者需要回放，请先隔离机器人及控制网络。不要在连接真机的 ROS 域直接执行 bag play。**
原包保留在机器人上；Mac 副本也保留。没有修改或删除原始录制。

## English handoff

ROS 2 Jazzy rosbag2, MCAP storage, CDR messages. Keep metadata.yaml and all eight MCAP chunks together.
Topics: /LIDAR/POINTS (sensor_msgs/msg/PointCloud2), /IMU (sensor_msgs/msg/Imu), /ODOM (nav_msgs/msg/Odometry).
All original files are SHA256-matched to the robot, all messages were deserialized without replay.
Point fields include ring and per-point timestamp. IMU frame_id is empty; TF is not recorded.
Driver transforms, multi-LiDAR timing and deskew status still need interface confirmation.
Configuration files are post-recording reference snapshots, not a frozen calibration certificate.
Use an isolated offline environment for playback; never publish this bag onto the live robot control network.
'''
(root/'README_先读我.md').write_text(readme)
files = sorted(p for p in root.rglob('*') if p.is_file() and p.name not in ('MANIFEST.json', 'package_receipt.json') and '__pycache__' not in p.parts)
entries = {str(p.relative_to(root)): dict(bytes=p.stat().st_size, sha256=sha(p)) for p in files}
(root/'MANIFEST.json').write_text(json.dumps(dict(created_at=dt.datetime.now(dt.timezone.utc).isoformat(),
    source_sha256_verified=True, files=entries), ensure_ascii=False, indent=2)+'\n')
files.append(root/'MANIFEST.json')
out = root.parents[1]/'deliverables'/'S10_048_LiDAR_IMU_20260917_170443.zip'
prefix = 'S10_048_LiDAR_IMU_20260917_170443'
with zipfile.ZipFile(out, mode='x', compression=zipfile.ZIP_DEFLATED, compresslevel=1, allowZip64=True) as z:
    for p in files:
        z.write(p, prefix+'/'+str(p.relative_to(root)))
        print('PACKED', p.relative_to(root), flush=True)
with zipfile.ZipFile(out) as z:
    assert len(z.infolist()) == len(files)
    for p in files:
        rel = str(p.relative_to(root))
        expected_hash = entries[rel]['sha256'] if rel in entries else sha(p)
        h = hashlib.sha256()
        with z.open(prefix+'/'+rel) as f:
            for block in iter(lambda: f.read(8 * 1024**2), b''):
                h.update(block)
        # Reading to EOF also checks the ZIP CRC.
        assert h.hexdigest() == expected_hash, f'ZIP SHA256 failure: {rel}'
result = dict(zip=str(out), bytes=out.stat().st_size, sha256=sha(out),
              file_count=len(files), source_sha256_verified=True, zip_crc_verified=True,
              all_zip_entry_sha256_verified=True)
(root/'package_receipt.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2), flush=True)
