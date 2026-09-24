# 步态采集性能基准与改进


> [!NOTE]
> Paths below in `code` are generated locally and are not tracked in this repository; rerun the tool to produce them.


2026-09-09，本地完成。100 份历史记录时，`/api/state` 中位耗时从 **61.08 ms 降至 39.56 ms（减少 35.2%）**。录制期间状态请求的采集锁持有时间从 **47.39 ms 降至 0.076 ms（减少 99.84%）**。

## 基准方法

- 环境：Windows 11、AMD Ryzen 9 7945HX、Python 3.12.4、已安装的 Flask 3.0.3。项目 requirements 指定 Flask 3.1.3，本次未更换本机依赖。
- `benchmark.py` 只使用标准库及应用已有 Flask；临时目录中的数据自动清理，不访问真实录制目录或机器人。
- 同一脚本对比修改前的工作区副本与修改后代码。基线包含用户已有的点云预览改动，并非 Git HEAD。
- 每项预热 5 次、计时 50 次，报告中位数与 nearest-rank P95。锁计时测量最外层持锁区间；独立 cProfile 测量 20 次状态快照，不混入接口计时。
- 固定 100 份 manifest，每份含全部 16 个话题及 30 条运动反馈事件；固定历史时间字段，总计 **1,999,500 字节**。另测空历史、录制期间状态、100 次合成 ingest+tick、10 万点抽样至 2500 点。
- 使用 Flask test client，包含认证和 JSON 响应序列化，但不包含 HTTP 网络、Waitress 排队或手机渲染。预热后的文件内容使用操作系统缓存。

## 结果

单位均为毫秒；每格为「中位数 / P95」。

| 场景 | 修改前 | 修改后 | 中位耗时变化 |
|---|---:|---:|---:|
| 空历史状态接口 | 0.673 / 0.927 | 0.492 / 0.581 | −26.9% |
| 100 份历史状态接口 | 61.077 / 69.488 | 39.558 / 51.823 | **−35.2%** |
| 100 份历史 snapshot（不含 HTTP/响应编码） | 46.799 / 53.806 | 27.678 / 34.398 | −40.9% |
| snapshot 持有采集锁 | 47.734 / 57.134 | 0.035 / 0.039 | **−99.93%** |
| 录制期间状态接口 | 59.639 / 69.779 | 39.523 / 46.698 | **−33.7%** |
| 录制期间状态请求持有采集锁 | 47.392 / 55.188 | 0.076 / 0.083 | **−99.84%** |
| 100 次合成 ingest+tick | 7.714 / 10.328 | 7.992 / 9.246 | +3.6% |
| 10 万点预览，输出 2500 点 | 4.550 / 7.612 | 4.420 / 5.009 | −2.9% |

ingest/tick 和点云算法均未修改，以上小幅差异不能归因于本次优化。接口仍返回完整历史内容，响应约 1.47 MB；实时健康字段使前后响应字节数略有差异。

## 瓶颈与修改

原 `snapshot()` 在采集共用的 `RLock` 内执行目录递归匹配、最多 100 次文件读取、JSON 解码和磁盘空间查询。cProfile 的 20 次快照共耗时 1.166 秒，其中 `read_text` 累计 0.560 秒，`json.loads` 0.313 秒，目录排序及 glob 遍历累计 0.279 秒。此时 ROS 回调中的 `ingest()` 需要等待同一把锁。

本次仅修改 `server.py` 的状态快照路径：

1. 历史文件处理与磁盘空间查询移到采集锁外；锁只保护当前录制、健康数据和运动反馈的内存快照。
2. 用一次 `os.scandir` 列出会话目录，按原有倒序规则选取最多 100 份有效 manifest，避免递归进入每个会话目录。
3. 缓存最近 100 份文件的文本，以修改时间纳秒值和大小判断是否重读。每次请求仍检查文件元数据，不引入缓存等待周期。新增、删除及修改会在下一次查询中反映。
4. 缓存以新字典整体替换，避免并发请求修改共享缓存；每次解码为独立历史对象，调用者修改返回数据不会污染后续响应。

缓存增加的文本内存与最近 100 份 manifest 的总大小成正比，本基准约 2 MB，另有 Python 对象开销。外部工具如果刻意保留相同文件大小及修改时间，缓存无法识别该修改。历史文件读取与实时状态不再构成同一锁内事务：并发开始或停止时，历史列表可能来自稍早时刻，`active` 字段反映随后获取锁时的当前状态。

复测 cProfile 中没有 manifest `read_text` 调用；剩余主要成本为 JSON 解码和文件元数据查询。原始录制写入、磁盘空间保护、ZIP 校验、点云预览和网页功能保持原有实现；未引入依赖。

## 验证与复现

以下检查已通过：

- `test_snapshot.py`：历史排序及 100 份上限、忽略无 manifest 的目录与 ZIP、缓存命中免重读、返回对象独立、文件更新和删除、API 认证、开始/标记/停止后的状态刷新。
- 同一脚本的并发检查：用事件阻塞历史文件读取，验证另一线程的 `ingest()` 在读取恢复前完成。
- 现有 `test_point_preview.py`：点云格式、大小端、抽样、非法数据和接口行为。
- `git diff --check -- tools/s10_gait_capture`。

在仓库根目录运行：

```powershell
python tools/s10_gait_capture/benchmark.py --server artifacts/s10-performance/server_before.py --output artifacts/s10-performance/before.json
python tools/s10_gait_capture/benchmark.py --output artifacts/s10-performance/after.json
python tools/s10_gait_capture/test_snapshot.py
python tools/s10_gait_capture/test_point_preview.py
```

本次原始输出（含 profile）：before.json `../../artifacts/s10-performance/before.json`、after.json `../../artifacts/s10-performance/after.json`；修改前源码副本 `../../artifacts/s10-performance/server_before.py`保留在本地 artifacts 中。

这些结果验证本机状态轮询耗时及锁占用的改善，不代表 Orin 上 ROS/CDR 序列化、SQLite 写入吞吐或真实丢帧率的改善。未部署到机器人。若继续优化，应先在 Orin 上复跑基准并测录制时接收间隔；只有完整历史响应确实成为网络瓶颈时，再考虑分页或摘要接口。
