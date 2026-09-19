# 持久目录 Unix socket：独立兼容检视

日期：2026-09-13。背景：master报告106系统服务在`RuntimeDirectory`阶段以233退出，worker尚未运行；决定只将应用socket迁入已准备的0700数据目录，不改`/run`或放宽系统权限。

## 结论

**最小迁移方案本地专项9/9通过，未发现本地阻断。** 原进程3/3和HTTP3/3复跑通过。实际106服务启动与端到端SSH转发由master部署验证，本reviewer没有SSH、部署、修改`/run`或操作机器人。

默认socket为：`/var/opt/robot/data/s10_field_assistant/worker.sock`，ASCII编码51字节。同样51字节长度的本地Unix socket已实际bind成功，不是只凭字符串检查推测。

| 验证 | 实际结果 |
| --- | --- |
| 默认路径一致 | `SOCKET`、`rpc_call`默认参数、CLI默认、系统unit和指南一致；无旧RuntimeDirectory配置 |
| forcedSSH转发 | mock AGX调用`field`，106受限后台调用同模块的`rpc_call`默认路径，没有另一份硬编码socket |
| 残留恢复 | 本地先创建并关闭Unix socket留文件，再启动fake worker，成功恢复服务 |
| 第二实例 | 同root第二进程被worker.lock拒绝；原socket inode未变，第一进程health仍可读 |
| 普通文件 | socket位置存在普通文件时拒绝启动，原内容不变 |
| 符号链接 | 指向真实socket的symlink与dangling symlink均拒绝，链接和目标保持原状 |
| 权限 | 已有0755父目录被拒绝且不自动chmod；合法父目录0700、socket0600 |
| 路径长度 | 与新生产默认相同的51字节路径本地实际bind成功 |
| 旧行为 | 大预览RPC、请求响应丢失幂等、运行任务重启中断不重放、HTTP认证/CSRF/下载Range均回归通过 |

## 执行

Python3.12本地运行，无ROS、SSH和真实后端：

```bash
python3.12 -B tools/s10_mapping_web/qa/test_socket_compat.py
python3.12 -B tools/s10_mapping_web/qa/test_independent_process.py
python3.12 -B tools/s10_mapping_web/qa/test_independent_http.py
```

结果分别为9、3、3项全部通过。`worker_fixture.py --socket-under-root`只改变QA临时目录布局；未改变原默认夹具，因此旧进程测试保持可运行。所有本地测试子进程已由夹具结束，临时文件回收。

## 边界

使用Code Reviewer技能审查权限、默认值一致性与残留恢复。生产代码由implementer修改，master负责上线。目录owner必须是运行用户，生产目录必须位于已挂载数据分区；这些代码检查保留。本专项没有变更用户组、系统目录权限、SSH访问范围、地图、录制或运动功能。

本地通过仅说明兼容改动及恢复/权限逻辑成立，不能代替106的真实systemd启动结果；也不代表手机现场完整验收或机器人安全停车认证。
