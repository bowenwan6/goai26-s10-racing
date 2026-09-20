# 48 号手机建图

手机连接 **S10 PRO-048-5G**，打开 [103 热点入口](http://10.21.41.1:8080/)。用户名 `golai`，密码已单独交接。使用遥控器控制行走。完整操作和算法说明见 [48 号 SLAM 指南](../../docs/S10_48_SLAM_ZH.md)。

## 实际分工

| 机器 | 部署内容 | 工作 |
|---|---|---|
| 103 / user | `/etc/systemd/system/s10-mapping-hotspot.socket`、同名 `.service` | 原生 socket-proxyd：`10.21.41.1:8080 → 10.21.33.102:8080` |
| 102 / golai | `~/s10_mapping_web/server.py`、`index.html`；用户服务 `s10-mapping-web` | HTTP、登录、点云累计、轨迹显示、SSH 命令 |
| 106 / user | `~/s10_mapping_web/robot_backend.py`、`backend.sh` | 官方 drmap 操作、按需数据读取；无 HTTP 服务 |

参考的本地 xwy 副本：`artifacts/xwy-migration-050-20260910-174950/files/home/xwy/s10_quick_mapping/` 中 `HOTSPOT_README.md`、`web_server.py`、`quick_mapping.py`、`robot_backend.py`。其链路也是 103 → 102 → 106，103 用 SSH 端口转发，102 使用 xwy 账号并下载地图。本实现复用这一分工及 SSH JSON 命令模式，账号改为 golai；使用已安装的 Paramiko 和 Python 标准库，不安装 Flask、Waitress 或新 SLAM。

106 复用已安装的 `map_manager.services.mapping.MappingService` 启动官方 mapping 服务，关闭 RViz 和自动激活。保存确认当前会话 InvocationID、非空地图文件和本次 journal 后处理完成标记，再停止建图、恢复定位；失败不强制杀 SLAM。没有照搬 xwy 后端永久 mask localization 的行为。

当前只实现启动、保存、实时预览和已有地图列表。没有迁移 xwy 标点、ZIP 下载或地图最终坐标修正。原始地图仍在 **106** `/var/opt/robot/data/maps/`，AGX 仅累计内存预览；它不会把最终完整 PCD 自动备份到 AGX。

## 预览与精度

官方 SLAM 的输入、配置和保存地图不变。每秒解码只针对额外的手机订阅，**不把官方 SLAM 降为 1 Hz**。页面大约每秒刷新、每帧最多 900 点；AGX 使用 20 cm 体素，最多累计 30,000 点和 2,000 个近期位姿。在线预览的稀疏程度、刷新率与官方地图精度是不同指标。回环后的完整地图应读取厂商保存结果。

读取使用 `raw=True`，先限频再调用系统 `deserialize_message`；预览的 CPU 优先级为 nice 10。没有人查看时，106 读取进程约在最后一次状态请求后 30 秒退出，官方 mapping 服务独立继续。重新打开页面时按需重建数据连接，连接断开不会自动重发 start/save。

当前未建图、显示实时雷达的 10 秒窗口，106 读取进程占约 **49.6% 单核 CPU**（8 核机器，约总容量 6.2%），RSS 约 **181 MiB**；原逐帧解码为单核 58.9%。这不是运行 SLAM 后的峰值测量。网页和累计图在 AGX；106 仍有读取成本，不能称为零负载。

## 部署文件与凭据

本目录代码已部署到 48 号。源码更新分别复制到上表目录，`.sh` 必须使用 LF 换行；修改 AGX 代码后重启它的用户服务，106 下一次 SSH 调用自动使用新后端。106 旧 `s10-mapping-web.service` 已禁用，不应重新启用。

- AGX 的 `~/s10_mapping_web/backend_key` 为模式 600 的独立密钥，`known_hosts` 固定经过核实的 48 号 106 主机密钥，连接拒绝未知主机。
- 106 `user/.ssh/authorized_keys` 中对应公钥使用 `restrict,from="10.21.33.102",command="/bin/bash /home/user/s10_mapping_web/backend.sh"` 限制来源与命令，不允许 shell 或端口转发。维护时只追加／撤销本工具对应条目，不覆盖其他密钥。
- AGX 的 `~/.config/s10-mapping-web/config.json`（600）只含 `login_hash`，其值为 `sha256("用户名:密码".encode()).hexdigest()`。明文 SSH 密码没有保存；公钥首次部署后后台使用受限密钥。
- 登录后使用 HttpOnly、SameSite=Strict 会话 Cookie，写操作另外校验 CSRF token；服务重启后重新登录。局域网 HTTP 入口不用于公网。
- `backend.sh` 使用系统 Python/ROS/Numpy，再追加厂商 Python 模块路径。不能把 SLAM 专用库路径全局加到 ROS 读取进程：本机验证会导致进程异常退出；仅保存命令单独使用该库路径。

重装时需先配置上述经核实的主机密钥、受限连接密钥和网页登录哈希，然后安装服务模板。仓库不包含可直接使用的密码或私钥。

## 维护

AGX，以 `golai` 执行（已启用 linger，无须保持电脑 SSH）：

```bash
systemctl --user daemon-reload
systemctl --user enable --now s10-mapping-web.service
systemctl --user status s10-mapping-web.service
```

103，以 `user` 执行：

```bash
systemctl status s10-mapping-hotspot.socket s10-mapping-hotspot.service
```

106 的数据读取没有独立的常驻服务。官方服务检查：

```bash
systemctl status mapping.service localization.service
```

要停止手机入口，只停 103 的 `s10-mapping-hotspot.socket` 和 `.service`，以及 AGX 的用户 `s10-mapping-web.service`；先等已提交的保存操作完成。不要用停止网页代替结束建图。

## 验证与边界

```powershell
python tools/s10_mapping_web/test_server.py
```

检查覆盖 PointCloud2 大小端、行填充、NaN、截断输入，以及保存失败不停止服务。真实热点 HTTP 验证了首页、登录、点云/IMU、地图列表、非法名称与缺少 CSRF 拒绝；约 890 点/帧来自实机雷达，不是示例数据。证据位于 `artifacts/s10-48-slam-20260911/phone-preview.json` 和 `phone-reader-performance.json`。

本次未启动真实建图或走场、未发送运动命令。建图输出 `/SLAM_ALIGNED_POINTS` 和 `/SLAM_ODOM` 要在现场启动后验收。浏览器自动化后续超时，尚未完成手机实际尺寸下的显示验收；页面使用响应式 Canvas，无外部 CDN。开机配置已启用，整机重启验收留待正常开机。
