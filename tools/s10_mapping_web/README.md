# 48 号手机建图

新增功能：主页绿色“现场助手”按钮进入 [`/field` 离线现场验收助手](FIELD_GUIDE_ZH.md)，包含持久任务、自检、地图验证、限时原始录制、位置草稿与报告。**2026-09-13 已部署到102/106，并通过实际热点HTTP登录、自检与目标地图预览验收**；原地图现场定位、真实录制及手机冷启动无WAN验收尚未完成。旧建图、定位和高度图均保留，详见[部署记录](DEPLOYMENT_20260913_ZH.md)。

手机连接 **<ROBOT_WIFI_SSID>**，打开 [103 热点入口](http://10.21.41.1:8080/)。用户名 `golai`，密码已单独交接。使用遥控器控制行走。完整操作和算法说明见 [48 号 SLAM 指南](../../docs/S10_48_SLAM_ZH.md)。

实时定位入口：[查看狗的位置](http://10.21.41.1:8080/localization)。沿用同一登录。当前显示`indoor_loop_01-20260911-203447`全部142,598个点，红色定位针底部对应狗的XYZ位置，上方箭头显示朝向。每秒更新位置图层，不刷新整页；默认关闭“跟随狗”，固定地图范围并保留旋转、缩放视角。高度分色设置可以展开调整。

定位显示复用106的`/ODOM`，并核对官方日志中的全局定位状态、消息时间、坐标系、服务会话和激活地图名。定位丢失、局部估计、状态过期或网络断开时隐藏定位针，避免继续显示旧坐标。新地图需要重新生成对应HTML；切换到其他地图时不会把坐标叠到本图上。

## 实际分工

| 机器 | 部署内容 | 工作 |
|---|---|---|
| 103 / user | `/etc/systemd/system/s10-mapping-hotspot.socket`、同名 `.service` | 原生 socket-proxyd：`10.21.41.1:8080 → 10.21.33.102:8080` |
| 102 / golai | `~/s10_mapping_web/server.py`、`index.html`、`localization.html`；用户服务 `s10-mapping-web` | HTTP、登录、点云累计、实时定位显示、SSH 命令 |
| 106 / user | `~/s10_mapping_web/robot_backend.py`、`backend.sh` | 官方 drmap 操作、按需数据读取；无 HTTP 服务 |

时间检查依赖106的`/home/user/check_s10_slam.py`（仓库`scripts/`同名文件）。开始建图前现场采样3秒，拒绝旧时间戳／倒退／明显跳变，然后才调用厂商会停止定位的启动接口。网页同时显示相对106系统时钟的测量延迟；持续到包不再等同于时间正常。

106另安装`scripts/wait_s10_ptp.py`到root所有的`/usr/local/lib/s10/wait_s10_ptp.py`，将`106-time-ready.conf`复制为`/etc/systemd/system/{yesense,rsdriver,hsLidar,localization,mapping}.service.d/48-time-ready.conf`后执行`systemctl daemon-reload`。每个服务启动时确认原103主钟、PTP状态和PHC/系统时间稳定，不改变厂商授时方向。配置已加载且用临时systemd服务验证；未为此重启传感器或整机。只需撤回本工具启动检查时，移除这五个明确命名的drop-in并reload即可，保留原厂商单元。

参考的本地 xwy 副本：`artifacts/xwy-migration-050-20260910-174950/files/home/xwy/s10_quick_mapping/` 中 `HOTSPOT_README.md`、`web_server.py`、`quick_mapping.py`、`robot_backend.py`。其链路也是 103 → 102 → 106，103 用 SSH 端口转发，102 使用 xwy 账号并下载地图。本实现复用这一分工及 SSH JSON 命令模式，账号改为 golai；使用已安装的 Paramiko 和 Python 标准库，不安装 Flask、Waitress 或新 SLAM。

106 复用已安装的 `map_manager.services.mapping.MappingService` 启动官方 mapping 服务，关闭 RViz 和自动激活。保存确认当前会话 InvocationID、非空地图文件和本次 journal 后处理完成标记，再停止建图、恢复定位；失败不强制杀 SLAM。没有照搬 xwy 后端永久 mask localization 的行为。

当前实现启动、保存、建图预览、已有地图列表和上述室内地图的实时定位查看。没有迁移 xwy 标点、ZIP 下载或地图最终坐标修正。原始地图仍在 **106** `/var/opt/robot/data/maps/`；AGX除内存预览外保存约5.94 MB的本图定位HTML，包含显示用XYZ数据，不会自动备份完整PCD和原始关键帧。

## 预览与精度

官方 SLAM 的输入、配置和保存地图不变。每秒解码只针对额外的手机订阅，**不把官方 SLAM 降为 1 Hz**。页面大约每秒刷新、每帧最多 900 点；AGX 使用 20 cm 体素，最多累计 30,000 点和 2,000 个近期位姿。在线预览的稀疏程度、刷新率与官方地图精度是不同指标。回环后的完整地图应读取厂商保存结果。

读取使用 `raw=True`，先限频再调用系统 `deserialize_message`；预览的 CPU 优先级为 nice 10。没有人查看时，106 读取进程约在最后一次状态请求后 30 秒退出，官方 mapping 服务独立继续。重新打开页面时按需重建数据连接，连接断开不会自动重发 start/save。

当前未建图、显示实时雷达的 10 秒窗口，106 读取进程占约 **49.6% 单核 CPU**（8 核机器，约总容量 6.2%），RSS 约 **181 MiB**；原逐帧解码为单核 58.9%。这不是运行 SLAM 后的峰值测量。网页和累计图在 AGX；106 仍有读取成本，不能称为零负载。

## 部署文件与凭据

本目录代码已部署到 48 号。源码更新分别复制到上表目录，`.sh` 必须使用 LF 换行；修改 AGX 代码后重启它的用户服务，106 下一次 SSH 调用自动使用新后端。106 旧 `s10-mapping-web.service` 已禁用，旧 HTTP 代码已撤下，不应重新启用。

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
python scripts/wait_s10_ptp.py --check
python scripts/check_s10_slam.py --check
python tools/s10_mapping_web/test_server.py
node tools/s10_mapping_web/test_localization.cjs
```

为已取回本地的另一地图生成定位页面：

```powershell
python -s artifacts/s10-48-map-review-20260911/render_full_cloud.py <本地地图目录> --live-map <106上的准确地图目录名>
```

将生成的`localization.html`上传到AGX的`~/s10_mapping_web/`即可。页面文件更新不需要重启定位或网页服务；浏览器需刷新一次加载新版。`localization.js`是生成器读取的源文件，已内嵌到HTML，无CDN依赖。新增检查覆盖定位日志过期、重启前日志、丢失／局部状态、地图不一致、旧消息和无效坐标；真实热点页面已验证登录、位置更新和定位针显示。

检查覆盖 PointCloud2 大小端、行填充、NaN、截断输入，以及保存失败不停止服务。真实热点 HTTP 验证了首页、登录、点云/IMU、地图列表、非法名称与缺少 CSRF 拒绝；约 890 点/帧来自实机雷达，不是示例数据。停止查看后已确认 106 读取进程退出、mapping 仍 inactive、localization 仍 active。证据位于 `artifacts/s10-48-slam-20260911/phone-preview.json` 和 `phone-reader-performance.json`。

用户已实际通过手机走场并保存，旧会话发生跳时及约103米首尾偏差，不通过地图质量验收。9月11日下午重启后传感器时间已恢复，随后部署上述保护；热点HTTP实测IMU测量延迟约10 ms、雷达约274 ms，异常时间戳拒绝和厂商启动前检查的离线测试通过。页面使用响应式Canvas，无外部CDN；新修改尚未完成手机尺寸截图检查。新增开机保护仍待下次正常整机启动验收。

9月11日20:34的`indoor_loop_01`复测已跑通手机建图、回环及保存：92关键帧，日志与文件一致的8条回环约束，60条状态心跳均报告同步正常。完整地图142,598点，后处理完成。首尾距离1.16 m与用户确认的实际停靠偏移一米多基本吻合，未做独立测量，不代表已测得绝对定位精度；见[复测报告](../../artifacts/s10-48-indoor-loop-20260911-203447/REVIEW.md)。
