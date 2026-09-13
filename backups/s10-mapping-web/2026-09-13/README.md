# 48 号当前手机 App：实机备份（2026-09-13）

这是从运行中的 48 号机器读取的应用快照，不是把仓库旧版再复制一遍。包含建图网页、静态定位预览、新增实时高度图、106 后台、103 热点转发和相关启动检查脚本。**备份没有向机器人写入文件、切换地图或重启服务。**

仓库基础提交：`defbb719ac3822941479caccdee2f35ef0547cf4`。本快照独立保存在 `backups/`，没有替换原有 `tools/s10_mapping_web/` 或修改主分支。

## 包含什么

| 目录 | 实机来源 | 内容 |
| --- | --- | --- |
| `agx102/` | 102，`golai` | `server.py`、首页、定位页、高度图页、部署时 README、实际用户服务文件 |
| `slam106/` | 106，`user` | `robot_backend.py`、`backend.sh`、新增 `heightmap.py` |
| `slam106/support/` | 106 | 建图前传感器检查 `check_s10_slam.py`、PTP 启动检查 `wait_s10_ptp.py` |
| `slam106/time-ready/` | 106，`/etc/systemd/system/` | mapping、localization、yesense、rsdriver、hsLidar 五项实际启动检查 drop-in |
| `relay103/` | 103，`/etc/systemd/system/` | 热点转发的 socket 与 service |

共 **18 个实机文件，6,009,896 字节（约 6.01 MB）**。每个文件的原路径、权限、修改时间、大小和 SHA-256 在 `manifest.json` 中。

`localization.html` 包含当前旧室内图的抽样显示资源和本地图形库，因此它约 5.93 MB；这是 App 正在使用的资源，不是本次 fullrun 的完整地图。未额外上传地图目录、PCD、录制或运行日志。

`agx102/README.md` 按实机原样保存，版本早于高度图更新，且其中相对链接可能依赖原来的仓库位置。当前快照说明以本文件为准；原目录文档可参考仓库的 `tools/s10_mapping_web/README.md`。

## 当时的部署状态

- 手机入口：`http://10.21.41.1:8080/`；103 将其转发至 102 的 `10.21.33.102:8080`。
- 102 用户服务 `s10-mapping-web.service`：active/running。Python 3.12.3，aarch64，Paramiko 软件包 `2.12.0-2ubuntu4.1`。
- 106：官方 localization active，mapping inactive；旧 HTTP 服务 `s10-mapping-web.service` disabled/inactive，不应重新启用。
- 106：Python 3.12.3，aarch64；`slam 3.5.1`、`slam-common-lib 1.1.3`、`ros-jazzy-rclpy 7.1.9-1noble.20260124.093049`、`ros-jazzy-grid-map-msgs 2.2.3`。
- 当时激活地图为 `indoor_loop_01-20260911-203447`。备份不激活任何地图，也不证明新图已通过定位验收。
- 103 socket active，配套转发 service 当时 inactive；它是按连接触发的 socket 服务，不能仅以该 service 当时 inactive 判为故障。

这些是采集时的状态/版本，不是对其他日期或重装后的保证。厂商 ROS、SLAM、PTP、Fast DDS 配置及系统库是外部依赖，不包含在本应用备份中。

## 校验备份

进入本目录后，macOS 执行：

```sh
shasum -a 256 -c SHA256SUMS
```

Linux 可使用：

```sh
sha256sum -c SHA256SUMS
```

18 项都应显示 `OK`。目录内 `.gitattributes` 禁用源文件的换行转换，保留实机原有字节，包括部分服务文件的 CRLF。不要先运行格式化器或自动换行转换再做校验。

本轮验证是文件完整性、源文件语法及敏感信息检查，不是重新运行机器人建图/定位的功能验收；没有为备份执行真实 start/save/切图操作。

## 明确排除的内容

- SSH 私钥 `backend_key`、其他私钥、公钥授权文件及 `known_hosts`。
- `~/.config/s10-mapping-web/config.json`、其中的网页登录哈希，以及任何 SSH/网页密码、令牌。
- 会话/锁文件、字节码、旧版 `.before-*` 文件、禁用的旧 106 HTTP 服务。
- 原始地图、录像/录制、厂商二进制及操作系统镜像。

使用显式文件白名单采集；发布前还对凭据模式和新增文件清单做检查。`.gitignore` 只是辅助防误提交，不代替凭据审查。

因此这是**应用代码及部署文件备份**，不是“空机器下载即可免配置恢复”的系统镜像。身份凭据必须另行安全保管/重新签发，不能放回 GitHub。

## 日后怎样恢复

以下是恢复清单，不是本次已经执行的动作。恢复涉及覆盖文件和重新加载服务，须另行确认机器人停稳、现场人员可接管且没有建图保存操作，再进行。

1. 先检查目标仍是同一台 48 号，确认兼容的厂商环境、ROS 2 Jazzy、Python 和依赖。再次备份待覆盖版本，核对本快照 SHA-256。
2. 按 `manifest.json` 的 `source_node` 和 `source_path`，把各文件恢复到对应板卡的原位置；恢复所有者与 `source_mode`。不要把三个目录混装到同一台板卡。
3. 102 恢复或重新配置独立后台密钥、核实过的 106 主机密钥和网页登录哈希，认证配置权限设为 600。106 对应授权公钥保持来源与强制命令限制；不要覆盖其他人的授权条目。
4. 106 保留官方地图/定位环境与 PTP 链路。`wait_s10_ptp.py` 固定检查已核验的 48 号 103 时钟身份，不应直接用于其他机器人。恢复 drop-in 后核对实际路径与依赖，不因复制文件而立即重启传感器。
5. 在安全维护窗口，针对实际恢复的服务执行必要的配置重载/重启。102 使用 `golai` 的用户服务；103 使用 socket 转发；106 不启用旧 HTTP 服务，也不把停止网页当作停止建图。
6. 人工复验登录、状态、地图列表、实时高度图和当前定位页。户外要另做无互联网手机测试；本快照不包含计划中的“现场助手”六项新增功能。

回退也按上述停稳、留存当前版本和凭据校验流程操作。不要未经审阅就把整个备份覆盖到正在工作的机器人上。
