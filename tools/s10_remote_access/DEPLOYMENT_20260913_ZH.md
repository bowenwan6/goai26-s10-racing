# Tailscale 用户态部署记录：配置及跨网验收完成

用户授权配置并验收远程 SSH，追加约束：Mac 的 Clash 必须始终保持开启。

## 当前已完成

- Mac：Homebrew 官方 core formula 安装 `tailscale 1.102.2`（只有这个新 formula，未启动 Homebrew 的 root 服务）。初次直连下载缓慢，停止本次下载后，仅为安装进程指定已有 `127.0.0.1:7890` 代理重新下载成功；没有修改 Clash 配置或系统代理。
- AGX：官方 ARM64 静态包 `1.102.4`，SHA256 `9dd1e6a592a014bbaea0103167ffe299adeda4ba14e078ce9c2895364f6c4c3f`，下载后校验通过。
- 两端 `tailscaled --tun=userspace-networking`，不创建 TUN 接口；状态查询显示 `TUN=false`。
- 禁止接受 DNS 和子网路由、不使用/发布出口节点、不发布内部网段、不开 Tailscale SSH 接管；保留原 OpenSSH。
- 用户已完成两端登录；两端状态均为 `Running`、`Online=true`。访问规则核验后，AGX 改为 `shields-up=false`；Mac 保持 `shields-up=true`。
- 初始部署已将默认全放行 grant 收紧为 `100.106.115.32` → `100.104.52.28` / `tcp:22`。随后经用户明确确认，新增朋友账号到同一机器人的 TCP 22 grant（见下文）。保留其他配置及原 Tailscale SSH 策略段；两端客户端 `RunSSH=false`，实际使用原 OpenSSH。
- 日志上传关闭（`--no-logs-no-support`），本地状态目录私有，认证状态不复制到仓库。
- 已确认节点：Mac `s10-48-mac` / `100.106.115.32`；AGX `s10-48-agx` / `100.104.52.28`，完整名称 `s10-48-agx.tail138c14.ts.net`。授权链接不保存在本文。

## 文件与服务

| 设备 | 位置 |
|---|---|
| Mac 命令行 | `/opt/homebrew/bin/tailscale`、`/opt/homebrew/bin/tailscaled` |
| Mac 启动项 | `/Users/xxxwbwxxx/Library/LaunchAgents/com.goai.s10-tailscale.plist`，launchd `gui/501/com.goai.s10-tailscale` |
| Mac 私有状态/套接字 | `/Users/xxxwbwxxx/.local/state/s10-tailscale/`、该目录的 `tailscaled.sock` |
| Mac SSH 别名 | `/Users/xxxwbwxxx/.ssh/s10-48-remote.conf`；仅在原 config 顶部增加 Include |
| 原 SSH 配置备份 | `/Users/xxxwbwxxx/.ssh/config.before-s10-ts-20260913` |
| AGX 二进制 | `/home/golai/.local/lib/s10-tailscale/tailscale_1.102.4_arm64/` |
| AGX 私有状态/套接字 | `/home/golai/.local/state/s10-tailscale/`、该目录的 `tailscaled.sock` |
| AGX 用户服务 | `/home/golai/.config/systemd/user/s10-tailscale.service`；已 enable/start，原 `Linger=yes` 未变 |
| AGX 安装材料 | `/home/golai/s10-tailscale-setup-20260913/` |

`golai` 的 `sudo -n true` 返回需要密码。采用不提权的用户态方案，未变更 sudoers、防火墙或网络配置。安装静态包不会登记到 dpkg，以用户服务和上述目录作为安装状态依据。

## 已做的测试

- 本地脚本语法和 plist 格式通过；AGX systemd 单元校验通过。
- 两端服务均在运行，且已完成登录；各自的节点状态能看到对端。
- 安装前，经现有 Clash 代理：Google HTTP 204（1.78 秒），GitHub HTTP 200（2.49 秒）。
- 两端 daemon 已启动后，经现有 Clash 代理：Google HTTP 204（1.61 秒）。
- 系统 HTTP/HTTPS/SOCKS 代理仍为 `127.0.0.1:7890`，没有修改或停止 Clash。
- 登录后，Tailscale 对 AGX 的三次 discovery ping 均响应（8、7、5 毫秒）。这不是 TCP 22 / SSH 测试，也不是跨网络验收。
- 登录后同时经 Clash 代理访问 Google 返回 HTTP 204（1.81 秒）、GitHub 返回 HTTP 200（2.08 秒）。证明本次短时探测下两者可同时联网，不代表已经验证所有 App 或长期稳定性。
- 内置浏览器管理页读取多次超时；改用 Arc，在用户登录后完整检查原规则。JSON 编辑器两次输入异常均未保存并已撤销，随后通过可视化表单准确修改唯一的全放行规则，控制台显示保存成功。没有为朋友创建共享邀请或新增权限。
- 在 AGX 读取到的实际 `PacketFilter` 只有来源 `100.106.115.32/32`、目标 `100.104.52.28/32`、协议 6（TCP）、端口 22；确认后才解除 AGX 入站总屏蔽。
- 15:34 左右，在机器人热点 `10.21.41.10` 下通过 `ssh s10-48-remote` 成功登录 `golai`，主机指纹严格校验，读取系统信息与服务状态成功，退出码 0。随后第二次独立 SSH 也成功。
- 通过 SCP 下载只读测试文件 `/etc/os-release`，本地和远端 SHA256 均为 `01af466feb100306498c86aa6bad1815e33036019aa34d4362c20f374ea5c829`。文件保存在 `artifacts/s10-remote-access/ssh-check-20260913.6LVr4n/agx-os-release.txt`。
- 同时经 Clash 访问 Google HTTP 204（1.10 秒）、GitHub HTTP 200（1.88 秒）；系统代理仍为 `127.0.0.1:7890`，默认网关仍为 `10.21.41.1`，两端接受 DNS/路由的选项仍关闭。
- 通过 SSH 本机 `127.0.0.1:18080` 转发原建图网页，GET `/` 返回 HTTP 200、`text/html`。仅验证入口，没有登录网页或操作建图/运动功能；验证后已关闭本次临时隧道。
- AGX 服务 `enabled`、`active`，既有 `Linger=yes`；Mac launchd 服务运行中。没有为验收重启机器人，因此重启后的实际恢复行为尚未测试。

### 初始部署时的网络 grant

```json
{
  "src": ["100.106.115.32"],
  "dst": ["100.104.52.28"],
  "ip": ["tcp:22"]
}
```

原规则为 `src=["*"]`、`dst=["*"]`、`ip=["*"]`。只修改这一条及其说明文字，没有覆盖整个策略文件。限制作用于 Tailscale 流量，不改变现场局域网访问。SSH 本身仍允许该 Linux 账号已有的文件、命令及隧道能力。

## 最终跨网验收：通过

- 时间：2026-09-13 15:38–15:39（Asia/Shanghai）。用户将 Mac 切回自己的手机热点；实测 `en0=172.20.10.4`，默认网关 `172.20.10.1`，不在机器人热点网段。
- 两端 Tailscale 均在线。通过 `ssh s10-48-remote` 新建连接，成功以 `golai` 登录，读取系统信息与 `/etc/os-release` 校验值，服务为 `active`；退出码 0。随后第二次独立 SSH 登录读取账号及时间也成功，没有复用先前热点内的连接。
- 通过 SCP 跨网下载 `/etc/os-release` 到 `artifacts/s10-remote-access/crossnet-check-20260913.93fhe1/agx-os-release.txt`，SHA256 与远端及热点内测试一致。
- Tailscale discovery ping 两次响应为 39、26 毫秒，报告使用公网端点。连通性结论以真实 SSH/SCP 测试为准，不只依赖 ping。
- 同时经既有 Clash 代理访问 Google HTTP 204（0.83 秒）、GitHub HTTP 200（2.71 秒）；系统 HTTP/HTTPS/SOCKS 代理仍开启，地址仍为 `127.0.0.1:7890`。
- 新建 SSH 网页隧道后，访问本机 `127.0.0.1:18080/` 返回 HTTP 200、`text/html`（0.07 秒）。只测试入口，未点击建图或运动按钮；验证后已关闭本次临时隧道。
- 结论：当前 Mac → 048 AGX 的跨网普通 SSH、SCP、网页入口转发和 Clash 共存验收通过。没有测试全部网页交互、长期掉线恢复或机器人重启恢复，不据此承诺这些行为。

## 朋友 SSH 授权：策略已保存，朋友端待测试

- 在 Users 页面发现用户新增的 Waiyiu Pong（`jackypong201246@gmail.com`），角色为 Admin，实际是同一 tailnet 成员，不是仅单机共享。
- 用户确认“给这个账号开通”后，在可视化策略编辑器保存下面的独立 grant。保存后的列表显示原 Mac 规则及朋友规则各一条，没有到 Mac 的新增放行。

```json
{
  "src": ["jackypong201246@gmail.com"],
  "dst": ["100.104.52.28"],
  "ip": ["tcp:22"]
}
```

- 保存后原 Mac 仍可通过 Tailscale SSH 读取 AGX 状态。Mac 本机复查 `ShieldsUp=true`、`RunSSH=false`，未改变其保护。
- AGX 当次 `PacketFilter` 仍只列原 Mac 的 IP；尚未验证朋友客户端是否已接入、是否生成对应设备规则，更未从朋友电脑实际登录。不能把管理页保存成功当作朋友端连通性通过。
- 用户这次确认的是 SSH 开通；没有明确要求修改现有角色，因此保留用户先前设置的 Admin，未降级为 Member。已告知 Admin 可修改网络访问规则，只需使用机器人时建议另行确认降级。
- 指南已按“同网成员”实际方式更新，不再要求这个已加入的账号重复领取单机共享邀请。SSH 使用共同的 `golai` 和原密码，指南不含密码。

## 可恢复停用（仅停本次新增服务）

Mac：

```bash
launchctl bootout gui/501 /Users/xxxwbwxxx/Library/LaunchAgents/com.goai.s10-tailscale.plist
```

AGX，通过保留的原内网 SSH：

```bash
systemctl --user disable --now s10-tailscale.service
```

这不删除状态/授权，之后可以恢复。若要完整卸载，应只删除本次明确列出的文件并撤销两台 Tailscale 节点，保留其他 SSH 配置，不直接覆盖用户后续修改。

参考：[用户态模式](https://tailscale.com/docs/concepts/userspace-networking)、[其他 VPN 共存](https://tailscale.com/docs/reference/faq/other-vpns)、[daemon 参数](https://tailscale.com/docs/reference/tailscaled)。按 `bash-script-template` 加入目标检查、失败停止、禁止覆盖已有安装及停用说明。
