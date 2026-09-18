# 50 号狗的网络共享原理与 48 号复现步骤

更新日期：2026-09-11；本文运行状态与验收结果以 2026-09-10 实测为准，本次仅补充文档。用户切换到 48 号并授权配置后，**48 号已完成直连 AGX 与热点共享上网，开机配置已保存**。第 1–5 节保留原理与迁移检查；48 号实际配置及回退见第 7–8 节。48 号复用了 AGX 已有外网，外网出口与 50 号的位置不同。

## 1. 能否复制，复杂吗？

**可以复制这套网络结构。** 如果 48 号已有和 50 号相同的热点、无线并发能力及管理权限，工作主要是连接外部 Wi-Fi、补齐路由与转发，复杂度不高。若无线驱动不支持同时连接外网和开热点，或 SSH 环境限制网络管理，则需要使用厂商提供的配置入口，或增加一个独立的上网接口。

50 号由 **103 做路由器和 NAT 网关**；48 号由 **103 转发、AGX 做 NAT 外网出口**。两者都使用网络层转发，不要求电脑先 SSH 登录 103。

补充硬件核验：50 号的 `wlan0/phy80211` 和 `wlan1/phy80211` **都指向 `phy0`**。这说明两个无线接口属于同一个无线 PHY，并非两块独立无线网卡，实测同时承担外网连接与热点功能。48 号也已确认这两个接口共用 phy0，但本次复用了 AGX 的外网，没有启用 103 的 wlan0 上联。

## 2. 50 号已验证的拓扑

```text
Windows：10.21.41.19/24
    │ 连接 S10 PRO-050-5G
    ▼
103 本体控制器
    ├─ wlan1：10.21.41.1/24       机器人热点、电脑的默认网关
    ├─ eth0 ：10.21.33.103/24
    │         10.21.33.1/24       同一个网口的第二个地址、AGX 的网关
    │              └── AGX end0：10.21.33.102/24
    ├─ eth1 ：10.21.32.103/24     另一个内部网段，本次不需调整
    └─ wlan0：10.18.2.125/22      外部网络，DHCP 地址
                   └── 默认网关 10.18.0.1 → 互联网
```

`10.18.2.125` 和 `10.18.0.1` 是当时外部网络分配的地址，不能作为 48 号的固定配置。外网 SSID 本次没有取得。

| 位置 | 实测配置或结果 |
|---|---|
| Windows | 默认路由经 `10.21.41.1`；另有 `10.21.33.0/24 → 10.21.41.1` 路由 |
| Windows DNS | `223.5.5.5`、`223.6.6.6` |
| 103 | `/proc/sys/net/ipv4/ip_forward` 为 `1` |
| 103 默认路由 | `default via 10.18.0.1 dev wlan0` |
| AGX 默认路由 | `default via 10.21.33.1 dev end0` |
| 电脑到 AGX | `tracert` 为 `10.21.41.1 → 10.21.33.102`，SSH 成功 |
| AGX 看到的 SSH 来源 | `10.21.41.19`，保留电脑源 IP，不是 103 代为建立的 SSH 会话 |
| 电脑外网 | 显式绕过代理、绑定 WLAN 地址访问百度 HTTPS，返回 HTTP 200 |
| AGX 外网 | 绕过代理访问百度 HTTPS，返回 HTTP 200 |
| 外网路径 | 电脑 traceroute 前两跳是 `10.21.41.1 → 10.18.0.1` |

Windows 的上述内网路由位于活动路由表，PersistentStore 中未查到对应项，来源本次没有确认。默认路由本身也能覆盖 AGX 目的地址，因此不要在 48 号尚未测试时重复加路由。

## 3. 50 号的转发配置与手机访问历史

`/etc/systemd/system/lidar-network.service` 已启用，启动执行 `/home/user/lidar_network_setup.sh`。该脚本内容为：

```bash
#!/bin/bash
# IP 转发
echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward > /dev/null
# iptables
sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT
sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
# The AGX is directly reachable at 10.21.33.102 via eth0.
# Do not override that connected route through Wi-Fi.
echo "S10 network setup done"
```

其中 `MASQUERADE` 把从 `wlan0` 出去的数据源地址转换为外网接口地址，让内部设备共用该上网出口。电脑到 AGX 则走 `wlan1 → eth0`，不用经过这个外网 NAT 出口。

**这份脚本是现场证据，不是完整的 48 号安装脚本：**

- 它没有写出 `wlan1` 相关的放行规则；这部分通信实测正常，但完整防火墙规则未取得，不能仅靠这几行重建全部热点网络。
- 本次启动日志中，写 `ip_forward` 报了只读文件系统错误；运行时该值确实已为 `1`。不能把服务显示成功当作每条命令都成功，也不能据此确认是谁持久化开启了转发。
- `iptables -A` 重复运行会追加重复规则。迁移时应检查已有规则，只补缺项。
- 103 当前 SSH 环境提示 `sec_docker` 策略不允许 `nmcli`，`sudo -n` 读取防火墙规则也要求密码。48 号应先确认其可用的管理入口；容器内受限时使用厂商支持的入口配置。

另有 `s10-mapping-hotspot.service` 和 `s10-gait-hotspot.service`，通过 SSH `-L` 把 103 的 `8080`、`8090` 转发到 AGX 的网页端口。它们服务于建图/采集网页，**不需要为了直连 SSH 或共享上网而复制**。

50 号手机访问失败的历史处理：原 PRO-050 热点下，iPhone/Mac 访问网页曾超时；抓包确认请求到达 AGX、应答也发回热点，但 TCP 握手未完成，根因没有确定。后来在 AGX 开设 `<CAPTURE_WIFI_SSID>` 独立热点，手机直接连接 AGX 并访问 `http://10.42.50.1:8091/`，用户确认 iPhone 可用，随后设置热点和采集服务开机自启。这是绕开原故障路径，不能用 48 号后来发现的回程路由问题反推当时原因。

**该入口现已下线。** [采集站部署与清理记录](../tools/s10_gait_capture/DEPLOYMENT_050.md)记载，2026-09-10 已删除该独立热点、8091 采集服务和对应旧中转目录；原 xwy 采集/建图服务保留。上述 SSID 和 URL 是历史解决方案，不代表目前仍能使用。它们也不参与本节的 PRO-050 共享上网路径。

## 4. 切到 48 号后，先做这些检查

先确认实物编号和实际 SSID。多台狗可能共用 `.102/.103`，不要用 IP 相同作为设备身份依据。保留 50 号的 SSH 主机密钥记录；48 号可以用独立 `HostKeyAlias`，首次指纹与现场交接信息核对。

### Windows

```powershell
netsh wlan show interfaces
Get-NetIPConfiguration
Get-NetRoute -AddressFamily IPv4
```

若现场确认 48 号也使用 `10.21.33.102`，再执行：

```powershell
tracert -d -h 4 -w 700 10.21.33.102
ssh -o HostKeyAlias=s10-48-agx <48号AGX账号>@10.21.33.102
```

若 AGX 不通而 103 可达，先用 48 号自己的账户登录 103 做检查；这是排障入口，不是最终必须保留的跳板方式。50 号现有的 SSH 密钥不保证能登录 48 号。

### 103：接口、路由、转发与管理权限

```bash
hostname
ip -br -4 addr
ip route
cat /proc/sys/net/ipv4/ip_forward
ls -l /sys/class/net/wlan*/phy80211
command -v iw nmcli
systemctl status lidar-network.service --no-pager
systemctl cat lidar-network.service
sudo -n iptables -S
sudo -n iptables -t nat -S
sudo -n nft list ruleset
```

某个工具或服务不存在只表示它没有沿用 50 号的实现。若 `iw` 已存在且允许使用，查看 `iw dev`、`iw list`，确认 AP 与 managed 接口的并发能力、接口数量和信道限制。若两个接口共享 PHY，外部 Wi-Fi 的信道选择可能影响现有热点。

### AGX：回程路由

```bash
ip -br -4 addr
ip route
ip route get <电脑当前Wi-Fi地址>
```

记录 AGX 发往电脑的下一跳。50 号是经 `10.21.33.1` 返回 103；48 号可以用自己的现有正确网关，不必为追求地址一致而新增 `.1`。

## 5. 按缺项复现，不直接覆盖现有网络

先打通电脑到 AGX，再处理外网；这两个目标可以分别验证。

| 顺序 | 需要达到的状态 | 最小处理 |
|---|---|---|
| 1. 保留热点 | 电脑持续连接 48 号热点，能到其网关 | 复用已有 AP、DHCP 和地址配置 |
| 2. 内网路由 | 电脑发往 AGX 的流量到达 103，103 从内部有线接口送到 AGX | 默认路由已覆盖就不加；确有缺项再补目的网段路由 |
| 3. 回程路由 | AGX 发往电脑 Wi-Fi 网段的流量回到 103 | 复用现有网关，或补精确网段路由；保留其他业务所需默认路由 |
| 4. 内网转发 | 103 开启 IPv4 forwarding，防火墙允许电脑网段访问 AGX 及其返回流量 | 在实际使用的规则体系中只补缺失规则，注意已有丢弃规则的顺序 |
| 5. 外网上联 | 选定的出口设备连到可用外部 Wi-Fi，获得地址、默认网关和 DNS；50 号出口为 103，48 号为 AGX | 优先复用已有外网；仅在 NetworkManager 确实管理接口且命令获准时使用 nmcli |
| 6. 外网共享 | 电脑网段、AGX 网段可经外网接口转发，返回流量被允许，并有对应 NAT | 复用现有共享功能；否则增加限定内部源网段的 MASQUERADE 与必要转发规则 |
| 7. 名称解析 | 电脑、103、AGX 都能解析外网域名 | 使用实际可达的 DNS，不只以 ping IP 作为上网成功 |
| 8. 持久化 | 临时验证通过后，正常重启仍保持上述状态 | 沿用实际管理服务保存配置，避免新增第二套热点/DHCP或相互覆盖的防火墙管理 |

外部 Wi-Fi 的 SSID、登录方式/密码及 48 号管理账户需要现场提供；密码不写入本文。若同一 PHY 的并发能力或管理权限不满足要求，再考虑独立 USB 无线网卡、有线上联等替代入口，不预先采购硬件。

**修改前记录将要调整的地址、路由、转发开关、规则和对应配置文件。** 通过可能被改断的热点远程配置时，先具备现场或有线恢复入口。回退只撤销本次新增项、恢复本次修改项；不要清空整个防火墙或整份路由表。不要修改机器人运动程序或重启本体来试网络。

## 6. 验收与待填结果

| 检查项 | 48 号结果 |
|---|---|
| 设备、SSID 与 SSH 身份记录 | 用户确认 48 号；SSID 实测为 `<ROBOT_WIFI_SSID>`；103 通过已有 known_hosts 检查，AGX 首次记录到独立别名 `s10-48-agx` |
| AP、外网上联、AGX 内网接口及 PHY 对应关系 | 103 的 wlan0/wlan1 共用 phy0，当前只启用热点；AGX 自己的无线接口连接外网 |
| 电脑直接 SSH 到 AGX，无 ProxyJump | 通过，原生 OpenSSH 验收脚本退出码 0 |
| AGX 的 SSH 连接来源与回程路径 | 来源 `10.21.41.19`；经 `end0 → 10.21.33.103` 返回 |
| 103 自己能访问外网 | ping 外网及 HTTP 200 通过；HTTPS 因本体系统时间错误而校验失败，见第 7 节 |
| 电脑绕过代理，经狗的 Wi-Fi 访问外网 | 绑定 WLAN 地址访问 HTTPS 返回 200；AGX 对应 NAT 计数增加 |
| AGX 绕过代理访问外网 | HTTPS 200 |
| 使用实际网络管理服务保存配置 | AGX 路由写入现有 NetworkManager 配置；两台新增服务均 enabled/active |
| 正常重启后的热点、直连 SSH、外网访问 | 未重启整机，待下一次正常开机验收 |

Windows 外网测试时把占位符替换为当时的狗 Wi-Fi 地址，避免其他网卡或电脑代理掩盖结果：

```powershell
curl.exe --noproxy '*' --interface <电脑当前Wi-Fi地址> --connect-timeout 5 --max-time 10 -I https://www.baidu.com
```

103、AGX 各自测试：

```bash
curl --noproxy '*' --connect-timeout 5 --max-time 10 -I https://www.baidu.com
```

已留下可运行的 [Windows 验收脚本](../artifacts/s10-network-048-20260910/check.ps1)，实际执行通过。运行时会交互询问 AGX 密码；若电脑的狗 Wi-Fi 地址变化，传入 `-WifiAddress <新地址>`。脚本验证电脑经指定 Wi-Fi 地址访问 HTTPS，以及直接 SSH 到 AGX 后访问 HTTPS。

## 7. 48 号实际配置与问题根因

### 实际路径

```text
电脑 10.21.41.19
    → <ROBOT_WIFI_SSID> / 103 wlan1：10.21.41.1
    → 103 eth0：10.21.33.103
    → AGX end0：10.21.33.102/28
    → AGX wlP1p1s0：10.18.2.147/22（云谷之芯）
    → 上级网关 10.18.0.1 → 互联网
```

50 号是 103 连接外网；48 号是 AGX 已连接外网，因此复用 AGX 做出口，避免另行连接 103 的 wlan0。AGX 必须开机且其外部 Wi-Fi 可用，热点共享上网才可用。AGX 的 DHCP 外网地址可以变化，NAT 按接口匹配。

电脑打开网页时，数据经过以下步骤：

1. 电脑把外网请求交给默认网关 `10.21.41.1`，即 103 的热点接口。
2. 103 根据新增默认路由，把请求交给 AGX `10.21.33.102`。
3. AGX 开启 IP 转发，把请求送向外部 Wi-Fi；NAT 将源地址从电脑的 `10.21.41.19` 转换为 AGX 当时的无线地址 `10.18.2.147`。上级网络因此不需要另外知道狗热点网段的返回路径。
4. 回复到达 AGX 后，按 NAT 连接记录还原目标地址，再根据回程路由经 103 返回电脑。DNS 负责把网站名称解析为目的 IP。

电脑直接 SSH 到 AGX 时，AGX 就是终点，不需要经过外网 NAT。共享上网额外需要 AGX 继续转发数据到外部网络；两项功能应分别验收。

### 直连 SSH 原先为什么不通

103 原有 IP forwarding 已经为 `1`，防火墙也已允许来自 `10.21.41.0/24` 的热点流量及返回连接。AGX 的有线地址为 `10.21.33.102/28`，有线网段为 `10.21.33.96/28`；103 的 `.103` 在同一网段，但 AGX 没有到电脑热点网段的路由。

修改前，AGX 的 `ip route get 10.21.41.19` 显示从 `wlP1p1s0` 发往外网网关 `10.18.0.1`。补上下面这条回程路由后，电脑立即可以直连 SSH：

```text
10.21.41.0/24 via 10.21.33.103 dev end0
```

这条路由已写入 AGX 原有 `Wired connection 1`，UUID 为 `1f85efec-ba59-372b-a0ee-bef275576557`。AGX 原来的 `/28` 地址和外网默认路由保持原值；没有重新激活正在使用的网卡。

该 `/24` 路由比外网默认路由更具体，因此目的为 `10.21.41.*` 的回复会交给 103。它覆盖整个热点网段，手机直接访问 `10.21.33.102:端口` 也会受益；具体网页仍需验证服务是否监听和允许访问。如果手机访问的是 103 上的 SSH 中转入口，AGX 看到的是 103 发起的连接，应按中转路径排查，不能直接套用手机回程缺失的结论。

### 用户截图中的 /29 与 /28 问题

用户提供的另一 agent 截图描述，AGX 当时有 `10.21.33.102/29` 和额外的 `10.21.33.103/28`。其中子网计算如下：

| AGX 配置 | 子网地址 | 普通主机地址范围 | 广播地址 |
|---|---|---|---|
| `10.21.33.102/29` | `10.21.33.96` | `.97–.102` | **`.103`** |
| `10.21.33.102/28` | `10.21.33.96` | `.97–.110` | `.111` |

按截图所述，`/29` 会把本体的 `.103` 地址视为广播地址；AGX 再把 `.103` 配给自己，还会与本体地址冲突。截图最后建议移除错误地址、仅保留 AGX 的 `.102/28`，方向合理。

**本次接手时，AGX 已经只有 `.102/28`。** `baseline.json` 记录了当时的有线配置；本次没有执行截图中的删地址操作，无法确认更早的修改由谁完成。本次发现并修复的是剩下的热点网段回程路由缺失。截图中的地址问题、此次路由问题和本机 SSH 身份检查问题属于不同环节。

### 新增的开机配置

| 机器 | 持久化位置 | 作用 |
|---|---|---|
| AGX | 现有 NetworkManager 有线配置的 `ipv4.routes` | 保存上述回程路由 |
| AGX | `/etc/systemd/system/s10-agx-internet.service` | 开机执行 `/usr/local/sbin/s10-agx-internet.sh` |
| AGX | `/usr/local/sbin/s10-agx-internet.sh` | 开启 IPv4 转发，并给两个指定源网段/地址添加外网 NAT |
| 103 | `/etc/systemd/system/s10-agx-gateway.service` | 开机执行 `/usr/local/sbin/s10-agx-gateway.sh` |
| 103 | `/usr/local/sbin/s10-agx-gateway.sh` | 设置 `default via 10.21.33.102 dev eth0 metric 700`，并写入可用 DNS |

AGX 新增 NAT 的源分别为 `10.21.41.0/24`（热点客户端）和 `10.21.33.103/32`（103 本身），出口均为 `wlP1p1s0`，标记 `s10-48-internet`。启动脚本先检查规则是否存在，再添加；临时配置后启动服务的验收确认仍恰好两条规则。

103 原有 `/etc/resolv.conf` 指向 `127.0.0.53`，但解析服务被 masked，导致本体无法解析域名。已改为 `223.5.5.5`、`223.6.6.6`，由上述网关服务在启动时写入；原文件内容保存在 baseline 中。未启动或解禁被屏蔽的服务。

**剩余独立问题：103 的系统日期为 `2026-03-14`。** DNS 修复后，它自身访问百度 HTTPS 报 `certificate is not yet valid`；外网 ping 和 HTTP 200 正常。电脑与 AGX 的 HTTPS 验收均通过。此次没有修改机器人时钟，时钟问题不影响其转发电脑数据包。

HTTPS 证书有生效和到期日期，103 的日期太早会把有效证书判断为“尚未生效”。因此 103 自己通过 HTTPS 下载软件或调用接口可能失败，日志日期也会错误；若数据使用该系统时间打戳，跨设备时间对齐也可能受影响。电脑和 AGX 使用各自的时间校验证书，103 转交数据包不需要代它们校验网站证书。后续应通过正常授时修正本体时钟，不能靠关闭证书校验解决；本次未修改时钟或验证冷启动后的授时状态。

### 本地证据与连接命令

配置副本与记录位于 `artifacts/s10-network-048-20260910/`：

- [baseline.json](../artifacts/s10-network-048-20260910/baseline.json)：修改前路由、转发状态、规则、有线配置及 103 原 DNS 文件内容，不含密码。
- [verified.json](../artifacts/s10-network-048-20260910/verified.json)：配置后的路由、服务、NAT 和验收结果。
- [AGX 启动脚本](../artifacts/s10-network-048-20260910/s10-agx-internet.sh)、[AGX 服务](../artifacts/s10-network-048-20260910/s10-agx-internet.service)、[103 启动脚本](../artifacts/s10-network-048-20260910/s10-agx-gateway.sh)、[103 服务](../artifacts/s10-network-048-20260910/s10-agx-gateway.service)：已安装文件的源码副本。

两台机器的上传副本分别保留在 `/home/ysc/s10-network-048-20260910/`、`/home/user/s10-network-048-20260910/`。

在当前电脑直接连接 AGX，使用独立主机密钥别名保留其他狗的记录：

```powershell
ssh -o HostKeyAlias=s10-48-agx -o StrictHostKeyChecking=yes ysc@10.21.33.102
```

### 为什么暂时不能省掉 HostKeyAlias

2026-09-10 复查确认，本机 `known_hosts` 中裸 IP `10.21.33.102` 仍对应旧设备的主机密钥；48 号 AGX 的密钥不同。因此直接执行 `ssh ysc@10.21.33.102` 会遇到主机指纹不匹配，尽管网络已经连通。`HostKeyAlias` 只选择用于核对身份的记录，不是跳板、代理或端口转发。

若希望在这台电脑上使用短命令，可以在 `C:/Users/Lenovo/.ssh/config` 中添加以下配置：

```sshconfig
Host 10.21.33.102
    HostKeyAlias s10-48-agx
```

这样短命令也会检查 48 号的身份。切回 50 号或其他同 IP 设备时，需要切换相应身份配置，不能继续把它当成 48 号。**以上只是配置示例，本次没有写入本机 SSH 配置，也没有删除旧设备密钥。**

## 8. 48 号回退方法（需要撤销时使用）

先核对后续没有其他业务依赖这些新增项。以下撤销会恢复“需要经 103 登录 AGX”的原状，并停止通过狗热点共享 AGX 外网。**先保留一条从 103 登录 AGX 的会话，再移除回程路由**，否则电脑直连 AGX 的会话会断开。

在 **103** 执行：

```bash
sudo systemctl disable --now s10-agx-gateway.service
sudo ip route del default via 10.21.33.102 dev eth0 metric 700
printf '%s\n' 'nameserver 127.0.0.53' 'options edns0 trust-ad' 'search .' | sudo tee /etc/resolv.conf >/dev/null
sudo rm -- /etc/systemd/system/s10-agx-gateway.service /usr/local/sbin/s10-agx-gateway.sh
sudo systemctl daemon-reload
```

通过保留的 **103 → AGX** 会话，在 AGX 执行：

```bash
sudo systemctl disable --now s10-agx-internet.service
sudo iptables -t nat -D POSTROUTING -s 10.21.41.0/24 -o wlP1p1s0 -m comment --comment s10-48-internet -j MASQUERADE
sudo iptables -t nat -D POSTROUTING -s 10.21.33.103/32 -o wlP1p1s0 -m comment --comment s10-48-internet -j MASQUERADE
sudo sysctl -w net.ipv4.ip_forward=0
sudo nmcli connection modify uuid 1f85efec-ba59-372b-a0ee-bef275576557 -ipv4.routes '10.21.41.0/24 10.21.33.103'
sudo ip route del 10.21.41.0/24 via 10.21.33.103 dev end0
sudo rm -- /etc/systemd/system/s10-agx-internet.service /usr/local/sbin/s10-agx-internet.sh
sudo systemctl daemon-reload
```

`ip_forward=0` 是 AGX 的本次原始值，仅在确认没有后续其他转发用途时恢复。回退命令只删除本次有标记的 NAT 和精确路由；不要用清空规则表或覆盖全部路由来替代。

## 9. 2026-09-12：离开房间后 Wi-Fi 无法自动恢复

AGX 的 `/var/log/syslog` 记录：17:19 漫游后发生 DHCP 失败，17:20 自动恢复；17:23 再次断线后出现多次 `ASSOC-REJECT`，17:25:10 最终以 `no-secrets` 失败，此后直到 18:23 重启没有记录到该 Wi-Fi 再次自动激活。房间网络仍使用已保存的 WPA-PSK，`psk-flags=0`，不能将 `no-secrets` 直接解释为用户没保存密码。

核对 NetworkManager 1.46.0 的 [Wi-Fi 实现](https://github.com/NetworkManager/NetworkManager/blob/1.46.0/src/core/devices/wifi/nm-device-wifi.c)：关联超时会进入 `handle_auth_or_fail`，认证重试耗尽时以 `NO_SECRETS` 失败；该失败会阻止继续自动连接。其调用的 [认证重试实现](https://github.com/NetworkManager/NetworkManager/blob/1.46.0/src/core/devices/nm-device.c) 将 `connection.auth-retries=0` 解释为无限重试。虽然 1.46 手册描述只提到 802.1X，此版本 Wi-Fi 代码也使用该设置。

已通过原有 **ysc 管理账号**修改房间 Wi-Fi 的现有配置；`golai` 没有 sudo 权限。仅改以下三个属性，未保存管理员密码：

```bash
sudo nmcli connection modify uuid 044a335e-093e-4184-a02f-7ed4c243ac87 \
  connection.auth-retries 0 connection.autoconnect-retries 0 802-11-wireless.powersave 2
```

认证和自动连接持续重试；关闭该连接的无线省电。省电模式是辅助调整，日志没有证明它是最初断线的原因。已有转发服务、NAT、路由及运动程序未改动，没有增加定时重启或额外守护程序。

验证：关闭 AGX 无线 8 秒再开启，NetworkManager 约 5 秒后自动完成连接和 DHCP；省电状态保持 `off`。验证窗口内电脑到 AGX ping 6/6 成功；AGX 绑定 `wlP1p1s0` 和 Windows 绑定狗热点地址 `10.21.41.19` 访问 HTTPS 均返回 200。配置前后快照见 [before.txt](../artifacts/s10-wifi-reconnect-20260912/before.txt)、[after.txt](../artifacts/s10-wifi-reconnect-20260912/after.txt)。这验证了无线重新开启后的自动恢复；实际出门、回屋时的弱信号和 AP 拒绝场景仍需现场复核。

若再次卡住，电脑保持连接狗热点，通过管理员账号只重连 AGX 的房间 Wi-Fi：

```powershell
ssh -t -o HostKeyAlias=s10-48-agx ysc@10.21.33.102 'sudo nmcli --wait 30 connection up uuid 044a335e-093e-4184-a02f-7ed4c243ac87'
```

需要撤销本节三个设置时，在 `ysc` 会话执行以下命令；最后一条会短暂重连外网 Wi-Fi：

```bash
sudo nmcli connection modify uuid 044a335e-093e-4184-a02f-7ed4c243ac87 \
  connection.auth-retries -1 connection.autoconnect-retries -1 802-11-wireless.powersave 0
sudo nmcli --wait 30 connection up uuid 044a335e-093e-4184-a02f-7ed4c243ac87
```
