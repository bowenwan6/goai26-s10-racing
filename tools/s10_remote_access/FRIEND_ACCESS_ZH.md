# 048 机器人：朋友远程 SSH 使用指南

你使用**自己的 Tailscale 账号**；登录机器人仍使用我们共同的 **`golai` 账号和原 SSH 密码**。不需要、也不要使用机器人的持有者的 Tailscale 登录密码。

状态（2026-09-13）：持有者的 Mac 已通过跨网验收。Waiyiu Pong（`jackypong201246@gmail.com`）已被持有者加入同一个 Tailscale 网络；经持有者确认，已保存该账号到 048 AGX 的 TCP 22 规则。朋友客户端的实际 SSH 登录尚待自己测试。

## 朋友怎么做

1. 从 [Tailscale 官网](https://tailscale.com/download) 安装适合自己电脑的客户端，用 **`jackypong201246@gmail.com`** 登录并连接。只登录管理网页不等于电脑客户端已经接入。
2. 确认客户端接入的是包含 **`s10-48-agx`** 的持有者网络，而不只是自己的其他网络。你的账号已经加入，不需要再次索取单机共享链接，也不要使用持有者的账号密码。
3. 在自己的 Tailscale 设备列表里找到 `s10-48-agx`，复制它的 IP。当前同一网络中的机器人地址是 **`100.104.52.28`**。如果以后改用跨网络单机共享，则以自己客户端显示的地址为准。[设备共享说明](https://tailscale.com/docs/features/sharing)
4. 打开终端（Windows 可用 PowerShell），将下方 `ROBOT_IP` 替换为刚复制的地址后执行：

   ```bash
   ssh golai@ROBOT_IP
   ```

5. 首次连接要核对主机身份。目前这台 048 的 ED25519 指纹为：

   ```text
   SHA256:6D+NMySIxflRihNF45StFNO/s/rQJgt9SfanyNYzTtM
   ```

   显示的算法或指纹不一致就先停下联系持有者，不要删除旧记录或关闭校验。匹配后确认连接，在终端输入共同的 SSH 密码；输入时不显示字符是正常的。密码另外沟通，不写进本文。
6. 登录后可先运行 `whoami`、`pwd`、`ls` 检查；退出输入 `exit`。

双方都要保持联网，但不要求连接同一个 Wi-Fi。机器人断电或失去互联网时不能远程连接。

## 日常可选操作

下载 AGX 上的文件到当前电脑目录，替换 IP 和文件路径：

```bash
scp golai@ROBOT_IP:/机器人上的文件绝对路径 .
```

原始地图在 106 板卡，不要直接把 106 的路径当作 AGX 路径。

建图网页可在持有者验收后，通过 SSH 转发打开：

```bash
ssh -N -L 127.0.0.1:18080:10.21.33.102:8080 golai@ROBOT_IP
```

保持终端开着，浏览器访问 `http://127.0.0.1:18080`，使用原网页账号。结束按 `Ctrl+C`。这不把网页公开到公网。

## 如果电脑也必须一直开 Clash / 其他 VPN

先告诉持有者，确认兼容配置后再使用。普通 Tailscale 客户端与其他 VPN 可能发生路由或 DNS 冲突；不要靠关闭必须保留的 VPN 解决。[官方兼容说明](https://tailscale.com/docs/reference/faq/other-vpns)

持有者当前 Mac 使用专门的用户态 Tailscale 和 `ssh s10-48-remote` 别名；这个别名不是你电脑自带的，不能直接照搬其本机路径和密钥配置。

## 持有者须知

- 本次实际采用的是朋友加入同一网络，再单独授予机器人 TCP 22 权限；不是单机共享。已保留持有者原 Mac 的 SSH 规则，没有增加到 Mac 的访问规则。Mac 的 `shields-up=true` 仍开启。
- 用户先前已给朋友 **Admin** 角色。本次只获确认开通 SSH，没有修改这个角色。Admin 可以修改整个网络的访问策略，因此“当前只放行机器人 22”不等于朋友无法自行修改规则；若只需使用机器人，建议由持有者另行确认降为 Member。[角色说明](https://tailscale.com/docs/reference/user-roles)
- 请朋友在自己的电脑上完成一次真实 SSH 登录。出现超时先核对客户端网络与在线状态；出现 `Permission denied` 则检查共同的 SSH 用户名和密码，不要改用持有者的 Tailscale 密码。
- 不再需要时，需按实际的同网成员方式撤销该账号的机器人规则及相应成员权限；仅找 Share 页面撤销邀请不适用于本次接入方式。
- 如果以后希望只共享一台机器人而不加入整个网络，可以另行按 [单机共享流程](https://tailscale.com/docs/features/sharing) 调整。本次没有擅自移除成员或切换授权方式。

这里使用的是 **Tailscale 网络上的普通 SSH**，不是接管身份认证的 “Tailscale SSH” 功能，因此机器人原有的 SSH 账号密码不变。[两种 SSH 的区别](https://tailscale.com/docs/features/tailscale-ssh)

注意：共用 `golai` 意味着共用该 Linux 账号的文件和程序权限；仅限制网络端口 22 并不等于只读权限，也不阻止 SSH 隧道。操作运动程序前必须有人在现场确认安全、掌握遥控与急停。撤销 Tailscale 权限不改变朋友已知的 SSH 密码，也不撤销其他内网接入方式。
