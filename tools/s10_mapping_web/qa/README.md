# 独立QA运行说明

只连接本地fake/demo，不连接机器人、不读取真实SSH密码/配置、不运行ROS。运行目录是repo根目录，Python需3.12+。

```bash
python3.12 -B -m unittest discover -s tools/s10_mapping_web/qa -p 'test_independent_*.py'
```

48项包含本地临时SQLite/Unix socket/HTTP与假录制子进程；其中一个真实等待10秒的截止时间测试不会启动rosbag。请勿把fixture改为真实机器人命令。

## 浏览器测试

需要预先准备Node.js、Playwright包及其Chromium浏览器（离线现场不安装依赖）。当前主机脚本默认使用Codex bundled Playwright路径；其他电脑必须用环境变量 `S10_QA_PLAYWRIGHT` 指向本机可解析的Playwright包路径，例如 `/absolute/path/to/node_modules/playwright`。包未附在仓库内。

终端一：

```bash
python3.12 -B tools/s10_mapping_web/field_demo.py --port 18080 --password field-demo-only
```

终端二：

```bash
S10_QA_PLAYWRIGHT=/absolute/path/to/node_modules/playwright node tools/s10_mapping_web/qa/test_browser.cjs
```

这是真正非安全HTTP origin的浏览器测试：fixture把 `http://s10-phone.invalid:18080` 请求仅转给 `127.0.0.1:18080`。不依赖外部DNS、CDN或机器人；浏览器`isSecureContext=false`。网络断开通过fixture abort加browser offline共同注入。模拟用户名/密码仅为`demo`/`field-demo-only`，不要填真机凭据。

覆盖登录、无randomUUID/SW、双击、地图XY/XZ、严格390px宽度（不用会随溢出扩大的innerWidth）、30秒任务中断联刷新不重发、定位人工确认、位置草稿和HTML转义、失败质量报告下载、陈旧localStorage会话恢复、真实长度地图SHA换行。

输出 `BROWSER_ALL_PASS` 并更新 `browser-phone.png`；结束后用Ctrl+C停止终端一的demo，它会清理自己的fake worker及临时数据。测试过程中不得把host改为机器人IP。

结果和现场未测关卡见[REVIEW.md](REVIEW.md)。本地通过不等于真机定位/安全/无外网验收通过。
