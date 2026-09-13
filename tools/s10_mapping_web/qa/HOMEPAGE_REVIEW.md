# 首页现场助手入口：独立专项检视

日期：2026-09-13。基线：`34b8e3f`。范围：implementer新增首页明显入口，master负责真机部署；reviewer只运行电脑本地demo，不SSH、不部署、不改生产源码。

## 结论

**本地专项通过，未发现需要返工的阻断问题。** 新按钮在手机首页首屏可见，登录跳转正确。这个结论不声称已部署成功；真实103→102转发、106 worker和手机网络由master另行核验。

`node tools/s10_mapping_web/qa/test_homepage_entry.cjs` 输出：

```text
ENTRY visible before login, first-screen >=46px, strict390px, old controls PASS
ENTRY unauthenticated click -> login -> field without URL loss PASS
ENTRY already-authenticated homepage click opens field directly PASS
ENTRY old pages and explicit404 retained; no write actions or external requests PASS
HOMEPAGE_ENTRY_ALL_PASS
```

另独立重跑原 `test_server.py` → `MAPPING_WEB_CHECK_OK`、`test_localization.cjs` → `LOCALIZATION_VIEW_CHECK_OK`。

## 验收证据

| 项目 | 结果 |
| --- | --- |
| 首页入口 | `#fieldAssistant` 为真实`href=/field`链接，绿色大卡片“现场助手”，包含验图/录制/标点/导出说明；首屏完整可见，点击高度超过46px |
| 手机布局 | Chromium390×844，严格`scrollWidth <= clientWidth`；不是用会随溢出变宽的innerWidth |
| 未登录路径 | 首页点击→`/field`显示原登录表单，不泄露field内容；field health返回401 |
| 登录后路径 | demo登录成功后仍在`/field`并加载现场助手，而不是掉回首页 |
| 已登录路径 | 回到首页再次点击→直接进入现场助手，无重复登录 |
| 原页面 | 原建图start/save/maps控件存在；`/localization`、`/heightmap`返回200并保留原内容 |
| 错误路径 | 已登录访问`/field-not-found`返回404与中文“没有这个页面” |
| 权限及网络 | 测试只有网页登录POST，无建图、切图、录制或现场动作POST，无外部或真机请求 |

截图：[homepage-entry-phone.png](homepage-entry-phone.png)，390×844，已人工查看。它是本地demo；旧遥测故意没有连接机器人，因此图中的“连接中断”不是对真机网络的诊断。

## 测试隔离及复现

使用Code Reviewer和Playwright E2E技能完成权限/回归检视和实际浏览器运行。Python3.12 loopback demo启动命令及Playwright依赖说明见[README.md](README.md)。本测试沿用不安全HTTP测试域名，仅由fixture转至127.0.0.1，不使用外部DNS、Tailscale或机器人。

```bash
python3.12 -B tools/s10_mapping_web/field_demo.py --port 18080 --password field-demo-only
S10_QA_PLAYWRIGHT=/absolute/path/to/node_modules/playwright node tools/s10_mapping_web/qa/test_homepage_entry.cjs
```

本轮reviewer开启的demo及其fake worker、浏览器在测试后关闭，保留测试代码与成功截图。

## 部署清单复查（非真机执行）

- 106导入闭包需要`field_core.py`、`field_worker.py`、`field_robot.py`、`field_preview.py`、`field_recorder.py`、`robot_backend.py`及`field-worker.sh`，配套systemd unit/config；原`heightmap.py`与受限SSH配置保留。指南未漏这些文件。`field_fake.py/field_demo.py`不是生产依赖，不应被误当生产后端启用。
- 102必须同步`server.py`和`field_core.py`，以及`index.html/field.html/field.js`；新server只从core导入`FieldError/ident`，不需要102装ROS。旧定位/高度图资源保留。
- `server.py`和`field_core.py`按Python3.8语法解析通过；这是静态兼容检查，不替代AGX实际`/usr/bin/python3`导入。master需实测目标解释器，不盲目从106版本推断AGX版本。
- 106 worker使用ROS Jazzy对应的Python3.12；wrapper保持`source /opt/ros/jazzy/setup.bash`之后才`set -u`，避免此前已发现的ROS环境未定义变量启动失败。
- 系统unit不依赖用户linger，保留持久数据目录和Unix socket权限；真实启动、权限、当前服务状态及103转发仍归master部署验收，本专项没有更改它们。
