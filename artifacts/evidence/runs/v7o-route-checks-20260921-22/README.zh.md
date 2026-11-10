# v7_o 路线上真机前的检查记录（2026-09-21 至 09-22）

[English](README.md)

9 月 21 日到 22 日，v7_o 室外赛道每一版路线从示教生成到装上机器人之前做过的检查，以及重跑这些检查要用的数据。文中时间都是北京时间。

## 先说清楚三件事

1. 路线不会并入地图。v7_o 地图是 x_nav 建好后就固定的。示教录下来的轨迹会被编译成一个单独的路线包（`route_v2.json` 等文件），用的是地图的坐标系，地图本身不改。
2. 这里的仿真是运动学仿真，不是 MuJoCo。[`sim_full_route.py`](../../../../robot/ros1_gateway/tests/nav/sim_full_route.py) 用真实的导航核心跑完整条路线，步态切换按控制器的规则模拟（停稳 1 秒以上才换），感知输入是平地。它检查的是路线形状、航点顺序、步态区、跟线器逻辑和用时，不模拟地形、接触和摔倒。论文里的 MuJoCo 仿真用的是 v3 地图和另一条参考路线，这里的示教路线没有进过 MuJoCo。
3. 碰撞检查是 9 月 21 日 08:48 左右才加上的。在那之前，路线只跑过运动学仿真。碰撞检查一加上，就发现当时 AGX 上的三条旧路线都会压到障碍：第一版拉直的路线有 107 处，直接用原始示教生成的路线有 29 个格子。三条都撤了下来。从那以后，每一版路线都要两道检查都过了才装到 AGX 上。

## 检查流程

写在 [`PIPELINE_AND_PLANNING_ZH.md`](../../../../robot/ros1_gateway/docs/PIPELINE_AND_PLANNING_ZH.md) 的 2.3 节。后来用 [`build_route.py`](../../../../robot/ros1_gateway/tools/build_route.py) 把这几步串起来，哪一步不过就停下，不出路线。

1. [`audit_teach_session.py`](../../../../robot/ros1_gateway/tools/audit_teach_session.py)：看哪几遍示教是完整的、有没有摔倒。
2. [`teach_line.py`](../../../../robot/ros1_gateway/tools/teach_line.py)：把几遍示教拼成一条线并拉直，自己先用车身轮廓扫一遍，修不好就退出。
3. [`teach_to_route.py`](../../../../robot/ros1_gateway/tools/teach_to_route.py)：编译成路线包，写出 `route_v2.json`、`maneuvers.json`、`terrain.json`、`report.json`。
4. [`verify_route_clearance.py`](../../../../robot/ros1_gateway/tools/verify_route_clearance.py)：硬关卡。用 0.9 × 0.5 m 的车身轮廓沿路线扫地图，车身压到示教时没压过的障碍就算违规（HARD），结果写进路线目录的 `clearance.json`。有违规的路线，导航节点拒绝加载。
5. `sim_full_route.py`：运动学全程仿真，必须打印 `SIM_FULL_ROUTE_OK`。

s5 另外在 Mac 上跑了起跳（`test_jump_lane.py`）和进步态区（`test_zone_entry.py`）两个测试，都通过了。

## 每一版的结果

| 版本 | 碰撞检查 | 运动学仿真（平地/台阶速度 m/s） | 被拦下的情况 | 真机 |
|---|---|---|---|---|
| 示教原线 `v7_o-teachline` | 09-21 08:49 到 09:08 修线期间报过 8 次违规；WP06 周围按操作员确认忽略 2.5 m 后，09:07 起通过 | 0.8/0.5：528、505、490、500 s，29/29 | 01:56 到 02:24 仿真失败 8 次。前 4 次停在路线 18 到 24 m 处（WP3 到 WP5 之间），原因是"中心线被挡、此段不许绕行"。修完 02:25 通过 | 未单独记录 |
| s1 | 12:12 通过 | 1.0/0.7：413、404、411 s | 无 | 未单独记录 |
| s2 | 14:21 通过 | 1.0/0.7：445 s；1.4 m/s 下最终 393 到 443 s | 1.4 m/s 时 3 次停在 28/29，位置在路线 268 m 处，状态是绕行。修订版 `20260921c` 又停过一次 24/29 | 未单独记录 |
| s3 | 15:40 通过 | 1.0、1.4、1.67：420、373、356 s；最快一次 297 s | 无 | 09-21 16:53 那一轮 |
| s4 | 21:30 通过 | 1.67/1.67：304、317 s | 20:41 第一次停在 24/29，位置在路线 239 m 处，状态是绕行 | 09-21 23:52 全程 29/29，用时 386 s |
| s5 | 09-22 01:08 到 01:13 连续 3 次违规，修好后 01:15 通过 | 1.67/1.67：317 s | WP21 和 WP22 之间有个细障碍压在线下面，生成器把几段打点轨迹当成一段连起来，漏检了；碰撞检查把它拦了下来 | 没上过真机 |

记录里一共有：运动学仿真失败 13 次，碰撞检查报违规 11 次，`build_route.py` 因此停下 3 次。s1 到 s5 每一版都是两道检查通过之后才装到 AGX 上的。

`clearance.json` 里的 `free_at` 是操作员确认可以忽略的地图障碍范围。示教原线忽略了 WP06 周围 2.5 m，地图里那里有 0.2 到 1.4 m 高的点，多半是标点时旁边站人留下的残影。s1 到 s3 忽略了 WP17 和 WP24 周围 1.5 m。s4 和 s5 没有忽略任何东西。

## 2026-11-10 用当前代码重跑

当时的仿真结果只打印在屏幕上，没有存文件。11 月 10 日用当前代码和当前参数，把 12 个路线包重跑了一遍，共 15 次，全部通过，结果在 `rerun_20261110/`。

| 路线 | 速度 | 用时 |
|---|---|---|
| 示教原线 | 0.8/0.5 | 398 s |
| s1、s2 | 1.0/0.7 | 371 s |
| s3 | 1.0/0.7、1.67/1.67 | 369 s、308 s |
| s4 | 1.0/0.7、1.67/1.67 | 380 s、315 s |
| s5 | 1.0/0.7、1.67/1.67 | 380 s、317 s |
| 各版短路线 | 同上 | 10 到 18 s |

s5 在 1.67 m/s 下是 317 s，和 9 月一样。前几版的数字和当时不同，因为导航核心和参数在 9 月 22 日之前一直在改，这次跑的是最后一版。

碰撞检查也用 s5 重跑过一次，和存档的 `clearance.json` 结果一致。

## 目录内容

```
routes/            12 个路线包，每个都有 route_v2.json、maneuvers.json、terrain.json、
                   report.json（编译报告）、clearance.json 和 clearance.png
teach_lines/       每一版拉直后的示教线：teach_line.json（车身扫掠、门点半径、步态计划、参数）、
                   图，以及 derived_session/（交给 teach_to_route 的输入）
audit/             示教审计结果
raw/session_0921/  s1 到 s5 用到的原始示教轨迹和打点记录
raw/logs_all/      控制事件日志，步态切换从这里取
history/           9 月的检查结果，从开发会话记录里摘出来的
  check_events_20260921-22.csv       每次检查一行：时间、检查类型、路线、速度、用时、到达点数、结果、停下原因
  check_output_lines_20260921-22.md  原始输出行
rerun_20261110/    11 月重跑的输出，summary.csv 是汇总
```

JSON 文件里的路径以 `<s10-real-readiness>/` 开头，指队里笔记本上的现场工作目录。每一版的参数和批准记录在 [`s10_params.yaml`](../../../../robot/ros1_gateway/config/s10_params.yaml) 和 [`s10_params.lock`](../../../../robot/ros1_gateway/config/s10_params.lock)。v7_o 地图有 228 MB，没放进仓库，队里另外分享。

## 怎么复现

在 `robot/ros1_gateway/` 下执行。导航核心在 `src/s10_auto_nav`，所以要设 `PYTHONPATH`。这两条命令 11 月 10 日用 `main` 上的代码试过，s5 的结果和存档一致。

```bash
PKG=../../artifacts/evidence/runs/v7o-route-checks-20260921-22
```

运动学仿真，不需要地图，约 1 分钟：

```bash
PYTHONPATH=../../src/s10_auto_nav python3 tests/nav/sim_full_route.py $PKG/routes/v7_o-s5-full 1.67 1.67
```

碰撞检查。它会改写路线目录里的 `clearance.json`，所以先复制一份再跑。`MAP` 是 v7_o 地图目录：

```bash
cp -R $PKG/routes/v7_o-s5-full /tmp/ && PYTHONPATH=../../src/s10_auto_nav python3 tools/verify_route_clearance.py /tmp/v7_o-s5-full $PKG/raw/session_0921 --map-dir $MAP
```

示教原线和 s1 到 s3 用过 `free_at`，复现时要加同样的 `--free-at` 参数，值见各自的 `clearance.json`。

## 局限

- `history/` 里的结果是从会话记录里摘的，不是当时存下的文件。直接在 AGX 上手动跑的检查不在会话记录里，所以可能有个别结果没被收录。
- 运动学仿真的感知输入是平地，通过只能说明路线和跟线逻辑能走通，不代表真机在台阶上不会摔。
- 9 月 20 日晚到 21 日早上装过的三条旧路线没有过碰撞检查，后来查出会压到障碍，已经撤下。
