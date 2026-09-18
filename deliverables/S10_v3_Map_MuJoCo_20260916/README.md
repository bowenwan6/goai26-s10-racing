# S10 v3 地图与 MuJoCo 场景包

地图：`0914_fr_v3-20260914-142008` · 整理日期：2026-09-16。

**包含 v3 全场点云；可运行的 mesh/碰撞场景只覆盖 Start＋B＋B 后约 4.88 米短平台，不是全场 mesh。** 原 B 和原始 v3 数据保留；没有重新建图、补洞、重新归因 IMU 或修改真机。

## 最快使用

1. 把 ZIP **完整解压**，不要只拖出一个 XML 或 OBJ。
2. 双击根目录 `index.html` 看图片、地图交互预览和地形浏览视频，无需联网。
3. 查看地图原始数据：`maps/v3/full_cloud.pcd`；查看合并网格：`mujoco/terrain/visual/Start_B_short_combined.ply`（也有 OBJ）。
4. 打开 MuJoCo：使用下面的环境和命令。

## 包内目录

```text
maps/v3/                       全场点云、二维栅格与原优化轨迹
  full_cloud.pcd               原导出点云，3,346,032 点，未改动
  trajectory.csv              1,435 个优化关键帧位姿；不是导航 WP
  occ_grid.pgm / occ_grid.yaml 二维栅格；YAML 已改为包内相对路径
  vendor_reference/           原 YAML/TOML 留档，含厂商机器上的路径
mujoco/terrain/                 三段地形、可视网格、碰撞体与工程验证证据
  scene_contact_v1.xml         推荐的地形入口（当前候选接触参数）
  visual/                     分区及合并 OBJ/PLY
  collision/                  有限凸碰撞体 XML，不能省略
  interactive.html            三段模型可旋转、缩放、切换图层
  media/                      三段总览、细节、截面、未知区、验证视频
mujoco/robot_scene/             同地形＋S10 机器人，便于接入自己的控制器
  scene.xml                   可直接由 MuJoCo 加载，附 start_pose keyframe
  robot/                      机器人 STL 资源
  simulation_route.yaml       15 个仿真测试点，不是现场采集 WP
visuals/                       v3 全场图与离线点云交互预览
scripts/                       只读校验、静态查看器
verification/                  复制来源与包内依赖检查
MANIFEST_SHA256.json            文件清单与 SHA-256
```

此包是**地图/场景交付**，不包含完整厂商 session、逐帧 LiDAR/IMU、ROS 工作区、策略权重或 Docker 镜像。它不是原始采集全量备份；不能仅凭合并点云重新去畸变。

## 在 MuJoCo 打开

推荐 Python 3.12。首次在新电脑建立环境（安装依赖需要网络，完成后使用场景不需要网络）：

```bash
python -m venv .venv
# macOS / Linux
source .venv/bin/activate
# Windows PowerShell 则使用：.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

已实测版本是 MuJoCo **3.13.0**、NumPy **2.5.3**，固定在 requirements.txt 中。其他版本需自行重新验证，不承诺完全相同的接触结果。

可选：自行运行只读校验和碰撞探针测试（本次打包未重新运行仿真）：

```bash
python scripts/check_package.py
```

打开地形查看器：

```bash
# macOS 原生窗口使用 mjpython
mjpython scripts/view_scene.py --scene terrain
# Linux / Windows 可使用
python scripts/view_scene.py --scene terrain
```

看机器人初始放置姿态：将 `--scene terrain` 换成 `--scene robot`。查看器**不进行物理步进、不运行 policy、不控制行走**；显示机器人不等于其能站稳或上楼。`--collision` 显示实际碰撞层，默认显示可视层。退出时关闭窗口即可。

无需 GUI 的加载示例，在包根目录执行：

```python
import mujoco
model = mujoco.MjModel.from_xml_path('mujoco/robot_scene/scene.xml')
data = mujoco.MjData(model)
key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'start_pose')
mujoco.mj_resetDataKeyframe(model, data, key)
mujoco.mj_forward(model, data)
print(model.nq, model.nv, model.nu)  # 23, 22, 16
```

需要 policy 行走时，还要接入 16 个执行器的控制、观测/动作映射、控制权管理和路线跟踪；本包不会自动运行这些内容。

## 导入其他项目时注意

- 坐标沿用 **v3 地图原坐标**，长度米、角度弧度，Z 向上；没有重新居中、缩放或旋转到照片坐标。XYZ 不是 GPS 或经纬度。轨迹 CSV 四元数字段为 `qx,qy,qz,qw`，MuJoCo qpos 为 `w,x,y,z`。
- `full_cloud.pcd` 是地图，不是带物理碰撞的 MuJoCo 场景。二维栅格也不能表达全部楼梯和多层关系。
- 可视层为 `geom group=2`、不碰撞；地形碰撞层为 `group=3`。机器人碰撞为 `group=1`。给地形做射线查询时应选 group 3，避免误选可视层或机器人。
- **不要把整个合并 OBJ 作为一个普通 mesh 碰撞体。** 本场景已将可视网格和有限凸碰撞体分开；整图凸包可能堵住楼梯、未知区和缺口。
- 直接加载场景最稳妥。若合入自己的 MJCF，要保留 `asset`、地形 `worldbody`、接触参数与引用路径，并处理名称冲突；不能只复制 `worldbody` 或只复制 OBJ。
- `scene_contact_v1.xml` 用候选 `solref=".01 1"`；原 B 自身的接触参数不变。`scene_baseline.xml` 是默认参数反例留档，不是推荐入口。
- `simulation_route.yaml` 仅为现有模型内的仿真测试路线，不能替代约 30 个现场 WP 的采集、确认和导航验收。

## 可视化从哪里看

| 文件 | 内容 |
|---|---|
| `visuals/full_map_top.png` | v3 全场点云俯视＋建图轨迹 |
| `visuals/full_map_3d.png` | v3 全场三维形态 |
| `visuals/full_map_interactive.html` | 离线旋转点云预览；显示抽样不改变原 PCD |
| `mujoco/terrain/media/10_mujoco_overview.png` | 三段场景实际 MuJoCo 总览 |
| `mujoco/terrain/media/11_mujoco_Start.png` | Start 细节 |
| `mujoco/terrain/media/12_mujoco_B.png` | B 台阶细节 |
| `mujoco/terrain/media/13_mujoco_Post.png` | B 后短平台 |
| `mujoco/terrain/media/03_source_sections.png` | 源点与拟合面固定截面对照 |
| `mujoco/terrain/media/04_support_masks.png` | 已建模与未知区域 |
| `mujoco/terrain/media/07_scene_tour.mp4` | 24 秒地形镜头浏览，不是机器人通过视频 |
| `mujoco/terrain/media/08_wheel_interfaces_1x.mp4` | 导轨轮接缝测试，不是自主策略测试 |
| `mujoco/terrain/media/09_contact_counterexample_1x.mp4` | 开/关碰撞的探针反例测试 |

## 已检查与未验收

- 局部可视网格共 **8,512 个三角面**；碰撞体共 **3,846 个**，其中原 B 为 32 个，原样保留。
- 包内提供先前接缝、射线、36 个落球探针和导轨轮工程检查。本次仅整理打包，检查复制哈希、包内文件引用和 ZIP 完整性，没有重新运行仿真或独立解压加载测试。
- 这些证明包内实现与几何的内部一致性，**不是现场尺寸、完整障碍覆盖或真机通过性的验收**。
- 原 B 局部不一致仍保留“未确定”；没有用未通过的平面修复替代它。未知空白不代表现实真的有洞；侧边障碍并未完整建模。
- 官方 policy 与后续楼梯策略实验未完成当前整条路线，不能把地图可加载解释为机器狗可以安全通过。本包不把实验失败通过改地形来掩盖。
- 本次仅做离线文件整理，没有连接或控制真机。

## 来源与完整性

本次整理由点云处理技能约束坐标、原始数据保护和网格层次，由 Sim-to-Real 技能区分工程加载检查与现场验收。`verification/source_copies.json` 记录来源与复制哈希；`MANIFEST_SHA256.json` 校验包内文件。原始 YAML 的绝对路径另存，供日后比对。

原 `mujoco/terrain/code_snapshot/` 是历史构建脚本，重建需要原工作区与额外输入，不是独立重建入口；包内已有场景、可视化与 `scripts/` 校验器才是本次便携交付入口。历史 provenance 中的绝对路径只是留档，不是运行依赖。
