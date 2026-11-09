# 视觉-激光融合补盲方案（任务一 + 任务三）

**文档状态**：设计稿 v2（未实现）
**涉及仓库**：
- 视觉侧：`/home/ubuntu/code/RM_Vision_2027`（负责人：视觉组）
- 导航侧：`/home/ubuntu/code/Navigation-2027-Jazzy`（负责人：导航组）
**运行形态**：同一台机器，两个独立进程，通过 ROS 2 Jazzy DDS **双向**通信

**版本要点（v3，2026-11-09 定稿）**：
- **双向通信**：导航 → 视觉提供位姿，视觉 → 导航提供点云
- **局部 BA 只做 M-A**（仅优化 3D 点，不动导航位姿）；M-B 位姿联合优化**不实现**
- **三条建图出口全部实现**：在线 costmap、2D 栅格、先验 PCD
- **三角化只跑左后一台相机（`omni_cam_3`）**，因为小yaw 遮挡在左后
- **建图与比赛是两次独立运行**（互斥），视觉侧需要两套配置
- **出口 C 只做离线 PCD**，不做在线累积节点

***

## 概述

我们发现，在激光雷达SLAM之中，只有简单的点云图数据和处理之后的栅格地图数据，这些数据过于的简单，提供给决策的信息非常的少，所以，我们决定启动四个全向相机的图像数据，做成一个视觉SLAM，提供更多的图像语义信息给我们导航的决策

***

## 目标

我们的激光雷达只有一个，并且安装在机器人的左前方，这会导致小yaw部分完全遮挡激光雷达的部分扫描，导致在栅格地图中，被遮挡的一大片区域都是「不可通过区域」，这对导航是非常不利的，于是，目标一：

**全向相机做到基础的点云提供，为激光雷达补充遮挡信息**

*是否四个相机都启动SLAM？完整率&计算性能*

同时，位了复杂的决策，我们打算提供比赛场面的语义信息，同时，自瞄的yolo提供了敌人，队友的检测（检测装甲板，红色和蓝色），以及基础的距离坐标（PnP结算）于是目标二：

**全向相机以自瞄yolo为基准提供场面语义信息，根据敌人和队友数量信息实现复杂决策**

*场面上的情况怎么样？队友有多少个？敌人有多少个？是强势占点还是孤军奋战？敌人距离我多远？队友距离我又多远？是退到队友身后？还是开启小陀螺并且左右摇晃躲子弹？*

***

## 开源研究

### 目标一，提示词是：*在激光雷达被遮挡时，多相机提供点云信息补充*

大肥鱼说暂无紧密的资料可以提供，提出了一个没有中刊的论文，我看一下，使用的方法是在SLAM的后端优化的部分，如果激光雷达提供的点云约束不足的话，就使用相机的数据来补充约束，这个如果论文口中的所谓「约束」可以不只是在里程计漂移导致的前端结算数据不精准导致「约束」效果差，而是能够扩展到直接遮挡，失效这一块的话（因为我们并不是激光雷达的信息约束失效，而是被遮挡导致无法得到被遮挡的一条通路上的反射点），那还是可以用的，但是该项目并没有提供对应代码，给了

### 目标二，提示词是：*知道某种特定点，想要在特征点聚类特定点周围的信息*

这个倒是有大量的实现，这个问题不大，感觉可以使用自己的yolo弄

***

# 开始干活吧

> 本章之后为**任务一（视觉点云补盲）+ 任务三（视觉辅助建图）**的完整实施方案。
> 任务二（语义信息）只在第 11 章预留接口。

---

## 1. 总体路线

### 1.1 一句话概括

导航侧的定位**已经足够准**，所以视觉不需要做 SLAM —— 它只需要做 **“已知位姿的结构恢复”（Structure from Known Motion）**：拿导航给的位姿，把连续帧的特征点三角化、再用局部 BA 精化，最后把 3D 点送进导航的三条建图链路。

由此产生一个必要设计：**双向通信**。

```
导航 → 视觉：位姿（逐点发布，精度 cm 级）      ← 视觉没有它就无法三角化
视觉 → 导航：3D 点云（三角化 + 局部 BA 后）    ← 导航没有它就无法补盲
```

图像不可能通过 DDS 传输（四路 1080p 压缩后仍是 MB/s 级），所以**三角化必须发生在视觉侧**，这使"导航 → 视觉的位姿"从"可选"变成"必须"。

### 1.2 三条建图出口（全部实现）

| 出口 | 路径 | 延迟 | 影响 | 优先级 | 运行时机 |
|---|---|---|---|---|---|
| **A. 在线 costmap** | 视觉点 → `terrain_map` → `IntensityObstacleLayer` | <100 ms | 实时避障，盲区可通行 | 高 | **比赛运行** |
| **B. 2D 栅格** | 视觉点 → `terrain_map` → `pointcloud_to_laserscan` → `slam_toolbox` | 1~2 s | 建图模式下的 `.pgm/.yaml` | 中 | **单独建图** |
| **C. 先验 PCD** | 视觉点 → 离线融合 → `pcd/*.pcd` → `small_gicp_relocalization` | 离线 | 重定位质量 | 高 | **离线单独跑** |

**关键实现简化**：出口 A 和 B **共用同一条链路**。`terrain_analysis` 发布的 `terrain_map` 同时被 costmap 和 `pointcloud_to_laserscan` 消费（见 `slam_launch.py:111-124`），所以只要把视觉点并进 `terrain_map`，A 和 B 自动同时得到支持，**不需要写两套代码**。

出口 C 是独立的第二条代码路径，**只做离线**（见 §7 N5），不引入在线累积节点。

> **运行模式区分（重要）**：B 和 C 都属于"单独建图运行"，与比赛运行**互斥**。因此视觉侧需要两套配置：
> - **比赛配置**：只开 C3（实时障碍点云）；三角化/BA 可关（省 CPU）
> - **建图配置**：开三角化 + BA + C4，与 B/C 出口配合
>
> 详见 §6 V8 的两份 yaml 片段。

### 1.3 数据流全图

```
┌───────────────────── RM_Vision_2027 ─────────────────────┐
│  5×Camera ──> 5×YOLO ──> PnP / 特征点                      │
│                              │                            │
│              ┌───────────────┴──────────────┐             │
│              ▼                              ▼             │
│      PnP 3D 点（已有）            特征点跟踪 + 三角化        │
│              │                     + 局部 BA 精化          │
│              └───────────────┬──────────────┘             │
│                              ▼                            │
│                  ObstacleCloudPublisher                   │
│         （独立线程，10Hz / 1Hz 两档，frame=gimbal_yaw）     │
└──────────────────────────────┬────────────────────────────┘
                               │  sensor_msgs/PointCloud2
                               ▼
┌──────────────────── Navigation-2027-Jazzy ────────────────┐
│  terrain_analysis                                          │
│    ├─ 订阅 vision/obstacle_cloud                           │
│    ├─ 静态外参 T_lidar←gimbal（一次查询）                    │
│    ├─ 变换到 odom、体素去重、量程/超时过滤                    │
│    └─ 并入 terrain_cloud_elev_                             │
│                     │                                      │
│          ┌──────────┴──────────┐                           │
│          ▼                     ▼                           │
│   costmap(mark+raytrace)  pointcloud_to_laserscan          │
│   出口 A                       │                           │
│                                ▼                           │
│                          slam_toolbox → /map               │
│                          出口 B                            │
└────────────────────────────────────────────────────────────┘

离线：rosbag ──> Point-LIO 全局轨迹 ──> 三角化+局部BA ──> PCD（出口 C）
         ▲                                                    │
         └──────────── 导航 → 视觉：aft_mapped_to_init ────────┘
```

### 1.4 分级实施

| 阶段 | 内容 | 依赖 | 工期 | 生效模式 |
|---|---|---|---|---|
| **L0** | PnP 装甲板点 → 出口 A/B | 无 | 1.5 周 | 比赛 |
| **L1** | 左后相机特征跟踪 + 三角化（固定位姿） | L0 的发布链路 | 1 周 | 建图 |
| **L2** | 局部 BA 精化（**仅 M-A 模式**） | L1 | 0.7 周 | 建图 |
| **L3** | 地面平面反投影（纯旋转兜底） | L1 | 3 天 | 建图 |
| **L4** | 离线 PCD 融合（出口 C） | L2 + rosbag | 1 周 | 离线 |
| ~~L5~~ | ~~深度网络半稠密~~ | — | — | **暂不排期** |

> L1~L4 只在"单独建图运行"时启用（见 §1.2 运行模式区分）。比赛运行时仅 L0 生效，`visual_triangulation.enable: false`，把 CPU 留给自瞄。

### 1.5 验收标准

| 编号 | 验收项 | 判据 | 归属阶段 |
|---|---|---|---|
| A1 | 双向话题通 | 两侧都能收到对方数据，帧率 ±10% | L0 |
| A2 | 空间对齐 | 静止时视觉点与激光同一障碍偏差 **< 0.15 m** | L0 |
| A3 | 旋转不错位 | 3 rad/s 自旋时漂移 **< 0.25 m** | L0 |
| A4 | 盲区可通行 | 遮挡雷达一侧，该方向 costmap 由 unknown 变为合理 free/occupied | L0 |
| A5 | 三角化有效 | 有平移时，三角化点重投影误差中位数 **< 2 px** | L1 |
| A6 | BA 有增益 | 局部 BA 后重投影误差下降 **≥ 40%**，深度抖动下降 **≥ 30%** | L2 |
| A7 | 点云质量 | 静态点 PCD 与激光 PCD 重叠区平均偏差 **< 0.2 m** | L4 |
| A8 | 重定位提升 | 加载带视觉点的 PCD 后，`small_gicp` 首次收敛率提升（盲区方向） | L4 |
| A9 | 无回归 | 三个出口全部关闭时，行为与当前版本完全一致 | 全部 |
| A10 | 实时性 | `terrain_analysis` 单帧增量 **< 3 ms**；视觉主链路 FPS 下降 **< 5%** | 全部 |

---

## 2. 现状事实（源码核对）

### 2.1 导航侧

```
livox_ros_driver2 → point_lio → loam_interface → terrain_analysis → costmap / slam_toolbox
```

| 事实 | 证据 |
|---|---|
| `lidar_odometry`：`frame_id=odom`，`child=front_mid360`，即**雷达在 odom 中的位姿** | `loam_interface/src/loam_interface.cpp:86-97` |
| `aft_mapped_to_init`：`frame_id=camera_init`，`child=body`，`publish_odometry_without_downsample: True` 时 **stamp = 该点时刻**（逐点发布） | `point_lio/src/laserMapping.cpp:269-275` |
| `terrain_map` 发布在 **`front_mid360` 系** | `terrain_analysis/src/terrain_analysis.cpp:584-598` |
| `terrain_cloud_elev_` 是 **`odom` 系**，每帧 `clear()` 后重建 | `terrain_analysis.cpp:480`、`:559-568` |
| `terrain_map` 的 `intensity` = **离地高度**（0~0.5） | `terrain_analysis.cpp:511`、`:564` |
| costmap 用 `IntensityObstacleLayer`，**intensity 必须 ∈ (0.1, 2.0)** | `config/reality/nav2_params.yaml:348-353` |
| costmap 高度过滤 0~2 m；射程 global `10/12`、local `5/8` | `nav2_params.yaml:405-409`、`:362-366` |
| `track_unknown_space: true` → 无观测区按不可通行 | `nav2_params.yaml:389` |
| 建图模式 `map→odom` 是**静态恒等**；`use_scan_matching: false`、`do_loop_closing: false` | `slam_launch.py:157-180`、`nav2_params.yaml:229,238` |
| 雷达装在 **`gimbal_yaw`**，偏移 `(-0.03569, 0.09268, 0.3627)`，俯仰 -30° | `pb2025_robot_description/resource/xmacro/uic2025_sentry_robot.sdf.xmacro:20` |
| `terrain_analysis` 已具备 `tf2/tf2_ros/tf2_geometry_msgs` 依赖 | `terrain_analysis/package.xml:20-22` |

### 2.2 视觉侧

| 事实 | 证据 |
|---|---|
| 实车入口 `sentry_multithread`，**唯一**创建 ROS2 节点的目标 | `CMakeLists.txt:83-90`、`src/sentry_multithread.cpp:67-73` |
| 非 ament 包；ROS2 源码编进 `io` 静态库 | `io/CMakeLists.txt:48-85` |
| `sensor_msgs` 已就绪；**`nav_msgs` 未引入**（双向通信需要补） | `io/CMakeLists.txt:48-56,65-80` |
| 全向 PnP 已有 `xyz_in_gimbal`（大yaw 系）+ `distance`，但被丢弃 | `tasks/omniperception/omni_pnp.hpp:139-155`、`decider.cpp:150-166` |
| `CameraResult` 无 3D 字段 | `tasks/auto_aim/multithread/yolo_pipeline.hpp:29-40` |
| 主相机 `Armor::xyz_in_gimbal` 在**小yaw 系**，不可与全向混用 | `tasks/auto_aim/armor.hpp:106-107`、`omni_pnp.hpp:119-130` |
| 当前只订阅 `/red_standard_robot1/cmd_vel` | `io/ros2/subscribe2nav.cpp:29-33` |
| 内部 tf 树 `lookup(from,to)` 返回 `T_to_from`；不发布 ROS TF | `io/tf/tf_tree.hpp:84` |
| `io::Camera` 通过 `sursign` 后缀区分 5 路相机配置 | `io/camera.cpp:20` |
| 配置校验严格，缺键可能直接退出 | `configs/soldier.yaml:2` 注释 |

### 2.3 两侧物理对应关系（方案成立的基础）

```
视觉 tf 树                        导航 tf 树
world                             map → odom → base_footprint → chassis
├── yaw_axis      ←── 同一块板 ──→  gimbal_yaw
│   ├── omni_cam_1..4                └── front_mid360（雷达，静态）
└── small_yaw                        └── gimbal_pitch
    └── main_cam
```

**结论：四台全向相机与雷达刚性固连在大yaw 板上，`T_lidar←omni_cam_N` 是常量。**

因此：
- 不需要对齐视觉 `world` 与导航 `map/odom`（两套独立漂移的世界系根本无法静态桥接）；
- 整条外参链只有 **Point-LIO 输出的 `T_camera_init←body` 一个动态量**；
- 主相机在小yaw 板，外参随 pitch 变化 → **优先使用四台全向相机**。

---

## 3. 三条红线

| 红线 | 说明 |
|---|---|
| **一、动态点绝不进 IVox 地图** | 装甲板点是移动目标。一旦 `ivox_->AddPoints()`，下一帧配准把它当静态基准，位姿被拉偏，目标离开后留下永久幽灵点。实时出口只走 `terrain_map`。 |
| **二、不在 YOLO 推理线程里发布** | 推理线程已绑核且周期敏感（`yolo_pipeline.hpp:82-89`）。点云序列化 + DDS 发布必须独立线程，主循环只投递。 |
| **三、不加第二套 TF 发布者** | `gimbal_yaw`、`front_mid360` 由 `robot_state_publisher` 统一发布。视觉侧只发点云 + `frame_id`，不发 TF。 |

---

## 4. 几何与算法基础

### 4.1 外参链（全静态）

```
T_camera_init←cam = T_camera_init←body · T_body←lidar · T_lidar←cam
                    └─ 逐点发布 ─┘  └──── 静态，标定一次 ────┘
```

- `T_body←lidar`：Point-LIO 的 `extrinsic_T/extrinsic_R`（`nav2_params.yaml:72-75`）
- `T_lidar←cam`：`lookupTransform(front_mid360, gimbal_yaw) · T_gimbal←cam`，全部静态

**结论**：视觉侧只需缓存 `T_lidar←omni_cam_N` 一次，之后每帧只做两次矩阵乘法。

### 4.2 位姿来源与插值

| 项 | 说明 |
|---|---|
| 话题 | `/red_standard_robot1/aft_mapped_to_init`（`nav_msgs/Odometry`） |
| 帧关系 | `camera_init` → `body` |
| 频率 | 逐点发布（`publish_odometry_without_downsample: True`），远高于图像帧率 |
| 插值 | 维护 500 ms 环形缓冲，按图像时间戳线性插值 SE(3)（四元数 slerp + 平移 lerp） |
| 缺失处理 | 超过 200 ms 找不到可用位姿 → 该帧不参与三角化，计入 `pose_miss` 计数 |

> Point-LIO **不填** `odometry.pose.covariance`（`set_posestamp` 只写 pose，`laserMapping.cpp:276`），所以位姿先验只能用常数或按运动速度的启发式，不能用协方差矩阵。

### 4.3 三角化（固定位姿）

对滑动窗口内对同一特征点的 \(N\) 次观测 \((R_i, t_i, u_i)\)：

**Step 1 — 归一化坐标**

\[
\mathbf{x}_i = K^{-1} \begin{bmatrix} u_i \\ v_i \\ 1 \end{bmatrix}, \quad \text{去畸变后}
\]

**Step 2 — DLT 求初值**

\[
\begin{bmatrix} [\mathbf{x}_i]_\times R_i & [\mathbf{x}_i]_\times t_i \end{bmatrix}
\begin{bmatrix} X \\ 1 \end{bmatrix} = 0
\]

堆叠 \(2N \times 4\) 矩阵，SVD 取最小奇异向量 → \(X_0\)。

**Step 3 — 重投影误差最小化（GN/LM，3~5 次迭代）**

\[
r_i = \pi\!\left(K(R_i X + t_i)\right) - u_i, \quad
J_i = \frac{\partial \pi}{\partial p}\bigg|_{p=R_iX+t_i} \!\!\cdot K R_i
\]

\[
H = \sum_i J_i^\top J_i,\quad b = \sum_i J_i^\top r_i,\quad
\Delta X = -H^{-1} b \ (\text{LM 加阻尼})
\]

**Step 4 — 质量判据**

| 判据 | 阈值 | 作用 |
|---|---|---|
| 视差角 | > 0.5° | 无基线时直接丢弃（纯旋转退化） |
| 三角化后 \(z\) | 0.4 ~ 12 m | 距离过大深度不可信 |
| 最大重投影误差 | < 3 px | 剔除误匹配 |
| 条件数 \(\kappa(H)\) | < 1e5 | 剔除病态解 |

### 4.4 局部 BA（滑动窗口，本期 M-A：只优化点）

> **这是 v2 新增的核心部分。** 目的是"让视觉眼中的点更准"，不是替代导航定位。

**优化变量**

\[
\theta = \big[\xi_1 \dots \xi_N,\ X_1 \dots X_M\big]
\]

- \(\xi_i\)：窗口内 \(N\) 个相机位姿（6DoF）
- \(X_j\)：\(M\) 个 3D 点

**代价函数**

\[
E = \sum_{(i,j)\in\mathcal{O}} \rho\!\left(\left\| \pi(K(R_i X_j + t_i)) - u_{ij} \right\|^2_{\Sigma_{px}}\right)
  + \sum_{i=1}^{N} \left\| \log\!\left(\xi_i^{-1} \xi_i^{\text{nav}}\right) \right\|^2_{\Sigma_{pose}}
\]

- 第一项：重投影残差（Huber 核 \(\rho\)，抑制误匹配）
- 第二项：**位姿先验**（来自导航 odom，\(\Sigma_{pose}\) 取 \(0.05\text{ m} / 0.02\text{ rad}\)，强约束）

**两种运行模式（本期只实现 M-A）**

| 模式 | 位姿 | 求解规模 | 状态 |
|---|---|---|---|
| **M-A：仅优化点** | 固定为导航位姿 | 每点独立 3×3 系统 | ✅ **唯一实现** |
| M-B：点 + 位姿联合 | 带强先验放开 | Schur 补消元 | ❌ 本期不实现，仅保留设计（见下） |

**为什么选 M-A**：
1. 位姿来自 Point-LIO，精度 cm 级，已经比视觉三角化准；
2. 若放开位姿，优化结果会与导航位姿不自洽 —— 输出的点用哪套位姿表达？用优化后的就与导航地图错位，用导航的就等于没优化。**M-A 从根上避免了这个矛盾**；
3. 实现量只有 M-B 的 1/5（约 150 行 vs 500 行），且计算量是 \(1/N\)；
4. 本期三角化只跑**一台相机**（左后），观测数少，联合优化位姿的可观性更差，M-B 收益更低。

> M-A 不平滑位姿，因此**不会污染导航定位**。它的作用只是"用同一个 3D 点在多帧上的观测互相约束"，把单次三角化的随机误差压下来。

**M-B 设计留存（本期不实现）**

若将来要放开位姿（例如外参存在系统性偏差），形式是：

\[
\begin{bmatrix} H_{cc} & H_{cp} \\ H_{pc} & H_{pp} \end{bmatrix}
\begin{bmatrix} \delta\xi \\ \delta X \end{bmatrix} =
\begin{bmatrix} b_c \\ b_p \end{bmatrix}
\Rightarrow
\begin{cases}
\delta X = H_{pp}^{-1}(b_p - H_{pc}\delta\xi) \\
\left(H_{cc} - H_{cp}H_{pp}^{-1}H_{pc}\right)\delta\xi = b_c - H_{cp}H_{pp}^{-1}b_p
\end{cases}
\]

每个点只被少数帧观测 → \(H_{pp}\) 块对角，求逆廉价。**本期不写这部分代码**，`mode` 参数保留但只接受 `points_only`。

**规模估算（M-A）**

| 项 | 值 |
|---|---|
| 窗口帧数 \(N\) | 5~10 |
| 每帧特征点 | 100~200（单相机） |
| 每点观测数 | 3~10 |
| 求解规模 | **每点独立 3×3**，总矩阵大小与点数无关 |
| 实际耗时（aarch64 单核） | **< 2 ms**（500 点） |
| 依赖 | 仅 Eigen |

> 不引入 Ceres/g2o：aarch64 上 Ceres 需要 abseil/glog/suitesparse，交叉编译成本远超收益。M-A 的 3×3 系统手写 LM 约 150 行。

**LM 主循环（伪代码）**

```cpp
// 对每个 track（至少 3 次观测）：
//   1) DLT 求初值 X0
//   2) for iter in 0..8:
//        线性化所有观测 → H(3x3), b(3x1)   [Huber 3px]
//        LM 阻尼：H += lam * diag(H)
//        dX = -H^{-1} b
//        X += dX；若代价下降 < 1e-4 则 break
//   3) 输出 p_world = R_world<-cam * X + t_world<-cam
```

**验收（A6）**：BA 后重投影误差中位数下降 ≥ 40%；同一静止点在相邻两个窗口的 3D 位置抖动下降 ≥ 30%。

**启用开关**

```yaml
visual_ba:
  enable: true
  mode: "points_only"        # 本期只支持 points_only
  window_size: 8             # 帧
  max_iterations: 8
  huber_threshold_px: 3.0
  min_observations: 3        # 少于该观测数的点不参与优化
```

> ⚠ 局部 BA 只优化"点"，**不修改导航位姿、不回写 odom**。输出点云始终在导航位姿定义的 `camera_init` / `gimbal_yaw` 系下表达。

### 4.5 地面平面反投影（纯旋转兜底）

机器人原地小陀螺时基线≈0，三角化必然失效。此时用单帧 + 位姿求地面点：

\[
\lambda \mathbf{d} = R^\top\!\left(\begin{bmatrix}X\\Y\\0\end{bmatrix} - t\right)
\quad\Rightarrow\quad
\lambda = \frac{-t_z}{d_z}
\]

其中 \(\mathbf{d}=K^{-1}[u,v,1]^\top\)，\(t=R_{world\leftarrow cam}\cdot 0 + t_{world\leftarrow cam}\)。

**适用**：只对"贴地物体"有效（地面、场地边线、矮挡板）。
**价值**：纯旋转时唯一能产出深度的方式，且**不需要基线**。
**限制**：像素射线必须与地面相交（即 \(d_z<0\)，看向下方）；射线朝上时无效。

### 4.6 退化检测汇总

| 检测项 | 阈值 | 触发动作 |
|---|---|---|
| 平移基线 \(\|t_i - t_j\|\) | < 0.15 m | 跳过三角化，切地面平面法 |
| 视差角 | < 0.5° | 丢弃该点 |
| 角速度 | > 3 rad/s | 降低窗口长度、提高重投影阈值 |
| 位姿缺失 | > 200 ms | 丢弃整帧 |
| 重投影误差 | > 3 px | 剔除该点 |
| 观测次数 | < 3 | 不进入 BA，仅保留原始三角化结果 |
| 多帧一致性 | 3 个窗口内位置差 > 0.2 m | 判为动态点，丢弃（见 §4.7） |

### 4.7 多帧一致性（动态点过滤）

```
同一个 3D 点在连续 K 个滑动窗口里的估计位置
  → 若最大两两距离 > 0.2 m，判为动态（落在敌人/队友身上）
  → 连续 3 次判为动态则永久丢弃该轨迹
```

**为什么必须做**：PCD 一旦混入动态点，`small_gicp` 重定位会被拉偏；costmap 一旦出现假障碍，规划会被堵死。

---

## 5. 通信规范（双向，双方签字后冻结）

### 5.1 总览

| 编号 | 方向 | 话题 | 类型 | 频率 | QoS | 用途 |
|---|---|---|---|---|---|---|
| C1 | 导航 → 视觉 | `/red_standard_robot1/aft_mapped_to_init` | `nav_msgs/Odometry` | 逐点（≫100 Hz） | BEST_EFFORT, KEEP_LAST 10 | 三角化 / BA 位姿 |
| C2 | 导航 → 视觉 | `/red_standard_robot1/tf`、`tf_static` | `tf2_msgs/TFMessage` | 动态/静态 | 默认 | 可选，外参校验 |
| C3 | 视觉 → 导航 | `vision/obstacle_cloud` | `sensor_msgs/PointCloud2` | 10 Hz | BEST_EFFORT, KEEP_LAST 1 | 出口 A/B（实时） |
| C4 | 视觉 → 导航 | `vision/map_points` | `sensor_msgs/PointCloud2` | 1 Hz | RELIABLE, KEEP_LAST 5 | 出口 C（**仅建图运行**，与比赛互斥） |
| C5 | 视觉 → 导航 | `vision/status` | `std_msgs/String`(JSON) | 1 Hz | RELIABLE, KEEP_LAST 1 | 诊断/心跳 |
| C6 | 视觉 → 导航 | `/red_standard_robot1/serial/gimbal_joint_state` | `sensor_msgs/JointState` | 100 Hz | 现有 | **已有**，不动 |

> 除 C1/C2 外，视觉侧话题名全部**带完整前缀**（照抄 `publish2nav.cpp:19-20` 的现有风格），因为视觉侧的节点没有设 namespace，只靠 topic 前缀区分。

### 5.2 C1 导航 → 视觉：位姿（新增，必需）

| 项 | 值 |
|---|---|
| 话题 | `/red_standard_robot1/aft_mapped_to_init` |
| 类型 | `nav_msgs/msg/Odometry` |
| `header.frame_id` | `camera_init` |
| `child_frame_id` | `body`（= IMU body，即 Point-LIO 的机体系） |
| `header.stamp` | 该状态对应的激光点时刻（逐点） |
| `pose.pose` | `T_camera_init←body` |
| `pose.covariance` | **全 0（`set_posestamp` 只写 pose，`laserMapping.cpp:233-263`，不要依赖）** |
| `twist` | **恒为 0**（Point-LIO 不填）。需要速度时用 `sensor_scan_generation` 发布的 `/odometry`（差分速度，接口未验收） |

**消费者侧要求**
- 维护 500 ms 环形缓冲；
- 按图像 `stamp` 线性插值（平移 lerp、四元数 slerp）；
- 找不到插值区间时取最近端点，并在 `status` 里上报 `pose_miss`。

**发布者侧要求（导航）**
- 不改 `publish_odometry_without_downsample`（保持 True）；
- 若实测频率过高导致 DDS 压力，改为 `False`（降为 20 Hz）需**双方确认**，因为插值精度会下降。

### 5.3 C3 视觉 → 导航：实时障碍点云（出口 A/B，**比赛运行**）

| 项 | 值 |
|---|---|
| 话题 | `/red_standard_robot1/vision/obstacle_cloud` |
| 类型 | `sensor_msgs/msg/PointCloud2` |
| `header.frame_id` | `gimbal_yaw` |
| `header.stamp` | **图像采集时刻**（不是发布时刻） |
| QoS | `SensorDataQoS()`（BEST_EFFORT, KEEP_LAST 1） |
| 频率 | 10 Hz（可配） |
| 点数上限 | 512 点/帧 |
| 字段 | `x, y, z, intensity`（4 × FLOAT32，`point_step=16`） |
| `intensity` | ∈ (0.1, 2.0)，统一 **1.0**（导航侧会覆盖） |
| 内容 | **只放距机器人 ≤ 5 m 的近距离点**（深度精度优先） |
| 空帧 | 无目标时发 `width=0` 的空点云，**不要停发** |

**订阅侧（导航）**：相对名 `vision/obstacle_cloud`，随 `/red_standard_robot1` 命名空间解析。

### 5.4 C4 视觉 → 导航：建图点云（出口 C，**仅建图运行**）

> **与比赛运行互斥**。比赛时不发这条话题（`publish_hz_map: 0` 或 `obstacle_cloud.enable: false` 的比赛配置）。
> 建图运行时导航侧只需要 `slam_launch.py`、不需要 Nav2 控制器，两边负载不会叠加。

| 项 | 值 |
|---|---|
| 话题 | `/red_standard_robot1/vision/map_points` |
| 类型 | `sensor_msgs/msg/PointCloud2` |
| `header.frame_id` | **`camera_init`**（世界系，便于直接拼 PCD） |
| `header.stamp` | 该批点中最新观测的时刻 |
| QoS | RELIABLE, KEEP_LAST 5 |
| 频率 | 1 Hz（攒 1 秒发一次） |
| 点数上限 | 20000 点/帧 |
| 字段 | 见下表 |
| 内容 | **允许远距离（≤ 12 m）、允许延迟**，带质量字段供离线筛选 |

**扩展字段（为离线筛选服务）**

| 字段 | 类型 | 含义 |
|---|---|---|
| `x, y, z` | float32 | `camera_init` 系，m |
| `intensity` | float32 | 固定 1.0（占位，兼容下游） |
| `normal_x` | float32 | 1/重投影误差均方根（越大越好） |
| `normal_y` | float32 | 观测次数 \(N\) |
| `normal_z` | float32 | 三角化视差角（度） |
| `curvature` | float32 | 「一致性得分」= 1/(1+多窗口位置方差) |

> 用 `PointXYZINormal` 布局承载质量信息，避免自定义 msg 带来的跨仓库类型耦合。离线工具按 `normal_x/normal_z` 阈值筛点，可直接剔除低质量与动态点。

**C4 的实现方式：只有离线一条**（已定，不做在线累积节点）

```
视觉侧建图程序
  → 累积 + 筛选（质量字段阈值）
  → 写本地 PCD 文件（如 RM_Vision_2027/output/vision_map.pcd）
  → 人工拷到 ros_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/
  → 与激光 PCD 合并（离线脚本，见 §7 N5）
```

C4 这条 ROS 话题只在"建图运行时需要在线看效果"时才启用；正式产出走文件，避免依赖网络与进程存活。

### 5.5 C5 视觉 → 导航：状态（诊断）

```json
{
  "cameras": 5,
  "frames_ok": 47,
  "pose_miss": 0,
  "features_total": 1840,
  "triangulated": 612,
  "ba_accepted": 498,
  "dynamic_rejected": 37,
  "reproj_px_median": 1.24,
  "publish_hz": 9.8,
  "stamp_lag_ms": 41,
  "degraded": false,
  "degraded_reason": ""
}
```

导航侧用 `ros2 topic echo` 就能判断"是没检测到、还是没发出来、还是被门限挡了"。

### 5.6 时间同步

| 环节 | 做法 |
|---|---|
| 相机取图时刻 | `io::Camera::read()` 输出的 `steady_clock::time_point`（`io/camera.hpp:33`） |
| `steady → ROS time` | 启动时标定一次 `offset = ros_now - steady_now`，之后 `stamp = steady_t + offset`；**禁止每帧重取 `now()`** |
| 标定残差监测 | 每 100 帧比较 `stamp` 与 `ros_now - 处理耗时`，偏差 > 20 ms 则告警 |
| 导航侧 | 用 `header.stamp` 做插值 / 超时判断；**不使用"收到时刻"代替** |
| 时钟域 | 两侧 `use_sim_time=false`，同一台机器 → 同一系统时钟 |

### 5.7 带宽与发现

| 话题 | 单帧大小 | 带宽 |
|---|---|---|
| C1 位姿 | ~200 B | 逐点发布，**实测确认**（可能 1~20 MB/s，见待确认 Q4） |
| C3 障碍点云 | 512×16 ≈ 8 KB | 80 KB/s |
| C4 建图点云 | 20000×48 ≈ 960 KB | 1 MB/s（**仅建图运行**，与比赛互斥） |
| C5 状态 | ~400 B | 0.4 KB/s |

**发现**：两侧 `ROS_DOMAIN_ID` 必须一致；RMW 实现一致（建议统一 `rmw_fastrtps_cpp`，与 Jazzy 默认一致）。同机不代表自动互通。

### 5.8 故障与降级

| 场景 | 视觉侧行为 | 导航侧行为 |
|---|---|---|
| C1 位姿中断 | 200 ms 后停止三角化，只发 PnP 点（C3），`status.degraded=true` | `terrain_map` 中的视觉点因超时被丢弃，行为回退到当前版本 |
| C3 中断 | — | `vision_timeout`（0.5 s）后丢弃，`vision_recv_count_` 不再增长 |
| C5 中断 | — | 仅失去诊断能力，无功能影响 |
| 视觉进程崩溃 | — | A 完全无影响（`terrain_map` 少几个点）；这是设计目标 |
| 导航进程崩溃 | 视觉仍能发 PnP 点，`degraded=true` | — |
| 时钟跳变 | 重新标定 `offset`，丢弃 1 s 内的点 | 超时判断天然免疫 |

**回滚开关（独立生效）**

```yaml
# 导航侧
terrain_analysis.ros__parameters.vision_cloud_enable: false
# 视觉侧
obstacle_cloud.enable: false
visual_triangulation.enable: false
```

### 5.9 启动顺序

```
1. 导航侧先起（提供 robot_state_publisher 的 TF + Point-LIO 的 odom）
2. 视觉侧起，等待 C1 超过 2 s 才开始三角化（超时则只发 PnP 点）
3. 联调时用 ros2 topic hz 双向确认
```

> 顺序不是硬依赖：视觉侧不等待也能发 PnP 点，只是三角化通道会处于 `degraded`。

---

## 6. 视觉侧任务

### V1 — 透传 PnP 3D 点（L0）

**现状**：`OmniPnPResult.xyz_in_gimbal` 已算出，`Decider::delta_angle()` 只取角度丢弃 xyz（`decider.cpp:150-166`）。

**改动**

1. `tasks/auto_aim/multithread/yolo_pipeline.hpp:29-40` 增加：

```cpp
struct ObstaclePoint {
  Eigen::Vector3d p_gimbal;   // 大yaw 系 (m)，x前 y左 z上
  int camera_id = -1;
  float confidence = 0.f;
};

struct CameraResult {
  // ... 现有字段保持不变 ...
  std::vector<ObstaclePoint> obstacles;   // 新增
};
```

2. `tasks/omniperception/decider.hpp` 增加只读访问器（不改现有调用点）：

```cpp
const OmniPnPResult & last_pnp() const { return last_pnp_; }
// 内部在 pnp.success 分支保存 last_pnp_ = pnp;
```

3. `tasks/omniperception/omni_jobs.cpp`（`take_detection_results` 内）填充 `obstacles`。

4. 主相机的点需先转到大yaw 系（`Armor::xyz_in_gimbal` 在小yaw 系）：

```cpp
io::tf::Transform T;   // T = T_yaw_axis <- small_yaw
if (tf_tree.lookup(io::tf::Tree::SMALL_YAW, io::tf::Tree::YAW_AXIS, T)) {
  p_gimbal = T.R * armor.xyz_in_gimbal + T.t;
}
```

### V2 — 新增 `nav_msgs` 依赖（双向通信前置）

`io/CMakeLists.txt` 当前**没有** `nav_msgs`，必须补：

```cmake
# 第 48-53 行区域追加
find_package(nav_msgs QUIET)

# 第 56 行的 guard 追加条件
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT geometry_msgs_FOUND
   OR NOT sensor_msgs_FOUND OR NOT nav_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND)

# 第 65-72 行 target_include_directories 追加
      ${nav_msgs_INCLUDE_DIRS}

# 第 73-80 行 target_link_libraries 追加
      ${nav_msgs_LIBRARIES}
```

> 漏改任一处都会编译失败或链接失败，这是最容易出问题的一步。

### V3 — 新增位姿订阅 `io/ros2/nav_pose_buffer`

**文件**：`io/ros2/nav_pose_buffer.hpp` / `.cpp`（加入 `io/CMakeLists.txt:62` 的 `target_sources`）

```cpp
class NavPoseBuffer
{
public:
  NavPoseBuffer(rclcpp::Node::SharedPtr node, const std::string & topic);

  // 按 ROS 时间戳插值，返回 T_camera_init<-body
  bool lookup(rclcpp::Time stamp, Eigen::Matrix3d & R, Eigen::Vector3d & t) const;

  bool healthy(rclcpp::Time now, double timeout = 0.2) const;
  size_t miss_count() const { return misses_; }

private:
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);   // 入环形缓冲

  mutable std::mutex mtx_;
  std::deque<std::pair<rclcpp::Time, Eigen::Isometry3d>> buf_;  // 500 ms
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::atomic<size_t> misses_{0};
};
```

**插值要点**：`buf_` 里存 `Isometry3d`，查找时定位相邻两个样本，平移 lerp、旋转 slerp；超出缓冲返回 false。

### V4 — 点云发布模块 `io/ros2/publish2nav_pointcloud`

**文件**：`io/ros2/publish2nav_pointcloud.hpp` / `.cpp`

- 独立工作线程 + 最新帧槽位（仿 `tasks/auto_aim/debug_stream.cpp:523-539`、`:608`）
- `submit()` 只投递不阻塞，已存在 `pending_` 则替换（丢旧帧）
- 手工填 `PointCloud2`（不引 PCL）：

```cpp
msg.height = 1; msg.width = n; msg.point_step = 16; msg.row_step = 16 * n;
// fields: x(0) y(4) z(8) intensity(12)，全部 FLOAT32
msg.is_bigendian = false; msg.is_dense = true;
msg.data.resize(msg.row_step);
float * d = reinterpret_cast<float *>(msg.data.data());
for (size_t i = 0; i < n; ++i) { d[4*i]=x; d[4*i+1]=y; d[4*i+2]=z; d[4*i+3]=1.0f; }
```

- C4 用 `PointXYZINormal` 布局（48 B/点），字段布局见 §5.4

### V5 — 特征点跟踪（L1）

**文件**：新增 `tasks/visual_depth/feature_tracker.{hpp,cpp}`

```cpp
struct FeatureObs {
  int track_id;
  int camera_id;
  rclcpp::Time stamp;
  cv::Point2f px;
};

class FeatureTracker
{
public:
  void push(const cv::Mat & gray, int camera_id, rclcpp::Time stamp);
  // 输出：跨帧的轨迹（track_id -> 多次观测）
  const std::vector<std::vector<FeatureObs>> & tracks() const;
private:
  // goodFeaturesToTrack + calcOpticalFlowPyrLK + 前后向校验
  // ROI：图像下半部（地平线以下），减少 60% 特征点
  // 上限：300 点/帧/相机
};
```

**参数建议**：`max_corners=300`、`quality=0.01`、`min_distance=8`、`winSize=21`、金字塔 3 层、前后向误差 < 1 px。

**相机选择：只跑左后一台（`omni_cam_3`）**

**依据**：小yaw 的遮挡出现在**左后方**，所以只需要覆盖该方向的相机。

配置中的方位映射（`configs/sentry_example.yaml:289-304` 的 `tf.frames`）：

| frame | `xyz`（大yaw 系，m） | `mount_deg`（yaw,pitch,roll） | 方位 |
|---|---|---|---|
| `omni_cam_1` | `[0.052692, -0.186271, 0.0]` | `[-45, 0, 0]` | 前右（y 负） |
| `omni_cam_2` | `[0.052692, 0.186271, 0.0]` | `[45, 0, 0]` | 前左（y 正） |
| **`omni_cam_3`** | **`[-0.147449, 0.092459, 0.003]`** | **`[135, 0, 0]`** | **左后（x 负、y 正）← 本任务用这台** |
| `omni_cam_4` | `[-0.147449, -0.092459, 0.003]` | `[-135, 0, 0]` | 右后（x 负、y 负） |

> ⚠ **必须核对**：配置里的 `mount_deg` 是 ±45°/±135° 的近似值，而实车实测后两台的光轴相对 `-x` 为 ±32.09°（即 147.9°/212.1°），与 135°/225° 差约 13°。**三角化前必须确认 `tf.frames.omni_cam_3` 用的是标定后的 `R`（9 数）或已修正的 `mount_deg`**，否则点的方位会整体偏 13°，8 m 外横向偏差可达 1.8 m。见待确认 Q12。

**工程约束**

| 项 | 值 | 理由 |
|---|---|---|
| 路数 | **1 路（omni_cam_3）** | 只需覆盖左后；单路 CPU 占用约 1 核 |
| 分辨率 | 640×480 | 1080p 光流耗时约 5 倍 |
| 频率 | 15 Hz（相机帧率） | 与 `frame_rate_omni3` 一致 |
| ROI | 图像下半部（`roi_bottom_ratio: 0.5`） | 障碍/地面都在地平线以下，省 60% 特征点 |
| 每帧特征点 | ≤ 300 | 足够覆盖左后区域 |

**注意**：只跑一台相机意味着三角化完全依赖**时间基线**（同相机跨帧），没有多相机立体可用（四台光轴彼此夹角 ≥73°，水平 FOV 约 72.8°，视野几乎不重叠）。这与 §4.6 的退化检测是同一个约束。

### V6 — 三角化 + 局部 BA（L2）

**文件**：新增 `tasks/visual_depth/triangulator.{hpp,cpp}`

```cpp
struct TriResult {
  Eigen::Vector3d p_world;
  int observations;
  double parallax_deg;
  double reproj_px;      // RMS
  bool valid;
};

class Triangulator
{
public:
  // 单条轨迹：DLT 初值 + LM 精化（固定位姿）
  TriResult triangulate(const std::vector<FeatureObs> & track,
                        const std::vector<PoseSample> & poses);

  // M-A 批量：窗口内所有轨迹一起处理（每点独立 3×3，无跨点耦合）
  std::vector<TriResult> triangulateWindow(
      const std::vector<Track> & tracks,
      const std::vector<PoseSample> & poses,
      const BaConfig & cfg);
};
```

> **没有 `localBA()` 函数**。本期 M-A 的"局部 BA"就是 `triangulateWindow()`：窗口内每个 3D 点用它在多帧上的全部观测做一次 3×3 LM，点与点之间没有耦合项，所以**不需要组装大矩阵**。这也是它能 < 2 ms 完成的原因。

**实现清单**

| 步骤 | 内容 | 行数估计 |
|---|---|---|
| DLT 初值 | 构造 \(2N\times4\) 矩阵 + SVD | ~40 |
| 投影与雅可比 | \(\pi(\cdot)\) 与 \(\partial\pi/\partial X\) | ~30 |
| LM 迭代（M-A） | 3×3 系统，逐点独立 | ~50 |
| 退化判据 | 视差 / 条件数 / 重投影 | ~30 |
| 多帧一致性 | 轨迹历史 + 阈值 | ~40 |
| ~~M-B 联合优化~~ | ~~Schur 补~~ | **本期不做** |

**总新增代码量**：约 190 行（三角化 + BA），不含测试。

### V7 — 主循环接入

**位置**：`src/sentry_multithread.cpp`，`take_detection_results(pipeline)`（约 L353）之后。

1. 此处 `tf_tree.set_imu(...)`（约 L267）已刷新，外参可用；
2. 收集 5 路 `CameraResult.obstacles`（PnP 点）+ **1 路**（`omni_cam_3`）三角化点；
3. 按运行模式分投递：
   - **比赛模式**：只投 C3（10 Hz），`visual_triangulation.enable: false`
   - **建图模式**：投 C3（10 Hz）+ C4（1 Hz），三角化 + BA 全开
4. 时间戳用 `CameraResult.timestamp`（`steady_clock`）+ 标定偏移；
5. **不要**放进 `CameraJob::postprocess`（`yolo_pipeline.hpp:70-77`，那是在绑核推理线程里跑）。

> 三角化必须用**本相机自己的**帧序列：左后相机的 `CameraResult` 从 `drain_omni()` 里按 `camera_id == 3` 挑出来。不要跨相机混用轨迹。

### V8 — 配置

`configs/sentry_example.yaml` / `sentry_virtual.yaml` 追加（`soldier.yaml` 中全部 `enable: false`）。

**因为比赛与建图是两次独立运行，建议用两个文件**：`sentry.yaml`（比赛）与 `sentry_mapping.yaml`（建图），只差三个开关。

#### 配置段（两套共用结构）

```yaml
#####-----视觉障碍点云（发给导航补盲，仅哨兵多相机需要）-----#####
# 整段可删：没有这一段就不启动点云发布与三角化。
obstacle_cloud:
  enable: true
  topic_obstacle: "/red_standard_robot1/vision/obstacle_cloud"
  topic_map: "/red_standard_robot1/vision/map_points"
  topic_status: "/red_standard_robot1/vision/status"
  frame_id_body: "gimbal_yaw"       # 出口 A/B 用
  frame_id_world: "camera_init"     # 出口 C 用（与导航 Point-LIO 一致）
  publish_hz_obstacle: 10
  publish_hz_map: 1                 # 比赛配置设为 0（不发 C4）
  max_points_obstacle: 512
  max_points_map: 20000
  max_range_obstacle: 5.0           # 近点走实时
  max_range_map: 12.0               # 远点走建图
  intensity: 1.0

nav_pose:                            # 双向通信：导航 → 视觉
  topic: "/red_standard_robot1/aft_mapped_to_init"
  buffer_ms: 500
  timeout_ms: 200

visual_triangulation:
  enable: true                      # 比赛配置设为 false（省 CPU）
  camera_id: 3                      # 左后（omni_cam_3），见 §6 V5
  resolution: [640, 480]
  rate_hz: 15
  roi_bottom_ratio: 0.5             # 只用图像下半部
  max_corners: 300
  min_baseline: 0.15                # m
  min_parallax_deg: 0.5
  max_reproj_px: 3.0
  min_triangulated_points: 20       # 低于此值本帧放弃

visual_ba:                           # M-A：仅优化点，不动位姿
  enable: true                      # 比赛配置设为 false
  mode: "points_only"               # 唯一支持值
  window_size: 8
  max_iterations: 8
  huber_threshold_px: 3.0
  min_observations: 3

visual_ground_projection:           # 纯旋转兜底
  enable: true
  max_range: 6.0

dynamic_filter:
  enable: true
  consistency_window: 3
  max_position_spread: 0.2          # m
  reject_streak: 3
```

#### 两套配置的差异

| 键 | 比赛（`sentry.yaml`） | 建图（`sentry_mapping.yaml`） |
|---|---|---|
| `obstacle_cloud.publish_hz_map` | `0` | `1` |
| `visual_triangulation.enable` | `false` | `true` |
| `visual_ba.enable` | `false` | `true` |

> 比赛只保留 L0（PnP 点），CPU 占用几乎为零；三角化/BA 全放在建图运行里，不抢自瞄的算力。

### V9 — 自测（不依赖导航侧）

```bash
ros2 topic list -t | grep vision
ros2 topic hz /red_standard_robot1/vision/obstacle_cloud --window 20
ros2 topic echo /red_standard_robot1/vision/obstacle_cloud --once --field width
ros2 topic echo /red_standard_robot1/vision/status --once
```

**判据**：`status` 里 `stamp_lag_ms < 80`；有平移时 `triangulated > 100`；纯旋转时 `triangulated ≈ 0` 但 `ground_pts > 0`（地面法生效）。

---

## 7. 导航侧任务

### N1 — `terrain_analysis` 订阅视觉点云（出口 A/B）

**文件**：`ros_ws/src/pb2025_sentry_nav/terrain_analysis/src/terrain_analysis.cpp`

**构造函数**（现有订阅之后，约 L95-106）：

```cpp
vision_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
  vision_cloud_topic_, rclcpp::SensorDataQoS(),
  [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lk(vision_mutex_);
    vision_msg_ = msg;
    vision_recv_count_++;
  });
```

**头文件新增成员**（`include/terrain_analysis/terrain_analysis.hpp`）

```cpp
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr vision_cloud_sub_;
std::mutex vision_mutex_;
sensor_msgs::msg::PointCloud2::ConstSharedPtr vision_msg_;
size_t vision_recv_count_ = 0;

bool vision_cloud_enable_ = false;
std::string vision_cloud_topic_ = "vision/obstacle_cloud";
std::string vision_frame_ = "gimbal_yaw";
double vision_intensity_ = 1.0;
double vision_voxel_size_ = 0.1;
double vision_max_range_ = 6.0;
double vision_timeout_ = 0.5;
std::vector<double> vision_yaw_offset_xyz_{0, 0, 0};
std::vector<double> vision_yaw_offset_rpy_{0, 0, 0};

std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
tf2::Transform tf_lidar_from_vision_;   // T_front_mid360 <- gimbal_yaw
bool tf_ready_ = false;

pcl::VoxelGrid<pcl::PointXYZI> vision_voxel_;
```

> `tf2/tf2_ros/tf2_geometry_msgs` **已在** `terrain_analysis/package.xml:20-22`，无需改依赖。

### N2 — 一次性取静态外参

```cpp
// 构造末尾或首次收到点云时
try {
  auto tf_stamped = tf_buffer_->lookupTransform(
    sensor_frame_, vision_frame_, tf2::TimePointZero, tf2::durationFromSec(2.0));
  tf2::fromMsg(tf_stamped.transform, tf_lidar_from_vision_);

  tf2::Matrix3x3 R_off;
  R_off.setRPY(vision_yaw_offset_rpy_[0], vision_yaw_offset_rpy_[1], vision_yaw_offset_rpy_[2]);
  tf2::Transform t_off(R_off, tf2::Vector3(
    vision_yaw_offset_xyz_[0], vision_yaw_offset_xyz_[1], vision_yaw_offset_xyz_[2]));
  tf_lidar_from_vision_ = tf_lidar_from_vision_ * t_off;
  tf_ready_ = true;
} catch (const tf2::TransformException & ex) {
  RCLCPP_WARN(get_logger(), "vision tf not ready: %s", ex.what());
}
```

语义：`p_lidar = tf_lidar_from_vision_ * p_gimbal`。

**前提核实**：TF 树里必须有 `gimbal_yaw` 与 `front_mid360`（由 `robot_state_publisher` 提供，整车启动时才有）。验证：

```bash
ros2 run tf2_ros tf2_echo front_mid360 gimbal_yaw --ros-args \
  -r /tf:=/red_standard_robot1/tf -r /tf_static:=/red_standard_robot1/tf_static
```

查不到 → 走 §10 降级 D1。

### N3 — 合并进 `terrain_map`（A/B 出口共用）

**位置**：`processLaserCloud()` 内，**`580`（`terrain_cloud_elev_` 填完）之后、`584`（`toROSMsg`）之前**。此时该云已在 `480` 行 `clear()` 并重建，插入点只对本帧生效。

```cpp
  // ---- 视觉补盲点并入（新增） ----
  // 注意：terrain_cloud_elev_ 是 odom 系，视觉点也必须变到 odom 系
  if (vision_cloud_enable_ && tf_ready_) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr vmsg;
    {
      std::lock_guard<std::mutex> lk(vision_mutex_);
      vmsg = vision_msg_;
    }
    const bool fresh =
      vmsg && (this->now() - vmsg->header.stamp) < rclcpp::Duration::from_seconds(vision_timeout_);

    if (fresh) {
      tf2::Transform tf_odom_to_lidar;
      tf2::fromMsg(odom_.pose.pose, tf_odom_to_lidar);      // lidar_odometry: odom -> front_mid360
      const tf2::Transform tf_odom_to_vision = tf_odom_to_lidar * tf_lidar_from_vision_;

      sensor_msgs::msg::PointCloud2 v_in_odom;
      pcl_ros::transformPointCloud("odom", tf_odom_to_vision, *vmsg, v_in_odom);

      pcl::PointCloud<pcl::PointXYZI> v_pts;
      pcl::fromROSMsg(v_in_odom, v_pts);
      pcl::PointCloud<pcl::PointXYZI>::Ptr vptr =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(v_pts);
      pcl::PointCloud<pcl::PointXYZI>::Ptr vfiltered(new pcl::PointCloud<pcl::PointXYZI>());
      vision_voxel_.setInputCloud(vptr);
      vision_voxel_.filter(*vfiltered);

      for (auto & p : vfiltered->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (std::hypot(p.x - odom_.pose.pose.position.x,
                       p.y - odom_.pose.pose.position.y) > vision_max_range_) continue;
        p.intensity = static_cast<float>(vision_intensity_);   // 必须覆盖，保证不被强度过滤
        terrain_cloud_elev_->push_back(p);
      }
    }
  }
```

**为什么放这里**：`terrain_cloud_elev_` 是 `odom` 系，之后统一做 `odom → sensor_frame_` 反变换（`:592-595`），视觉点自动跟着走；且 `terrain_map` 同时喂 costmap（出口 A）和 `pointcloud_to_laserscan`（出口 B），**一处改动同时支持两个出口**。

**v1 时间近似**：用最新 `odom_`，视觉点可能落后 50~100 ms；1 m/s 直行时误差 ≈ 0.1 m。若 A3 不达标，把 `lidar_odometry` 改成 200 ms 环形缓冲按 `vmsg->header.stamp` 插值。

### N4 — 出口 B（2D 栅格）的注意事项

`terrain_map` 会自动流到 `pointcloud_to_laserscan → slam_toolbox`（`slam_launch.py:111-140`），**无需额外代码**，但要知道三个既有限制：

| 限制 | 值 | 影响 |
|---|---|---|
| `min_laser_range` / `max_laser_range` | 0.3 / 10.0 m | 视觉点超出此范围不进栅格 |
| `throttle_scans` | 1 | 每帧都处理 |
| `map→odom` | **静态恒等**（`slam_launch.py:157-180`） | 栅格质量 = Point-LIO 累积漂移，视觉点会跟着漂 |
| `use_scan_matching` / `do_loop_closing` | 均为 false | 无回环修正 |

**因此**：出口 B 只能"让地图看起来更完整"，**不能提升定位精度**。真正提升定位的是出口 C（先验 PCD）+ `small_gicp`。

### N5 — 出口 C：先验 PCD 融合（**只做离线**）

**已定**：不做在线累积节点，全部走离线文件流程。

```
① 视觉侧建图运行（sentry_mapping.yaml）产出 vision_map.pcd
② 人工拷到 pb2025_nav_bringup/pcd/reality/
③ 运行离线合并脚本（一次性的，见下）
④ 生成融合后的 rmul_2024.pcd → small_gicp_relocalization 加载
```

**目录约定**

```
pb2025_nav_bringup/pcd/reality/
├── rmul_2024.pcd            # 激光 PCD（Point-LIO 产出，现有）
├── vision_map.pcd           # 视觉 PCD（本任务产出）
└── rmul_2024_fused.pcd      # 融合结果（small_gicp 用这个）
```

**离线合并脚本（一次性工具，`tools/merge_pcd.cpp` 或 Python）**

```python
# 伪代码
lidar = load("rmul_2024.pcd")       # 激光点，保留
vis   = load("vision_map.pcd")      # 视觉点，带质量字段

# 1) 质量筛选
vis = vis[(vis.normal_x > 0.7) & (vis.normal_z > 1.0)]   # 重投影/视差
vis = vis[vis.curvature > 0.5]                            # 一致性
# 2) 剔除与激光重复的区域（KD-tree 最近邻 0.2m）
vis = remove_if_nearest(lidar, vis, radius=0.2)
# 3) 只保留盲区方向（左后 ±60° 扇区 + 5m 内无激光点）
vis = filter_by_sector(vis, yaw_center=-135, half_width=60)
# 4) 合并 + 打标签
fused = lidar + tag(vis, intensity=0.5)
write("rmul_2024_fused.pcd", fused)
```

**融合策略（关键）**

| 规则 | 原因 |
|---|---|
| 与最近激光点距离 < 0.2 m → **丢弃视觉点** | 激光更准，避免重复与噪声叠加 |
| `normal_x > 0.7 && normal_z > 1.0`（质量阈值） | 剔除低质量三角化结果 |
| `curvature > 0.5`（一致性得分） | 剔除动态点 |
| 只保留左后 ±60° 扇区 | 其他方向本就有激光覆盖，补了反而引入误差 |
| 视觉点打标签 `intensity=0.5` | 可与激光点区分；必要时在 `small_gicp` 中降权 |

> **为什么这里要"限制扇区"**：视觉点精度永远不如激光。只在激光真的没有覆盖的方向补，才不会让整体 PCD 质量下降。这和"全局融合"是两个不同的思路，本任务明确选前者。

### N6 — 参数

`config/reality/nav2_params.yaml` 的 `terrain_analysis:` 段（L107-135）追加，同步加进 `config/simulation/nav2_params.yaml`：

```yaml
    # ---- 视觉补盲（任务一）----
    vision_cloud_enable: true
    vision_cloud_topic: "vision/obstacle_cloud"
    vision_frame: "gimbal_yaw"
    vision_intensity: 1.0
    vision_voxel_size: 0.1
    vision_max_range: 6.0
    vision_timeout: 0.5
    vision_yaw_offset_xyz: [0.0, 0.0, 0.0]
    vision_yaw_offset_rpy: [0.0, 0.0, 0.0]
```

### N7 — 调试

```bash
# 双向都通
ros2 topic hz /red_standard_robot1/aft_mapped_to_init
ros2 topic hz /red_standard_robot1/vision/obstacle_cloud
ros2 topic echo /red_standard_robot1/vision/status --once

# terrain_map 里的视觉点（RViz 按 intensity 着色：激光 0~0.5，视觉 1.0）
ros2 topic echo /red_standard_robot1/terrain_map --once --field width

# 静态外参应接近 x=-0.036 y=0.093 z=0.363, pitch=-30°
ros2 run tf2_ros tf2_echo front_mid360 gimbal_yaw --ros-args \
  -r /tf:=/red_standard_robot1/tf -r /tf_static:=/red_standard_robot1/tf_static
```

---

## 8. 联调步骤

| 阶段 | 内容 | 通过判据 | 工期 |
|---|---|---|---|
| 0 | 环境：`ROS_DOMAIN_ID` / RMW 一致；TF 树有 `gimbal_yaw`；双方确认第 5 章契约 | 话题可见 | 0.5 天 |
| 1 | 双向打通：视觉发空点云 + 订阅 odom；导航订阅 + 计数 | A1 | 1 天 |
| 2 | PnP 点可见：静止 + 3 m 目标 | 视觉点与激光重合 | 1 天 |
| 3 | 静态对齐：4 方向目标，调 `vision_yaw_offset_*` | A2 | 1.5 天 |
| 4 | 旋转：1/2/3 rad/s 自旋 | A3 | 1 天 |
| 5 | 盲区通过性：遮挡雷达侧后方 | A4 | 1 天 |
| 6 | 三角化：缓慢平移，看重投影误差 | A5 | 1 天 |
| 7 | 局部 BA：打开/关闭对比 | A6 | 1 天 |
| 8 | 出口 B：建图模式跑一圈，看栅格 | 盲区不再大片 unknown | 1 天 |
| 9 | 出口 C：离线 PCD 融合 + `small_gicp` 重定位 | A7/A8 | 2 天 |
| 10 | 长稳与性能（10 min） | A10 | 1 天 |

---

## 9. 变更清单

### 9.1 RM_Vision_2027

| 文件 | 改动 | 行数 |
|---|---|---|
| `tasks/auto_aim/multithread/yolo_pipeline.hpp` | `CameraResult` + `ObstaclePoint` | +10 |
| `tasks/omniperception/decider.hpp/.cpp` | `last_pnp()` 访问器 | +10 |
| `tasks/omniperception/omni_jobs.cpp` | 填充 `obstacles` | +10 |
| `io/ros2/nav_pose_buffer.hpp/.cpp` | **新增**位姿订阅 + 插值 | ~180 |
| `io/ros2/publish2nav_pointcloud.hpp/.cpp` | **新增**点云发布（C3/C4/C5） | ~260 |
| `tasks/visual_depth/feature_tracker.hpp/.cpp` | **新增**特征跟踪（左后单路） | ~200 |
| `tasks/visual_depth/triangulator.hpp/.cpp` | **新增**三角化 + M-A（无 M-B） | ~190 |
| `tasks/visual_depth/ground_projection.hpp/.cpp` | **新增**地面反投影 | ~90 |
| `tasks/visual_depth/pcd_writer.hpp/.cpp` | **新增**建图 PCD 落盘 | ~80 |
| `io/CMakeLists.txt` | 加 `nav_msgs` + 3 处新源文件 | +7 |
| `CMakeLists.txt` | 加 `visual_depth` 库 | +5 |
| `src/sentry_multithread.cpp` | 主循环聚合 + 节流 + 主相机点变换 | +60 |
| `configs/sentry_example.yaml` / `sentry_virtual.yaml` / `soldier.yaml` | 各加 1 段 + 建图专用副本 | +70 ×3 |

**合计**：约 **1030 行**（含新增模块；较 v1 的 1600 行减少，因为砍掉 M-B 与在线累积节点）

### 9.2 Navigation-2027-Jazzy

| 文件 | 改动 | 行数 |
|---|---|---|
| `terrain_analysis/include/.../terrain_analysis.hpp` | 新增成员 | +25 |
| `terrain_analysis/src/terrain_analysis.cpp` | 订阅 + TF + 合并 + 参数 | +90 |
| `pb2025_nav_bringup/config/reality/nav2_params.yaml` | `terrain_analysis` 段 +9 参数 | +9 |
| `pb2025_nav_bringup/config/simulation/nav2_params.yaml` | 同上 | +9 |
| `pb2025_nav_bringup/pcd/reality/` | 新增 `vision_map.pcd`、`rmul_2024_fused.pcd` | 数据文件 |
| `tools/merge_pcd.cpp`（或 `.py`） | **离线合并脚本**（一次性工具，不入包） | ~120 |
| `terrain_analysis/package.xml` | **无需改动**（tf2 已具备） | 0 |

**合计**：约 **134 行**（不含数据文件与离线脚本）

---

## 10. 风险与降级

| 编号 | 风险 | 影响 | 处理 |
|---|---|---|---|
| R1 | 单目无可靠远距离深度 | 三角化 8 m 外误差 0.43 m，15 m 外 1.5 m | 分级：C3 只用 ≤5 m，C4 允许 ≤12 m 并带质量字段 |
| R2 | 静态外参原点不重合 | 盲区点整体偏移，旋转时放大 | 阶段 3 用 `vision_yaw_offset_*` 补偿；> 0.3 m 回查机械 |
| R3 | 时间戳用了发布时刻 | 旋转时点被甩出 | 强制 §5.6；`status.stamp_lag_ms` 监控 |
| R4 | 动态点污染 PCD | `small_gicp` 重定位被拉偏 | §4.7 多帧一致性 + §7 N5 质量筛选 |
| R5 | `gimbal_yaw` frame 缺失 | N2 失败 | 降级 D1 |
| R6 | 纯旋转深度退化 | 三角化无输出 | 地面平面反投影兜底（§4.5） |
| R7 | 视觉误检 | costmap 假障碍堵路 | 颜色过滤 + 置信度阈值 + `vision_timeout` |
| R8 | 局部 BA 收敛到局部极小 | 点更差 | LM 阻尼 + 初值来自 DLT + 代价不降则回退原值 |
| R9 | 多路高分辨率光流跑不动 | 主链路 FPS 下降 | **只跑 1 路（`omni_cam_3`）640×480**，ROI 下半部；比赛配置直接关闭 |
| R11 | 相机外参口径不一致（135° vs 147.9°） | 点方位整体偏 13° | Q12：三角化前用标定 `R` 覆盖 `mount_deg` |
| R10 | DDS 带宽（逐点 odom） | 网络/CPU 压力 | 实测 Q4；必要时降 `publish_odometry_without_downsample` |

**降级方案**

| 编号 | 触发 | 方案 |
|---|---|---|
| D1 | 无 `gimbal_yaw` frame | 视觉改发 `frame_id: "camera_init"`（世界系），导航用一次性 4-DoF 对齐；精度下降 |
| D2 | 三角化质量差 | 只跑 L0（PnP 点）+ 地面反投影 |
| D3 | 局部 BA 拖慢实时 | `visual_ba.enable: false`，只做 DLT + 3 次 GN |
| D4 | 完全不可用 | 两侧 `enable: false`，行为回退当前版本 |

---

## 11. 后续任务预留

| 任务 | 内容 | 与本任务的关系 |
|---|---|---|
| **任务二** | 语义信息（敌我数量、距离、方位）→ `pb_nav_goal_decision` | 复用 `CameraResult.obstacles` 与 C5；新增 `vision/semantic`（JSON）；导航侧新增订阅 + 决策节点输入 |
| 任务四 | 视觉点参与 `point_lio` 的 IVox 地图 | **只有静态点允许**。需线程安全改造（IVox 无锁）+ 静态判定 |
| 任务五 | 视觉重定位（辅助 `small_gicp` 初始化） | 需要回环或先验地图匹配 |

---

## 12. 分工与里程碑

| 里程碑 | 视觉侧 | 导航侧 | 联合验收 | 工期 |
|---|---|---|---|---|
| M0 | 确认第 5 章契约 | 确认第 5 章契约 | 签字 | 0.5 天 |
| M1 | V2+V4（空点云 + C5） | N1+N6 | A1 | 1 天 |
| M2 | V1+V7（PnP 点） | N2+N3 | 可视 | 1 天 |
| M3 | V9 自测 | N7 调试 | A2 | 1.5 天 |
| M4 | — | 参数微调 | A3 | 1 天 |
| M5 | — | costmap 调参 | A4 | 1 天 |
| M6 | V5+V6（左后单路三角化） | — | A5 | 1 周 |
| M7 | M-A 窗口优化（`triangulateWindow`） | — | A6 | 0.7 周 |
| M8 | 离线 PCD 生成（`pcd_writer`） | N5 融合 | A7/A8 | 2 天 |
| M9 | 性能优化 | 性能统计 | A10 | 1 天 |

**协作约定**

1. 第 5 章话题名、消息类型、`frame_id`、`intensity` 语义 **冻结后不得单方面修改**；
2. 改契约先在本文档更新 + PR 记录，双方确认后改代码；
3. 两侧独立可编译、独立可回滚，不允许"必须同时升级才能跑"；
4. 不允许跳过阶段 3、4 直接调阶段 5。

---

## 13. 待确认清单（联调前必须填完）

| 编号 | 问题 | 责任方 | 结论 |
|---|---|---|---|
| Q1 | `gimbal_yaw` 与视觉 `yaw_axis` 原点是否重合？实测偏差多少？ | 双方 | 待测 |
| Q2 | `steady_clock → ROS time` 偏移标定方法与残差 | 视觉 | 待定 |
| Q3 | 全向相机实际帧率（`frame_rate_omni1..4`）与 `publish_hz` 匹配 | 视觉 | 待测 |
| Q4 | `aft_mapped_to_init` 实测频率与带宽（逐点发布可能极高） | 导航 | 待测 |
| Q5 | 只发敌方还是敌我双方？ | 视觉 | 待定 |
| Q6 | 主相机是否参与（小yaw 板随俯仰动，误差更大） | 双方 | 待定 |
| Q7 | `vision_timeout = 0.5 s` 是否合适 | 导航 | 待定 |
| Q8 | 三角化跑哪几路相机 | 视觉 | ✅ **已定：只跑左后 `omni_cam_3`**（小yaw 遮挡在左后）。**待核**：`tf.frames.omni_cam_3` 的 `mount_deg` 是 135°，实车实测为 147.9°，差 13°，须用标定值 |
| Q9 | 比赛中是否保留 C4（建图点云） | 双方 | ✅ **已定：建图单独跑**，比赛 `publish_hz_map: 0` |
| Q10 | 局部 BA 是否开 M-B（位姿联合优化） | 视觉 | ✅ **已定：只做 M-A（仅优化点）**，M-B 不实现 |
| Q11 | 出口 C 离线还是在线累积节点 | 双方 | ✅ **已定：只做离线文件**，不做累积节点 |
| Q12 | `omni_cam_3` 的 yaw 外参用 135° 还是实测 147.9°？ | 视觉 | ⬜ **新增待核对**（决定 8 m 外 1.8 m 的横向偏差） |
| Q13 | 建图运行时是否需要同时录 rosbag（用于离线复算） | 视觉 | ⬜ 待定 |
