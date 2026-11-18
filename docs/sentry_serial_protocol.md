# Sentry 视觉节点通信协议（iceoryx2 中转架构）

> 适用入口：`src/sentry_multithread.cpp`、`tests/sentry_multithread_debug.cpp`
> 通信载体：**iceoryx2 publish-subscribe（同机共享内存，零拷贝）**
> 对端进程：`info_relay`（独立仓库，独占串口 / CAN）

本文件描述视觉进程**从中转进程接收什么**、**向中转进程发送什么**、以及这些字段如何对应到下位机帧。
串口收发、分帧、CRC、协议解析、下行报文打包**全部不在本工程内**，实现与帧结构请见 `info_relay` 仓库的
`msg/relay_msgs.hpp`、`src/relay.cpp`、`IO/`。

---

## 1. 进程边界与职责

```
下位机 MCU ──串口/CAN──▶ info_relay ──iceoryx2──▶ 视觉进程（本工程）
                              ▲                        │
                              └───────iceoryx2─────────┘
```

| 职责 | info_relay | 视觉进程 |
| --- | --- | --- |
| 打开 / 持有串口或 CAN | **是（唯一持有者）** | 否，不打开任何设备 |
| 分帧、CRC、赛季协议解析 | **是** | 否 |
| 解析结果发布到共享内存 | **是** | 否 |
| 云台瞄准指令落地成下位机帧 | **是** | 否，只发布 `cmd_aim` |
| 相机采集、YOLO、PnP、跟踪、瞄准决策 | 否 | **是** |
| 是否开火的最终判据 | 否 | **是**（热量 + 链路状态） |
| 录像（H.264 硬编） | 否 | **是**（`tools::Recorder`） |

视觉进程里已**不存在** `io/serial/`、`io/can/`、`io/ros2/` 中的导航桥接代码。
唯一的通信入口是 `io/relay/relay_client.{hpp,cpp}`。

---

## 2. 主题与载荷

主题全名为 `<relay_prefix>/<topic>`，默认前缀 `rm_vision`（视觉侧配置项 `relay_prefix`）。
载荷定义在 `info_relay/msg/relay_msgs.hpp`，本工程持有一份**字节兼容副本** `io/relay/relay_msgs.hpp`，
两侧都必须通过尺寸与 `kSchemaVersion` 校验（当前 **3**）。

| 主题 | 载荷 | 尺寸 | 频率 | 方向 | 本工程 |
| --- | --- | --- | --- | --- | --- |
| `imu` | `Imu` | 48 B | 500 Hz（有新帧才发） | relay → 视觉 | **订阅** |
| `robot` | `Robot` | 40 B | 1 Hz 及变化即发 | relay → 视觉 | **订阅** |
| `game` | `Game` | 24 B | 1 Hz | relay → 视觉 | **订阅** |
| `health` | `Health` | 48 B | 10 Hz | relay → 视觉 | **订阅** |
| `chassis` | `Chassis` | 40 B | 100 Hz | relay → 视觉 | **不订阅**（无消费者） |
| `cmd_aim` | `AimCmd` | 24 B | 视觉帧率 | 视觉 → relay | **发布** |
| `cmd_nav` | `NavCmd` | 24 B | 导航帧率 | 视觉/导航 → relay | **不发布**（不参与导航） |

只订阅实际消费的四个主题，可减少接收线程唤醒与共享内存占用。声明见
`io/relay/relay_contract.hpp` 的 `kSubscribeChassis = false` / `kPublishNavCmd = false`。

### 2.1 `Imu` → 姿态与自瞄门控

| 字段 | 类型 / 单位 | 视觉侧用途 |
| --- | --- | --- |
| `t_ns` | uint64 / ns | 姿态插值的唯一时间基准（中转打点，本侧不重打） |
| `seq` | uint64 | 丢样统计 |
| `uptime_ms` | uint32 / ms | 数值回落 → `relay_client.reset_pending()` → `tracker.reset()` |
| `gimbal_type` | uint8 | 0 无 / 1 单yaw / 2 双yaw / 3 折叠，决定关节角解释 |
| `gimbal_enable` | uint8 | 云台软件使能（只服从，不做二次裁决） |
| `autoaim_enabled` | uint8 | MCU 对自瞄的最终裁决 |
| `pitch_rad` / `yaw_rad` / `roll_rad` | float / rad | 合成 `R_gimbal2world` 送 `Solver` |
| `upper_yaw_rad` | float / rad | 小yaw 关节角；`lower_yaw = yaw_rad - upper_yaw_rad` |
| `fold_yaw_rad` | float / rad | 折叠关节角，本赛制恒为 0 |

**角度约定**：上电时车头朝向为 0，世界系绝对角，`[0, 2π)`，逆时针为正；pitch 向上为负。
单yaw 构型下 `upper_yaw_rad = 0`，`yaw_rad` 即唯一 yaw。

视觉侧读取接口：

```cpp
const ImuState s = serial_board.imu_state_at(timestamp - 1ms);   // 线性插值
R_gimbal2world = Rz(s.lower_yaw) * Ry(s.pitch) * Rx(s.roll);     // 小yaw 板（主相机）
```

### 2.2 `Robot` → 敌我颜色、热量、血量

| 字段 | 视觉侧用途 |
| --- | --- |
| `robot_id` | 红方 1~11 / 蓝方 101~111，0 表示未收到 |
| `own_team_red` | 1 红 / 0 蓝 / 2 未知；取反作为敌人颜色，覆盖配置 `enemy_color` |
| `current_hp` / `maximum_hp` | 血量状态与调试显示 |
| `barrel_heat_17mm` / `heat_limit` | `relay_client.can_shoot()` 的热量门限 |
| `valid_mask` | bit0 机器人状态有效 / bit1 比赛状态有效 / bit2 主机状态有效 |

### 2.3 `Game` → 开赛门控

`Game.in_battle == 1` 时 `is_game_started()` 返回 true，主循环才开启推理；否则整段跳过（只出图不推理）。
`game_progress` 用于日志，视觉侧不再读取 `use_referee_system` / `game_start_progress_threshold`——
开赛判定完全由中转发布的 `in_battle` 决定（见 `io/relay/relay_client.cpp`）。

### 2.4 `Health` → 链路健康与版本握手

| 字段 | 用途 |
| --- | --- |
| `transport_ok` | 中转侧链路判定，与本地超时判定互相印证 |
| `last_rx_ns` | 定位"中转没收到"还是"视觉没收到" |
| `last_latency_us` | 视觉发指令到中转写串口的耗时，用于开火时序标定 |
| `schema_version` | 收到首个 Health 时比对本地 `kSchemaVersion`，不符即判链路失效并打 ERROR |

### 2.5 `AimCmd` → 下发的唯一指令

| 字段 | 类型 / 单位 | 语义 |
| --- | --- | --- |
| `t_ns` | uint64 / ns | 视觉发出时刻，中转据此算端到端延迟 |
| `yaw_rad` | float / rad | 世界系绝对 yaw |
| `pitch_rad` | float / rad | 世界系绝对 pitch，向上为负 |
| `shoot` | uint8 | 1 表示请求开火 |

**没有 `control` 字段。** 下位机报文 `0x0400` 的 yaw/pitch 被无条件执行，唯一受控的是开火位
（`shoot`）。因此"不接管"必须显式下发**当前姿态**，而不是留空或填 0——这由 `Fsm::finalize()` 统一保证。

---

## 3. 接收约定

`RelayClient` 自建**一条接收线程**，对外接口从主循环调用，规则如下：

| 项 | 约定 |
| --- | --- |
| 取样方式 | 每拍把订阅队列**排空**，只保留最新一条；姿态样本额外入有界队列（容量 64）供插值，**绝不排队消费** |
| 姿态插值 | `imu_state_at(t)` 在相邻样本间线性插值；目标时刻早于/晚于样本区间的端点直接返回端点，不外推 |
| time base | 全部用中转打的 `t_ns`；禁止在订阅侧重新打时间戳 |
| 链路失效 | 超过 `relay_timeout_ms`（默认 200 ms）无 IMU，或 `schema_version` 不符 → `link_ok() = false` → `can_shoot() = false`，并强制下发 `shoot = 0` |
| 安全收敛 | 链路失效时绝不发弹（强制 `shoot=0`）；无有效瞄准角时由状态机自旋决策接管云台；`AimCmd` 不排队，每次只保留最新一条 |
| 未启动中转 | 视觉正常出图，只是不开火；`RelayClient` 构造不抛异常 |
| 下发 | `send(Command)` 只写待发槽（非阻塞），接收线程下一拍发布 `cmd_aim` |

启动日志中应看到：

```
[RelayClient] 已连接主题前缀 rm_vision（订阅 imu/robot/game/health）
[RelayClient] 收到首个 IMU 样本，链路建立
```

---

## 4. 中转侧的下位机帧映射（供对照，视觉不直接接触）

下位机帧的 cmd_id、CRC、字段布局全部属于 `info_relay` 的实现范围，此处只列出与视觉消费字段的对应关系：

| 下位机上行帧 | 频率 | 映射到主题 | 视觉使用字段 |
| --- | --- | --- | --- |
| `0x0500` 协议版本与构型 | 1 Hz | `imu` / `robot` | `gimbal_type`、`uptime_ms` |
| `0x0501` 机器人状态 | 1 Hz | `robot` | `enable_bits`、`module_online` |
| `0x0502` 底盘反馈 | 100 Hz | `chassis`（视觉不订阅） | — |
| `0x0503` 云台姿态 | 500 Hz | `imu` | `pitch/yaw/roll_rad`、`upper_yaw_rad`、两个使能位 |
| `0x0001` 比赛状态 | 1 Hz | `game` | `game_progress`、`in_battle` |
| `0x0201` 机器人状态 | 1~10 Hz | `robot` | `robot_id`、`hp`、`heat_limit` |
| `0x0202` 发射状态 | 10 Hz | `robot` | `barrel_heat_17mm` |

下行：视觉 `AimCmd` → 中转 `0x0400`（`pitch` f32 / `yaw` f32 / `shoot` u8）；中转另有 `0x0401` 导航帧，
本工程不使用。完整的帧结构与 CRC 规则见 `info_relay/IO/serial/protocol/`。

---

## 5. 配置

| 配置 | 文件 | 作用 |
| --- | --- | --- |
| `relay_prefix` | 视觉入口配置 | 主题前缀，必须与中转一致 |
| `relay_timeout_ms` | 视觉入口配置 | 无 IMU 判定链路失效的阈值，默认 200 ms |
| `bullet_speed` | 视觉入口配置 | 弹速 m/s，供 `Aimer` 解算 |
| `link_type` / `com_port` / `baud_rate` / `can_interface` / `can_id` / `io_timeout_ms` | `info_relay/configs/info_relay.yaml` | 中转侧硬件链路，视觉侧**不读取** |

主题前缀不匹配时不会报连接错误，只会"收不到数据"，表现为视觉侧始终不开火——部署时务必核对双端配置。

---

## 6. ROS 2 与调试数据流

- 视觉进程**不再**订阅 `/cmd_vel`、不再发布云台关节与 `referee/*` 三类导航话题；
- `tests/sentry_multithread_debug.cpp` 仍使用 `auto_aim::DebugStream`（rclcpp → Foxglove），
  用于开发阶段画面与状态观测，**只订阅业务数据，不参与控制**；
- 当前 `info_relay` 实现中未包含 ROS 2 节点。若恢复导航/裁判桥接，应在中转侧新增，
  视觉侧不重新引入。

---

## 7. 本次架构变更的影响

| 变更 | 影响 |
| --- | --- |
| 串口/CAN 实现整体迁出视觉工程 | 视觉不再需要串口设备权限；无法单独打开下位机调试，须先启动中转 |
| `io::Command::control` 删除 | 下位机报文本就没有该位；"不接管时保持姿态"由状态机 `finalize()` 统一回填当前角度 |
| 导航速度不再经视觉进程 | 导航链路延迟更低；`cmd_nav` 通道在视觉侧移除 |
| 增加 `Health.schema_version` 握手 | 两端头文件不同步会在启动时明确报错，而非静默错位 |
| 自旋由 `tasks/spin_decision` 统一决策 | 无有效瞄准角时 yaw 扫描 + pitch 点头，连续丢失 <5 帧保持姿态；带 5 帧短滞回，从失瞄点接续 |
| 下行多一跳 IPC | 实测 `last_latency_us` 用于开火时序重新标定；通常由几十微秒级开销 |
| 下发集中在主循环末尾 | 每轮最多一次 `send`，且无有效瞄准角时强制停火 |

---

## 8. 构建与联调

```bash
# 视觉工程（需系统已安装 iceoryx2）
cmake -S . -B build -DSP_CONFIG=configs/sentry_example.yaml
cmake --build build --parallel

# 中转工程（独立目录，单独构建）
cd info_relay && cmake -S . -B build && cmake --build build
./build/info_relay                       # 默认 configs/info_relay.yaml

# 视觉入口
./build/sentry_multithread configs/sentry_example.yaml
```

`SP_CONFIG` 决定编译哪些相机 / YOLO 后端（见 README「配置驱动的编译裁剪」）；换车时必须重新配置编译。

自检与观测：

```bash
./info_relay/build/ipc_selftest                         # 期望 PASS
ros2 topic list | grep referee                          # 若中转侧已恢复 ROS2 桥接
grep -E "RelayClient|fsm" logs/*.log | tail             # 链路建立与状态转移
```

> 无硬件环境下无法端到端验证；当前仓库已移除虚拟相机与虚拟下位机。
