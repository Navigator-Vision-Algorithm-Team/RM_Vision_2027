# Sentry 视觉节点通信协议

> 适用入口：`src/sentry_multithread_debug.cpp`
> 数据来源：下位机（MCU / 串口）+ 导航模块（ROS2）

本文件描述 `sentry_multithread_debug` 入口从下位机**接收**的数据，以及为完成闭环**发送**给下位机的数据。

---

## 1. 通信链路

| 链路 | 载体 | 用途 | 代码 |
| --- | --- | --- | --- |
| 串口 | `io::SerialBoard` | 与 MCU（云台/底盘主控）互发数据帧 | `io/serial_board.cpp` |
| ROS2 | `io::ROS2` | 订阅导航速度指令 | `io/ros2/subscribe2nav.cpp` |

串口参数（`configs/sentry.yaml` + `io/serial_board.cpp`）：

| 参数 | 值 | 配置项 |
| --- | --- | --- |
| 波特率 | 921600 | `baud_rate` |
| 数据位 | 8 | 固定 |
| 校验位 | 无 | 固定 |
| 停止位 | 1 | 固定 |
| 流控 | 无 | 固定 |
| 弹速 | 28.0 m/s | `bullet_speed` |
| 裁判系统开关 | false | `use_referee_system` |
| 比赛开始进度阈值 | 4 | `game_start_progress_threshold` |

---

## 2. 帧结构（串口）

所有串口数据包都以统一的 `Header` 开头，结构体按 `__attribute__((packed))` 紧凑排列，**多字节字段为小端序（Little-Endian）**。定义见 `io/gimbal/packet.hpp`。

### 2.1 通用帧头 Header（7 字节）

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `header` | uint8 | 帧起始标志，固定 `0xA5` |
| 1 | `data_length` | uint16 | 数据段长度（不含帧头与 CRC16） |
| 3 | `seq` | uint8 | 序号（当前发送恒为 0） |
| 4 | `crc8` | uint8 | 帧头校验，CRC8 结果 |
| 5 | `cmd_id` | uint16 | 命令 ID，区分包类型 |

### 2.2 校验规则

| 校验 | 初始值 | 覆盖范围 | 存放位置 | 实现 |
| --- | --- | --- | --- | --- |
| CRC8 | `0xFF`（poly `0x2F`，RoboMaster 标准） | 帧头前 4 字节（偏移 0~3） | 偏移 4（`header.crc8`） | `crc8::Append_CRC8_Check_Sum` |
| CRC16 | `0xFFFF`（poly `0x1021`，RoboMaster 标准 / CCITT） | 除最后 2 字节外的整包 | 最后 2 字节，小端 | `crc16::Append_CRC16_Check_Sum` |

> 导航命令 `0x0405` 与自瞄 `0x0402`、遥测回传（0x502 等）**共用同一套 RM 标准 CRC**（init `0xFF` / `0xFFFF`），不存在特例。`SerialBoard::send(NavCommand)` 同样调用上面的 `crc8::Append_CRC8_Check_Sum` / `crc16::Append_CRC16_Check_Sum`。

CRC 表与算法见 `io/gimbal/protocol_crc.cpp`。接收端解析流程（`SerialBoard::receiveThread`）：读取 `0xA5` → 读帧头剩余 6 字节 → 校验 CRC8 → 按 `cmd_id` 读取定长 payload → 校验 CRC16。

---

## 3. 接收：下位机 → 视觉

视觉在 `receiveThread` 中按 `cmd_id` 分发处理。各包大小按 packed 结构推算。

### 3.1 `0x502` IMU 姿态（IMUPacket）

**最重要的一路数据**，用于 `serial_board.imu_at()` 获取云台世界系姿态。

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `header` | Header | 见 2.1，`data_length = 15` |
| 7 | `pitch` | float | 俯仰角（rad） |
| 11 | `roll` | float | 横滚角（rad） |
| 15 | `yaw` | float | 偏航角（rad） |
| 19 | `roboid` | uint8 | 机器人 ID |
| 20 | `id` | uint8 | 保留/来源标识 |
| 21 | `timeseries` | uint8 | 时间序列号 |
| 22 | `crc16` | uint16 | CRC16 |

总长 **24 字节**。

**视觉侧使用**（`io/serial_board.cpp`）：

```cpp
Eigen::Quaterniond q =
  Eigen::AngleAxisd(yaw,   UnitZ()) *
  Eigen::AngleAxisd(pitch, UnitY()) *
  Eigen::AngleAxisd(roll,  UnitX());
```

即按 **Z-Y-X（yaw-pitch-roll）** 顺序将欧拉角转为四元数。接收线程为每帧记录 `steady_clock` 时间戳，`imu_at(t)` 按时间戳对相邻两帧做 **slerp 球面插值**，与相机曝光时间对齐（入口传入 `timestamp - 1ms`）。

### 3.2 `0x503` 底盘速度（classisPacket）

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `header` | Header | `data_length = 13` |
| 7 | `vx` | float | 底盘 X 方向速度 |
| 11 | `vy` | float | 底盘 Y 方向速度 |
| 15 | `vz` | float | 底盘 Z / 角速度 |
| 19 | `enable` | uint8 | 使能标志 |
| 20 | `crc16` | uint16 | CRC16 |

总长 **22 字节**。当前入口仅做 CRC 校验，未参与决策。

### 3.3 `0x504` 按键 / 扳机（bottonPacket）

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `header` | Header | `data_length = 1` |
| 7 | `start` | bool(1B) | 按键 / 启动状态 |
| 8 | `crc16` | uint16 | CRC16 |

总长 **10 字节**。当前入口仅做 CRC 校验。

### 3.4 `0x505` 机器人底盘状态（bassPacket）

**复位信号来源**：`serial_board.reset_pending()`。

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `header` | Header | `data_length = 11` |
| 7 | `roboid` | uint8 | 机器人 ID |
| 8 | `reset` | bool(1B) | **复位标志**：为真时入口 `tracker.reset()` |
| 9 | `position[2]` | uint8×2 | 位置信息 |
| 11 | `HP` | uint8 | 血量 |
| 12 | `robomode` | uint8 | 机器人模式 |
| 13 | `canshoot` | bool(1B) | 允许发射 |
| 14 | `shootmode` | uint8 | 发射模式 |
| 15 | `superpower` | bool(1B) | 超级电容状态 |
| 16 | `online` | bool(1B) | 在线状态 |
| 17 | `two` | bool(1B) | 保留标志 |
| 18 | `crc16` | uint16 | CRC16 |

总长 **20 字节**。

### 3.5 `0x0001` / `0x0100` 比赛状态（GameStatusPacket）

**比赛开始判定来源**：`serial_board.is_game_started()`。

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `header` | Header | `data_length = 11` |
| 7 | `game_info` | uint8 | 低 4 位 = `game_type`，高 4 位 = `game_progress` |
| 8 | `stage_remain_time` | uint16 | 阶段剩余时间（s） |
| 10 | `sync_timestamp` | uint64 | 同步时间戳 |
| 18 | `crc16` | uint16 | CRC16 |

总长 **20 字节**。

解析规则（`SerialBoard::handleGameStatus`）：

- `game_type = game_info & 0x0F`
- `game_progress = (game_info >> 4) & 0x0F`
- 当 `use_referee_system = true` 时，`game_progress >= 4` 判定为比赛开始；否则（无人裁判系统）启动即视为已开始。

### 3.6 `0x0201` / `0x0102` 机器人状态（GameRobotStatusPacket）

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `header` | Header | `data_length = 13` |
| 7 | `robot_id` | uint8 | 机器人 ID |
| 8 | `robot_level` | uint8 | 等级 |
| 9 | `remain_hp` | uint16 | 剩余血量 |
| 11 | `max_hp` | uint16 | 最大血量 |
| 13 | `shooter_cooling_rate` | uint16 | 枪口冷却速率 |
| 15 | `shooter_heat_limit` | uint16 | 枪口热量上限 |
| 17 | `chassis_power_limit` | uint16 | 底盘功率上限 |
| 19 | `mains_power_state` | uint8 | bit0=云台供电, bit1=底盘供电, bit2=发射机构供电 |
| 20 | `crc16` | uint16 | CRC16 |

总长 **22 字节**。

`mains_power_state` 位定义（`SerialBoard::handleGameRobotStatus`）：

| bit | 含义 |
| --- | --- |
| 0 | 云台供电 `mains_power_gimbal` |
| 1 | 底盘供电 `mains_power_chassis` |
| 2 | 发射机构供电 `mains_power_shooter` |

### 3.7 未识别包

`default` 分支按 `header.data_length` 读取并丢弃，仅用于对齐后续数据流。

---

## 4. 发送：视觉 → 下位机

### 4.1 `0x0402` 自瞄控制指令（SendPacket）

由 `SerialBoard::send(Command)` 下发，入口每帧调用。

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `header` | Header | `data_length = 13` |
| 7 | `id` | uint8 | 恒为 0 |
| 8 | `robo_id` | uint8 | 恒为 0 |
| 9 | `pitch` | float | 目标俯仰**世界系绝对角**（rad） |
| 13 | `yaw` | float | 目标偏航**世界系绝对角**（rad） |
| 17 | `accuracy` | uint8 | 恒为 50 |
| 18 | `shoot` | uint8 | `(control && shoot) ? 1 : 0` |
| 19 | `timeseries` | uint8 | 时间戳低 8 位（接收方不使用，占位 0） |
| 20 | `crc16` | uint16 | CRC16 |

总长 **22 字节**。

> 关键约定：`yaw` / `pitch` 为 **IMU 世界系绝对角**，与 `0x502` 的 yaw 同一定义。入口中 `aimer.aim()` 输出已满足该定义直接下发；`decider.decide()` 与全向切换的 `delta_yaw/delta_pitch` 为云台系角，需叠加 `gimbal_pos[0]` 后下发。`control` 为假时 `shoot` 强制为 0。

### 4.2 `0x0405` 导航控制指令（NavigationPacket）

由 `SerialBoard::send(NavCommand)` 下发，来源为 ROS2 订阅。

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `header` | Header | `data_length = 12` |
| 7 | `vx` | float | 前向速度 |
| 11 | `vy` | float | 横向速度 |
| 15 | `wz` | float | 角速度 |
| 19 | `crc16` | uint16 | CRC16 |

总长 **21 字节**（结构体 `packed, aligned(1)`）。

---

## 5. ROS2 输入：导航速度

入口 `ros2.subscribe_get_data()` → `Subscribe2Nav::subscribe_move_info()`（`io/ros2/subscribe2nav.cpp`）。

| 项 | 值 |
| --- | --- |
| 话题 | `/red_standard_robot1/cmd_vel` |
| 消息类型 | `geometry_msgs/msg/Twist` |
| 超时清理 | 收到 ≥2 条消息后启动 1500ms 定时器，超时清空缓存 |

字段映射：

| NavCommand | Twist 字段 |
| --- | --- |
| `vx` | `linear.x` |
| `vy` | `linear.y` |
| `wz` | `angular.z` |

> 旋转角速度取自 `angular.z`（与导航规范 `cmd_vel.angular.z` 一致，且已含 `fake_vel_transform` 叠加的自旋分量）。

---

## 6. 接收数据在入口中的用途速查

| 数据 | 包 / 来源 | 入口中的用途 |
| --- | --- | --- |
| IMU 姿态 | `0x502` | `solver.set_R_gimbal2world(q)`、`gimbal_pos`、自旋角计算 |
| 比赛开始 | `0x0001`/`0x0100` | `is_game_started()` 门控自瞄逻辑 |
| 复位信号 | `0x505` (reset) | `tracker.reset()` |
| 弹速 | `bullet_speed`（yaml） | `aimer.aim(..., bullet_speed)` |
| 导航速度 | ROS2 `cmd_vel` | 下发 `0x0405` |

---

## 7. 结构体定义位置索引

| 结构体 | 文件 |
| --- | --- |
| `Header` / 所有 Packet | `io/gimbal/packet.hpp` |
| `Command` / `NavCommand` | `io/command.hpp` |
| CRC 算法 | `io/gimbal/protocol_crc.cpp` / `protocol_crc.hpp` |
| 串口收发与解析 | `io/serial_board.cpp` / `serial_board.hpp` |
| ROS2 订阅 | `io/ros2/subscribe2nav.cpp` |
