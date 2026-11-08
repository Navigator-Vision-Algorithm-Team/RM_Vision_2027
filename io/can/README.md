# CAN 通信模块

本目录包含两套用途不同的 CAN 通信实现：

- `cboard.*`：原有 CBoard 接口，保留给哨兵/原 CAN 协议使用。
- `soldier_can_*`：步兵 Mini-PC ↔ C 板协议，依据电控提供的 `视觉对接.md`、`minipc_config.h` 和 `minipc_can.py`。

## 步兵文件职责

| 文件 | 职责 |
| --- | --- |
| `soldier_can_config.hpp` | 协议 ID、版本、DLC 和 100ms 有效期等常量。 |
| `soldier_can_protocol.hpp/.cpp` | 定义协议数据类型；编码 `0x510`，解码 `0x511–0x514`。 |
| `soldier_can_board.hpp/.cpp` | 姿态历史、超时、序号、同组状态快照、会话变化和发送安全检查。 |
| `soldier_can_socketcan.cpp` | 将步兵协议回调接到 Linux SocketCAN，并读取 YAML 中的 `can_interface`。 |
| `socketcan.hpp` | 底层 Linux SocketCAN 收发线程、错误恢复和帧读写；由步兵和原 CBoard 共用。 |
| `CMakeLists.txt` | 将两套实现编进 `io_can` 静态库。 |

## 电控文件逐项映射

### `视觉对接.md`

- 物理层：Classic CAN 2.0A、11 位标准帧、DLC 8、1 Mbit/s。Linux CAN 网卡的波特率仍需在系统侧设置。
- Mini-PC → C 板：`0x510`。字节依次为序号、版本/标志、地面 yaw（mrad）、pitch 编码器原始角（mrad）、参考姿态时间戳。
- C 板 → Mini-PC：`0x511` 姿态、`0x512` 底盘、`0x513` 机器人/弹速状态、`0x514` 热量状态。四帧仅在序号相同时合成一个状态快照。
- 多字节数：小端序；角度在协议线上为 `int16 mrad`，程序结构体中解码为弧度。
- 命令频率：上层按 50–100Hz 提交最新结果；步兵入口限速到最高 100Hz。无目标持续发送 `tracking=0`。
- 超时和新鲜度：100ms；不排队重发旧目标。发送时回显本次图像解算所使用的 `0x511.reference_ms`。
- `yaw`：地面参考 yaw，Mini-PC 输出为弧度，编码范围约 `[-pi, pi]`。
- `pitch_raw`：电机编码器原始角，不是世界系 pitch。`soldier.cpp` 使用 `pitch_encoder_zero_rad` 和 `pitch_encoder_direction` 转换；未标定时禁止发送跟踪目标。
- `robot_id`：1–11 表示红方，101–111 表示蓝方；Mini-PC 据此选敌方颜色，ID 无效时保持空闲。
- 弹速：只有 `bullet_speed_valid` 时传给 Aimer；无效弹速不进入瞄准计算。
- `0x512` 底盘模式/yaw/yaw-rate/相对 yaw 和 `0x514` 热量数据已解码并纳入同序号快照；当前自瞄主链路暂不消费它们，因为 Aimer 只需云台姿态、目标状态和有效弹速。
- `fire_advice`：步兵当前程序固定为 false；最终发射仍由操作手/C 板逻辑控制。
- 下发门控同时要求 `0x511.autoaim_active`（C 板实际接管）和 `0x513.autoaim_requested`（操作手请求），并要求机器人已使能、云台展开、姿态/弹速有效。
- 比赛开始位、显式复位位：当前协议没有与旧 `SerialBoard::is_game_started()` / `reset_pending()` 一一对应的字段。步兵改用 CAN 状态使能/自瞄请求门控，并在通信 session 变化时清空 Tracker。

### `minipc_can.py`

Python 文件是纯编解码参考，不负责打开 CAN 设备。C++ `soldier_can_protocol.cpp` 对应 `pack_command()` 和 `unpack_telemetry()`：相同的 ID、little-endian、有符号字段、单位、边界和 idle 编码。CAN 标准帧标志和 DLC 检查放在 SocketCAN 适配层完成。

### `minipc_config.h`

- `COMMAND_ID`、`TELEMETRY_ID`、`TIMEOUT_MS` 已映射到 `soldier_can_config.hpp`。
- `TASK_PERIOD_MS` 描述 C 板遥测排程，不控制 Mini-PC 的视觉循环。
- `YAW_TARGET_RATE`、`PITCH_TARGET_RATE`、`FIRE_YAW_ERROR`、`FIRE_PITCH_ERROR` 和 `REQUIRE_FIRE_ADVICE` 属于 C 板控制/火控侧参数；当前 Mini-PC 通信层不重复执行这些控制策略。步兵程序暂不发送开火建议。
- `ENABLED` 是电控侧总开关，不等同于 Linux CAN 接口是否启用。

## 接入状态与限制

`src/soldier.cpp` 已使用 `SoldierCanBoard` 收发步兵协议；哨兵原 `CBoard` 保持独立。CAN 姿态只提供 ground yaw 和 pitch encoder，没有 roll/四元数；当前通过标定过的 pitch 零点和方向构造云台到世界旋转，并直接传给 `Solver`，避免把这组云台角再次当作 IMU 姿态套用 `R_gimbal2imubody`。坡面补偿不在此协议能力内。

`pose_at()` 用本机接收时刻近似关联相机帧与 C 板姿态；没有硬件触发或两端时钟同步。没有摄像头或实车时，可以编译通信模块，但不能据此确认相机时间关联、编码器标定或硬件动作正确。
