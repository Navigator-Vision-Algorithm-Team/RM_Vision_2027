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
| `soldier_can_damiao.cpp` | 通过达妙 SDK 打开 USB 转接器，配置经典 CAN 2.0、1 Mbps，并收发标准数据帧。 |
| `socketcan.hpp` | 原 CBoard 使用的 Linux SocketCAN 收发和错误恢复。 |
| `CMakeLists.txt` | 分别构建 `io_soldier_can` 和 `io_can`。 |

## 在小电脑上构建步兵程序

在项目根目录运行以下命令。达妙 SDK 使用官方仓库的 v1.1.0 C/C++ 库，CMake 按目标架构选取 `.so`。

```bash
git clone https://github.com/dmBots/dm-device-sdk.git "$HOME/dm-device-sdk"
git -C "$HOME/dm-device-sdk" checkout --detach 36a8b3971dc35608701d128f353a32cad3ee6cdb
cmake -S . -B build -DDMCAN_SDK_ROOT="$HOME/dm-device-sdk"
cmake --build build --target soldier -j2
```

小电脑需要能通过达妙 SDK 打开转接器（使用兼容的 USB 固件模式，并按厂商说明给当前用户配置 USB 访问权限）。只连接一个达妙设备，在 YAML 设置 `can_channel: 0`；双通道设备按实际接线可用 `1`。程序启动后设置并回读经典 CAN 2.0、1 Mbps，无需创建 `can0`。如果转接器中途拔掉，重新插上后重启 `soldier`。

### Ubuntu 22.04 运行环境（libusb ≥ 1.0.27）

达妙 `libdm_device.so` 使用 `libusb_init_context`（libusb 1.0.27 起提供）。Ubuntu 22.04 自带 1.0.25，运行时报：

```
symbol lookup error: .../libdm_device.so: undefined symbol: libusb_init_context
```

用脚本一次性配置（新版 libusb 只装到 `/usr/local`，不覆盖系统库；顺带为当前用户写入 udev 规则）：

```bash
./io/can/setup_damiao_env.sh                    # 检查 + 安装 libusb 1.0.27 + 写 udev 规则
./io/can/setup_damiao_env.sh --check            # 只检查，不改动系统
VID=xxxx PID=xxxx ./io/can/setup_damiao_env.sh  # 自动识别不到设备时手动指定 VID:PID
```

配置完成后**无需重新编译**，直接运行即可。脚本最后会用 `ldd -r` 校验 `libdm_device.so` 的依赖符号是否全部可解析。

## 步兵 Foxglove 调试

在运行 CMake 前先加载小电脑所用 ROS 2 发行版的 `setup.bash`，再按上文构建 `soldier`。启动程序和桥接的两个终端都加载同一 ROS 2 环境，并使用相同的 `ROS_DOMAIN_ID`。

```bash
# 终端一，启动步兵调试配置
cmake --build build --target soldier_debug -j2
./build/tests/soldier_debug

# 终端二，启动桥接
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

在 Foxglove 连接 `ws://<小电脑IP>:8765`，查看 `/sp_debug/soldier/image` 和 `/sp_debug/soldier/status`。调试配置沿用达妙 CAN 2.0 收发，ROS 2 负责发布调试数据。`debug_ros2.enable: true` 时，如果调试流无法初始化，程序会报错退出。`waiting_pose` 表示 YOLO 已有结果，程序正在等可用于该帧的 `0x511` 姿态。

## 电控文件逐项映射

### `视觉对接.md`

- 总线格式：Classic CAN 2.0A、11 位标准帧、DLC 8、1 Mbit/s。波特率由步兵程序通过达妙 SDK 设置。
- Mini-PC → C 板：`0x510`。字节依次为序号、版本/标志、地面 yaw（mrad）、pitch 编码器原始角（mrad）、参考姿态时间戳。
- C 板 → Mini-PC：`0x511` 姿态、`0x512` 底盘、`0x513` 机器人/弹速状态、`0x514` 热量状态。四帧仅在序号相同时合成一个状态快照。
- 多字节数：小端序；角度在协议线上为 `int16 mrad`，程序结构体中解码为弧度。
- 命令频率：上层按 50–100Hz 提交最新结果；步兵入口限速到最高 100Hz。无目标持续发送 `tracking=0`。
- 超时和新鲜度：100ms；不排队重发旧目标。发送时回显本次图像解算所使用的 `0x511.reference_ms`。
- `yaw`：地面参考 yaw，Mini-PC 输出为弧度，编码范围约 `[-pi, pi]`。
- `pitch_raw`：电机编码器原始角。步兵使用 `pitch_encoder_zero_rad` 和 `pitch_encoder_direction` 转换，并限制在协议范围内。
- 步兵默认跟随 YOLO 返回的第一个装甲板，使用 PnP 位置计算角度，PnP 无法解算时使用检测中心的视线方向。
- `0x512`、`0x513`、`0x514` 仍可解码。步兵发送链路使用 `0x511` 的姿态和参考时间戳。
- `fire_advice`：步兵当前程序固定为 false；最终发射仍由操作手/C 板逻辑控制。
- 实际接管由 C 板决定。C 板接受跟踪指令后回报 `0x511.autoaim_active=1`，步兵在终端记录接管状态变化。

### `minipc_can.py`

Python 文件是纯编解码参考，不负责打开 CAN 设备。C++ `soldier_can_protocol.cpp` 对应 `pack_command()` 和 `unpack_telemetry()`：相同的 ID、little-endian、有符号字段、单位、边界和 idle 编码。达妙适配层检查通道、帧格式和 DLC。

### `minipc_config.h`

- `COMMAND_ID`、`TELEMETRY_ID`、`TIMEOUT_MS` 已映射到 `soldier_can_config.hpp`。
- `TASK_PERIOD_MS` 描述 C 板遥测排程，不控制 Mini-PC 的视觉循环。
- `YAW_TARGET_RATE`、`PITCH_TARGET_RATE`、`FIRE_YAW_ERROR`、`FIRE_PITCH_ERROR` 和 `REQUIRE_FIRE_ADVICE` 属于 C 板控制/火控侧参数；当前 Mini-PC 通信层不重复执行这些控制策略。步兵程序暂不发送开火建议。
- `ENABLED` 是电控侧总开关，不等同于 Linux CAN 接口是否启用。

## 接入状态与限制

`src/soldier.cpp` 已使用 `SoldierCanBoard` 收发步兵协议；哨兵原 `CBoard` 保持独立。CAN 姿态只提供 ground yaw 和 pitch encoder，没有 roll/四元数；当前通过标定过的 pitch 零点和方向构造云台到世界旋转，并直接传给 `Solver`，避免把这组云台角再次当作 IMU 姿态套用 `R_gimbal2imubody`。坡面补偿不在此协议能力内。

`pose_at()` 用本机接收时刻近似关联相机帧与 C 板姿态。已有前后两个有效样本时插值，否则用图像时刻之前最近的有效样本。回传给 C 板的参考时间戳始终来自真实 CAN 样本。实际时间对齐仍取决于 USB 延迟，精确同步需要硬件触发或两端时钟同步。
Foxglove 的 `/sp_debug/soldier/status` 会记录调试采样时最近一次 `can_tx`，其中序号、角度、标志和回显时间来自实际编码后的 `0x510`。结果码依次为 0 正常、1 参考过期后改发空闲、2 角度无效后改发空闲、3 提交失败、4 控制条件失效后改发空闲。`sdk_accepted` 只表示 SDK 接受提交。完整收发和接收频率仍以 C 板或总线记录为准。

`frame_us` 和 `can_tx.attempt_us` 都使用 Mini-PC 的单调时钟。`frame_to_attempt_ms` 从本机图像时间戳算到提交 CAN 之前。`observed_armor_world_m` 记录所跟随装甲板的 PnP 世界坐标。
