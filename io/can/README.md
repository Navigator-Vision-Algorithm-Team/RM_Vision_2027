# 步兵 CBoard CAN 通信

`SoldierCBoard` 沿用现有 `CBoard` 的 YAML 配置和 SocketCAN 回调形式，
按电控提供的《视觉对接.md》、`minipc_config.h`、`minipc_can.py` 实现步兵协议。
原有 `CBoard` 类继续提供哨兵接口。

## 协议和数据流

经典 CAN 标准帧，11 位 ID，8 字节数据，小端编码；真实设备接口需配置为 1 Mbit/s。

| ID | 方向 | 内容 |
| --- | --- | --- |
| `0x510` | 小电脑 → 电控 | 跟踪状态、开火建议、目标 yaw、编码器 pitch、参考时间戳 |
| `0x511` | 电控 → 小电脑 | yaw、编码器 pitch、姿态有效位、自瞄/展开状态、时间戳 |
| `0x512` | 电控 → 小电脑 | 底盘模式、yaw、yaw 角速度、相对 yaw |
| `0x513` | 电控 → 小电脑 | 使能、自瞄请求、机器人 ID、弹速及有效位、射击计数 |
| `0x514` | 电控 → 小电脑 | 热量、上限、冷却值及有效位 |

`minipc_codec` 只负责数据编码与解码。`SoldierCBoard` 负责姿态缓存、
序号检查、100ms 有效期、状态分组和发送。Linux 适配层负责打开 SocketCAN 接口。

## 使用接口

```cpp
#include "io/can/soldier_cboard.hpp"

io::SoldierCBoard board(config_path);  // YAML 中配置 can_interface: can0
```

收到 CAN 帧后自动进入接收回调。`snapshot()` 返回最新未过期姿态，
只附带同序号、未过期的其他遥测；缺失字段为 `std::nullopt`。
调用方必须检查姿态、弹速、热量等有效位。

图像处理时保存 `pose_at(image_timestamp)` 返回的样本，发送时使用
`send(command, reference)`。图像时间戳必须与 `steady_clock` 使用同一时间基准。
通信类填写发送序号，回显解算实际使用的样本时间戳。

- 目标角度使用弧度，yaw 范围 `[-pi, pi]`，编码器 pitch 范围 `[-0.25, 0.4]`。
- 上层以 50–100Hz 提交最新结果；持续无目标时也调用 `send({})`。
  通信类自身不启动周期发送线程。
- 无效角度或过期参考会转为空闲指令，清除角度和开火建议，返回相应错误状态。
- `sent` 表示接口接受发送，不能视为电控确认；`transport_error` 表示提交失败。
- `session()` 变化后上层需重置旧跟踪状态；确认电控重启时可调用 `reset()`。
- 注入 `Writer` 的构造函数可用于虚拟收发；Writer 不应回调同一个对象的方法。

## 编译和检查

在已经完成项目环境配置的 Linux 中编译实际收发层：

```bash
cmake --build build-debug --target io_can --parallel 2
```

以下独立检查只要求 C++20 编译器和线程库，无需 ROS2、相机 SDK、
OpenVINO 或 CAN 硬件。在仓库根目录运行：

```bash
cmake -S io/can/tests -B build-debug/can-checks -DCMAKE_BUILD_TYPE=Release
cmake --build build-debug/can-checks --parallel 2
ctest --test-dir build-debug/can-checks --output-on-failure
```

检查覆盖协议样例、角度边界、有符号字段、全部状态位、非法帧、时间戳回显、
100ms 超时、收发序号回绕、会话重置、快照分组、发送失败和并发访问。
Release 模式也会执行检查断言。

## 当前范围与后续工作

当前交付通信模块，尚未接入 `soldier.cpp` 或 `virtual_run.cpp`。
真实 USB-CAN 与电控收发尚需硬件验证。

电控提供 yaw 和编码器 pitch，没有完整四元数，不能直接替代旧的 `imu_at()`。
主程序接入前需确定 pitch 零点、方向和相机外参。
图像关联暂以本机接收时刻近似，未实现硬件触发或两端时钟同步。
协议没有明确启动标识，序号和时间戳回绕无法可靠区分每一次电控重启；
会话自动清理基于超时、时钟倒退和状态变化等可观测条件。
