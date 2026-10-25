# 保留但未使用的代码清单

本文件记录仓库里**仍然保留、但当前不参与构建或没有调用方**的目录与文件，说明它们是干什么的、
为什么不用、以及将来要恢复时怎么改。

分三类：

- **A 类**：完全不参与构建（CMake 里已注释掉）
- **B 类**：参与构建，但没有任何调用方（编译了没人用）
- **C 类**：留在仓库里，但当前入口不会读取（配置、参数）

> 判断依据是当前 `develop` 分支上仍然会生成的入口：
> `mt_standard`、`soldier`、`sentry_multithread`（需 ROS2）、`virtual_run`，
> 以及 `capture` / `calibrate_*` / `split_video` 五个标定工具。

---

## A 类：完全不参与构建

### 1. `tasks/auto_buff/` —— 打符（能量机关）

**是什么**

| 文件 | 作用 |
| --- | --- |
| `buff_type.cpp` | 能量机关类型/状态定义 |
| `buff_detector.cpp`、`yolo11_buff.cpp` | 能量机关识别（YOLO11 检测 + 传统方法） |
| `buff_target.cpp` | 目标建模：大符/小符旋转模型，内含自己的 `Voter` 判断顺/逆时针 |
| `buff_solver.cpp` | PnP 解算能量机关位姿 |
| `buff_aimer.cpp` | 选打哪个叶片、算提前量 |

依赖方向是 `auto_buff → auto_aim`（复用 `Solver`、`YOLO`、PnP 等基础设施）。

**为什么不使用**

1. **入口已经删了**：原先只有 `src/auto_buff_debug.cpp`、`src/auto_buff_debug_mpc.cpp`
   和 `src/uav.cpp` 会创建 `auto_buff::*` 对象，前两个已删除，`uav` 也没启用。
2. **串口协议没有"模式切换"**：`src/mt_standard.cpp` 里保留的原始调用位置上方写着
   `// [CAN] 打符模块，串口协议暂不支持模式切换`。当前下行帧 `0x0402`
   （`io/serial/gimbal/packet.hpp`）只有 `yaw` / `pitch` / `shoot`，没有表达
   "现在打符" 的字段，电控侧也无法切换到打符状态。
3. 当前交付重心是自瞄（步兵 / 哨兵），打符不是本期任务。

**怎么恢复**

```cmake
# 顶层 CMakeLists.txt
add_subdirectory(tasks/auto_buff)
# 并给对应入口目标加上 auto_buff
target_link_libraries(<your_exe> ... auto_buff tools io io_can)
```

若要用 MPC 版打符（原 `auto_buff_debug_mpc`），还需同时恢复下面的 `planner`。

---

### 2. `src/uav.cpp` —— 无人机入口

**是什么**：把自瞄与打符串在同一条主循环里的入口，通过 `io::CBoard`（CAN）收发，
不走 `SerialBoard`（串口）。

**为什么不使用**：目标兵种是步兵 / 哨兵，没有无人机平台；`uav` 目标在
`CMakeLists.txt` 里从未启用过；且它依赖 CAN 版 `CBoard` 与打符模块。

**怎么恢复**

```cmake
add_executable(uav src/uav.cpp)
target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
```

注意它用的是 CAN（`io/can`），不是串口。

---

### 3. `tasks/auto_aim/planner/` 与 `planner/tinympc/` —— MPC 云台平滑

**是什么**

- `planner/tinympc/`：vendored 的 [TinyMPC](https://github.com/TinyMPC/TinyMPC)
  （CMU），用 ADMM 解 MPC 的 QP，纯 C++/Eigen、无外部依赖；附带嵌入式 C 代码生成
  （`codegen.cpp`）与 ADMM 步长 ρ 的调参基准（`rho_benchmark.cpp`）。
- `planner/planner.cpp`：用 TinyMPC 给云台建两个独立的 1 维 MPC——
  状态 `[角度, 角速度]`、输入 `角加速度`、`DT = 10ms`、`N = 100`（1 秒视窗），
  约束是 `±max_yaw_acc` / `±max_pitch_acc`。参考轨迹由 `aim()` 每 10ms 预测目标
  得到，求解后取窗口正中间（当前时刻）的角度下发；并用"参考角 vs 平滑角"的偏差
  小于 `fire_thresh` 作为开火条件。

  简言之：**不是"指哪打哪"，而是"以不超过电机加速能力的方式贴着目标未来 1 秒的
  轨迹走"**，用来抑制云台抖动与超调。

**为什么不使用**

1. **没有状态反馈**：求解器的初始状态取的是参考轨迹起点的状态，而不是实测云台角度
   （`planner.cpp` 的 `plan()` 里 `x0 << traj(0, 0), traj(1, 0)`）。所以它本质上是
   **开环轨迹整形**，不是闭环控制。
2. **协议传不出前馈**：`io::Command` 只有 `yaw` / `pitch`
   （`io/command.hpp`），`Plan` 里算出来的 `yaw_vel / yaw_acc / pitch_vel / pitch_acc`
   没有字段可发。少了前馈，它就退化成"平滑器"，而平滑的代价是相位滞后。
3. **时间没对齐**：`delay_time` 用的是写死的经验常数
   （`high_speed_delay_time` / `low_speed_delay_time`），`SendPacket.timeseries`
   是恒 0 的占位字段（注释："接收方不使用"）。视觉与电控的时间基没有对齐，
   基于预测的收益会被延迟误差吃掉。
4. **纯 CPU 开销**：每帧 yaw、pitch 各解一次 ADMM（`N = 100`、`max_iter = 10`）。
   x86 上无所谓，ARM 上是实打实的负担，而换来的只有平滑。
5. 唯一会实例化 `Planner` 的 `*_mpc` 入口（`standard_mpc`、`auto_aim_debug_mpc`、
   `auto_buff_debug_mpc`）已删除。

**如果要做成真正的 MPC，还缺两件事**：把实测云台姿态喂进 `x0`（上行 `0x502` 的
四元数可以用），以及在下行帧里增加角速度/角加速度字段（需要电控固件同步改），
再统一双端时间戳。

**怎么恢复**

```cmake
# tasks/auto_aim/CMakeLists.txt
add_subdirectory(planner/tinympc)                       # 取消注释
# 源列表里加回 planner/planner.cpp
target_link_libraries(auto_aim io openvino::runtime tinympcstatic)
```

配置侧需要补齐 `Q_yaw / R_yaw / max_yaw_acc / Q_pitch / R_pitch / max_pitch_acc /
fire_thresh / yaw_offset`（可直接参考 `configs/standard3.yaml`）。

---

## B 类：参与构建但没有调用方

### 4. `tasks/auto_aim/multithread/commandgener.*`

**是什么**：`CommandGener` 起一个独立线程，把 `Aimer` 的结果推给 `CBoard`（CAN）
并按需绘图。

**为什么不使用**：它依赖 CAN 版 `CBoard`；改串口后，同样的工作在 `mt_standard`
主循环里内联完成了。`src/mt_standard.cpp` 里对应代码整段注释：

```cpp
// [CAN] CommandGener 依赖 CBoard，串口版在内联完成
// auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter);
```

**现状**：仍在 `tasks/auto_aim/CMakeLists.txt` 的源列表里，会编译但从不被实例化。
想彻底停编，删掉源列表里的 `multithread/commandgener.cpp` 即可。

---

### 5. `tasks/auto_aim/voter.*`

**是什么**：`auto_aim::Voter`，按历史角度投票判断装甲板旋转方向。

**为什么不使用**：全仓库搜不到任何实例化点。`auto_buff` 里有另一份独立的
`Voter`（定义在 `buff_target.hpp`），两份同名但互不相关。属于重构后的遗留物。

**现状**：仍随 `auto_aim` 编译。想停编，删掉源列表里的 `voter.cpp`。

---

## C 类：留在仓库但当前入口不读取

### 6. `configs/` 下的历史配置

所有入口的配置都是命令行传入的（如 `./bin/mt_standard configs/mtstandard.yaml`），
没有硬编码默认值。因此下面这些配置**不对应任何一个现存入口**，保留作为参数参考
（标定值、调参经验都在里面，换车时经常直接拷来改）：

| 配置 | 原使用者 |
| --- | --- |
| `standard3.yaml` / `standard4.yaml` | 已删除的 `standard` / `standard_mpc` |
| `demo.yaml` / `example.yaml` | 早先的演示与最小示例 |
| `mvs.yaml` / `ascento.yaml` | 迈德威视相机车、Ascento 底盘的旧配置 |
| `uav.yaml` | 未启用的 `src/uav.cpp` |
| `sentry_perf.yaml` / `sentry_example.yaml` | 哨兵性能测试与示例 |

仍在使用的：`mtstandard.yaml`（`mt_standard`）、`soldier.yaml`（`soldier`）、
`sentry.yaml` / `sentry_virtual.yaml`（`sentry_multithread`）、
`virtual_*.yaml`（`virtual_run`）。

### 7. "睡着"的 MPC 参数

`standard3.yaml`、`standard4.yaml`、`demo.yaml` 里保留了完整的
`Q_yaw / R_yaw / max_yaw_acc / Q_pitch / R_pitch / max_pitch_acc / fire_thresh`。
这些键只在 `Planner` 构造时被读取，而 `Planner` 已不参与构建——
**现在它们既不生效、也不会报错**（在役的 `mtstandard.yaml` 已只剩 `yaw_offset`）。
恢复 MPC 时可以直接复用。

---

## 附：看起来没用、其实在用的东西（别删）

| 路径 | 用途 |
| --- | --- |
| `io/sim/` | 虚拟数据源（`VirtualMcu`、伪相机、录制/回放），`virtual_run` 和 `source_mode: fake/replay` 都依赖它 |
| `sim/virtual_mcu.py` | 可选的**外部**虚拟下位机（pty 伪串口），`fake_serial_external: true` 时使用；不做真机也能联调 |
| `tools/trajectory.hpp` | 弹道解算（飞行时间、下坠），`aimer.cpp` 在用 |
| `calibration/` | 5 个标定工具，都在构建并安装 |
| `3rd_part/openvino_2024.6.0/` | 两套内置 OpenVINO 运行库，构建时按架构取用 |

---

## 附：本期删除的入口（对照 git 历史）

`src/standard.cpp`、`src/standard_mpc.cpp`、`src/auto_aim_debug_mpc.cpp`、
`src/mt_auto_aim_debug.cpp`、`src/auto_buff_debug.cpp`、`src/auto_buff_debug_mpc.cpp`、
`src/sentry_multithread_debug.cpp`、`src/uav_debug.cpp`。

它们原先都只是"调试/对比"入口（单线程版、MPC 版、打符版），
功能分别被 `mt_standard`（多线程 + 串口）与 `sentry_multithread` 覆盖。
