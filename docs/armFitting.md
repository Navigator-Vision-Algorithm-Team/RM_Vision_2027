# ARM 架构适配文档（armFitting.md）

> 分支：`pure_cpp_Serial_port_Arm`
> 目标：将项目从 x86_64 平台迁移到 ARM（aarch64）平台编译并运行
> 分析结论：代码层几乎没有架构相关硬伤，主要工作量在**工具链安装、OpenVINO 版本、推理设备配置、运行时库路径**四个方面。

---

## 0. 重要决策：OpenVINO 内置到 `3rd_part/`

**OpenVINO 不通过系统安装（不使用 `/opt/intel`），直接放入项目根目录 `3rd_part/`**，作为随项目分发的第三方预编译依赖。理由：

- x86 / ARM 各自在 `3rd_part/` 中放置对应架构的 OpenVINO，源码与 CMake 完全一致，**无需架构分支**；
- 换机器、重装系统无需重新安装配置 OpenVINO，放置目录即可编译；
- 版本锁定在项目内，避免系统升级导致行为漂移。

目标结构：

```
RM_Vision_2027/
├── 3rd_part/                          # 第三方预编译依赖（架构相关二进制）
│   └── openvino_2024.6.0/               # OpenVINO 2024.6.0（aarch64 版）
│       └── runtime/
│           ├── cmake/                   # find_package(OpenVINO) 入口
│           ├── include/                 # C++ 头文件
│           └── lib/aarch64/             # 运行库 + plugins/（CPU 插件所在）
├── io/
│   ├── hikrobot/lib/arm64/libMvCameraControl.so    # 海康 SDK（已内置）
│   └── mindvision/lib/arm64/libMVSDK.so            # 迈德威视 SDK（已内置）
└── ...
```

> 注意：`3rd_part/` 实际体积约 102MB（arm64 版仅含 CPU 插件），已加入 `.gitignore`，不要直接提交二进制进 git。

---

## 1. 现状分析

### 1.1 分支状态

`git diff pure_cpp_Serial_port pure_cpp_Serial_port_Arm` 为空，即当前分支与上一分支内容**完全一致**，尚未开始任何 ARM 适配修改。

### 1.2 项目在 x86 上的运行方式（基线）

- 平台：x86_64 + Ubuntu 20.04/22.04
- 编译器：g++（C++17）
- 构建：CMake ≥ 3.16，`cmake -B build && make -C build -j$(nproc)`
- 推理：OpenVINO 2024.6.0（原为 x86 系统安装，现改为 `3rd_part/` 内置），部分配置使用 `device: GPU`（Intel 核显）

---

## 2. 依赖总览与 ARM 适配性分析

### 2.1 系统 / apt 依赖（均需在 ARM 上重新安装）

| 依赖 | 用途 | ARM(aarch64) 适配性 |
|---|---|---|
| `libopencv-dev` | 图像处理、`cv::dnn` 分类器 | 无问题，apt 原生 arm64 包 |
| `libeigen3-dev` | 线性代数（纯模板库） | 无问题，头文件库 |
| `libfmt-dev` | 日志格式化 | 无问题 |
| `libspdlog-dev` | 日志 | 无问题 |
| `libyaml-cpp-dev` | 配置文件解析 | 无问题 |
| `nlohmann-json3-dev` | JSON | 无问题，纯头文件 |
| `libusb-1.0-0-dev` | MindVision 相机 USB reset | 无问题，apt 原生 arm64 包 |
| `can-utils` | SocketCAN 命令行工具 | 无问题（`linux/can.h` 为内核 API，ARM Linux 同样支持） |
| `cmake / g++ / git / screen / openssh-server` | 工具链 | 无问题 |

### 2.2 工业相机 SDK（仓库内置预编译 .so）

| SDK | 库文件 | amd64 | arm64 | 动态依赖（arm64 实测 readelf） |
|---|---|---|---|---|
| 海康机器人 HikRobot MVS | `io/hikrobot/lib/*/libMvCameraControl.so` | 已有 | **已有** | `libpthread / libdl / libstdc++ / libm / libgcc_s / libc` |
| 迈德威视 MindVision | `io/mindvision/lib/*/libMVSDK.so` | 已有 | **已有** | `librt / libpthread / libdl / libm / libc` |

**结论：两个相机 SDK 的 arm64 库均已随仓库提供**，只依赖 glibc 基础库，无需额外安装运行库。这是本项目适配工作量小的重要原因。

仍需注意：

- 运行时必须让动态链接器找到这两个 `.so`（见 §5.4）。
- 相机为 USB 设备，ARM 工控机/开发板上需要配置 **udev 规则**（否则无权限访问 `/dev/bus/usb/*`），且 USB 控制器建议使用原生 USB3 端口。

### 2.3 OpenVINO 2024.6.0（**适配重点，已决策：内置 `3rd_part/`**）

| 项目 | x86 现状 | ARM 要求 |
|---|---|---|
| 获取方式 | 原为系统安装 `/opt/intel/openvino_2024.6.0/`（x86_64 版） | **已完成**：下载官方 ubuntu20 arm64 包（33MB 压缩/102MB 解压，SHA256 已验证），解压到 `3rd_part/openvino_2024.6.0/` |
| 可用推理设备 | CPU / GPU(iGPU) | **仅 CPU**（ARM 上无 Intel GPU/NPU 插件） |
| CMake 查找路径 | 硬编码 `/opt/intel/openvino_2024.6.0/runtime/cmake`（4 处，见 §3） | 改为 `${CMAKE_SOURCE_DIR}/3rd_part/openvino_2024.6.0/runtime/cmake`（见 §5.1） |

关键限制：

- **OpenVINO 的安装包是架构绑定的**，x86 上解压好的 OpenVINO 目录**不能**直接拷贝到 ARM 使用，必须单独下载 aarch64 版本；
- `runtime/lib/aarch64/` 下的 `plugins/` 子目录（CPU 插件）必须随库一起分发，不能只拷 `libopenvino.so`；
- 内置后 4 处 CMakeLists 的硬编码路径必须同步修改，否则 `find_package(OpenVINO)` 直接失败。

### 2.4 模型文件（架构无关，可直接复用）

| 文件 | 格式 | ARM 可用性 |
|---|---|---|
| `assets/yolo11.xml + yolo11.bin`、`assets/yolov5.*`、`assets/yolov8.*`、`assets/yolo11_buff_int8.*` | OpenVINO IR | 可用。IR 是中间表示，跨架构通用，ARM 上由 CPU 插件即时编译 |
| `assets/tiny_resnet.onnx`、`assets/best2-sim.onnx` | ONNX | 可用。`cv::dnn::readNetFromONNX` 与 OpenVINO 均可直接读取 |

> 提示：`yolo11_buff_int8` 是 INT8 量化 IR，量化标定在 x86 CPU 上完成，在 ARM CPU 上精度通常可接受，但建议实测确认。

### 2.5 ROS2（可选，仅哨兵多线程目标需要）

以下目标在检测到 ROS2 时才编译，其余全部目标不依赖 ROS2：

- `src/sentry_multithread.cpp`、`src/sentry_multithread_debug.cpp`
- `io` 库中的 `ros2/publish2nav.cpp`、`ros2/ros2.cpp`、`ros2/subscribe2nav.cpp`

ARM 上若使用 Ubuntu 22.04 arm64，可直接安装官方 ROS2 Humble arm64 包；若不需要哨兵链路，可完全不装 ROS2。

### 2.6 Ceres Solver（名存实亡的依赖）

- `tasks/auto_buff/CMakeLists.txt` 中 `find_package(Ceres REQUIRED)`，根 `CMakeLists.txt` 的 `auto_buff_debug*` 链接 `${CERES_LIBRARIES}`。
- 但全项目源码中**没有任何 `#include <ceres/...>`**（已全局搜索确认）。
- 处置建议二选一：
  1. 安装 `libceres-dev`（Ubuntu 22.04 arm64 为 Ceres 2.0.0）保持构建不变；
  2. 从 CMakeLists 中移除 Ceres 相关内容（推荐，可减少一个大型依赖）。

### 2.7 仓库内置源码库（无外部依赖，无适配问题）

- `io/serial/`：wjwwood/serial 的 vendored 副本（`rt/pthread`）。
- `tasks/auto_aim/planner/tinympc/`：vendored TinyMPC 静态库。
- 均为纯 C++ 源码编译，与架构无关。

---

## 3. CMakeLists 逐文件分析

### 3.1 根 `CMakeLists.txt`

| 行号 | 内容 | ARM 影响 |
|---|---|---|
| 11-16 | `find_package`: OpenCV / fmt / Eigen3 / spdlog / yaml-cpp / nlohmann_json | 均需 apt 安装 arm64 版本 |
| **19** | 原 `set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")` | **已修改**：改为架构感知路径解析，ARM 指向 `3rd_part/`（见 §5.1） |
| 20 | `find_package(OpenVINO REQUIRED)` | 依赖上一条路径 |
| 44-50 | 可执行文件链接 `${CERES_LIBRARIES}` | 根作用域中该变量为空（`find_package(Ceres)` 只在子目录），当前依赖 OBJECT 库传递链接，能工作但脆弱；与架构无关 |
| 109-131 | ROS2 条件编译 | 无 ROS2 时自动跳过，ARM 上行为一致 |

### 3.2 `io/CMakeLists.txt`

| 行号 | 内容 | ARM 影响 |
|---|---|---|
| **28-34** | `CMAKE_SYSTEM_PROCESSOR` 匹配 `aarch64` → 链接 `hikrobot/lib/arm64` | **已适配** ✅ |
| **38-44** | 同上，mindvision → `mindvision/lib/arm64` | **已适配** ✅ |
| 46 | `target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial)` | 需 `libusb-1.0-0-dev`；两个 SDK 库在对应 arm64 目录中存在 |
| 48-79 | ROS2 条件编译 | 可选 |

### 3.3 `tasks/auto_aim/CMakeLists.txt`、`tasks/auto_buff/CMakeLists.txt`、`tasks/omniperception/CMakeLists.txt`

- 三处同样硬编码过 `set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")`（分别位于第 3/2/3 行），**已一并改为与根 CMakeLists 相同的架构感知逻辑**（见 §5.1）。
- `auto_buff` 额外 `find_package(Ceres REQUIRED)`（见 §2.6）。
- 其余内容与架构无关。

### 3.4 `io/serial/CMakeLists.txt`、`tools/CMakeLists.txt`、`tasks/auto_aim/planner/tinympc/CMakeLists.txt`

无架构相关内容，无需改动。

---

## 4. 代码层分析

- **无 x86 SIMD intrinsics**：全局搜索 `immintrin.h / _mm_* / __x86_64__ / cpuid` 等仅命中海康 SDK 头文件 `MvISPErrorDefine.h` 内的错误码注释，无实际使用。
- **无端序相关代码**：未发现手写 `htole/le32` 等字节序处理，串口协议均按小端主机序直接 memcpy，ARM/x86 均为小端，行为一致。
- **SocketCAN**（`io/can/socketcan.hpp`）：使用内核 `linux/can.h` + epoll，内核 API 跨架构一致。
- **相机 SDK 调用**：海康仅枚举 `MV_USB_DEVICE`（USB 相机），MindVision 用 libusb reset，均无架构差异。
- **线程 / 时间戳**：`std::thread`、`std::chrono::steady_clock` 自建时间戳体系（见 README 约定 4），与平台无关。

---

## 5. 需要改动清单

### 5.1 【已完成】OpenVINO 内置 `3rd_part/` + 修改 4 处 CMakeLists 路径

> **后续变更（x86 / arm 分支合并后）**：本节描述的"4 处 CMakeLists 各写一份架构判断"
> 已被收敛为**唯一**的 `cmake/platform.cmake`。根 CMakeLists 只 `include(platform)`，
> 各子目录直接使用 `SP_HIKROBOT_LIB_DIR` / `SP_MINDVISION_LIB_DIR` / `SP_OPENVINO_*`
> 等变量，不再出现 `CMAKE_SYSTEM_PROCESSOR` 判断；x86 与 aarch64 共用同一份源码。
> OpenVINO 目录也由扁平的 `3rd_part/openvino_2024.6.0/runtime` 调整为按架构分目录的
> `3rd_part/openvino_2024.6.0/{aarch64,x86_64}/runtime`，x86_64 一套同样入库。
> 本节其余内容保留为当时的适配记录。

**步骤 1：放置 OpenVINO aarch64 版（已完成）**

目标设备为 Ubuntu 26.04 arm64，但官方 2024.6.0 的 arm64 构建只有 **Ubuntu 20.04** 一版
（新版 2025.3+ 才有 ubuntu22 arm64 构建，官方 ARM 构建最高只到 Ubuntu 22.04）。
由于 glibc 向后兼容，ubuntu20 构建的二进制可在 26.04 正常运行，因此保持项目锁定的 2024.6.0：

```bash
# 实际下载并校验（SHA256 已验证通过）
URL="https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux"
PKG="l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_arm64.tgz"   # 33MB 压缩 / 102MB 解压
mkdir -p 3rd_part && cd 3rd_part
curl -fSL -o openvino_arm64.tgz "$URL/$PKG"
sha256sum -c   # e094e7c3e3a3931213aeeb6d6abb6cc4d654f5cb4df8da2914158ba62e25e33a
tar -xzf openvino_arm64.tgz
mv l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_arm64 openvino_2024.6.0
rm openvino_arm64.tgz
```

已核实：`runtime/lib/aarch64/libopenvino_arm_cpu_plugin.so` 为 aarch64 ELF；
`libopenvino.so` 仅依赖 glibc/libstdc++/libtbb；TBB 已随包附带（`runtime/3rdparty/tbb/lib`）。

**步骤 2：修改 4 处硬编码路径（已完成）**

根 `CMakeLists.txt`（第 19 行）、`tasks/auto_aim/CMakeLists.txt`（第 3 行）、
`tasks/auto_buff/CMakeLists.txt`（第 2 行）、`tasks/omniperception/CMakeLists.txt`（第 3 行），
统一替换为以下架构感知逻辑：

```cmake
# OpenVINO 路径解析（详见 armFitting.md §5.1）：
# - ARM(aarch64)：使用项目内置 3rd_part/openvino_2024.6.0（架构相关二进制，不进 git）
# - x86 手动解压安装：/opt/intel/openvino_2024.6.0
# - x86 apt 安装（libopenvino-dev）：不设置 OpenVINO_DIR，由 find_package 在系统路径搜索
# 任意场景均可通过 -DOpenVINO_DIR=<path> 覆盖
if(NOT DEFINED OpenVINO_DIR)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(OpenVINO_DIR "${CMAKE_SOURCE_DIR}/3rd_part/openvino_2024.6.0/runtime/cmake")
    elseif(EXISTS "/opt/intel/openvino_2024.6.0/runtime/cmake")
        set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
    endif()
endif()
```

要点：

- 用 `CMAKE_SOURCE_DIR`（始终指向顶层源码目录）而非 `PROJECT_SOURCE_DIR`，保证子目录 CMakeLists 中同样有效；
- 三场景兼容：ARM → `3rd_part/`；x86 手动解压安装 → `/opt/intel`（保持旧行为）；x86 apt 安装（如本机的 `libopenvino-dev-2026.2.0`）→ 不设变量，`find_package` 自动在系统路径找到 `/usr/lib/cmake/openvino*`；
- 全程可用 `-DOpenVINO_DIR=<path>` 覆盖，无需改代码。

**步骤 3：版本管理（已完成）**

- `3rd_part/` 已加入 `.gitignore`（实际体积 102MB，远小于预估的 1GB+，但仍不应进 git 历史）；
- ARM 设备部署时按步骤 1 的脚本重新下载即可（脚本已固化 URL 与 SHA256，可复现）；
- 后续如需内网分发，只需把 URL 换成内网 Nexus/MinIO 地址。

### 5.2 【已完成】修改配置文件中的推理设备

OpenVINO 在 ARM 上只有 CPU 插件，`device: GPU` 会直接抛异常。**已完成**：以下 5 个配置文件的 `device` 均已改为 `CPU`：

| 文件 | 原值 |
|---|---|
| `configs/ascento.yaml` | `GPU` → `CPU` |
| `configs/standard3.yaml` | `GPU` → `CPU` |
| `configs/sentry.yaml` | `GPU` → `CPU` |
| `configs/standard4.yaml` | `GPU` → `CPU` |
| `configs/mvs.yaml` | `GPU` → `CPU` |

（`example.yaml`、`demo.yaml`、`mtstandard.yaml`、`uav.yaml`、`sentry_example.yaml` 原本已是 `CPU`。`classifier.cpp` 使用 `"AUTO"` 设备，在 ARM 上自动回退到 CPU 插件，无需改动。）

| 文件 | 当前值 |
|---|---|
| `configs/ascento.yaml` | `GPU` |
| `configs/standard3.yaml` | `GPU` |
| `configs/sentry.yaml` | `GPU` |
| `configs/standard4.yaml` | `GPU` |
| `configs/mvs.yaml` | `GPU` |

（`example.yaml`、`demo.yaml`、`mtstandard.yaml`、`uav.yaml`、`sentry_example.yaml` 已是 `CPU`，无需改。）

### 5.3 【必须】README 补充 ARM 依赖安装指令

见 README「3.2 编译方式 → 安装依赖项」（已更新）：apt 清单不变；OpenVINO 改为下载 aarch64 包放置到 `3rd_part/`；相机 SDK 说明 arm64 库已内置、只需 udev 规则与运行时库路径。

### 5.4 【必须】运行时动态库路径

相机 SDK 与 OpenVINO 的运行库均在项目内（编译期靠 `target_link_directories` 链接），但**运行期**动态链接器默认不知道这些路径。推荐 ldconfig 一次性配置：

```bash
echo "$(pwd)/3rd_part/openvino_2024.6.0/runtime/lib/aarch64" | sudo tee /etc/ld.so.conf.d/openvino.conf
echo "$(pwd)/3rd_part/openvino_2024.6.0/runtime/3rdparty/tbb/lib" | sudo tee -a /etc/ld.so.conf.d/openvino.conf
echo "$(pwd)/io/hikrobot/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvs.conf
echo "$(pwd)/io/mindvision/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvsdk.conf
sudo ldconfig
```

注意：

- **TBB 必须配置**：`libopenvino.so` 依赖 `libtbb.so.12`，包内自带 aarch64 版 TBB（`runtime/3rdparty/tbb/lib/libtbb.so.12.13`）；若系统装有 `libtbb2` 也可替代，但内置方式最省事；
- OpenVINO 的 `runtime/lib/aarch64` 下的 `plugins/`（CPU 插件）必须整目录可被找到，只链 `libopenvino.so` 会在推理时找不到插件；
- 相机 SDK 的 `.so` 路径含 `$PWD`，ldconfig 配置写在绝对路径里，换部署目录后需重新生成 conf。

### 5.5 【必须】相机 udev 规则（USB 设备权限）

ARM 开发板/工控机同样需要，否则相机打开失败（`MV_E_ACCESS_DENIED` 类错误）：

- 海康：安装 MVS 自带的 udev 规则（如 `51-mvs.rules`，内容为海康相机 VID/PID 的 `MODE="0666"`）；
- 迈德威视：安装其 SDK 的 `*.rules`；
- 无桌面环境时这一点很容易被忽略，务必确认。

### 5.6 【已完成】Ceres 依赖处置

见 §2.6。已确认全项目源码无 `#include <ceres/...>` 或 `ceres::` 使用，因此**已移除**该依赖：
- 删除 `tasks/auto_buff/CMakeLists.txt` 的 `find_package(Ceres REQUIRED)` 与 `target_link_libraries(... ${CERES_LIBRARIES})`
- 删除根 `CMakeLists.txt` 第 60-61 行 `auto_buff_debug*` 中的 `${CERES_LIBRARIES}`

移除后 ARM 全新系统少一个 apt 依赖（`libceres-dev`），构建更健壮。

### 5.7 【建议】部署脚本硬编码路径

- `autostart.sh` 中写死 `~/Desktop/sp_vision_25/` 与不存在的 `watchdog.sh`（未随仓库分发）。ARM 部署时需按实际路径修改，或将 `watchdog.sh` 纳入仓库。

### 5.8 【验证】推理精度与性能

- INT8 量化 IR（`yolo11_buff_int8`）在 ARM CPU 上的精度需实测；
- ARM CPU 无 AVX-512/AVX2 指令集，OpenVINO 会退化为 ARM 优化内核（fp16/neon），延迟与 x86 不同，需重新评估帧率预算；
- 若 SoC 带 NPU（RK3588 / Jetson Orin 等），OpenVINO 无对应插件，需另行评估（换 TensorRT / RKNN），属于超出本清单的额外工作。

---

## 6. 目标平台假设

- OS：**Ubuntu 26.04 LTS arm64 / aarch64**（全新系统，无预装）
- 推理后端：项目内置 `3rd_part/openvino_2024.6.0`（官方 ubuntu20 arm64 构建，glibc 兼容 26.04），`device: CPU`
- 相机：HikRobot USB 工业相机（主）/ MindVision（备）
- 不需要 ROS2 时可跳过全部 ROS 目标（CMake 自动跳过）

---

## 7. ARM 编译部署流程（汇总）

```bash
# 1) 系统依赖（完整清单见 README）
sudo apt install -y git g++ cmake can-utils libopencv-dev libfmt-dev libeigen3-dev \
    libspdlog-dev libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev libceres-dev \
    openssh-server screen

# 2) 放置 OpenVINO（ubuntu20 arm64 构建，可在 26.04 运行）到 3rd_part/
mkdir -p 3rd_part && cd 3rd_part
curl -fSL -o openvino_arm64.tgz "https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_arm64.tgz"
echo "e094e7c3e3a3931213aeeb6d6abb6cc4d654f5cb4df8da2914158ba62e25e33a  openvino_arm64.tgz" | sha256sum -c -
tar -xzf openvino_arm64.tgz && rm openvino_arm64.tgz
mv l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_arm64 openvino_2024.6.0
# 仅 C++ 推理需要 runtime/，删除非必需目录（docs/python/samples/install_dependencies/setupvars.sh），体积降至约 70MB
rm -rf openvino_2024.6.0/docs openvino_2024.6.0/install_dependencies \
       openvino_2024.6.0/python openvino_2024.6.0/samples openvino_2024.6.0/setupvars.sh
cd ..

# 3) 运行时库路径（OpenVINO + TBB + 两个相机 SDK）
echo "$(pwd)/3rd_part/openvino_2024.6.0/runtime/lib/aarch64" | sudo tee /etc/ld.so.conf.d/openvino.conf
echo "$(pwd)/3rd_part/openvino_2024.6.0/runtime/3rdparty/tbb/lib" | sudo tee -a /etc/ld.so.conf.d/openvino.conf
echo "$(pwd)/io/hikrobot/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvs.conf
echo "$(pwd)/io/mindvision/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvsdk.conf
sudo ldconfig

# 4) udev 规则（海康/迈德威视各自的 .rules 拷入 /etc/udev/rules.d/）
sudo cp <SDK中的>.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger

# 5) 配置文件 device: GPU -> CPU（见 §5.2）

# 6) 编译（4 处 CMakeLists 已按 §5.1 改为架构感知路径，ARM 自动走 3rd_part）
cmake -B build && make -C build -j$(nproc)

# 7) 运行
./build/standard configs/standard3.yaml
```

---

## 8. 风险与检查清单

- [x] `3rd_part/openvino_2024.6.0` 放置的是 **ubuntu20 arm64 版**（SHA256 已验证，`libopenvino_arm_cpu_plugin.so` 为 aarch64 ELF）
- [x] 4 处 CMakeLists 的 `OpenVINO_DIR` 已改为架构感知逻辑（ARM → `3rd_part/`，x86 → `/opt/intel` 或系统搜索，可 `-D` 覆盖）
- [x] `3rd_part/` 已加入 `.gitignore`
- [ ] 目标机（Ubuntu 26.04 arm64）上首次 `cmake -B build` 能通过 `find_package(OpenVINO)`（ubuntu20 构建 + glibc 兼容性实测）
- [x] 所有配置文件 `device` 均为 `CPU`（无 `GPU`）
- [ ] 运行时库可被链接器找到（`ldconfig -p | grep -E 'MvCameraControl|MVSDK|openvino|tbb'`），OpenVINO `plugins/` 与 TBB 目录在同一路径下
- [ ] 相机 udev 规则生效，非 root 可访问 USB 设备
- [ ] INT8 模型在 ARM CPU 上精度实测通过
- [ ] 帧率/延迟在 ARM CPU 上满足决策链路预算
- [ ] 若用哨兵链路：ROS2 arm64 环境可用
