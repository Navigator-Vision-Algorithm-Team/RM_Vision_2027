# BNBU 2027 Auto-Aim

## 本项目重写了同济大学的自瞄（感谢同济开源喵）

### 3.2 编译方式

> **单一分支说明**：x86_64 与 aarch64 共用 `develop` 分支，源码完全一致，**架构差异只存在于
> 构建系统**（`cmake/platform.cmake`）。该文件是项目中唯一允许出现架构判断的地方，它根据
> `CMAKE_SYSTEM_PROCESSOR` 自动选择：相机 SDK 目录（`io/*/lib/amd64` 或 `io/*/lib/arm64`）、
> OpenVINO 来源（系统 / `/opt/intel` / 内置 `3rd_part`）、安装包里打包的 `.so`。
> 业务代码、配置文件、脚本都不需要按架构分叉。新增平台差异时只改这一个文件。
>
> **ARM（aarch64）适配说明**：仓库同时提供海康 / 迈德威视相机 SDK 的 amd64 与 arm64 动态库
> （`io/hikrobot/lib/{amd64,arm64}/libMvCameraControl.so`、`io/mindvision/lib/{amd64,arm64}/libMVSDK.so`）。
> OpenVINO 在 ARM 上采用**内置方式**：下载 aarch64 版 OpenVINO 2024.6.0 放入项目 `3rd_part/`
> 目录即可，无需系统安装；x86 上则使用系统安装（apt 的 `libopenvino-dev` 或 `/opt/intel`）。
> 以下安装步骤以 **Ubuntu 26.04 LTS arm64（全新系统）** 为例，详细的适配分析见
> [armFitting.md](docs/armFitting.md)；x86 开发机见 2.0 / 2.2。

1. 安装依赖项：

   **1.1 系统依赖（apt 源直接安装，arm64 原生）：**
   ```bash
   sudo apt update
   sudo apt install -y \
       git \
       g++ \
       cmake \
       can-utils \
       libopencv-dev \
       libfmt-dev \
       libeigen3-dev \
       libspdlog-dev \
       libyaml-cpp-dev \
       libusb-1.0-0-dev \
       nlohmann-json3-dev \
       openssh-server \
       screen
   ```

   **1.2 OpenVINO（内置方式：下载后放入项目 `3rd_part/` 目录，不安装到系统）：**
   ```bash
   # 目标平台 Ubuntu 26.04 arm64：官方 2024.6.0 的 arm64 构建为 ubuntu20 版
   # （glibc 向后兼容，可在 26.04 正常运行），33MB 压缩，仅保留 runtime/ 后约 70MB
   mkdir -p 3rd_part && cd 3rd_part
   curl -fSL -o openvino_arm64.tgz "https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_arm64.tgz"
   echo "e094e7c3e3a3931213aeeb6d6abb6cc4d654f5cb4df8da2914158ba62e25e33a  openvino_arm64.tgz" | sha256sum -c -
   tar -xzf openvino_arm64.tgz && rm openvino_arm64.tgz
   mv l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_arm64 openvino_2024.6.0
   # 删除非必需内容（仅 C++ 推理需要 runtime/），解压后体积从 ~102MB 降至 ~70MB
   rm -rf openvino_2024.6.0/docs openvino_2024.6.0/install_dependencies \
         openvino_2024.6.0/python openvino_2024.6.0/samples openvino_2024.6.0/setupvars.sh
   cd ..
   # cmake/platform.cmake 会按目标架构自动选择：aarch64 指向 3rd_part/openvino_2024.6.0，
   # x86 走系统安装（/opt/intel 或 apt 的 libopenvino-dev），详见 docs/armFitting.md §5.1
   # 内置包自带 lib/ 子目录名（aarch64 / intel64），架构不匹配时不会误用
   ```

   **1.3 相机 SDK（arm64 动态库已随仓库提供，无需额外安装 SDK 本体）：**
   - [MindVision SDK](https://mindvision.com.cn/category/software/sdk-installation-package/)或[HikRobot SDK](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_231116.zip)
   - 仅需从上述 SDK 安装包中提取 **udev 规则**（如 `51-mvs.rules`），否则无权限访问 USB 相机：
   ```bash
   sudo cp <SDK中的>.rules /etc/udev/rules.d/
   sudo udevadm control --reload && sudo udevadm trigger
   ```
   - 运行时库路径配置（二选一，推荐 ldconfig）：
   ```bash
   # 方式一：ldconfig（推荐，一次性生效；注意 TBB 也必须配置）
   echo "$(pwd)/3rd_part/openvino_2024.6.0/runtime/lib/aarch64" | sudo tee /etc/ld.so.conf.d/openvino.conf
   echo "$(pwd)/3rd_part/openvino_2024.6.0/runtime/3rdparty/tbb/lib" | sudo tee -a /etc/ld.so.conf.d/openvino.conf
   echo "$(pwd)/io/hikrobot/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvs.conf
   echo "$(pwd)/io/mindvision/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvsdk.conf
   sudo ldconfig

   # 方式二：LD_LIBRARY_PATH（临时生效）
   export LD_LIBRARY_PATH="$(pwd)/3rd_part/openvino_2024.6.0/runtime/lib/aarch64:$(pwd)/3rd_part/openvino_2024.6.0/runtime/3rdparty/tbb/lib:$(pwd)/io/hikrobot/lib/arm64:$(pwd)/io/mindvision/lib/arm64:$LD_LIBRARY_PATH"
   ```

   **1.4 注意事项：**
   - 推理设备统一写 `device: auto`：有可用 GPU 时自动用 GPU，否则回退 CPU。ARM 板没有 GPU
      插件，会打印一行 `GPU 不可用，改用 CPU 推理` 后继续，属正常现象；也可显式写 `CPU` / `GPU`。
   - ROS2（可选）：仅哨兵多线程目标需要，ARM 上可安装 ROS2 Humble arm64 官方包；不装则 CMake 自动跳过 ROS 相关目标。
   - Ceres 已从构建中移除（代码从未使用），无需再安装 `libceres-dev`。

   2. 编译：

   **2.0 x86_64 本机编译（开发机默认）：**

   ```bash
   sudo apt install -y libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev \
       libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev libopenvino-dev
   cmake -B build            # platform.cmake 自动识别 x86_64：系统 OpenVINO + lib/amd64
   cmake --build build -j$(nproc)
   ```

   **2.1 ARM 本机编译（aarch64 上直接编译）：**

   ```bash
   cmake -B build
   cmake --build build -j$(nproc)
   ```

   > 三条命令唯一的区别就是 `-DCMAKE_TOOLCHAIN_FILE`，其余完全一致；架构选择由
   > `cmake/platform.cmake` 依据 `CMAKE_SYSTEM_PROCESSOR` 自动完成。

   **2.2 x86 开发机交叉编译到 ARM（工具链文件 `cmake/aarch64-linux-gnu.cmake`）：**

   ```bash
   # 安装交叉编译器（一次性）
   sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

   # 配置（指定工具链文件即可：platform.cmake 会把 CMAKE_SYSTEM_PROCESSOR 识别为 aarch64，
   # 自动指向 3rd_part/ 的 OpenVINO 与 io/*/lib/arm64 的相机 SDK）
   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake
   cmake --build build -j$(nproc)
   ```

   > 交叉编译需要 arm64 版的 apt 依赖（OpenCV / fmt / Eigen3 / spdlog / yaml-cpp /
   > nlohmann_json / libusb-1.0 等），通过 sysroot 提供，二选一：
   >
   > **方式 A：从目标 ARM 板同步根文件系统**（依赖与目标机完全一致，最稳妥）
   > ```bash
   > rsync -a --exclude=/proc --exclude=/sys --exclude=/dev user@arm-board:/ /opt/arm64-sysroot/
   > ```
   >
   > **方式 B：本地用 mmdebstrap 生成 arm64 rootfs 并装入依赖**（无需 ARM 板）
   > ```bash
   > sudo apt install -y mmdebstrap qemu-user-static
   > CODENAME=$(lsb_release -cs)   # 26.04 为 resolute
   > sudo mmdebstrap --arch=arm64 --variant=important "$CODENAME" /opt/arm64-sysroot \
   >     http://ports.ubuntu.com/ubuntu-ports \
   >     --hook='copy-in -v /usr/bin/qemu-aarch64-static /usr/bin/' \
   >     --customize-hook='chroot "$1" apt-get update' \
   >     --customize-hook='chroot "$1" apt-get install -y --no-install-recommends \
   >         libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev \
   >         nlohmann-json3-dev libusb-1.0-0-dev'
   > ```
   >
   > 然后带 sysroot 配置编译：
   > ```bash
   > cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake \
   >       -DAARCH64_SYSROOT=/opt/arm64-sysroot
   > cmake --build build -j$(nproc)
   > ```
   > 也可以不建 sysroot，用 `-DOpenCV_DIR=<arm64路径>` 等逐个显式指定。

   ```bash
   #不输出wearning，单线程编译。
   cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS_DEBUG="-O0 -g -w -fdiagnostics-color=always -fmax-errors=10"
    
   cmake --build build -j1 2>&1 | tee build_error.log
   ```

### 数据源：real / fake / replay（无真实下位机与相机也能跑通）
>
> 业务算法不直接依赖硬件，只依赖抽象数据源（`io::sim::QuaternionSource` / `io::sim::CameraSource`），
> 通过配置在 `real` / `fake` / `replay` 之间切换，**业务代码一行不改**。
> 实现位于 `io/sim/`，Python 虚拟下位机位于 `sim/virtual_mcu.py`。

**1) 抽象接口与实现**

| 方向 | 抽象 | real | fake | replay |
| --- | --- | --- | --- | --- |
| 串口四元数 | `io::sim::QuaternionSource` | `RealSerialSource` | `FakeSerialSource` | `ReplaySerialSource` |
| 相机图像 | `io::sim::CameraSource` | `RealHikCameraSource` | `FakeCameraSource` | `ReplayCameraSource` |

三种实现共用同一套「真实协议字节解析」与同一套 `io::CameraBase` 适配器，因此**帧率、时间戳语义、
报文格式完全一致**：fake 由内置虚拟下位机按 0xA5 + CRC8/CRC16 打包 0x502/0x503/0x505/0x0001。

**2) 配置切换（示例见 `configs/virtual_example.yaml` / `virtual_replay.yaml` / `virtual_fault.yaml`）**

```yaml
source_mode: "fake"          # real | fake | replay（可用 serial_mode / camera_mode 单独覆盖）
serial_mode: "fake"          # 串口四元数源
camera_name: "fake"          # 相机源：fake=伪相机，replay=回放相机
fake_serial_external: false  # true 时改用外部 Python 虚拟下位机提供的 pty
fake_camera_source: "synth"  # synth(合成图案) | sequence(图片序列) | video(视频文件)
record: { enable: true, dir: "logs/virtual_session", save_images: true }
```

**3) 无硬件运行**

```bash
# 纯内置伪数据（推荐）：四元数 500Hz + 合成图像 30fps，同时记录
./build/virtual_run configs/virtual_example.yaml --duration=8 --show

# 业务入口直接跑（只改配置：camera_name fake + source_mode fake）
./build/mt_standard configs/mtstandard_virtual.yaml
```

**4) Python 虚拟下位机（pty 伪串口）**

```bash
python3 sim/virtual_mcu.py --port-file /tmp/vmcu.pts          # 打印 /dev/pts/N
python3 sim/virtual_mcu.py --drop-rate 0.05 --spike-every 200 --corrupt-crc-rate 0.02
python3 sim/virtual_mcu.py --replay logs/virtual_session/serial.bin   # 回放录制字节
# 然后设置 com_port: "<上面打印的 /dev/pts/N>"，serial_mode: real（主程序按普通串口打开）
```
> 注意：Python 侧 CRC16 表与 `io/serial/gimbal/protocol_crc.cpp` 的固件表**严格对齐**（索引 47 为 `0xD974`），
> 修改任一侧都必须同步。

**5) 异常注入与记录回放**

- 可注入：串口丢包/延迟/跳变/校验错误/角度噪声；相机丢帧/重复帧/黑屏/过曝/冻结/噪声/延迟。
  固定 `fault_seed` 保证异常序列可复现。
- 日志用 `[REAL]` / `[FAKE]` / `[REPLAY]` 标识数据来源，`[FAULT:drop]`、`[FAULT:crc]`、`[FAULT:black]`
  等标识注入的异常，`[FAULT] frame ... injected anomaly` 标识下游收到的异常帧。
- 记录内容：`serial.bin`（串口原始字节 + 方向 + 时间戳）、`camera.tsv` + `camera/*.png`（帧 + 时间戳 + 帧信息），
  时间戳统一基于会话单调时钟；改 `source_mode: replay` 即按原时间轴复现。

**6) 局限**：不覆盖海康 SDK 初始化、设备枚举、驱动异常等底层问题，仍需真实硬件验证。

### 支持兵种并维护

哨兵，步兵，英雄

### 项目信息

#### calibration

标定相机使用，里面有很多脚本可以用来标定包括相机参数，手眼参数等

#### configs

存放配置文件，你能看见似乎有很多的配置文件，但是其中大部分的内容已经被优化在一个文件中，所以大部分文件已经不在使用，主要使用参数文件：configs/sentry_example.yaml

##### 参数细节

包含了敌人颜色，串口通信约定，相机参数配置，决策器等参数配置。

#### IO

里面包含了相机的通信代码，串口读取代码和串口的发送代码（现在还包含一些其他的，后面会删除的）

#### src

这是程序的主入口，有着哨兵，步兵，无人机等RM机器人的启动文件。当然，我们要注意的是哨兵和步兵，为什么不删除——谁知道以后会不会被使用吗？

#### tasks

我们真正的业务代码，里面包含了自瞄相关的决策代码，全向感知的相关代码，至于auto buff好像是为了做符用的，我们也用不上

#### test

里面提供了一些链路的test工具，可以仔细看看作用，只不过我在专注更改代码的时候忽略了这个东西，现在如果想要使用test的话，也许需要一些功夫适配现在的代码了

#### tool

提供了一些代码会使用到的通用工具。

### 约定

1. yaw的方向：根据实验结果认定，在云台顺时针旋转的时候yaw值减小。也就是在镜头的左边是yaw增大，在镜头中央的右边的时候yaw是减小的。
2. 此代码作用于3V3，所以我们将yolo识别的数量只限【哨兵--图标哨兵号】【步兵--3】【英雄--1】
3. yolo输出约定和后续处理约定：
    - 英雄：1 - “One”
    - 步兵：3 - “Three” 在7V7中我们的步兵有3个，分别是3 4 5
    - 哨兵：0 - “semtry“
    因为我们只在乎3V3，所以我们在yolo之后的处理只是这样的。只在乎这三个yolo输出。
4. 我们的相机时间和IMU时间约定：老实说，我们没有任何的硬件方式同步我们相机和IMU的时间，我们采用的策略是不使用任何硬件设备的时间标签，而是使用我们的自己的，具体来说。我们在相机初始化和串口连接上的时候就会来事开启一个线程来接收两者的数据，并且自动的打上我们自己的时间标签，这个时间标签在代码运行之初就已经确定。在我们读取的时候，也只是使用这个时间标签来读取我们自己的数据。
5. 我们对于大装甲板和普通装甲板的约定在代码中--slover代码中，宏定义部分
6. yaw的约定-2:我们yaw的范围是[-pi, pi]，并且电机初始化的逻辑是，在上电的那一瞬间的yaw值是0。所以在后续的代码中，我们需要注意结果的yaw的具体描述，是增加量，还是直接的相较于零的目标数值

### 协作约定

在该项目的工作者可以直接将代办事项写在TODO文件中，如果你接取了代办事项需要在代办事项后进行标记，比如写上你的名字。然后请记住立马push！！如果你完成了事项，可以删除该事项。