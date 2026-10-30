# BNBU 2027 Auto-Aim

## 本项目重写了同济大学的自瞄（感谢同济开源喵）

### 3.2 编译方式

> **单一分支说明**：x86_64 与 aarch64 共用 `develop` 分支，源码完全一致，**架构差异只存在于
> 构建系统**（`cmake/platform.cmake`）。该文件是项目中唯一允许出现架构判断的地方，它根据
> `CMAKE_SYSTEM_PROCESSOR` 自动选择：相机 SDK 目录（`io/*/lib/amd64` 或 `io/*/lib/arm64`）、
> OpenVINO 目录（`3rd_part/openvino_2024.6.0/{aarch64,x86_64}/runtime`）、安装包里打包的 `.so`。
> 业务代码、配置文件、脚本都不需要按架构分叉。新增平台差异时只改这一个文件。
>
> **两套内置二进制**：海康 / 迈德威视相机 SDK 提供 amd64 与 arm64 版
> （`io/hikrobot/lib/{amd64,arm64}/libMvCameraControl.so`、`io/mindvision/lib/{amd64,arm64}/libMVSDK.so`），
> OpenVINO 2024.6.0 也提供 aarch64 与 x86_64 两套运行库，均随仓库分发，**两个平台都无需系统安装**。
> 以下安装步骤以 **Ubuntu 26.04 LTS arm64（全新系统）** 为例；x86 开发机见 2.0 / 2.2。
>
> **保留但未参与构建的代码**（打符 `auto_buff`、`uav`、MPC `planner`/`tinympc` 等）
> 的作用、停用原因与恢复方法，见 [docs/unused_code.md](docs/unused_code.md)。

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

   **1.2 OpenVINO（已随仓库提供 aarch64 与 x86_64 两套运行库，无需安装）：**
   ```
   3rd_part/openvino_2024.6.0/
     aarch64/runtime/   # 官方 l_openvino_toolkit_ubuntu20_2024.6.0_arm64
     x86_64/runtime/    # 官方 l_openvino_toolkit_ubuntu22_2024.6.0_x86_64
   ```
   `cmake/platform.cmake` 按目标架构自动选择，两条硬件线共用同一个版本（2024.6.0），
   因此不会出现"开发机 apt 装的是新版、板子上是 2024.6"的版本漂移。
   来源、SHA256 校验值、裁剪内容与重新获取步骤见
   [3rd_part/openvino_2024.6.0/README.md](3rd_part/openvino_2024.6.0/README.md)。

   想改用系统安装的 OpenVINO（如 apt 的 `libopenvino-dev`）时加
   `-DOpenVINO_DIR=/path/to/runtime/cmake` 覆盖即可。

   **1.3 相机 SDK（arm64 动态库已随仓库提供，无需额外安装 SDK 本体）：**
   - [MindVision SDK](https://mindvision.com.cn/category/software/sdk-installation-package/)或[HikRobot SDK](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_231116.zip)
   - 仅需从上述 SDK 安装包中提取 **udev 规则**（如 `51-mvs.rules`），否则无权限访问 USB 相机。
     仓库已附带一份等价规则 `io/hikrobot/udev/51-mvs.rules`，直接安装即可：
   ```bash
   sudo cp io/hikrobot/udev/51-mvs.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   # 然后重新插拔相机
   ```
   - **海康运行时库必须与 `libMvCameraControl.so` 同目录**：SDK 内部只从自身所在目录
     加载 `libMvUsb3vTL.so`（USB3 传输层）与 `libMediaProcess.so` 等辅助库，缺库时
     `MV_CC_EnumDevices` 会返回 `MV_E_RESOURCE (0x80000006)`，现象是一台相机都枚举不到。
     仓库已在 `io/hikrobot/lib/{amd64,arm64}/` 中提供这套运行库，安装时会一起复制到
     `install/bin/`。
   - **海康 SDK 版本锁定为 `3.2.1.0`**：`io/hikrobot/include`、`lib/{amd64,arm64}` 下的
     控制库、传输层、图像处理库与头文件必须来自**同一个 MVS 版本**。只换其中一部分时，
     枚举（`MV_CC_EnumDevices`）往往还能过，但一开设备（`MV_CC_OpenDevice`）就返回
     `MV_E_VERSION (0x80000009)` —— 本项目踩过这个坑：控制库 4.1.2.2 + 传输层 3.2.1.0。
     升级 SDK 请整套替换（include + `libMvCameraControl.so` + `libMvUsb3vTL.so` +
     `libMediaProcess.so` + `libMVRender.so` + `libFormatConversion.so`）。
   - 运行时库路径配置：**用 `install/` 部署时不需要任何配置**（`bin/` 内 `.so` 通过
     `$ORIGIN` 自包含）；只有在 `build/` 里直接运行时可执行文件才需要下面二选一：
   ```bash
   # 方式一：ldconfig（推荐，一次性生效；注意 TBB 也必须配置）
   echo "$(pwd)/3rd_part/openvino_2024.6.0/aarch64/runtime/lib/aarch64" | sudo tee /etc/ld.so.conf.d/openvino.conf
   echo "$(pwd)/3rd_part/openvino_2024.6.0/aarch64/runtime/3rdparty/tbb/lib" | sudo tee -a /etc/ld.so.conf.d/openvino.conf
   echo "$(pwd)/io/hikrobot/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvs.conf
   echo "$(pwd)/io/mindvision/lib/arm64" | sudo tee /etc/ld.so.conf.d/mvsdk.conf
   sudo ldconfig

   # 方式二：LD_LIBRARY_PATH（临时生效；x86 把 aarch64 换成 x86_64、arm64 换成 amd64）
   export LD_LIBRARY_PATH="$(pwd)/3rd_part/openvino_2024.6.0/aarch64/runtime/lib/aarch64:$(pwd)/3rd_part/openvino_2024.6.0/aarch64/runtime/3rdparty/tbb/lib:$(pwd)/io/hikrobot/lib/arm64:$(pwd)/io/mindvision/lib/arm64:$LD_LIBRARY_PATH"
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
       libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev
   cmake -B build            # platform.cmake 自动识别 x86_64：内置 x86_64 OpenVINO + lib/amd64
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