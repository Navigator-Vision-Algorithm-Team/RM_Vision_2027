# BNBU 2027 Auto-Aim

## 本项目重写了同济大学的自瞄（感谢同济开源喵）

### 3.2 编译方式
1. 安装依赖项：
   - [MindVision SDK](https://mindvision.com.cn/category/software/sdk-installation-package/)或[HikRobot SDK](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_231116.zip)
   - [OpenVINO](https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-archive-linux.html)
   - [Ceres](http://ceres-solver.org/installation.html)
   - 其余：
    ```bash
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

2. 编译：
    ```bash
    cmake -B build
    # 普通编译
    make -C build/ -j`nproc`
    ```

    ```bash
    #不输出wearning，单线程编译。
    cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS_DEBUG="-O0 -g -w -fdiagnostics-color=always -fmax-errors=10"
    
    cmake --build build -j1 2>&1 | tee build_error.log
    ```

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