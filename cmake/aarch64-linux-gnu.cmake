# =====================================================================================
#  aarch64 交叉编译工具链文件
#
#  用途：在 x86_64 开发机上交叉编译出运行于 ARM（aarch64）机器人的可执行文件。
#
#  使用方式（在项目根目录执行）：
#    # 1) 安装交叉编译器（Ubuntu/Debian，一次性）
#    sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
#
#    # 2) 配置 + 编译
#    cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake
#    cmake --build build -j$(nproc)
#
#  可选变量：
#    CROSS_PREFIX     交叉编译器前缀，默认 aarch64-linux-gnu-
#                     也可改用 -DCMAKE_C_COMPILER=... -DCMAKE_CXX_COMPILER=... 直接指定
#    AARCH64_SYSROOT  目标机（aarch64）根文件系统路径。交叉编译时 OpenCV / fmt /
#                     Eigen3 / spdlog / yaml-cpp 等 apt 依赖需要 arm64 版本，
#                     通常由 sysroot 提供（从目标 ARM 板 rsync 的根文件系统，或
#                     debootstrap --arch=arm64 生成的 rootfs）：
#                       cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake \
#                             -DAARCH64_SYSROOT=/opt/arm64-sysroot
#                     也可不建 sysroot，用 -DOpenCV_DIR=... 等逐个显式指定 arm64 库路径。
#
#  说明：
#    本文件生效后 CMAKE_SYSTEM_PROCESSOR=aarch64，项目 CMakeLists 的架构感知逻辑
#    （OpenVINO 指向 3rd_part/openvino_2024.6.0、相机 SDK 选择 io/*/lib/arm64）
#    会自动切换到 ARM 分支，无需修改任何 CMakeLists。
# =====================================================================================

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# ---------- 交叉编译器（未显式指定时才设置，允许命令行 -DCMAKE_*_COMPILER 覆盖） ----------
if(NOT DEFINED CROSS_PREFIX)
  set(CROSS_PREFIX "aarch64-linux-gnu-")
endif()

if(NOT DEFINED CMAKE_C_COMPILER)
  find_program(CMAKE_C_COMPILER NAMES "${CROSS_PREFIX}gcc")
endif()
if(NOT DEFINED CMAKE_CXX_COMPILER)
  find_program(CMAKE_CXX_COMPILER NAMES "${CROSS_PREFIX}g++")
endif()

if(NOT CMAKE_C_COMPILER OR NOT CMAKE_CXX_COMPILER)
  message(FATAL_ERROR
    "未找到 ${CROSS_PREFIX}gcc / ${CROSS_PREFIX}g++，请先安装：\n"
    "  sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu")
endif()

# ---------- 可选 sysroot（arm64 目标库来源） ----------
if(DEFINED AARCH64_SYSROOT AND NOT "${AARCH64_SYSROOT}" STREQUAL "")
  set(CMAKE_SYSROOT "${AARCH64_SYSROOT}")
  set(CMAKE_FIND_ROOT_PATH "${AARCH64_SYSROOT}")
endif()

# ---------- 查找规则 ----------
# 可执行程序（cmake / ninja / pkg-config 等）使用宿主机；
# 库、头文件、CMake 包配置只在交叉环境（sysroot / 显式 -D<Package>_DIR）中查找，
# 防止误链宿主机的 x86_64 库。
# 项目内置的 3rd_part/openvino_2024.6.0 由 CMakeLists 直接 set(OpenVINO_DIR ...)
# 指定绝对路径，不受此查找规则影响，可正常找到。
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
