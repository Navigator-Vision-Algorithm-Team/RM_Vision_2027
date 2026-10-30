cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

# ---------- 平台抽象（架构差异唯一来源） ----------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(platform)

# ---------- 安装目录与 RPATH ----------
# 默认安装到项目根目录下的 install/（可用 -DCMAKE_INSTALL_PREFIX=... 覆盖）
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Installation prefix" FORCE)
endif()
# 让 install/bin 下的可执行文件通过 $ORIGIN 找到同目录内的 .so（相机 SDK / OpenVINO / TBB）
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

# 用 DT_RPATH 而不是默认的 DT_RUNPATH：RUNPATH 只对直接依赖生效且不传递，
# 于是程序能找到同目录的 libopenvino.so，但 libopenvino.so 自己找不到同目录的
# libtbb.so，干净机器上启动即失败。DT_RPATH 是全局搜索列表，可让 bin/ 真正自包含。
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")

set(CMAKE_CXX_STANDARD 20)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
endif()
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
# fastcdr / fastrtps 由 ROS2 环境提供，仅哨兵多线程目标间接需要；本项目源码未直接
# 包含其头文件，故不在此 find_package，避免未装 ROS2 的开发机配置失败。
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
# tasks/auto_buff（打符）与 src/uav.cpp 均不参与构建：
# 打符入口 auto_buff_debug* 已删除，uav 目标暂不启用。
# 需要时把 `add_subdirectory(tasks/auto_buff)` 与 uav 的
# add_executable / target_link_libraries 两行恢复即可。
add_subdirectory(tasks/omniperception)

####################src###################
# 多线程自瞄主程序（配置示例 configs/mtstandard.yaml）
add_executable(mt_standard src/mt_standard.cpp)
# 步兵主程序：YOLO 检测 + PnP 位姿定位，默认配置 configs/soldier.yaml
add_executable(soldier src/soldier.cpp)
# 虚拟数据源运行器（无硬件可跑通 real / fake / replay）
add_executable(virtual_run src/virtual_run.cpp)

target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io io_can)
target_link_libraries(soldier ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io io_can)
target_link_libraries(virtual_run ${OpenCV_LIBS} fmt::fmt yaml-cpp io tools)

##################calibration################
# 旧的标定工具（capture / calibrate_camera / calibrate_handeye /
# calibrate_robotworld_handeye / split_video）已清空，标定方案见 calibration/README.md。
# 目前只有内参标定工具，外参（相机 -> 转轴）的手眼标定工具按方案后续补。
add_executable(calibrate_intrinsics calibration/calibrate_intrinsics.cpp)
target_link_libraries(calibrate_intrinsics ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

# 外参标定：相机 -> tf 父帧（物理测量 / 手眼算法两种模式）
add_executable(calibrate_extrinsic calibration/calibrate_extrinsic.cpp)
target_link_libraries(calibrate_extrinsic ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
# find_package(sp_msgs QUIET)
# 只有在找到 ROS 环境时才会添加 sentry
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND)
    # 编译 sentry
    add_executable(sentry_multithread src/sentry_multithread.cpp)

    target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io io_can)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()

##################benchmarks##############
# EKF 离线基准（合成数据生成 + 评分）：不需要相机 / 串口 / ROS2，
# 数据与报告落在 logs/ekf_bench/（logs/ 已被 .gitignore 覆盖）
# add_subdirectory(benchmarks/ekf)

####################tests###################
# 调试版入口（sentry_multithread_debug / soldier_debug）：业务逻辑与 src/ 完全一致，
# 只额外通过 ROS2 向 Foxglove 推送图像与位姿，仅在找到 ROS2 时构建
add_subdirectory(tests)

################## install ##################
# 把编译产物与所有运行所需资源打包到 <prefix>（默认项目根 install/），
# 形成自包含部署目录：
#   install/bin/      可执行文件 + 相机 SDK(.so) + OpenVINO 运行库 + TBB
#   install/configs/ 推理配置
#   install/assets/  模型文件
# 注意：OpenCV / fmt / Eigen / spdlog / yaml-cpp / nlohmann 等为系统库（apt 安装），
#       不打包，目标机需先装好这些 apt 依赖。

# 1) 可执行文件（if(TARGET) 保证只在对应目标被生成时安装；sentry 仅在找到 ROS2 时存在）
set(ALL_EXES
    soldier sentry_multithread)
foreach(ex ${ALL_EXES})
    if(TARGET ${ex})
        install(TARGETS ${ex} RUNTIME DESTINATION bin)
    endif()
endforeach()

# 2) 相机 SDK 动态库（目录由 cmake/platform.cmake 按架构给出，打包进 bin/ 以便 $ORIGIN 找到）
#    海康的 libMvUsb3vTL.so / libMediaProcess.so 等必须与 libMvCameraControl.so 同目录，
#    SDK 才能找到 USB3 传输层（否则枚举直接返回 MV_E_RESOURCE），所以整目录一起打包
install(DIRECTORY "${SP_HIKROBOT_LIB_DIR}/" DESTINATION bin FILES_MATCHING PATTERN "*.so*")
install(FILES "${SP_MINDVISION_LIB_DIR}/libMVSDK.so" DESTINATION bin)

# 3) OpenVINO 运行库 + TBB（仅当链接的是项目内置 3rd_part 版本时打包，使 install/ 自包含）
if(SP_OPENVINO_BUNDLED)
    # 这两个目录里有 pkgconfig/、cmake/ 等子目录；FILES_MATCHING 只过滤文件、
    # 空目录仍会被创建，故一并 EXCLUDE，保证 bin/ 里只有可执行文件与 .so
    install(DIRECTORY "${SP_OPENVINO_LIB_DIR}/" DESTINATION bin
            FILES_MATCHING PATTERN "*.so*"
            PATTERN "pkgconfig" EXCLUDE PATTERN "cmake" EXCLUDE)
    install(DIRECTORY "${SP_OPENVINO_RUNTIME}/3rdparty/tbb/lib/" DESTINATION bin
            FILES_MATCHING PATTERN "*.so*"
            PATTERN "pkgconfig" EXCLUDE PATTERN "cmake" EXCLUDE)
endif()

# 4) 配置、模型与启动脚本
install(DIRECTORY configs assets DESTINATION .)

message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Install layout: bin/ (executables + SDK/OpenVINO .so), configs/, assets/")
