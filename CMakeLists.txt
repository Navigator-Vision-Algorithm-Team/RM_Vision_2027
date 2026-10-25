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

set(CMAKE_CXX_STANDARD 17)
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
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

####################src###################
add_executable(standard src/standard.cpp)
add_executable(mt_standard src/mt_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug src/auto_buff_debug.cpp)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
# 步兵主程序：YOLO 检测 + PnP 位姿定位，默认配置 configs/soldier.yaml
add_executable(soldier src/soldier.cpp)
# 虚拟数据源运行器（无硬件可跑通 real / fake / replay）
add_executable(virtual_run src/virtual_run.cpp)
# add_executable(uav src/uav.cpp)
# add_executable(uav_debug src/uav_debug.cpp)

# 自瞄/哨兵类：算法 + 串口 + CAN
foreach(ex standard mt_standard standard_mpc auto_aim_debug_mpc mt_auto_aim_debug)
    target_link_libraries(${ex} ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io io_can)
endforeach()

# 打符类：视觉 + 串口/CAN（auto_buff_debug 需要 io::CBoard 读取 IMU）
foreach(ex auto_buff_debug auto_buff_debug_mpc)
    target_link_libraries(${ex} ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io io_can)
endforeach()

target_link_libraries(soldier ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io io_can)
target_link_libraries(virtual_run ${OpenCV_LIBS} fmt::fmt yaml-cpp io tools)
# target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io io_can)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

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
    add_executable(sentry_multithread_debug src/sentry_multithread_debug.cpp)

    target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io io_can)
    target_link_libraries(sentry_multithread_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io io_can)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()

################## install ##################
# 把编译产物与所有运行所需资源打包到 <prefix>（默认项目根 install/），
# 形成自包含部署目录：
#   install/bin/      可执行文件 + 相机 SDK(.so) + OpenVINO 运行库 + TBB
#   install/configs/ 推理配置
#   install/assets/  模型文件
#   install/autostart.sh
# 注意：OpenCV / fmt / Eigen / spdlog / yaml-cpp / nlohmann 等为系统库（apt 安装），
#       不打包，目标机需先装好这些 apt 依赖。

# 1) 可执行文件（按目标是否生成决定是否安装；ROS2 相关目标仅在其被编译时安装）
set(ALL_EXES standard mt_standard standard_mpc auto_aim_debug_mpc mt_auto_aim_debug
             auto_buff_debug auto_buff_debug_mpc virtual_run soldier
             capture calibrate_camera calibrate_handeye calibrate_robotworld_handeye split_video)
foreach(ex ${ALL_EXES})
    if(TARGET ${ex})
        install(TARGETS ${ex} RUNTIME DESTINATION bin)
    endif()
endforeach()
foreach(ex sentry_multithread sentry_multithread_debug)
    if(TARGET ${ex})
        install(TARGETS ${ex} RUNTIME DESTINATION bin)
    endif()
endforeach()

# 2) 相机 SDK 动态库（目录由 cmake/platform.cmake 按架构给出，打包进 bin/ 以便 $ORIGIN 找到）
install(FILES "${SP_HIKROBOT_LIB_DIR}/libMvCameraControl.so" DESTINATION bin)
install(FILES "${SP_MINDVISION_LIB_DIR}/libMVSDK.so" DESTINATION bin)

# 3) OpenVINO 运行库 + TBB（仅当链接的是项目内置 3rd_part 版本时打包，使 install/ 自包含）
if(SP_OPENVINO_BUNDLED)
    install(DIRECTORY "${SP_OPENVINO_LIB_DIR}/" DESTINATION bin FILES_MATCHING PATTERN "*.so*")
    install(DIRECTORY "${SP_OPENVINO_RUNTIME}/3rdparty/tbb/lib/" DESTINATION bin
            FILES_MATCHING PATTERN "*.so*")
endif()

# 4) 配置、模型与启动脚本
install(DIRECTORY configs assets DESTINATION .)
install(FILES autostart.sh DESTINATION .)

message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Install layout: bin/ (executables + SDK/OpenVINO .so), configs/, assets/")
