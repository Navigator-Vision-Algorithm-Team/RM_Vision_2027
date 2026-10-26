cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(3rdparty/serial)

# CAN 通信部分独立成库（io_can），与串口部分分开维护
add_subdirectory(can)

# 创建目标 io
add_library(io STATIC
    hikrobot/hikrobot.cpp
    hikrobot/hik_camera_manager.cpp
    mindvision/mindvision.cpp
    usbcamera/usbcamera.cpp
    camera.cpp
    dm_imu/dm_imu.cpp
    # ---- 坐标变换树（io/tf/）：标定值 -> 各坐标系之间的刚体变换 ----
    tf/tf_tree.cpp
    # ---- 串口通信（io/serial/）----
    serial/serial_board.cpp
    serial/gimbal/gimbal.cpp
    serial/gimbal/protocol_crc.cpp
    # ---- 虚拟数据源 (real / fake / replay) ----
    sim/transport.cpp
    sim/virtual_mcu.cpp
    sim/recorder.cpp
    sim/quaternion_source.cpp
    sim/camera_source.cpp
)

target_include_directories(io
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 相机 SDK：具体目录由 cmake/platform.cmake 按目标架构给出（amd64 / arm64）
target_include_directories(io PUBLIC hikrobot/include)
target_link_directories(io PUBLIC ${SP_HIKROBOT_LIB_DIR})

target_include_directories(io PUBLIC mindvision/include)
target_link_directories(io PUBLIC ${SP_MINDVISION_LIB_DIR})

target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial)

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
# find_package(sp_msgs QUIET)
# 如果找不到 ROS 环境，则跳过 ROS 相关的部分
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT geometry_msgs_FOUND OR NOT sensor_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 添加 ROS2 相关的文件
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

    # 设置 ROS2 依赖
    target_include_directories(io PUBLIC
      ${rclcpp_INCLUDE_DIRS}
      ${std_msgs_INCLUDE_DIRS}
      ${geometry_msgs_INCLUDE_DIRS}
      ${sensor_msgs_INCLUDE_DIRS}
      # ${sp_msgs_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(io
      ${rclcpp_LIBRARIES}
      ${std_msgs_LIBRARIES}
      ${geometry_msgs_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
      # ${sp_msgs_LIBRARIES}
      ${rosidl_typesupport_cpp_LIBRARIES}
    )

    # 供上层（tasks/auto_aim/debug_stream.cpp）判断是否具备 ROS2 能力，
    # 没有该宏时调试数据流整体退化为空实现，业务代码无需 #ifdef
    target_compile_definitions(io PUBLIC SP_WITH_ROS2=1)
endif()
