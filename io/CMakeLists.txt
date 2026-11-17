cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

# 达妙 USB-CAN 驱动单独维护（io_soldier_can）；SocketCAN 板卡通信已移除
add_subdirectory(can)

# 创建目标 io
# 职责边界：本库只保留相机（海康/迈德威视/USB）、坐标变换树与相机侧数据源/记录。
#   串口通信、协议解析、虚拟下位机、串口字节记录与 ROS2 导航桥接已迁至 info_relay 模块，
#   视觉进程不再打开串口，改由 io/relay 的客户端与中转进程通信。
add_library(io STATIC
    hikrobot/hikrobot.cpp
    hikrobot/hik_camera_manager.cpp
    mindvision/mindvision.cpp
    usbcamera/usbcamera.cpp
    camera.cpp
    # ---- 坐标变换树（io/tf/）：标定值 -> 各坐标系之间的刚体变换 ----
    tf/tf_tree.cpp
    # ---- 中转客户端：视觉侧只通过它与 info_relay 交换串口数据 ----
    relay/relay_client.cpp
    relay/relay_codec.cpp
)

# 中转客户端依赖 iceoryx2，按系统库查找（与 info_relay 工程共用同一份安装）。
# 缺失时 find_package 会直接报错，提示需要先安装 iceoryx2 到系统前缀。
find_package(iceoryx2-cxx REQUIRED)

target_include_directories(io
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 相机 SDK：具体目录由 cmake/platform.cmake 按目标架构给出（amd64 / arm64）
target_include_directories(io PUBLIC hikrobot/include)
target_link_directories(io PUBLIC ${SP_HIKROBOT_LIB_DIR})

target_include_directories(io PUBLIC mindvision/include)
target_link_directories(io PUBLIC ${SP_MINDVISION_LIB_DIR})

# ${CMAKE_DL_LIBS}：hik_camera_manager 用 dladdr 定位 SDK 所在目录做运行库自检
target_link_libraries(io
    MvCameraControl MVSDK usb-1.0 yaml-cpp ${CMAKE_DL_LIBS}
    # 中转客户端：PUBLIC 让各入口直接拿到 iceoryx2 的头文件与库
    iceoryx2-cxx::static-lib-cxx)

################## 检查 ROS 环境 ##################
# 说明：导航桥接的源文件已迁到 info_relay，但 tasks/auto_aim/debug_stream.cpp 仍直接使用
#       rclcpp 做 Foxglove 图像调试，它通过 auto_aim -> io 取得编译宏与头文件路径。
#       因此这里的 ROS2 探测、头文件路径、链接库与 SP_WITH_ROS2 宏必须保留。
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT geometry_msgs_FOUND OR NOT sensor_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, enabling SP_WITH_ROS2 for debug stream.")

    target_include_directories(io PUBLIC
      ${rclcpp_INCLUDE_DIRS}
      ${std_msgs_INCLUDE_DIRS}
      ${geometry_msgs_INCLUDE_DIRS}
      ${sensor_msgs_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(io
      ${rclcpp_LIBRARIES}
      ${std_msgs_LIBRARIES}
      ${geometry_msgs_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
      ${rosidl_typesupport_cpp_LIBRARIES}
    )

    # 供上层（tasks/auto_aim/debug_stream.cpp）判断是否具备 ROS2 能力，
    # 没有该宏时调试数据流整体退化为空实现，业务代码无需 #ifdef
    target_compile_definitions(io PUBLIC SP_WITH_ROS2=1)
endif()
