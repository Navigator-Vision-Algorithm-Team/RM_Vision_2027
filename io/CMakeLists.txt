cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

# 达妙 USB-CAN 驱动单独维护（io_soldier_can）；SocketCAN 板卡通信已移除
add_subdirectory(can)

# =====================================================================================
#  相机包按配置裁剪
#
#  SP_CAMERAS_ENABLED 由顶层 cmake/config_features.cmake 依据 SP_CONFIG 解析得到：
#  只把配置里 camera_name* 引用到的后端编进来，其余后端的源文件、SDK 头文件路径与
#  链接库一概不参与构建。未启用的后端同时关闭对应的 SP_WITH_* 宏，camera.cpp 里的
#  分支会被预处理器裁掉，因此不产生任何运行期判断开销。
# =====================================================================================
set(IO_SOURCES
    camera.cpp
    # ---- 坐标变换树（io/tf/）：标定值 -> 各坐标系之间的刚体变换 ----
    tf/tf_tree.cpp
    # ---- 中转客户端：视觉侧只通过它与 info_relay 交换串口数据 ----
    relay/relay_client.cpp
    relay/relay_codec.cpp
)

set(IO_LINK_LIBS yaml-cpp)

if("hikrobot" IN_LIST SP_CAMERAS_ENABLED)
    list(APPEND IO_SOURCES hikrobot/hikrobot.cpp hikrobot/hik_camera_manager.cpp)
endif()
if("mindvision" IN_LIST SP_CAMERAS_ENABLED)
    list(APPEND IO_SOURCES mindvision/mindvision.cpp)
endif()
if("usbcamera" IN_LIST SP_CAMERAS_ENABLED)
    list(APPEND IO_SOURCES usbcamera/usbcamera.cpp)
endif()

# 创建目标 io
# 职责边界：本库只保留相机（按配置裁剪）、坐标变换树与中转客户端。
#   串口通信、协议解析与 ROS2 导航桥接已迁至 info_relay 模块，
#   视觉进程不再打开串口，改由 io/relay 的客户端与中转进程通信。
add_library(io STATIC ${IO_SOURCES})

target_include_directories(io
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 中转客户端依赖 iceoryx2，按系统库查找（与 info_relay 工程共用同一份安装）。
# 缺失时 find_package 会直接报错，提示需要先安装 iceoryx2 到系统前缀。
find_package(iceoryx2-cxx REQUIRED)
# PUBLIC 让各入口直接拿到 iceoryx2 的头文件与库
list(APPEND IO_LINK_LIBS iceoryx2-cxx::static-lib-cxx)

# ---------------- 海康 MVS：仅在配置引用 hikrobot 时引入 ----------------
if("hikrobot" IN_LIST SP_CAMERAS_ENABLED)
    # 相机 SDK：具体目录由 cmake/platform.cmake 按目标架构给出（amd64 / arm64）
    target_include_directories(io PUBLIC hikrobot/include)
    target_link_directories(io PUBLIC ${SP_HIKROBOT_LIB_DIR})
    # ${CMAKE_DL_LIBS}：hik_camera_manager 用 dladdr 定位 SDK 所在目录做运行库自检
    list(APPEND IO_LINK_LIBS MvCameraControl ${CMAKE_DL_LIBS})
    target_compile_definitions(io PUBLIC SP_WITH_HIKROBOT=1)
endif()

# ---------------- 迈德威视：仅在配置引用 mindvision 时引入 ----------------
if("mindvision" IN_LIST SP_CAMERAS_ENABLED)
    target_include_directories(io PUBLIC mindvision/include)
    target_link_directories(io PUBLIC ${SP_MINDVISION_LIB_DIR})
    list(APPEND IO_LINK_LIBS MVSDK usb-1.0)
    target_compile_definitions(io PUBLIC SP_WITH_MINDVISION=1)
endif()

# ---------------- USB 相机：只依赖 OpenCV，无额外 SDK ----------------
if("usbcamera" IN_LIST SP_CAMERAS_ENABLED)
    target_compile_definitions(io PUBLIC SP_WITH_USBCAMERA=1)
endif()

target_link_libraries(io ${IO_LINK_LIBS})

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
