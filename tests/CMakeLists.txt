cmake_minimum_required(VERSION 3.16.3)

# ============================================================================
#  串口通信自检（不依赖 ROS2 / 相机 SDK，也不链接 io，避免拖入相机运行库）
#    ./serial_communication_test [串口] [波特率] [yaw角速度 rad/s] [发送频率 Hz] [--nav]
#  只复用协议定义与 CRC 实现，收到的帧按结构体字段打印，同时下发 0x0402 让云台缓转
# ============================================================================
add_executable(serial_communication_test
    serial_communication_test.cpp
    ${PROJECT_SOURCE_DIR}/io/serial/gimbal/protocol_crc.cpp)
target_link_libraries(serial_communication_test serial)

# ============================================================================
#  串口通信延迟测试（回环 RTT，协议见 time_delay_test.cpp 文件头）
#    ./time_delay_test        # 无参数：/dev/ttyACM0 921600 100Hz 30s，写 delay.csv
#  发 0x0410（带时间戳，帧格式不变），下位机把 data 段原样回传（cmd_id 不限），
#  上位机统计 RTT / 单向延迟 / 抖动 / 丢包，同时记录发送频率与写串口耗时
# ============================================================================
add_executable(time_delay_test
    time_delay_test.cpp
    ${PROJECT_SOURCE_DIR}/io/serial/gimbal/protocol_crc.cpp)
target_link_libraries(time_delay_test serial)

# 交互式云台响应测试：输入绝对 yaw/pitch（度），每行发送一条 0x0402 指令。
add_executable(testgimbalspeed testgimbalspeed.cpp)
target_link_libraries(
    testgimbalspeed
    ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io io_can)
install(TARGETS testgimbalspeed RUNTIME DESTINATION bin)

# ============================================================================
#  调试版入口（仅供局域网内 Foxglove 实时观瞄，正式比赛请用 src/ 下的目标）
#
#  tests/ 下的可执行文件与 src/ 下的正式版业务逻辑完全一致，只是在主循环里
#  额外插入了 auto_aim::DebugStream（ROS2 -> Foxglove）：
#      相机原图 / YOLO 预处理后的输入图 / 检测框 + 敌人数字编号 /
#      YOLO 处理帧率(FPS) / 当前云台位姿 / 下发给云台的目标位姿
#  调试数据不参与控制；yaml 里删掉 debug_ros2 段即可整体关闭。
# ============================================================================

# 调试入口依赖 ROS2（图像走 sensor_msgs/CompressedImage）；
# 找不到就整段跳过，不影响 src/ 下的正式目标，也不影响无 ROS2 的开发机配置
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)

if(NOT (ament_cmake_FOUND AND rclcpp_FOUND AND sensor_msgs_FOUND))
    message(STATUS "ROS2 not found, skipping debug executables (tests/).")
    return()
endif()

# 步兵调试入口默认读取 configs/soldier_debug.yaml，使用达妙 USB-CAN SDK。
if(TARGET io_soldier_can)
    add_executable(soldier_debug soldier_debug.cpp)
    target_link_libraries(
        soldier_debug
        ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io io_soldier_can)
    install(TARGETS soldier_debug RUNTIME DESTINATION bin)
endif()

# 哨兵调试入口：多线程自瞄（主相机 + 全向），默认配置 configs/sentry.yaml
add_executable(sentry_multithread_debug sentry_multithread_debug.cpp)
target_link_libraries(
    sentry_multithread_debug
    ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io io_can)

# 仅测试主相机 YOLO：不启动串口、PnP 或多相机处理链路。
add_executable(yolodebug yolodebug.cpp)
target_link_libraries(yolodebug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io io_can)

install(TARGETS sentry_multithread_debug yolodebug RUNTIME DESTINATION bin)
