cmake_minimum_required(VERSION 3.16.3)

# ============================================================================
#  串口自检工具已迁出
#
#  serial_communication_test / time_delay_test 需要直接编译串口协议实现，
#  而协议代码随串口链路一起迁到了独立工程 info_relay。视觉进程不再打开串口，
#  也没有这些源文件，因此这两个目标从本工程移除；需要时在 info_relay 仓库构建。
#  源码保留在本目录备查，不参与构建。
# ============================================================================

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
# 与 soldier 共用同一份业务代码（src/soldier.cpp 的 run_soldier）。
if(TARGET io_soldier_can)
    add_executable(soldier_debug soldier_debug.cpp "${PROJECT_SOURCE_DIR}/src/soldier.cpp")
    # io_can：auto_aim 的 commandgener.cpp 仍引用 io::CBoard::send，需一并链接。
    target_link_libraries(
        soldier_debug
        ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io io_can io_soldier_can)
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

# ============================================================================
#  裁判状态转发自测：虚拟下位机 0x0001/0x0201 -> SerialBoard -> ROS 话题
#    ./referee_forward_test [config] [timeout_s]
#  校验导航决策节点订阅的三个 referee 话题的名字、类型与字段值。
# ============================================================================
# 裁判转发自测已停用：转发逻辑迁入 info_relay，校验方式改为订阅 relay 的裁判主题，
# 待 relay 端到端联调后重写。源码保留备查，不参与构建。
# add_executable(referee_forward_test referee_forward_test.cpp)
# target_link_libraries(referee_forward_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

# soldier_debug 只在 io_soldier_can 存在时定义，其 install 写在上面那个 if 块里。
# 裁判转发自测停用后已从安装列表移除
install(TARGETS sentry_multithread_debug yolodebug
        RUNTIME DESTINATION bin)
