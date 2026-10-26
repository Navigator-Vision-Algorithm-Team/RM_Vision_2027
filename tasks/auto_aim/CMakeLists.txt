cmake_minimum_required(VERSION 3.16)

# OpenVINO 的查找路径由顶层 cmake/platform.cmake 统一决定，这里只管使用
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# planner/（MPC 云台平滑）与 planner/tinympc/（CMU TinyMPC 求解器）不参与构建：
# 唯一会实例化 Planner 的 *_mpc 入口已删除，详见 docs/unused_code.md。
# 恢复时取消下一行注释，并把 planner/planner.cpp 与 tinympcstatic 加回本文件。
# add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    # 【改动】新增公共 PnP 内核，主相机 Solver 与全向相机共用
    pnp.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    # ROS2 实时调试数据流（-> Foxglove）；无 ROS2 环境下自动退化为空实现
    debug_stream.cpp
    # 设备选择与降级（x86 GPU / arm64 CPU 共用同一份配置）
    ov_device.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    # planner/planner.cpp            # MPC，暂不编译，见 docs/unused_code.md
)

target_link_libraries(auto_aim io openvino::runtime)
# target_link_libraries(auto_aim io openvino::runtime tinympcstatic)  # 随 planner 恢复