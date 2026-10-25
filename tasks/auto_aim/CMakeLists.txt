cmake_minimum_required(VERSION 3.16)

# OpenVINO 的查找路径由顶层 cmake/platform.cmake 统一决定，这里只管使用
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

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
    # 设备选择与降级（x86 GPU / arm64 CPU 共用同一份配置）
    ov_device.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)