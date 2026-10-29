cmake_minimum_required(VERSION 3.16)

# OpenVINO 的查找路径由顶层 cmake/platform.cmake 统一决定，这里只管使用
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(omniperception OBJECT 
    decider.cpp
    # 每路全向相机一个 YOLO/线程的任务工厂（供 auto_aim::multithread::YoloPipeline 使用）
    omni_jobs.cpp
    # 【改动】全向相机的 PnP 胶水层（复用 auto_aim 的公共 PnP 内核）
    omni_pnp.cpp
)
target_link_libraries(omniperception openvino::runtime)