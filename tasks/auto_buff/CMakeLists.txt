cmake_minimum_required(VERSION 3.16)
# OpenVINO 的查找路径由顶层 cmake/platform.cmake 统一决定，这里只管使用
find_package(OpenVINO REQUIRED)

add_library(auto_buff OBJECT 
    buff_type.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    yolo11_buff.cpp
)

target_link_libraries(auto_buff auto_aim openvino::runtime)