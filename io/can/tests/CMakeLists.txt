cmake_minimum_required(VERSION 3.16.3)
project(soldier_can_checks LANGUAGES CXX)

# Standalone checks: no ROS2, camera SDK, OpenVINO or CAN device required.
find_package(Threads REQUIRED)
enable_testing()
get_filename_component(REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

add_library(soldier_can_core STATIC
    ../minipc_codec.cpp
    ../soldier_cboard.cpp)
target_compile_features(soldier_can_core PUBLIC cxx_std_20)
set_target_properties(soldier_can_core PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(soldier_can_core PUBLIC "${REPOSITORY_ROOT}")
target_link_libraries(soldier_can_core PUBLIC Threads::Threads)

foreach(check IN ITEMS minipc_codec_check soldier_cboard_check)
    add_executable(${check} ${check}.cpp)
    target_link_libraries(${check} PRIVATE soldier_can_core)
    add_test(NAME ${check} COMMAND ${check})
    set_tests_properties(${check} PROPERTIES TIMEOUT 30)
endforeach()
