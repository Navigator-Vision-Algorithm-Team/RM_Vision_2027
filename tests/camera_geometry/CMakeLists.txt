cmake_minimum_required(VERSION 3.16)
project(camera_geometry_check LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_executable(camera_geometry_test
    camera_geometry_test.cpp
    ${REPO_ROOT}/tasks/auto_aim/solver.cpp
    ${REPO_ROOT}/tasks/auto_aim/pnp.cpp
    ${REPO_ROOT}/tasks/auto_aim/armor.cpp
    ${REPO_ROOT}/tools/math_tools.cpp)
target_include_directories(camera_geometry_test PRIVATE ${REPO_ROOT} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_geometry_test PRIVATE ${OpenCV_LIBS} Eigen3::Eigen yaml-cpp)
enable_testing()
add_executable(trajectory_direction_test
    trajectory_direction_test.cpp
    ${REPO_ROOT}/tools/trajectory.cpp)
target_include_directories(trajectory_direction_test PRIVATE ${REPO_ROOT})
add_test(NAME trajectory_direction COMMAND trajectory_direction_test)
add_test(NAME soldier_geometry COMMAND camera_geometry_test ${REPO_ROOT}/configs/soldier.yaml)
add_test(NAME soldier_virtual_geometry COMMAND camera_geometry_test ${REPO_ROOT}/configs/soldier_virtual.yaml)
