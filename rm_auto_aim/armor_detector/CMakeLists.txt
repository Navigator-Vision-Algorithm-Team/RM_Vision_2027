cmake_minimum_required(VERSION 3.10)
project(armor_detector)

# Auto-detect ROS distro and set C++ standard + preprocessor define
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
else()
  set(ROS_DISTRO "humble")
endif()
if(ROS_DISTRO STREQUAL "humble")
  set(CMAKE_CXX_STANDARD 14)
  add_definitions(-DROS_DISTRO_HUMBLE)
else()
  set(CMAKE_CXX_STANDARD 17)
  add_definitions(-DROS_DISTRO_JAZZY)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

# [Optional] OpenVINO for YOLO detection
# Install: https://docs.openvino.ai/2024/get-started/install-openvino.html
# Ubuntu 22.04: apt install openvino-2024.6.0
# Ubuntu 24.04: apt install openvino-2024.6.0
find_package(OpenVINO QUIET)
if(OpenVINO_FOUND)
  message(STATUS "Found OpenVINO: YOLO detector enabled")
  add_definitions(-DHAS_OPENVINO)
  set(OpenVINO_LIBS openvino::runtime)
else()
  message(STATUS "OpenVINO not found: YOLO detector disabled (set use_yolo: false)")
endif()

###########
## Build ##
###########

# Core sources (always built)
set(ARMOR_DETECTOR_SOURCES
  src/detector.cpp
  src/detector_node.cpp
  src/number_classifier.cpp
  src/pnp_solver.cpp
  tools/math_tools.cpp
)

# YOLO detector (only if OpenVINO is available)
if(OpenVINO_FOUND)
  list(APPEND ARMOR_DETECTOR_SOURCES src/yolo_detector.cpp)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${ARMOR_DETECTOR_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if(OpenVINO_FOUND)
  target_link_libraries(${PROJECT_NAME} ${OpenVINO_LIBS})
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup ${PROJECT_NAME})

  ament_add_gtest(test_number_cls test/test_number_cls.cpp)
  target_link_libraries(test_number_cls ${PROJECT_NAME})

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  model
)
