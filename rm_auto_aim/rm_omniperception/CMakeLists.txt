cmake_minimum_required(VERSION 3.10)
project(rm_omniperception)

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

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_omniperception::OmniPerceptronNode
  EXECUTABLE omni_perceptron_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
