cmake_minimum_required(VERSION 3.10)
project(rm_serial_driver)

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
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/rm_serial_driver.cpp
  src/crc.cpp
  src/solve_trajectory.cpp
  tools/aimer.cpp
  tools/planner/planner.cpp
  tools/planner/tinympc/admm.cpp
  tools/planner/tinympc/codegen.cpp
  tools/planner/tinympc/rho_benchmark.cpp
  tools/planner/tinympc/tiny_api.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_serial_driver::RMSerialDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
