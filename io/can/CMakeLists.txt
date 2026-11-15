# CAN 通信部分（独立维护）
# 基于 Linux SocketCAN 的 CBoard 实现，与 io/serial/ 下的 SerialBoard 相互独立。
# 仅当机器人仍通过 CAN 总线与 MCU 通信时才需要此模块，
# 链接此库的可执行文件见根 CMakeLists.txt。
add_library(io_can STATIC
    cboard.cpp
)

target_include_directories(io_can
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(io_can
  PUBLIC
    fmt::fmt
    yaml-cpp
)

# 步兵：达妙 USB-CAN（dm-device-sdk）封装。
# SDK 不在仓库内，用 -DDMCAN_SDK_ROOT=/path/to/dm-device-sdk 指定；
# 未指定时只告警并跳过，不影响其他目标。
if(SP_BUILD_SOLDIER)
  set(DMCAN_SDK_ROOT "" CACHE PATH "Root of the official dm-device-sdk checkout")
  if(NOT DMCAN_SDK_ROOT)
    message(WARNING "Skipping soldier: set -DDMCAN_SDK_ROOT=/path/to/dm-device-sdk")
  else()
    set(_dmcan_sdk "${DMCAN_SDK_ROOT}/C&C++/lib/v1.1.0")
    set(_dmcan_library "${_dmcan_sdk}/linux/${SP_DMCAN_ARCH}/libdm_device.so")
    if(NOT EXISTS "${_dmcan_sdk}/dmcan.h" OR NOT EXISTS "${_dmcan_library}")
      message(FATAL_ERROR "Missing Damiao SDK v1.1.0 header or library under ${_dmcan_sdk}")
    endif()

    add_library(dmcan_sdk SHARED IMPORTED)
    set_target_properties(dmcan_sdk PROPERTIES
      IMPORTED_LOCATION "${_dmcan_library}"
      INTERFACE_INCLUDE_DIRECTORIES "${_dmcan_sdk}")

    add_library(io_soldier_can STATIC
      soldier_can_protocol.cpp
      soldier_can_board.cpp
      soldier_can_damiao.cpp)
    target_link_libraries(io_soldier_can PRIVATE dmcan_sdk fmt::fmt yaml-cpp)
    install(FILES "${_dmcan_library}" DESTINATION bin)
  endif()
endif()
