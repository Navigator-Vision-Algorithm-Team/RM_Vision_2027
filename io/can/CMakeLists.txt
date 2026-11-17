# =====================================================================================
#  达妙 USB-CAN 驱动（io_soldier_can）
#
#  职责：通过官方 dm-device-sdk 读写步兵的达妙电机与板卡数据。
#  状态：默认不参与构建（SP_BUILD_SOLDIER 默认 OFF）。
#        中转进程目前对外发布的是 0x05xx 协议数据，没有达妙专属读取通路；
#        视觉侧的统一通信入口是 io::RelayClient（iceoryx2），不再直接持有 CAN 设备。
#
#  恢复条件（三者缺一不可）：
#    1) 配置时打开 -DSP_BUILD_SOLDIER=ON，并给 -DDMCAN_SDK_ROOT=/path/to/dm-device-sdk；
#    2) tasks/soldier/soldier.cpp 的收发改为 RelayClient：上行取 IMU / 机器人状态，下行发 AimCmd；
#    3) 恢复根 CMakeLists 中 soldier / soldier_debug 目标的链接项。
#  在此之前，SocketCAN 的 CBoard 实现已按规范删除，本目录只保留达妙驱动源码备查。
# =====================================================================================

set(DMCAN_SDK_ROOT "" CACHE PATH "Root of the official dm-device-sdk checkout")

if(SP_BUILD_SOLDIER)
  if(NOT DMCAN_SDK_ROOT)
    message(FATAL_ERROR
      "SP_BUILD_SOLDIER 为 ON，但未指定 DMCAN_SDK_ROOT。\n"
      "请给 -DDMCAN_SDK_ROOT=/path/to/dm-device-sdk，或把 SP_BUILD_SOLDIER 设为 OFF。")
  endif()

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
else()
  message(STATUS "io_soldier_can 未启用（SP_BUILD_SOLDIER=OFF），达妙驱动源码保留但不参与构建")
endif()
