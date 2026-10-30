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
