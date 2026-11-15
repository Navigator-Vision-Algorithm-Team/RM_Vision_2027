cmake_minimum_required(VERSION 3.16)

# FFmpeg 的发现与链接在顶层 CMakeLists.txt 完成：
#   · 发现：pkg_check_modules(FFMPEG REQUIRED libavcodec libavformat libavutil libswscale)
#   · 链接：link_libraries(${FFMPEG_LIBRARIES})，对所有可执行目标生效
# 这里不能再声明 target_link_libraries(tools PUBLIC ...)：tools 是 OBJECT 库，
# 其链接接口不会传播到最终可执行文件，写了也只会制造"已声明依赖"的错觉，
# 实际仍会在链接期报 undefined reference to `av_*` / `sws_*`。

add_library(tools OBJECT 
    cpu_core_manager.cpp
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    # 录像：FFmpeg 硬件 H.264 编码（h264_vaapi），依赖由顶层统一链接
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)
