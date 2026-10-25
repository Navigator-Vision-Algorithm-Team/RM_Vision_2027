# =====================================================================================
#  platform.cmake —— 本项目唯一的架构（x86_64 / aarch64）差异来源
#
#  约定：
#    * 除本文件外，任何 CMakeLists 与源码都不允许出现 CMAKE_SYSTEM_PROCESSOR 判断，
#      也不允许硬编码 lib/amd64、lib/arm64、3rd_part/openvino* 等与架构绑定的路径。
#      新增平台差异时，只改本文件，业务代码保持零改动。
#    * 交叉编译只需要指定工具链：
#        cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake
#      工具链文件把 CMAKE_SYSTEM_PROCESSOR 置为 aarch64，本文件随之整体切换。
#
#  对外提供：
#    SP_PROCESSOR           x86_64 / aarch64     归一化后的目标架构名（日志用）
#    SP_ARCH                amd64 / arm64        仓库内二进制目录名（相机 SDK 用）
#    SP_HIKROBOT_LIB_DIR    海康 MVS SDK 动态库目录
#    SP_MINDVISION_LIB_DIR  迈德威视 SDK 动态库目录
#    SP_OPENVINO_RUNTIME    项目内置 OpenVINO runtime 目录
#    SP_OPENVINO_BUNDLED    TRUE 表示链接并打包项目内置 OpenVINO（3rd_part/）
#    SP_OPENVINO_LIB_DIR    内置 OpenVINO 动态库目录（仅 BUNDLED 时有效）
# =====================================================================================

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _sp_proc)

if(_sp_proc MATCHES "^(x86_64|amd64|x64)$")
  set(SP_PROCESSOR "x86_64")
  set(SP_ARCH "amd64")
elseif(_sp_proc MATCHES "^(aarch64|arm64|armv8.*)$")
  set(SP_PROCESSOR "aarch64")
  set(SP_ARCH "arm64")
else()
  message(FATAL_ERROR
    "不支持的处理器架构：${CMAKE_SYSTEM_PROCESSOR}\n"
    "当前仅支持 x86_64 与 aarch64；交叉编译请使用 "
    "-DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake")
endif()

message(STATUS "[platform] 目标架构 ${SP_PROCESSOR}（仓库内二进制目录 ${SP_ARCH}）")

# ---------- 相机 SDK：仓库同时提供 amd64 / arm64 两套动态库 ----------
set(SP_HIKROBOT_LIB_DIR "${CMAKE_SOURCE_DIR}/io/hikrobot/lib/${SP_ARCH}")
set(SP_MINDVISION_LIB_DIR "${CMAKE_SOURCE_DIR}/io/mindvision/lib/${SP_ARCH}")

foreach(_sdk_lib
    "${SP_HIKROBOT_LIB_DIR}/libMvCameraControl.so"
    "${SP_MINDVISION_LIB_DIR}/libMVSDK.so")
  if(NOT EXISTS "${_sdk_lib}")
    message(FATAL_ERROR "[platform] 缺少 ${SP_PROCESSOR} 版的相机 SDK 动态库：${_sdk_lib}")
  endif()
endforeach()

# ---------- OpenVINO ----------
# 仓库自带两套官方运行库（架构相关，必须分开存放）：
#   3rd_part/openvino_2024.6.0/aarch64/runtime/  l_openvino_toolkit_ubuntu20_2024.6.0_arm64
#   3rd_part/openvino_2024.6.0/x86_64/runtime/   l_openvino_toolkit_ubuntu22_2024.6.0_x86_64
# 两套包里的 OpenVINOConfig.cmake / OpenVINOTargets-release.cmake 把 lib 路径写死为
# 各自架构（lib/aarch64 与 lib/intel64），无法合并成一份，故按 <架构>/runtime 分目录。
#
# 查找优先级：
#   1. -DOpenVINO_DIR=<path>        手动指定，最高优先级
#   2. 仓库内置 <架构>/runtime       默认路径，保证所有机器版本一致（2024.6.0）
#   3. /opt/intel/openvino_2024.6.0 手动解压安装的 Intel 官方包
#   4. 以上都没有                    交给 find_package 搜索系统（apt 安装，版本可能更新）
set(SP_OPENVINO_ROOT "${CMAKE_SOURCE_DIR}/3rd_part/openvino_2024.6.0")
set(SP_OPENVINO_RUNTIME "${SP_OPENVINO_ROOT}/${SP_PROCESSOR}/runtime")

# 各架构包内 lib/ 的子目录名不同
if(SP_PROCESSOR STREQUAL "x86_64")
  set(_sp_ov_lib_subdir "intel64")
else()
  set(_sp_ov_lib_subdir "aarch64")
endif()

if(NOT DEFINED OpenVINO_DIR)
  if(EXISTS "${SP_OPENVINO_RUNTIME}/cmake/OpenVINOConfig.cmake"
     AND EXISTS "${SP_OPENVINO_RUNTIME}/lib/${_sp_ov_lib_subdir}")
    set(OpenVINO_DIR "${SP_OPENVINO_RUNTIME}/cmake")
  elseif(EXISTS "/opt/intel/openvino_2024.6.0/runtime/cmake")
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
  endif()
endif()

if(OpenVINO_DIR STREQUAL "${SP_OPENVINO_RUNTIME}/cmake")
  set(SP_OPENVINO_BUNDLED TRUE)
  set(SP_OPENVINO_LIB_DIR "${SP_OPENVINO_RUNTIME}/lib/${_sp_ov_lib_subdir}")
else()
  set(SP_OPENVINO_BUNDLED FALSE)
  set(SP_OPENVINO_LIB_DIR "")
endif()

if(OpenVINO_DIR)
  message(STATUS "[platform] OpenVINO 配置目录 ${OpenVINO_DIR}")
else()
  message(STATUS "[platform] OpenVINO 使用系统安装（find_package 搜索）")
endif()
if(SP_OPENVINO_BUNDLED)
  message(STATUS "[platform] OpenVINO 运行库随安装包分发：${SP_OPENVINO_LIB_DIR}")
endif()

unset(_sp_proc)
unset(_sp_ov_lib_subdir)
