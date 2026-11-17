# =====================================================================================
#  配置驱动的编译裁剪
#
#  职责：在 CMake 配置阶段读取部署用的 YAML，取其中的相机关键词（camera_name*）与
#        YOLO 关键词（yolo_name*），映射成「需要编译的包集合」，供各子目录的源列表与
#        宏定义使用。
#
#  对外提供：
#    SP_CAMERAS_ENABLED   需要编译的相机包，取值 hikrobot / mindvision / usbcamera 的子集
#    SP_YOLOS_ENABLED     需要编译的 YOLO 包，取值 yolov5 / yolov8 / yolo11 的子集
#    sp_resolve_build_features(<config_path>)   执行解析与校验，结果写入上述变量
#
#  规则与边界：
#    * 只做配置期文本解析，不生成任何运行时代码，也不引入运行时判断；
#    * 关键词全部小写归一后比较，`Usbcamera` 与 `usbcamera` 视为同一个包；
#    * 配置里出现未知关键词 → 配置期 FATAL_ERROR 并列出合法取值，
#      避免打出一个缺少所需后端、只能在赛场上运行期报错的二进制；
#    * 同一关键词出现多次（多相机）→ 去重后取并集；
#    * 一个关键词都没有（键缺失，例如标定工具用的精简配置）→ 回退为「全部编译」并告警，
#      保证非部署类配置仍可构建。
# =====================================================================================

# 已知的包集合，配置里出现其他取值即为错误
set(SP_KNOWN_CAMERAS "hikrobot" "mindvision" "usbcamera")
set(SP_KNOWN_YOLOS "yolov5" "yolov8" "yolo11")

# 功能：从配置文件里取出以某个前缀开头的键的取值集合
# 输入：
#   cfg_file    配置文件绝对路径
#   key_regex   匹配键名的正则，如 "^[ \t]*camera_name"
#   out_var     输出变量名，结果为小写、去重、去空的值列表
# 输出：写入 out_var（PARENT_SCOPE），文件不存在或没有匹配时为空列表
# 备注：按行做正则提取，只支持本仓库配置使用的 "key: value  # 注释" 形式
function(sp_read_key_values cfg_file key_regex out_var)
  set(values "")
  if(NOT EXISTS "${cfg_file}")
    set(${out_var} "" PARENT_SCOPE)
    return()
  endif()

  file(STRINGS "${cfg_file}" lines REGEX "${key_regex}")
  foreach(line IN LISTS lines)
    # 去掉行内注释，再取第一个冒号之后的内容作为取值
    string(REGEX REPLACE "#.*$" "" body "${line}")
    string(REGEX MATCH "^[^:]*:(.*)$" _ "${body}")
    set(raw "${CMAKE_MATCH_1}")
    string(REPLACE "\"" "" raw "${raw}")
    string(REPLACE "'" "" raw "${raw}")
    string(STRIP "${raw}" raw)
    if(NOT raw STREQUAL "")
      string(TOLOWER "${raw}" raw)
      list(APPEND values "${raw}")
    endif()
  endforeach()

  if(values)
    list(REMOVE_DUPLICATES values)
  endif()
  set(${out_var} "${values}" PARENT_SCOPE)
endfunction()

# 功能：校验关键词集合，遇到未知取值直接终止配置
# 输入：
#   kind      类别名（用于报错文案），如 "相机"
#   values    实际取到的关键词列表
#   known     合法取值列表
# 输出：无
# 备注：报错信息同时给出配置路径与合法取值，便于当场修正
function(sp_validate_keywords kind values known)
  foreach(v IN LISTS values)
    if(NOT v IN_LIST known)
      string(REPLACE ";" " | " hint "${known}")
      message(FATAL_ERROR
        "[features] 配置 ${SP_CONFIG} 中的${kind}关键词 \"${v}\" 无法识别。\n"
        "合法取值：${hint}\n"
        "请修正配置，或在 -DSP_CONFIG= 指向正确的部署配置后重新配置。")
    endif()
  endforeach()
endfunction()

# 功能：解析配置并确定需要编译的相机包与 YOLO 包
# 输入：
#   cfg_path  部署配置路径（建议传绝对路径）
# 输出：设置 SP_CAMERAS_ENABLED / SP_YOLOS_ENABLED（CACHE INTERNAL，供各子目录读取）
# 备注：缺关键词时回退为全部编译并告警；未知关键词直接 FATAL_ERROR
function(sp_resolve_build_features cfg_path)
  set(_cfg "${cfg_path}")
  if(NOT IS_ABSOLUTE "${_cfg}")
    set(_cfg "${CMAKE_SOURCE_DIR}/${_cfg}")
  endif()

  if(NOT EXISTS "${_cfg}")
    message(FATAL_ERROR "[features] 找不到驱动编译裁剪的配置文件：${_cfg}")
  endif()

  sp_read_key_values("${_cfg}" "^[ \t]*camera_name" _cameras)
  sp_read_key_values("${_cfg}" "^[ \t]*yolo_name" _yolos)

  if(NOT _cameras)
    message(WARNING
      "[features] ${_cfg} 中没有 camera_name* 键，回退为编译全部相机包。\n"
      "           若这是部署配置，请补上 camera_name_* 以裁剪无用的相机 SDK。")
    set(_cameras ${SP_KNOWN_CAMERAS})
  else()
    sp_validate_keywords("相机" "${_cameras}" "${SP_KNOWN_CAMERAS}")
  endif()

  if(NOT _yolos)
    message(WARNING
      "[features] ${_cfg} 中没有 yolo_name 键，回退为编译全部 YOLO 包。\n"
      "           若这是部署配置，请补上 yolo_name 以裁剪无用的检测后端。")
    set(_yolos ${SP_KNOWN_YOLOS})
  else()
    sp_validate_keywords("YOLO" "${_yolos}" "${SP_KNOWN_YOLOS}")
  endif()

  set(SP_CAMERAS_ENABLED "${_cameras}" CACHE INTERNAL "配置驱动的相机包集合" FORCE)
  set(SP_YOLOS_ENABLED "${_yolos}" CACHE INTERNAL "配置驱动的 YOLO 包集合" FORCE)

  string(REPLACE ";" ", " _camera_text "${_cameras}")
  string(REPLACE ";" ", " _yolo_text "${_yolos}")
  message(STATUS "[features] 配置 ${SP_CONFIG}")
  message(STATUS "[features]   相机包：${_camera_text}")
  message(STATUS "[features]   YOLO 包：${_yolo_text}")
endfunction()
