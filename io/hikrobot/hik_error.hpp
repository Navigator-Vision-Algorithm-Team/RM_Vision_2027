#ifndef IO__HIKROBOT__HIK_ERROR_HPP
#define IO__HIKROBOT__HIK_ERROR_HPP

#include <string>

#include "MvErrorDefine.h"

namespace io
{
// 把 MVS 返回码转成可读名字。日志里同时打印十六进制与名字，便于现场判断：
//   0x80000006 MV_E_RESOURCE   多为缺 USB3 传输层 / 权限不足
//   0x80000009 MV_E_VERSION    控制库与传输层版本不匹配（升级 SDK 时容易踩）
inline std::string hik_error_name(unsigned int ret)
{
  switch (ret) {
    case MV_OK:
      return "MV_OK";
    case MV_E_HANDLE:
      return "MV_E_HANDLE";
    case MV_E_SUPPORT:
      return "MV_E_SUPPORT";
    case MV_E_PARAMETER:
      return "MV_E_PARAMETER";
    case MV_E_RESOURCE:
      return "MV_E_RESOURCE";
    case MV_E_NODATA:
      return "MV_E_NODATA";
    case MV_E_PRECONDITION:
      return "MV_E_PRECONDITION";
    case MV_E_VERSION:
      return "MV_E_VERSION";
    case MV_E_NOENOUGH_BUF:
      return "MV_E_NOENOUGH_BUF";
    case MV_E_LOAD_LIBRARY:
      return "MV_E_LOAD_LIBRARY";
    default:
      return "UNKNOWN";
  }
}
}  // namespace io

#endif  // IO__HIKROBOT__HIK_ERROR_HPP
