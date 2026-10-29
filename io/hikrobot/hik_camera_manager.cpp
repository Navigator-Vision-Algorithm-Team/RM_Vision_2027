#include "hik_camera_manager.hpp"

#include <dlfcn.h>
#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>

#include "tools/logger.hpp"

namespace io
{
namespace
{
std::filesystem::path sdk_path()
{
  Dl_info info{};
  if (dladdr(reinterpret_cast<void *>(&MV_CC_EnumDevices), &info) == 0 || !info.dli_fname) {
    return {};
  }
  return std::filesystem::path(info.dli_fname).parent_path();
}

std::string sdk_version_string()
{
  const unsigned int v = MV_CC_GetSDKVersion();
  return fmt::format("{}.{}.{}.{}", (v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
}
}  // namespace

HikCameraManager & HikCameraManager::instance()
{
  static HikCameraManager manager;
  return manager;
}

std::string HikCameraManager::sdk_directory()
{
  const auto dir = sdk_path();
  return dir.empty() ? std::string("<unknown>") : dir.string();
}

std::string HikCameraManager::error_name(unsigned int ret)
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
    case MV_E_NOENOUGH_BUF:
      return "MV_E_NOENOUGH_BUF";
    case MV_E_LOAD_LIBRARY:
      return "MV_E_LOAD_LIBRARY";
    default:
      return "UNKNOWN";
  }
}

bool HikCameraManager::has_hik_udev_rules()
{
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dir("/etc/udev/rules.d");
  if (!fs::is_directory(dir, ec)) return false;

  try {
    for (const auto & entry : fs::directory_iterator(dir, ec)) {
      std::string name = entry.path().filename().string();
      std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      if (name.find("mvs") != std::string::npos || name.find("hik") != std::string::npos ||
          name.find("mvusb") != std::string::npos) {
        return true;
      }
    }
  } catch (const std::exception &) {
    return false;
  }

  return false;
}

void HikCameraManager::report_enum_failure(unsigned int ret)
{
  const auto dir = sdk_path();
  const auto transport = dir.empty() ? std::filesystem::path() : dir / "libMvUsb3vTL.so";

  tools::logger()->warn(
    "[HikCameraManager] MV_CC_EnumDevices 失败：{:#x}（{}），SDK 目录：{}", ret,
    error_name(ret), sdk_directory());

  if (!transport.empty() && !std::filesystem::exists(transport)) {
    // 这是最常见的“枚举不到相机”：海康 SDK 只从 libMvCameraControl.so 所在目录
    // 加载 USB3 传输层，缺了它枚举会直接返回 MV_E_RESOURCE。
    tools::logger()->warn(
      "[HikCameraManager] 缺少 USB3 传输层：{}（与 libMvCameraControl.so 必须同目录）",
      transport.string());
    tools::logger()->warn(
      "[HikCameraManager] 修复：把 MVS 安装目录 lib/64（arm64 为 lib/aarch64）下的 "
      "libMvUsb3vTL.so 复制到上面这个目录；或安装官方 MVS 后重新运行。");
    return;
  }

  if (ret == MV_E_RESOURCE) {
    tools::logger()->warn(
      "[HikCameraManager] 传输层已就位，但仍返回 MV_E_RESOURCE：通常是 USB 权限不足。");
    if (!has_hik_udev_rules()) {
      tools::logger()->warn(
        "[HikCameraManager] /etc/udev/rules.d 下没有找到 MVS/Hik 相关规则，"
        "请从 MVS 安装包复制 udev 规则后执行：\n"
        "  sudo cp <MVS安装包>/udev/51-mvs.rules /etc/udev/rules.d/\n"
        "  sudo udevadm control --reload-rules && sudo udevadm trigger");
    }
  }
}

void * HikCameraManager::create_handle(const std::string & serial_number)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enumerated_) rescan();

  const auto * info = find(serial_number);
  if (!info) {
    if (device_list_.nDeviceNum > 0) {
      // 有相机但序列号对不上：说明配置文件写错了
      tools::logger()->warn(
        "[HikCameraManager] Camera with serial '{}' not found ({} device(s) enumerated)",
        serial_number, device_list_.nDeviceNum);
    }
    return nullptr;
  }

  void * handle = nullptr;
  const unsigned int ret = MV_CC_CreateHandle(&handle, info);
  if (ret != MV_OK) {
    tools::logger()->warn(
      "[HikCameraManager] MV_CC_CreateHandle('{}') failed: {:#x} ({})", serial_number, ret,
      error_name(ret));
    return nullptr;
  }

  return handle;
}

void HikCameraManager::invalidate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  enumerated_ = false;
}

void HikCameraManager::rescan()
{
  memset(&device_list_, 0, sizeof(device_list_));

  // 无论成功与否都标记为已枚举，保证一个失败周期内最多枚举一次
  enumerated_ = true;

  if (!logged_sdk_info_) {
    logged_sdk_info_ = true;
    tools::logger()->info(
      "[HikCameraManager] MVS SDK {}，运行库目录 {}", sdk_version_string(), sdk_directory());
  }

  const unsigned int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list_);

  if (ret != MV_OK) {
    // 同一个错误码只详细打印一次，避免相机守护线程重连时刷屏
    if (ret != last_enum_error_) {
      last_enum_error_ = ret;
      logged_no_camera_ = false;
      report_enum_failure(ret);
    }
    return;
  }

  last_enum_error_ = MV_OK;

  if (device_list_.nDeviceNum == 0) {
    if (!logged_no_camera_) {
      logged_no_camera_ = true;
      tools::logger()->warn(
        "[HikCameraManager] 未枚举到 USB 相机：SDK 工作正常，请检查相机是否上电、"
        "USB 线是否插好（如仅某台相机缺失，核对配置里的序列号）");
    }
    return;
  }

  logged_no_camera_ = false;

  for (unsigned int i = 0; i < device_list_.nDeviceNum; i++) {
    const auto * info = &device_list_.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo;
    tools::logger()->info(
      "[HikCameraManager] Found camera [{}]: serial='{}' model='{}'", i,
      reinterpret_cast<const char *>(info->chSerialNumber),
      reinterpret_cast<const char *>(info->chModelName));
  }
}

const MV_CC_DEVICE_INFO * HikCameraManager::find(const std::string & serial_number) const
{
  for (unsigned int i = 0; i < device_list_.nDeviceNum; i++) {
    const auto * info = device_list_.pDeviceInfo[i];
    const auto * sn = reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
    if (serial_number == sn) return info;
  }

  return nullptr;
}

}  // namespace io
