#include "hik_camera_manager.hpp"

#include <dlfcn.h>
#include <fmt/format.h>

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

#include "hik_error.hpp"
#include "tools/logger.hpp"

namespace io
{
namespace
{
// 海康 USB3 工业相机的 USB 厂商号
constexpr const char * kHikVendorId = "2bdf";

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

struct UsbDevice
{
  std::string sysfs_name;
  int busnum = -1;
  int devnum = -1;
};

// 直接从 sysfs 找海康 USB 设备：能判断“相机有没有被系统识别”，
// 不依赖 SDK 是否工作正常。
std::vector<UsbDevice> find_hik_usb_devices()
{
  namespace fs = std::filesystem;
  std::vector<UsbDevice> devices;

  std::error_code ec;
  const fs::path root("/sys/bus/usb/devices");
  if (!fs::is_directory(root, ec)) return devices;

  try {
    for (const auto & entry : fs::directory_iterator(root, ec)) {
      const auto dir = entry.path();

      std::ifstream vid_file(dir / "idVendor");
      std::string vid;
      if (!(vid_file >> vid) || vid != kHikVendorId) continue;

      UsbDevice dev;
      dev.sysfs_name = dir.filename().string();
      std::ifstream(dir / "busnum") >> dev.busnum;
      std::ifstream(dir / "devnum") >> dev.devnum;
      devices.push_back(dev);
    }
  } catch (const std::exception &) {
    // 诊断代码本身不允许影响主流程
  }

  return devices;
}

std::string usb_devnode(int busnum, int devnum)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "/dev/bus/usb/%03d/%03d", busnum, devnum);
  return buffer;
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
      // 海康官方规则文件名是 80-drivers-SDK-2bdf.rules（不含 mvs/hik），
      // 所以这里同时按厂商号 2bdf 与 sdk 匹配。
      if (name.find("mvs") != std::string::npos || name.find("hik") != std::string::npos ||
          name.find("mvusb") != std::string::npos || name.find("2bdf") != std::string::npos ||
          name.find("sdk") != std::string::npos) {
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
    hik_error_name(ret), sdk_directory());

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

  if (ret != MV_E_RESOURCE) return;

  // 传输层就位仍返回 MV_E_RESOURCE：用 sysfs 把“没插好 / 权限不足 / 版本不匹配”区分开。
  const auto devices = find_hik_usb_devices();
  if (devices.empty()) {
    tools::logger()->warn(
      "[HikCameraManager] 系统中没有发现海康 USB 设备（VID {}）：请检查相机供电、USB 线，"
      "以及是否插在可用的 USB3 口上（可先用 lsusb 确认）",
      kHikVendorId);
    return;
  }

  bool any_accessible = false;
  for (const auto & dev : devices) {
    const auto node = usb_devnode(dev.busnum, dev.devnum);

    struct stat st {};
    if (::stat(node.c_str(), &st) != 0) {
      tools::logger()->warn(
        "[HikCameraManager] 发现相机 {}，但设备节点 {} 不存在（检查 usbfs 挂载）",
        dev.sysfs_name, node);
      continue;
    }

    const bool rw = ::access(node.c_str(), R_OK | W_OK) == 0;
    any_accessible = any_accessible || rw;
    tools::logger()->warn(
      "[HikCameraManager] 发现相机 {} -> {}，权限 {:03o}（uid={} gid={}）：{}", dev.sysfs_name,
      node, st.st_mode & 0777, st.st_uid, st.st_gid,
      rw ? "当前用户可读写" : "当前用户无读写权限");
  }

  if (!any_accessible) {
    tools::logger()->warn(
      "[HikCameraManager] 结论：USB 权限不足（udev 规则未生效）。仓库已附带规则文件，执行：\n"
      "  sudo cp io/hikrobot/udev/51-mvs.rules /etc/udev/rules.d/\n"
      "  sudo udevadm control --reload-rules && sudo udevadm trigger\n"
      "然后重新插拔相机再试（/etc/udev/rules.d 中{}海康相关规则）",
      has_hik_udev_rules() ? "已存在" : "没有");
    return;
  }

  tools::logger()->warn(
    "[HikCameraManager] 结论：设备存在且当前用户可读写，但 SDK 仍枚举失败，"
    "多半是 libMvCameraControl.so 与 libMvUsb3vTL.so 版本不匹配。"
    "请用同一份 MVS 安装包的 lib/64（arm64 为 lib/aarch64）整目录替换 {} 后重试。",
    sdk_directory());
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
      hik_error_name(ret));
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
