#include "hik_camera_manager.hpp"

#include <cstring>

#include "tools/logger.hpp"

namespace io
{
HikCameraManager & HikCameraManager::instance()
{
  static HikCameraManager manager;
  return manager;
}

void * HikCameraManager::create_handle(const std::string & serial_number)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enumerated_) rescan();

  const auto * info = find(serial_number);
  if (!info) {
    tools::logger()->warn(
      "[HikCameraManager] Camera with serial '{}' not found ({} device(s) enumerated)",
      serial_number, device_list_.nDeviceNum);
    return nullptr;
  }

  void * handle = nullptr;
  const unsigned int ret = MV_CC_CreateHandle(&handle, info);
  if (ret != MV_OK) {
    tools::logger()->warn(
      "[HikCameraManager] MV_CC_CreateHandle('{}') failed: {:#x}", serial_number, ret);
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

  const unsigned int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list_);

  // 无论成功与否都标记为已枚举，保证一个失败周期内最多枚举一次
  enumerated_ = true;

  if (ret != MV_OK || device_list_.nDeviceNum == 0) {
    tools::logger()->warn("[HikCameraManager] MV_CC_EnumDevices failed or no cameras: {:#x}", ret);
    return;
  }

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
