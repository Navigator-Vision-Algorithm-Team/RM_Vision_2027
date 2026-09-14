#ifndef IO__HIK_CAMERA_MANAGER_HPP
#define IO__HIK_CAMERA_MANAGER_HPP

#include <mutex>
#include <string>

#include "MvCameraControl.h"

namespace io
{
// 海康工业相机设备管理器：统一负责枚举设备与创建句柄。
// 多台相机共享同一个实例，避免各自枚举设备造成的 USB 总线竞争，
// 也避免每个 HikRobot 通过全局 static 偷偷共享枚举结果。
// 所有公有接口内部加锁，可被多台相机的守护线程并发调用。
class HikCameraManager
{
public:
  static HikCameraManager & instance();

  // 按序列号创建相机句柄（缓存中没有该序列号时会重新枚举一次）。
  // 成功返回有效句柄，失败返回 nullptr。
  void * create_handle(const std::string & serial_number);

  // 使下一次 create_handle 强制重新枚举设备，用于相机拔出后插回的自动恢复。
  void invalidate();

private:
  HikCameraManager() = default;
  HikCameraManager(const HikCameraManager &) = delete;
  HikCameraManager & operator=(const HikCameraManager &) = delete;

  // 以下两个函数都必须在持有 mutex_ 时调用
  void rescan();
  const MV_CC_DEVICE_INFO * find(const std::string & serial_number) const;

  std::mutex mutex_;
  MV_CC_DEVICE_INFO_LIST device_list_{};
  bool enumerated_{false};
};

}  // namespace io

#endif  // IO__HIK_CAMERA_MANAGER_HPP
