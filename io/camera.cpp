#include "camera.hpp"

#include <stdexcept>

#ifdef SP_WITH_HIKROBOT
#include "hikrobot/hikrobot.hpp"
#endif
#ifdef SP_WITH_MINDVISION
#include "mindvision/mindvision.hpp"
#endif
#include "tools/logger.hpp"
#include "tools/yaml.hpp"
#ifdef SP_WITH_USBCAMERA
#include "usbcamera/usbcamera.hpp"
#endif

namespace io
{

cv::Mat CameraBase::read() { return {}; }

// note: the config file will change
Camera::Camera(const std::string & config_path, std::string sursign)
{
  auto key = [&](const std::string & k) { return sursign.empty() ? k : k + "_" + sursign; };

  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, key("camera_name"));

  // optional camera position for omniperception ("left", "right", "back", etc.)
  // Falls back to sursign (e.g. "main", "omni1", "omni2") so logs identify which camera
  std::string device_name;
  auto position_key = key("camera_position");
  if (yaml[position_key])
    device_name = yaml[position_key].as<std::string>();
  else if (!sursign.empty())
    device_name = sursign;

  // 各分支由 CMake 依据配置里的 camera_name* 决定是否编译：未启用的后端在预处理阶段
  // 整段消失，运行期只剩一次字符串比较。每个 #ifdef 块以 else 收尾，保证任意子集组合
  // 下 if/else 链都完整——缺哪个后端都不会留下悬空的 else。
#ifdef SP_WITH_MINDVISION
  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, key("gamma"));
    auto vid_pid = tools::read<std::string>(yaml, key("vid_pid"));
    auto exposure_ms = tools::read<double>(yaml, key("exposure_ms"));
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid, device_name);
  } else
#endif

#ifdef SP_WITH_HIKROBOT
  if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, key("gain"));
    auto exposure_ms = tools::read<double>(yaml, key("exposure_ms"));
    auto serial_number =
      yaml[key("serial_number")] ? yaml[key("serial_number")].as<std::string>() : std::string();
    auto frame_rate = yaml[key("frame_rate")] ? yaml[key("frame_rate")].as<double>() : 30.0;
    auto transfer_size =
      yaml[key("transfer_size")] ? yaml[key("transfer_size")].as<unsigned int>() : 0;
    camera_ = std::make_unique<HikRobot>(
      exposure_ms, gain, device_name, serial_number, frame_rate, transfer_size);
  } else
#endif

#ifdef SP_WITH_USBCAMERA
  if (camera_name == "Usbcamera") {
    auto open_name = tools::read<std::string>(yaml, key("open_name"));
    camera_ = std::make_unique<USBCamera>(open_name, config_path);
  } else
#endif

  {
    // 走到这里说明运行期配置与编译期裁剪不一致（构建用的不是这份配置），直接报错
    throw std::runtime_error(
      "camera_name 未在本次构建中编译：" + camera_name + "。请用 -DSP_CONFIG= 指向该配置后重新构建。");
  }

}

Camera::~Camera() = default;

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

std::string Camera::device_name() const { return camera_->device_name(); }

}  // namespace io