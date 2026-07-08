#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"
#include "usbcamera/usbcamera.hpp"

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
  std::string device_name;
  auto position_key = key("camera_position");
  if (yaml[position_key]) device_name = yaml[position_key].as<std::string>();

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, key("gamma"));
    auto vid_pid = tools::read<std::string>(yaml, key("vid_pid"));
    auto exposure_ms = tools::read<double>(yaml, key("exposure_ms"));
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid, device_name);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, key("gain"));
    auto vid_pid = tools::read<std::string>(yaml, key("vid_pid"));
    auto exposure_ms = tools::read<double>(yaml, key("exposure_ms"));
    int device_index = yaml[key("device_index")] ? yaml[key("device_index")].as<int>() : 0;
    std::string serial_number = yaml[key("serial_number")] ? yaml[key("serial_number")].as<std::string>() : "";
    double frame_rate = yaml[key("frame_rate")] ? yaml[key("frame_rate")].as<double>() : 30.0;
    unsigned int grab_timeout_ms =
      yaml[key("grab_timeout_ms")] ? yaml[key("grab_timeout_ms")].as<unsigned int>() : 100;
    unsigned int transfer_size =
      yaml[key("transfer_size")] ? yaml[key("transfer_size")].as<unsigned int>() : 0;
    camera_ = std::make_unique<HikRobot>(
      exposure_ms, gain, vid_pid, device_name, device_index, serial_number, frame_rate,
      grab_timeout_ms, transfer_size);
  }

  else if (camera_name == "Usbcamera") {
    auto open_name = tools::read<std::string>(yaml, key("open_name"));
    camera_ = std::make_unique<USBCamera>(open_name, config_path);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

std::string Camera::device_name() const { return camera_->device_name(); }

}  // namespace io