#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "io/sim/camera_source.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"
#include "usbcamera/usbcamera.hpp"

namespace io
{
namespace
{

/*
 * 全向相机的默认图像旋转，按安装方位一一对应：
 *   omni1 前右 -> 顺时针 90      omni2 前左 -> 逆时针 90
 *   omni3 后左 -> 顺时针 90      omni4 后右 -> 逆时针 90
 * 主相机及其它 sursign 不旋转（用户要求：只有全向相机做此处理）。
 */
int default_omni_rotation(const std::string & sursign)
{
  if (sursign == "omni1" || sursign == "omni3") return 90;
  if (sursign == "omni2" || sursign == "omni4") return -90;
  return 0;
}

}  // namespace

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

  // 全向相机的图像方向修正。yaml 的 rotate_<sursign> 可覆盖内置映射
  //（模组装反时把符号反过来即可）；主相机不读这个键，恒为 0。
  rotate_degrees_ = default_omni_rotation(sursign);
  if (!sursign.empty() && yaml[key("rotate")]) {
    const int value = yaml[key("rotate")].as<int>();
    if (value == 90 || value == -90 || value == 0) {
      rotate_degrees_ = value;
    } else {
      tools::logger()->warn(
        "[{}] rotate_{} = {} 无效（只支持 90 / -90 / 0），沿用内置值 {}°", device_name, sursign,
        value, rotate_degrees_);
    }
  }
  if (rotate_degrees_ != 0) {
    tools::logger()->info(
      "[{}] 图像旋转修正 {}°（{}）", device_name, rotate_degrees_,
      rotate_degrees_ > 0 ? "顺时针" : "逆时针");
  }

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, key("gamma"));
    auto vid_pid = tools::read<std::string>(yaml, key("vid_pid"));
    auto exposure_ms = tools::read<double>(yaml, key("exposure_ms"));
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid, device_name);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, key("gain"));
    auto exposure_ms = tools::read<double>(yaml, key("exposure_ms"));
    std::string serial_number = yaml[key("serial_number")] ? yaml[key("serial_number")].as<std::string>() : "";
    double frame_rate = yaml[key("frame_rate")] ? yaml[key("frame_rate")].as<double>() : 30.0;
    unsigned int transfer_size =
      yaml[key("transfer_size")] ? yaml[key("transfer_size")].as<unsigned int>() : 0;
    camera_ = std::make_unique<HikRobot>(
      exposure_ms, gain, device_name, serial_number, frame_rate,
      transfer_size);
  }

  else if (camera_name == "Usbcamera") {
    auto open_name = tools::read<std::string>(yaml, key("open_name"));
    camera_ = std::make_unique<USBCamera>(open_name, config_path);
  }

  // 伪相机 / 回放相机：业务代码只认 io::Camera，切换 real/fake/replay 无需改动
  else if (camera_name == "fake" || camera_name == "replay") {
    const auto mode =
      (camera_name == "replay") ? sim::SourceMode::Replay : sim::SourceMode::Fake;
    auto source = sim::make_camera_source(yaml, config_path, sursign, mode, true);
    camera_ = std::make_unique<sim::CameraSourceAdapter>(std::move(source), device_name);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }

  // 帧记录（可选）：与 SerialBoard 的串口记录写入同一目录，构成完整可回放会话
  auto record = sim::RecordConfig::load(yaml);
  if (record.enable) {
    recorder_ = std::make_unique<sim::CameraRecorder>(record.dir, record.save_images);
  }
}

Camera::~Camera() = default;

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
  if (img.empty()) return;

  // 先记录「未旋转」的原始帧：replay 模式同样经过本函数，若这里存的是旋转后的
  // 图像，回放时会被再转一次。
  if (recorder_) recorder_->record(img, timestamp, frame_id_++, img.type());

  // 全向相机的方向修正（主相机 rotate_degrees_ = 0，直接跳过，无额外开销）。
  if (rotate_degrees_ != 0) {
    cv::Mat rotated;
    cv::rotate(
      img, rotated,
      rotate_degrees_ > 0 ? cv::ROTATE_90_CLOCKWISE : cv::ROTATE_90_COUNTERCLOCKWISE);
    img = rotated;
  }
}

std::string Camera::device_name() const { return camera_->device_name(); }

}  // namespace io