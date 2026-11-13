#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io::sim
{
class CameraRecorder;
}

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual cv::Mat read();
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  virtual std::string device_name() const { return device_name_; }

protected:
  std::string device_name_;
};

class Camera
{
public:
  Camera(const std::string & config_path, std::string sursign = "");
  ~Camera();
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  std::string device_name() const;

  // 该相机的图像旋转修正（度）：90 = 顺时针，-90 = 逆时针，0 = 不旋转。
  // 只有全向相机（sursign = omni1..omni4）默认非零，主相机恒为 0。
  // PnP 侧需要用它把标定内参变换到旋转后的图像坐标系（见 omni_pnp 里的
  // rotate_camera_matrix），否则解出的角度会整体偏 90°。
  int rotation() const { return rotate_degrees_; }

private:
  std::unique_ptr<CameraBase> camera_;
  // 记录相机帧与时间戳（record.enable=true 时），供 replay 模式复现
  std::unique_ptr<sim::CameraRecorder> recorder_;
  uint64_t frame_id_ = 0;
  int rotate_degrees_ = 0;
};

}  // namespace io

#endif  // IO__CAMERA_HPP