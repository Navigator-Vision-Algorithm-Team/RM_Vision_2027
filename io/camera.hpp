#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

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
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  std::string device_name() const;

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP