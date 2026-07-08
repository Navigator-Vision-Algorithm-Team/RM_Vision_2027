#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class HikRobot : public CameraBase
{
public:
  HikRobot(
    double exposure_ms, double gain,
    const std::string & device_name, const std::string & serial_number,
    double frame_rate = 30.0, unsigned int transfer_size = 0);
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  double exposure_us_;
  double gain_;
  double frame_rate_;
  unsigned int transfer_size_;

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  void * handle_;
  std::thread capture_thread_;
  std::atomic<bool> capturing_;
  std::atomic<bool> capture_quit_;
  std::atomic<bool> first_frame_received_;
  tools::ThreadSafeQueue<CameraData> queue_;

  std::string serial_number_;

  void capture_start();
  void capture_stop();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);

  // SDK BGR callback — called from SDK internal thread when a frame is ready.
  // The SDK manages USB transfers internally, avoiding per-camera polling threads
  // competing for shared USB resources.
  static void __stdcall on_image_callback(
    unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo, void * pUser);
  void on_frame(unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo);
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP
