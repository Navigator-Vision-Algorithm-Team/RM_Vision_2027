#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
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

  std::string serial_number_;

  // 相机生命周期（打开、参数下发、掉流巡检、停流、重启）全部由这一个线程负责，
  // 海康 SDK 回调线程只处理帧数据，两者通过下面的原子量与队列通信。
  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_{false};

  void * handle_{nullptr};              // 仅守护线程访问
  std::atomic<bool> opened_{false};     // MV_CC_OpenDevice 是否成功
  std::atomic<bool> grabbing_{false};   // MV_CC_StartGrabbing 是否成功
  std::atomic<bool> stopping_{false};   // 相机正在 shutdown，回调应立即返回
  std::atomic<bool> first_frame_received_{false};
  std::atomic<uint64_t> frame_counter_{0};  // 真实掉流检测：每收到一帧自增
  std::atomic<uint64_t> restart_count_{0};  // 重启次数，用于排查 USB 稳定性

  // healthy 状态不单独保存，直接由 frame_counter_ 推导。
  // 队列满时丢弃旧帧、保留最新帧，回调线程永不阻塞。
  tools::ThreadSafeQueue<CameraData, true> queue_;

  bool capture_start();
  void capture_stop();
  void watchdog_frames();
  void dump_camera_config();

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
