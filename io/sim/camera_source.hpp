#ifndef IO__SIM__CAMERA_SOURCE_HPP
#define IO__SIM__CAMERA_SOURCE_HPP

// 相机数据源抽象：业务算法只依赖此接口。
//   RealHikCameraSource  — 真实相机（海康/迈德威视/USB，由 io::Camera 工厂决定）
//   FakeCameraSource     — 伪帧：合成图案 / 图片序列 / 视频，可注入丢帧、重复、黑屏、过曝、噪声、延迟
//   ReplayCameraSource   — 按录制时间戳回放相机帧
// 伪帧的分辨率、像素格式、帧率、时间戳、帧号与真实相机保持一致。

#include <opencv2/opencv.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "io/camera.hpp"
#include "io/sim/common.hpp"
#include "io/sim/recorder.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io::sim
{

struct Frame
{
  TimePoint timestamp{};
  uint64_t frame_id = 0;
  int width = 0;
  int height = 0;
  int format = CV_8UC3;  // cv::Mat::type()
  cv::Mat data;
  SourceMode mode = SourceMode::Fake;
  bool injected = false;  // 是否来自注入的异常帧
};

class CameraSource
{
public:
  using Ptr = std::unique_ptr<CameraSource>;
  virtual ~CameraSource() = default;

  virtual void open() = 0;
  // timeout_ms <= 0 表示阻塞直到有帧
  virtual bool read(Frame & out, int timeout_ms = 0) = 0;
  virtual void close() = 0;
  virtual std::string name() const = 0;
  virtual SourceMode mode() const = 0;
  virtual std::string device_name() const = 0;
};

// ============================ 真实相机 ============================
class RealHikCameraSource : public CameraSource
{
public:
  RealHikCameraSource(std::string config_path, std::string sursign);

  void open() override;
  bool read(Frame & out, int timeout_ms) override;
  void close() override;
  std::string name() const override { return "RealCamera"; }
  SourceMode mode() const override { return SourceMode::Real; }
  std::string device_name() const override;

private:
  std::string config_path_;
  std::string sursign_;
  std::unique_ptr<io::Camera> camera_;
  uint64_t frame_id_ = 0;
};

// ============================ 伪相机 ============================
// 高帧率设计（支持 5 路 x 300fps）：
//   1) 底图只渲染一次（build_base），帧缓冲由「池 + 单槽 holder」复用；
//   2) 产帧路径零全帧拷贝：每帧只把 overlay 小块从底图复原再画计数文字；
//   3) 与真实相机一致，「永远给最新帧」：新帧覆盖 holder 中的旧帧，被覆盖的算 overwritten；
//   4) 只有真正被消费的帧才在 read() 里 clone 一份给业务代码独占（消费端通常只有几 fps，
//      这点拷贝可忽略；产帧端 1500fps 的路径上没有任何全帧写入）。
class FakeCameraSource : public CameraSource
{
public:
  FakeCameraSource(
    const CameraSourceConfig & cfg, const FaultConfig & fault, int seed, std::string tag = "");
  ~FakeCameraSource() override;

  void open() override;
  bool read(Frame & out, int timeout_ms) override;
  void close() override;
  std::string name() const override { return "FakeCamera"; }
  SourceMode mode() const override { return SourceMode::Fake; }
  std::string device_name() const override { return device_name_; }

private:
  void run();
  cv::Mat next_image();

  // —— 高帧率产帧路径 ——
  void build_base();                   // 一次性渲染底图 + overlay 复原块
  void render_overlay(cv::Mat & buf);  // 每帧只画这一小块
  int take_free_buffer();              // 取空闲池缓冲；池内全忙返回 -1
  void publish_frame(const cv::Mat & img, int pool_idx, bool injected);

  CameraSourceConfig cfg_;
  FaultConfig fault_;
  FaultInjector injector_;
  std::string device_name_ = "fake";
  std::string tag_ = "cam";

  // 图片序列 / 视频
  std::vector<std::string> files_;
  size_t file_index_ = 0;
  cv::VideoCapture video_;

  // 预渲染底图 + overlay 复原块
  cv::Mat base_;
  cv::Mat patch_;
  cv::Rect overlay_roi_{0, 0, 0, 0};

  // 帧缓冲池 + 单槽 holder
  std::vector<cv::Mat> pool_;
  std::vector<bool> pool_free_;
  Frame holder_;
  int holder_idx_ = -1;
  std::mutex holder_mutex_;
  std::condition_variable holder_cv_;
  cv::Mat freeze_;  // 冻结注入用，仅在 camera_freeze_rate > 0 时分配

  std::thread thread_;
  std::atomic<bool> quit_{false};

  uint64_t frame_id_ = 0;
  long long emitted_ = 0;
  long long delivered_ = 0;
  long long overwritten_ = 0;  // 产得比消费快，被新帧覆盖掉的帧数（消费端瓶颈指标）
  long long dropped_ = 0;      // 注入的丢帧
  long long starved_ = 0;      // 池内无空闲缓冲而放弃的帧
  long long duplicated_ = 0;
  long long injected_count_ = 0;
};

// ============================ 回放相机 ============================
class ReplayCameraSource : public CameraSource
{
public:
  ReplayCameraSource(const CameraSourceConfig & cfg, const FaultConfig & fault, int seed);
  ~ReplayCameraSource() override;

  void open() override;
  bool read(Frame & out, int timeout_ms) override;
  void close() override;
  std::string name() const override { return "ReplayCamera"; }
  SourceMode mode() const override { return SourceMode::Replay; }
  std::string device_name() const override { return device_name_; }

private:
  void run();

  CameraSourceConfig cfg_;
  bool loop_ = true;
  std::string device_name_ = "replay";
  CameraReplayReader reader_;

  std::thread thread_;
  std::atomic<bool> quit_{false};
  tools::ThreadSafeQueue<Frame, true> queue_{1};
  bool ready_ = false;
  long long emitted_ = 0;
};

// ============================ CameraBase 适配器 ============================
// 让 Fake/Replay 相机可以直接被现有 io::Camera 工厂返回，业务代码无需改动。
class CameraSourceAdapter : public io::CameraBase
{
public:
  CameraSourceAdapter(CameraSource::Ptr source, std::string device_name);
  ~CameraSourceAdapter() override;

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  std::string device_name() const override;

private:
  CameraSource::Ptr source_;
};

// 按配置创建相机数据源。use_force=true 时忽略配置中的 camera_mode，强制使用 force。
CameraSource::Ptr make_camera_source(
  const YAML::Node & yaml, const std::string & config_path, const std::string & sursign = "",
  SourceMode force = SourceMode::Fake, bool use_force = false);

}  // namespace io::sim

#endif  // IO__SIM__CAMERA_SOURCE_HPP
