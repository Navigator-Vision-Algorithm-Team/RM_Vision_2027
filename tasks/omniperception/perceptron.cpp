#include "perceptron.hpp"

#include <fmt/core.h>

#include <chrono>
#include <memory>
#include <thread>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace omniperception
{
Perceptron::Perceptron(
  const std::vector<OmniCameraConfig> & omni_configs, const std::string & config_path,
  std::shared_ptr<auto_aim::YOLO> yolo, std::vector<tools::Recorder *> recorders)
: detection_queue_(10),
  yolo_(std::move(yolo)),
  decider_(config_path),
  stop_flag_(false),
  omni_recorders_(std::move(recorders))
{
  if (omni_configs.empty()) {
    tools::logger()->info("Perceptron initialized with 0 cameras.");
    return;
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 创建线程进行并行推理，所有相机共享同一个 YOLO
  for (size_t i = 0; i < omni_configs.size(); i++) {
    threads_.emplace_back(
      [this, cfg = omni_configs[i], i] { parallel_infer(cfg, yolo_, i); });
  }

  tools::logger()->info("Perceptron initialized with {} cameras.", omni_configs.size());
}

Perceptron::~Perceptron()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_flag_ = true;
  }
  condition_.notify_all();

  for (auto & t : threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
  tools::logger()->info("Perceptron destructed.");
}

std::vector<DetectionResult> Perceptron::get_detection_queue()
{
  std::vector<DetectionResult> result;
  DetectionResult temp;

  while (!detection_queue_.empty()) {
    detection_queue_.pop(temp);
    result.push_back(std::move(temp));
  }

  return result;
}

void Perceptron::parallel_infer(
  const OmniCameraConfig & cfg, const std::shared_ptr<auto_aim::YOLO> & yolo, size_t index)
{
  if (!cfg.camera) {
    tools::logger()->error("[Perceptron] Camera pointer is null!");
    return;
  }
  tools::logger()->info(
    "[Perceptron] Thread started for camera '{}' mount_yaw={:.1f} mount_pitch={:.1f}",
    cfg.camera->device_name(), cfg.mount_yaw, cfg.mount_pitch);

  tools::Recorder * recorder = nullptr;
  if (index < omni_recorders_.size()) recorder = omni_recorders_[index];

  int frame_count = 0;
  int detect_count = 0;
  auto last_log = std::chrono::steady_clock::now();

  try {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;
      }

      cv::Mat img;
      std::chrono::steady_clock::time_point ts;
      cfg.camera->read(img, ts);
      if (img.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }

      frame_count++;
      auto armors = yolo->detect(img);

      // --- draw detection boxes and record ---
      if (recorder) {
        cv::Mat annotated = img.clone();
        for (const auto & armor : armors) {
          std::vector<cv::Point> pts;
          for (const auto & p : armor.points) pts.emplace_back(p);
          cv::polylines(annotated, pts, true, {0, 255, 0}, 2);
          auto label = fmt::format(
            "{} {:.2f}", auto_aim::ARMOR_NAMES[armor.name], armor.confidence);
          cv::putText(
            annotated, label, armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.6,
            {0, 255, 0}, 1);
        }
        // use zero quaternion for omni cameras (no gimbal pose)
        recorder->record(annotated, Eigen::Quaterniond::Identity(), ts);
      }

      if (!armors.empty()) {
        detect_count++;
        auto da = decider_.delta_angle(armors, cfg);

        DetectionResult dr;
        dr.armors = std::move(armors);
        dr.timestamp = ts;
        dr.delta_yaw = da[0] / 57.3;
        dr.delta_pitch = da[1] / 57.3;
        detection_queue_.push(dr);
      }

      // --- periodic status (0.2 Hz) ---
      auto now = std::chrono::steady_clock::now();
      if (tools::delta_time(now, last_log) >= 5.0) {
        tools::logger()->info(
          "[Perceptron] camera='{}' frames={} detects={} detect_rate={:.1f}%",
          cfg.camera->device_name(), frame_count, detect_count,
          frame_count > 0 ? 100.0 * detect_count / frame_count : 0.0);
        frame_count = 0;
        detect_count = 0;
        last_log = now;
      }
    }
  } catch (const std::exception & e) {
    tools::logger()->error("[Perceptron] Exception in parallel_infer: {}", e.what());
  }
}

}  // namespace omniperception
