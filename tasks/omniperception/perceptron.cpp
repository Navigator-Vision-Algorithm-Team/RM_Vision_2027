#include "perceptron.hpp"

#include <chrono>
#include <fmt/core.h>
#include <memory>
#include <thread>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/recorder.hpp"

namespace omniperception
{
Perceptron::Perceptron(
  const std::vector<OmniCameraConfig> & omni_configs, const std::string & config_path,
  std::shared_ptr<auto_aim::YOLO> yolo)
: detection_queue_(10), yolo_(std::move(yolo)), decider_(config_path), stop_flag_(false)
{
  if (omni_configs.empty()) {
    tools::logger()->info("Perceptron initialized with 0 cameras.");
    return;
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 创建线程进行并行推理，所有相机共享同一个 YOLO
  for (size_t i = 0; i < omni_configs.size(); i++) {
    threads_.emplace_back(
      [this, cfg = omni_configs[i]] { parallel_infer(cfg, yolo_); });
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
  const OmniCameraConfig & cfg, const std::shared_ptr<auto_aim::YOLO> & yolo)
{
  if (!cfg.camera) {
    tools::logger()->error("Camera pointer is null!");
    return;
  }
  try {
    while (true) {
      cv::Mat img;
      std::chrono::steady_clock::time_point ts;

      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;
      }

      cfg.camera->read(img, ts);
      if (img.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }

      auto armors = yolo->detect(img);
      cv::Mat display_img = img.clone();
      for (const auto & armor : armors) {
        cv::rectangle(display_img, armor.box, cv::Scalar(0, 255, 0), 2);
        std::string label = fmt::format(
          "{} {} {:.2f}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
          armor.confidence);
        cv::putText(display_img, label, cv::Point(armor.box.x, armor.box.y - 5),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        for (const auto & pt : armor.points) {
          cv::circle(display_img, pt, 2, cv::Scalar(0, 0, 255), -1);
        }
      }

      if (cfg.recorder && !display_img.empty()) {
        cfg.recorder->record(display_img, Eigen::Quaterniond::Identity(), ts);
      }

      if (!armors.empty()) {
        auto da = decider_.delta_angle(armors, cfg);

        // Calibration diagnostic: log raw detection data for each detection.
        // centre_norm (0=left, 1=right), pixel centre, and computed yaw.
        auto & a = armors.front();
        tools::logger()->info(
          "[{}] detect {}: centre_norm=({:.3f},{:.3f}) px=({:.0f},{:.0f}) "
          "da_yaw={:.2f}° da_pitch={:.2f}°",
          cfg.camera->device_name(), auto_aim::ARMOR_NAMES[a.name],
          a.center_norm.x, a.center_norm.y, a.center.x, a.center.y,
          da[0], da[1]);

        DetectionResult dr;
        dr.armors = std::move(armors);
        dr.timestamp = ts;
        dr.delta_yaw = da[0] / 57.3;
        dr.delta_pitch = da[1] / 57.3;
        detection_queue_.push(dr);
      }
    }
  } catch (const std::exception & e) {
    tools::logger()->error("Exception in parallel_infer: {}", e.what());
  }
}

}  // namespace omniperception
