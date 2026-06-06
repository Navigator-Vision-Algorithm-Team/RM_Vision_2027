#include "perceptron.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace omniperception
{
Perceptron::Perceptron(
  const std::vector<OmniCameraConfig> & omni_configs, const std::string & config_path)
: detection_queue_(10), decider_(config_path), stop_flag_(false)
{
  if (omni_configs.empty()) {
    tools::logger()->info("Perceptron initialized with 0 cameras.");
    return;
  }

  // 创建 YOLO 模型，每个相机一个
  for (size_t i = 0; i < omni_configs.size(); i++) {
    yolos_.push_back(std::make_shared<auto_aim::YOLO>(config_path, false));
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 创建线程进行并行推理，每个相机一个线程
  for (size_t i = 0; i < omni_configs.size(); i++) {
    threads_.emplace_back(
      [this, cfg = omni_configs[i], yolo = yolos_[i]] { parallel_infer(cfg, yolo); });
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
      if (!armors.empty()) {
        auto da = decider_.delta_angle(armors, cfg);

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