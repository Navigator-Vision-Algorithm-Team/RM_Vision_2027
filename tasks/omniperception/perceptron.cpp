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
  //在这个parallelinfer函数中，读取了目标的信息在detector_queue中
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
      // 这个地方是锁，四个相机争抢，这里的相机争抢必然涉及到主相机的争抢，但是我们应该保证主相机的线程，并且希望四个全向感知不这样争抢
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;
      }

      cfg.camera->read(img, ts);
      if (img.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }

      //得到一个img，然后使用yolo推理，这个时候yolo是提供出来的了一个框的，这个时候我们把原图和框放在一起保存起来。
      auto armors = yolo->detect(img);

      // 画框图只有一个用途：交给 recorder 存档。没有 recorder 时整帧 clone（1440x1080
      // 约 4.4MB）+ 画框纯属浪费，300fps 压力测试下会直接吃掉可观带宽，因此仅在需要时做。
      if (cfg.recorder) {
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

        if (!display_img.empty()) {
          cfg.recorder->record(display_img, Eigen::Quaterniond::Identity(), ts);
        }
      }
      //至此，都是记录。

      // 先按主循环完全一致的规则做过滤、定优先级、按优先级排序，再计算 delta_yaw/delta_pitch，
      // 保证它们描述的始终是 armors.front() 这块装甲板。
      // （若先算角度再过滤/排序，角度会与最终 front() 指向的目标不一致。）
      decider_.armor_filter(armors);
      // 【已停用】无法可靠判断兵种，暂停按兵种设置优先级；
      // 此时下面按 priority 的排序所有元素相等，退化为稳定排序（保持原顺序）
      // decider_.set_priority(armors);
      armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
        return a.priority < b.priority;
      });

      if (!armors.empty()) {
        // 【改动】把当前帧尺寸传给 delta_angle：内部改用 PnP 解算，
        // 在没有标定内参时会用 fov + 图像尺寸估算内参。
        auto da = decider_.delta_angle(armors, cfg, img.size());

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
