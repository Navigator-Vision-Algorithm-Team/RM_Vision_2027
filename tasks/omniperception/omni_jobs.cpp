#include "omni_jobs.hpp"

#include <fmt/core.h>

#include <Eigen/Geometry>
#include <list>
#include <memory>
#include <utility>

#include "decider.hpp"
#include "tools/logger.hpp"
#include "tools/recorder.hpp"

namespace omniperception
{

std::vector<auto_aim::multithread::CameraJob> make_omni_jobs(
  const std::vector<OmniCameraConfig> & omni_configs, const std::string & config_path)
{
  std::vector<auto_aim::multithread::CameraJob> jobs;
  jobs.reserve(omni_configs.size());

  for (std::size_t i = 0; i < omni_configs.size(); i++) {
    // 拷贝一份配置给 lambda：任务的生命周期长于本函数
    auto cfg = omni_configs[i];

    // 每路一个 Decider：过滤/优先级规则是只读的，但各自持有可以避免跨线程共享状态
    auto decider = std::make_shared<Decider>(config_path);

    auto_aim::multithread::CameraJob job;
    job.camera = cfg.camera;    job.camera_id = static_cast<int>(i);
    job.keep_image = false;  // 全向结果不把整帧图像带出线程，省内存带宽

    job.postprocess = [cfg, decider](
                        const cv::Mat & img, std::chrono::steady_clock::time_point ts,
                        std::list<auto_aim::Armor> & armors,
                        auto_aim::multithread::CameraResult & result) {
      // 录像：只有配置了 recorder 时才复制图像并画框（与原 Perceptron 的行为一致）
      if (cfg.recorder && !img.empty()) {
        cv::Mat display_img = img.clone();
        for (const auto & armor : armors) {
          cv::rectangle(display_img, armor.box, cv::Scalar(0, 255, 0), 2);
          std::string label = fmt::format(
            "{} {} {:.2f}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
            armor.confidence);
          cv::putText(
            display_img, label, cv::Point(armor.box.x, armor.box.y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
          for (const auto & pt : armor.points) {
            cv::circle(display_img, pt, 2, cv::Scalar(0, 0, 255), -1);
          }
        }
        cfg.recorder->record(display_img, Eigen::Quaterniond::Identity(), ts);
      }

      // 先过滤、定优先级、排序，再算角度：保证 delta_yaw/delta_pitch 描述的目标
      // 与 armors.front() 始终一致（否则角度会指向已被过滤掉的装甲板）。
      decider->armor_filter(armors);
      decider->set_priority(armors);
      armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
        return a.priority < b.priority;
      });

      result.delta_yaw = 0.0;
      result.delta_pitch = 0.0;
      if (armors.empty()) return;

      const auto da = decider->delta_angle(armors, cfg, img.size());
      result.delta_yaw = da[0] / 57.3;
      result.delta_pitch = da[1] / 57.3;

      const auto & armor = armors.front();
      tools::logger()->info(
        "[{}] detect {}: centre_norm=({:.3f},{:.3f}) px=({:.0f},{:.0f}) "
        "da_yaw={:.2f}° da_pitch={:.2f}°",
        cfg.camera ? cfg.camera->device_name() : "omni", auto_aim::ARMOR_NAMES[armor.name],
        armor.center_norm.x, armor.center_norm.y, armor.center.x, armor.center.y, da[0], da[1]);
    };

    jobs.push_back(std::move(job));
  }

  return jobs;
}

std::vector<DetectionResult> take_detection_results(
  auto_aim::multithread::YoloPipeline & pipeline)
{
  std::vector<auto_aim::multithread::CameraResult> raw;
  pipeline.drain_omni(raw);

  std::vector<DetectionResult> results;
  results.reserve(raw.size());
  for (auto & item : raw) {
    DetectionResult detection;
    detection.armors = std::move(item.armors);
    detection.timestamp = item.timestamp;
    detection.delta_yaw = item.delta_yaw;
    detection.delta_pitch = item.delta_pitch;
    results.push_back(std::move(detection));
  }

  return results;
}

}  // namespace omniperception
