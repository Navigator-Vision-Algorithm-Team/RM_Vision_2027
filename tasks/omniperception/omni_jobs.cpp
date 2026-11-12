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
  const std::vector<OmniCameraConfig> & omni_configs, const std::string & config_path,
  OmniSharedStatePtr shared)
{
  std::vector<auto_aim::multithread::CameraJob> jobs;
  jobs.reserve(omni_configs.size());

  // 各路 lambda 的颜色缓存初值。共享状态非空时以它为准，保证第一帧不会用
  // Decider 构造时的配置值把正确的颜色又覆盖回去。
  const auto initial_color =
    shared ? shared->enemy_color.load() : auto_aim::Color::red;

  for (std::size_t i = 0; i < omni_configs.size(); i++) {
    // 拷贝一份配置给 lambda：任务的生命周期长于本函数
    auto cfg = omni_configs[i];

    // 每路一个 Decider：过滤/优先级规则是只读的，但各自持有可以避免跨线程共享状态
    auto decider = std::make_shared<Decider>(config_path);

    auto_aim::multithread::CameraJob job;
    job.camera = cfg.camera;    job.camera_id = static_cast<int>(i);
    job.keep_image = false;  // 全向结果不把整帧图像带出线程，省内存带宽

    // 全向只负责搜索，绑到能效核；性能核全部留给不绑核的主相机（OpenVINO
    // 线程池在空闲的大核上自主展开）。能效核不足时 YoloPipeline 会告警并回退。
    job.core_preference = tools::CpuCoreType::Efficiency;

    job.postprocess = [cfg, decider, shared, last_color = initial_color](
                        const cv::Mat & img, std::chrono::steady_clock::time_point ts,
                        std::list<auto_aim::Armor> & armors,
                        auto_aim::multithread::CameraResult & result) mutable {
      // 录像：必须 clone 后再画框。虽然 hikrobot/mindvision 每帧返回的都是独立缓冲，
      // 但伪相机走零拷贝缓冲池、只重画左上角 overlay，直接在原图画框会污染池缓冲
      // 并在后续帧留下残影。这路相机 keep_image=false，原图无人使用，克隆出来的
      // 副本正好可以直接画。
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

      // 同步主线程的敌方颜色（串口上报 robot_id 后可能已换色）。
      // 不同步的后果：armor_filter 会把本该保留的装甲板全部剔除，armors 为空 ->
      // 结果不入队 -> 全向感知表现为「完全失效」。
      if (shared) {
        const auto color = shared->enemy_color.load();
        if (color != last_color) {
          decider->set_enemy_color(color);
          last_color = color;
        }
      }

      // 先过滤、排序，再算角度：保证 delta_yaw/delta_pitch 描述的目标
      // 与 armors.front() 始终一致（否则角度会指向已被过滤掉的装甲板）。
      const auto detected = armors.size();
      decider->armor_filter(armors);
      if (armors.empty() && detected > 0) {
        tools::logger()->debug(
          "[{}] {} 块装甲板全部被敌方颜色过滤（enemy_color={}），请检查串口 robot_id 与配置",
          cfg.camera ? cfg.camera->device_name() : "omni", detected,
          auto_aim::COLORS[static_cast<int>(last_color)]);
      }
      // 【已停用】无法可靠判断兵种，暂停按兵种设置优先级（与 sentry_multithread 保持一致）；
      // 下方按 priority 的排序在全相等时退化为稳定排序，实际由“离图像中心最近”决定。
      // decider->set_priority(armors);
      armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
        return a.priority < b.priority;
      });

      result.delta_yaw = 0.0;
      result.delta_pitch = 0.0;
      if (armors.empty()) return;

      const auto da = decider->delta_angle(armors, cfg, img.size());
      result.delta_yaw = da[0] / 57.3;
      result.delta_pitch = da[1] / 57.3;

      // 每检测到目标就打印一次；4 路相机各 30~60fps 时 info 会刷满日志并拖慢推理线程，
      // 需要时用 debug 级别查看。
      const auto & armor = armors.front();
      tools::logger()->debug(
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
