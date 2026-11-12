#include "omni_jobs.hpp"

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <list>
#include <memory>
#include <string>
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

  // ---- 全向检测后端 ----
  // "traditional"（默认）：纯传统算法（阈值 + 轮廓 + 灯条配对 + tiny_resnet 编号），
  //                        该路不再加载 YOLO 模型，也不占 YOLO 的推理线程池。
  // "yolo"：保持旧行为（每路一个 YOLO 实例，可选 use_traditional 角点矫正）。
  const auto yaml = YAML::LoadFile(config_path);
  const std::string backend =
    yaml["omni_detector"] ? yaml["omni_detector"].as<std::string>() : "traditional";
  if (backend != "traditional" && backend != "yolo") {
    tools::logger()->warn(
      "[omni] omni_detector='{}' 无法识别，按 traditional 处理（可选值：traditional | yolo）",
      backend);
  }
  const bool use_traditional = backend != "yolo";
  tools::logger()->info(
    "[omni] {} 路全向相机检测后端：{}", omni_configs.size(),
    use_traditional ? "traditional（不加载 YOLO）" : "yolo");

  for (std::size_t i = 0; i < omni_configs.size(); i++) {
    // 拷贝一份配置给 lambda：任务的生命周期长于本函数
    auto cfg = omni_configs[i];

    // 每路一个 Decider：过滤/优先级规则是只读的，但各自持有可以避免跨线程共享状态
    auto decider = std::make_shared<Decider>(config_path);

    auto_aim::multithread::CameraJob job;
    job.camera = cfg.camera;    job.camera_id = static_cast<int>(i);
    job.keep_image = false;  // 全向结果不把整帧图像带出线程，省内存带宽

    // 检测后端：传统算法不创建 YOLO 实例。参数前缀 "omni_" 让 Detector 读取
    // omni_threshold / omni_min_confidence 等覆盖键，与主相机共用一份 yaml 但
    // 各自一套阈值（全向相机曝光、分辨率、安装角度都与主相机不同）。
    job.use_traditional = use_traditional;
    if (use_traditional) job.traditional_key_prefix = "omni_";

    // 全向只负责搜索，绑到能效核；性能核全部留给不绑核的主相机（OpenVINO
    // 线程池在空闲的大核上自主展开）。能效核不足时 YoloPipeline 会告警并回退。
    // 传统后端下绑核不再是为了限制推理线程池，而是限流：单帧耗时会随场景
    // 波动（高亮物多则轮廓多），独占一颗核可以避免它挤占主相机的时间片。
    job.core_preference = tools::CpuCoreType::Efficiency;

    job.postprocess = [cfg, decider](
                        const cv::Mat & img, std::chrono::steady_clock::time_point ts,
                        std::list<auto_aim::Armor> & armors,
                        auto_aim::multithread::CameraResult & result) {
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

      // 先过滤、排序，再算角度：保证 delta_yaw/delta_pitch 描述的目标
      // 与 armors.front() 始终一致（否则角度会指向已被过滤掉的装甲板）。
      decider->armor_filter(armors);
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
