#include "omni_jobs.hpp"

#include <fmt/core.h>

#include <Eigen/Geometry>
#include <iterator>
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
  int keep_image_camera_id)
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
    // 全向结果默认不把整帧图像带出线程（省内存带宽）；只有视觉三角化那一路例外，
    // 它需要每帧原图做特征跟踪。always_push 同理：没有装甲板也要产生结果。
    job.keep_image = static_cast<int>(i) == keep_image_camera_id;
    job.always_push = job.keep_image;

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

      const auto & armor = armors.front();

      // PnP 3D 点透传（方案 V1）：delta_angle 内部刚解出的「装甲板中心在大yaw 系」
      // 坐标，原来只取了角度把 xyz 丢掉，现在一并交给主循环汇总成障碍点云。
      // 注意用的是 front()（与 delta_yaw/delta_pitch 描述的是同一块装甲板）。
      if (const auto & pnp = decider->last_pnp(); pnp.success) {
        auto_aim::multithread::ObstaclePoint obstacle;
        obstacle.p_gimbal = pnp.xyz_in_gimbal;
        obstacle.camera_id = result.camera_id;
        obstacle.confidence = static_cast<float>(armor.confidence);
        result.obstacles.push_back(obstacle);
      }
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
  auto_aim::multithread::YoloPipeline & pipeline, const TakeOptions & options)
{
  std::vector<auto_aim::multithread::CameraResult> raw;
  pipeline.drain_omni(raw);

  std::vector<DetectionResult> results;
  results.reserve(raw.size());
  for (auto & item : raw) {
    // 三角化那一路：把原图与时间戳搬出去（只移动 cv::Mat 句柄，不拷贝像素）。
    // 这一路即使没检测到装甲板也会有结果（always_push），照样要取。
    if (options.keep_image && item.camera_id == options.keep_image_camera_id) {
      options.keep_image->camera_id = item.camera_id;
      options.keep_image->timestamp = item.timestamp;
      options.keep_image->img = std::move(item.img);
    }

    // PnP 3D 点与「有没有装甲板」无关地汇总：本帧有障碍物就交给视觉补盲链路
    if (options.obstacles && !item.obstacles.empty()) {
      options.obstacles->insert(
        options.obstacles->end(), std::make_move_iterator(item.obstacles.begin()),
        std::make_move_iterator(item.obstacles.end()));
    }

    // 没有装甲板的结果不进 DetectionResult：下游（sort/track）只关心候选目标，
    // 塞空条目只会让队列里多出无意义的元素。
    if (item.armors.empty()) continue;

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
