#ifndef OMNIPERCEPTION__OMNI_JOBS_HPP
#define OMNIPERCEPTION__OMNI_JOBS_HPP

#include <string>
#include <vector>

#include "detection.hpp"
#include "tasks/auto_aim/multithread/yolo_pipeline.hpp"

namespace omniperception
{

// 为一组全向相机创建推理任务：每路相机一个 YOLO 实例（由 YoloPipeline 在各线程内
// 构造）、一个绑定固定核的线程。推理之后在这里完成：
//   敌方过滤 -> 装甲板优先级 -> 按优先级排序 -> 相对大yaw 的 delta_yaw/delta_pitch
//   以及（可选）录像、PnP 3D 点（CameraResult.obstacles）。
// camera_id 从 0 开始按 omni_configs 顺序编号。
//
// keep_image_camera_id：需要保留原图的那一路（0 基，视觉三角化用）。传 -1 表示
// 都不保留。该路同时会被置为 always_push —— 三角化需要连续帧，不能像自瞄那样
// 只在检测到装甲板时才产生结果。
std::vector<auto_aim::multithread::CameraJob> make_omni_jobs(
  const std::vector<OmniCameraConfig> & omni_configs, const std::string & config_path,
  int keep_image_camera_id = -1);

// take_detection_results 的附加出口（视觉补盲用，缺省全关时行为与旧版本一致）
struct TakeOptions
{
  // 需要把原图 + 时间戳取出来的那一路（0 基）；-1 = 不取
  int keep_image_camera_id = -1;
  // 汇总所有相机的 PnP 3D 点（大yaw 系），非空时追加
  std::vector<auto_aim::multithread::ObstaclePoint> * obstacles = nullptr;
  // 输出：keep_image_camera_id 那一路的结果（只搬原图与时间戳，armors 不复制）
  auto_aim::multithread::CameraResult * keep_image = nullptr;
};

// 取走推理池里积压的全向结果，并转换成 tracker/decider 使用的 DetectionResult。
std::vector<DetectionResult> take_detection_results(
  auto_aim::multithread::YoloPipeline & pipeline, const TakeOptions & options = TakeOptions{});

}  // namespace omniperception

#endif  // OMNIPERCEPTION__OMNI_JOBS_HPP
