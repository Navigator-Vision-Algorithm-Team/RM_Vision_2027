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
//   以及（可选）录像。
// camera_id 从 0 开始按 omni_configs 顺序编号。
std::vector<auto_aim::multithread::CameraJob> make_omni_jobs(
  const std::vector<OmniCameraConfig> & omni_configs, const std::string & config_path);

// 取走推理池里积压的全向结果，并转换成 tracker/decider 使用的 DetectionResult。
std::vector<DetectionResult> take_detection_results(
  auto_aim::multithread::YoloPipeline & pipeline);

}  // namespace omniperception

#endif  // OMNIPERCEPTION__OMNI_JOBS_HPP
