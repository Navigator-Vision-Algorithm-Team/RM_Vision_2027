#ifndef OMNIPERCEPTION__OMNI_JOBS_HPP
#define OMNIPERCEPTION__OMNI_JOBS_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "detection.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/multithread/yolo_pipeline.hpp"

namespace omniperception
{

/*
 * 全向推理线程与主线程共享的运行时状态。
 *
 * make_omni_jobs 给每路相机创建一个独立的 Decider（避免跨线程共享 count_ 等
 * 可变成员），代价是主线程 Decider::set_enemy_color() 传不进推理线程。
 *
 * 颜色不同步的后果非常隐蔽：串口判定敌方为蓝方、而配置里还是红方时，
 * 推理线程里的 armor_filter 会把检测到的装甲板全部剔除，armors 为空 ->
 * 结果不入队 -> 表现为「全向感知完全失效」（画面有目标，但系统永远收不到）。
 *
 * 这里只同步颜色这一个量：主线程调用 set_enemy_color()，各推理线程在每帧
 * 过滤前读一次并和自己的缓存比较，不同才更新本路的 Decider。
 */
struct OmniSharedState
{
  void set_enemy_color(auto_aim::Color color) { enemy_color.store(color); }

  // 主相机是否处于 tracking。true 时各全向推理线程暂停 YOLO 推理：
  // 主相机已经锁定目标，全向的搜索没有意义，省下这部分算力。
  // 颜色层仍然每帧运行（保持命中证据），因此主相机一丢目标，
  // 全向下一帧就能直接进入推理，不需要重新去抖。
  void set_main_tracking(bool tracking) { main_tracking.store(tracking, std::memory_order_relaxed); }

  // 下位机上报的己方编号（0 = 还没收到）。颜色门控用它判断 enemy_color 是
  // 「串口真实值」还是「配置兜底值」：后者可能是反的，开赛初期必须红蓝都查。
  void set_robot_id(std::uint8_t id) { robot_id.store(id, std::memory_order_relaxed); }

  std::atomic<auto_aim::Color> enemy_color{auto_aim::Color::red};
  std::atomic<bool> main_tracking{false};
  std::atomic<std::uint8_t> robot_id{0};
};

using OmniSharedStatePtr = std::shared_ptr<OmniSharedState>;

// 为一组全向相机创建推理任务：每路相机一个 YOLO 实例（由 YoloPipeline 在各线程内
// 构造）、一个绑定固定核的线程。推理之后在这里完成：
//   敌方过滤 -> 装甲板优先级 -> 按优先级排序 -> 相对大yaw 的 delta_yaw/delta_pitch
//   以及（可选）录像。
// camera_id 从 0 开始按 omni_configs 顺序编号。
//
// shared 非空时，各路 Decider 的敌方颜色会跟随主线程同步；为空则各用自己的
// 配置值（与旧行为一致）。
std::vector<auto_aim::multithread::CameraJob> make_omni_jobs(
  const std::vector<OmniCameraConfig> & omni_configs, const std::string & config_path,
  OmniSharedStatePtr shared = nullptr);

// 取走推理池里积压的全向结果，并转换成 tracker/decider 使用的 DetectionResult。
std::vector<DetectionResult> take_detection_results(
  auto_aim::multithread::YoloPipeline & pipeline);

}  // namespace omniperception

#endif  // OMNIPERCEPTION__OMNI_JOBS_HPP
