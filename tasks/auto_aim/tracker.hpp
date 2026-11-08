#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <opencv2/core/types.hpp>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/detection.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
class Tracker
{
public:
  // solve_armors=false 时跳过 Solver::solve（供离线合成数据回放使用，
  // 此时 Armor 的 xyz/ypd 已由数据源直接给出）
  // imm_config_path 为空时自动取主 config 同目录下的 IMM.yaml（如 configs/IMM.yaml），
  // 这样项目根、install/ 下、以及任意自定义配置目录都能找到；读不到则用代码内默认值
  Tracker(
    const std::string & config_path, Solver & solver, bool solve_armors = true,
    const std::string & imm_config_path = "");

  std::string state() const;
  void reset();

  // 返回内部跟踪目标的只读指针，nullptr 表示当前没有可用目标。
  // 用指针而不是 std::list<Target> 返回值：Target 内含 3 个 EKF（矩阵 + NIS 历史窗口），
  // 按值返回会让主循环每帧深拷贝一次。
  //
  // 【生命周期】返回值指向 Tracker 内部的 target_，只在下一次 track()/reset() 之前有效，
  // 且内容会随下一次 track() 被原地修改。因此：
  //   · 只在"取到指针 -> 同帧紧邻消费"的模式下使用（aimer/shooter 都满足）；
  //   · 不要存成成员、不要跨帧持有、不要交给异步任务；
  //   · 需要一份稳定快照时自行拷贝（Aimer 内部就是这么做的，它要 predict）。
  const Target * track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true, cv::Size image_size = cv::Size());

  std::tuple<omniperception::DetectionResult, const Target *> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true,
    cv::Size image_size = cv::Size());

private:
  Solver & solver_;
  bool solve_armors_;
  TargetConfig target_config_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  // 3V3 无前哨站，前哨站专用丢失容忍计数已停用
  // int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t, double dt);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP