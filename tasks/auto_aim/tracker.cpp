#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <tuple>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
// IMM 配置默认放在主 config 同目录，兼容项目根运行与 install/ 部署
std::string resolve_imm_config(const std::string & config_path, const std::string & override_path)
{
  if (!override_path.empty()) return override_path;

  std::filesystem::path p(config_path);
  const auto dir = p.parent_path();
  if (dir.empty()) return "IMM.yaml";
  return (dir / "IMM.yaml").string();
}
}  // namespace

Tracker::Tracker(
  const std::string & config_path, Solver & solver, bool solve_armors,
  const std::string & imm_config_path)
: solver_{solver},
  solve_armors_{solve_armors},
  target_config_(TargetConfig::load(resolve_imm_config(config_path, imm_config_path))),
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  // 3V3 无前哨站，前哨站专用丢失容忍计数已停用
  // outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
}

std::string Tracker::state() const { return state_; }

void Tracker::reset()
{
  state_ = "lost";
  pre_state_ = "lost";
  detect_count_ = 0;
  temp_lost_count_ = 0;
  max_temp_lost_count_ = normal_temp_lost_count_;
  tools::logger()->info("[Tracker] Reset by MCU signal.");
}

const Target * Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color,
  cv::Size image_size)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  const cv::Point2f img_center =
    (image_size.width > 0 && image_size.height > 0)
      ? cv::Point2f(image_size.width * 0.5F, image_size.height * 0.5F)
      : cv::Point2f(720.0F, 540.0F);

  // 一次排序同时完成两件事：优先级升序（1 最高），同优先级按离图像中心距离升序。
  // 原来的写法是先按中心距离排一遍、再按优先级稳定排序一遍，第一次排序完全被第二次
  // 覆盖掉（只影响同优先级的相对顺序），合并后结果等价、少一次 O(n log n)。
  armors.sort([&](const Armor & a, const Armor & b) {
    if (a.priority != b.priority) return a.priority < b.priority;
    return cv::norm(a.center - img_center) < cv::norm(b.center - img_center);
  });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t, dt);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    // tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return nullptr;
  }

  // 收敛效果检测：按时间窗统计 NIS 超限比例（阈值由观测维度的卡方 95% 分位确定），
  // 窗口内样本足够多才判失效，避免冷启动阶段的少量样本误杀
  if (
    target_.nis_window_samples() >= target_.min_nis_samples() &&
    target_.nis_failure_rate() >= target_config_.tracker_fail_rate) {
    // tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return nullptr;
  }

  if (state_ == "lost") return nullptr;

  // 预测的位置和速度：(x, vx, y, vy, z, vz)
  // 预测的角度和角速度：(a, w)
  // 几何参数：半径 r、长短轴差 l、高度差 h
  // 目标类型信息：机器人名称、装甲板类型、优先级等
  // 卡尔曼滤波器状态：完整的 EKF 状态向量和协方差矩阵
  // 收敛状态：是否已经稳定跟踪
  return &target_;
}

/*
  这个方法用来挑选最应该攻击的装甲板，并且得出结算
*/
std::tuple<omniperception::DetectionResult, const Target *> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color, cv::Size image_size)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  // 队列里优先级最高的一路全向结果。只取引用：原来的写法每帧都会把整条
  // list<Armor>（含 pattern 图像）拷贝一份 temp_target，而这里只读几个字段。
  const omniperception::DetectionResult * temp_target =
    detection_queue.empty() ? nullptr : &detection_queue.front();
  (void)use_enemy_color;  // 敌我过滤用的是成员 enemy_color_，该参数仅为接口兼容保留

  //这个时间来自于上一次相机的读取的时间
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  const cv::Point2f img_center =
    (image_size.width > 0 && image_size.height > 0)
      ? cv::Point2f(image_size.width * 0.5F, image_size.height * 0.5F)
      : cv::Point2f(720.0F, 540.0F);

  armors.sort([&](const Armor & a, const Armor & b) {
    if (a.priority != b.priority) return a.priority < b.priority;

    auto da = a.center - img_center;
    auto db = b.center - img_center;

    float dist2_a = da.x * da.x + da.y * da.y;
    float dist2_b = db.x * db.x + db.y * db.y;

    return dist2_a < dist2_b;
  });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);  // 这个armors是已经排好序了的
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    // tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  // 全向感知中的目标优先级排序并不完善似乎。
  // 如果在全向的目标会比主相机找到的更佳的好，并不是直接替换目标，而是吧这个全向中的目标标记称为switch_target，进入switching状态，等待主相机的目标切换
  else if (
    state_ == "tracking" && temp_target != nullptr && !temp_target->armors.empty() &&
    temp_target->armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target->armors, t, temp_target->delta_yaw, temp_target->delta_pitch};
    omni_target_priority_ = temp_target->armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t, dt);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {std::move(switch_target), nullptr};  // 返回switch_target，无跟踪目标
  }

  // 收敛效果检测：与主相机路径保持一致
  if (state_ != "lost") {
    if (
      target_.nis_window_samples() >= target_.min_nis_samples() &&
      target_.nis_failure_rate() >= target_config_.tracker_fail_rate) {
      tools::logger()->debug("[Tracker] Bad converge found (NIS).");
      state_ = "lost";
      return {std::move(switch_target), nullptr};
    }
  }

  if (state_ == "lost") return {std::move(switch_target), nullptr};

  return {std::move(switch_target), &target_};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
      // 换目标已经有主相机确认，保留 detect_count_ 让新目标直接进入 tracking（原行为）；
      // 只把丢失计数清零，避免沿用上一段跟踪残留的值
      temp_lost_count_ = 0;
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      // 3V3 无前哨站，前哨站的特殊丢失容忍已停用，统一使用普通目标阈值
      // if (target_.name == ArmorName::outpost)
      //   max_temp_lost_count_ = outpost_max_temp_lost_count_;
      // else
      max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

/*
  使用已经排好序的armors来设置目标targe
*/
bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  // 解算失败时 armor 的 xyz/ypr/ypd 仍是默认零值，用它初始化目标等于在原点起一个
  // 假目标，必须放弃这一帧（原来忽略了返回值）。
  if (solve_armors_ && !solver_.solve(armor)) return false;

  // 根据兵种优化初始化参数
  //TODO:这个地方的决策要改，在之前的armers的构建通路中已经将装甲板约束在了0 1 3 之中
  // 并且这里很多的配置都是为了旧的兵种和7V7中的一些设施准备的。
  // auto is_balance = (armor.type == ArmorType::big) &&
  //                   (armor.name == ArmorName::three || armor.name == ArmorName::four ||
  //                    armor.name == ArmorName::five);  // 知道兵种，并使用对应的卡尔曼参数。

  // if (is_balance) {
  //   Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
  //   target_ = Target(armor, t, 0.2, 2, P0_dig);
  // }

  // else if (armor.name == ArmorName::outpost) {
  //   Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
  //   target_ = Target(armor, t, 0.2765, 3, P0_dig);
  // }

  // else if (armor.name == ArmorName::base) {
  //   Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
  //   target_ = Target(armor, t, 0.3205, 3, P0_dig);
  // }

  // else {
  //   Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
  //   target_ = Target(armor, t, 0.2, 4, P0_dig);
  // }
  // 3V3 无前哨站，前哨站的 3 板模型（120° 间隔、半径 0.2765m）已停用
  // if (armor.name == ArmorName::outpost) {
  //   target_ =
  //     Target(armor, t, 0.2765, 3, Eigen::VectorXd{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}});
  // } else {
  // 其余兵种（含哨兵、步兵、英雄）统一按 4 块装甲板、半径 0.2m 建模
  // P0 由 configs/IMM.yaml 的 initial_covariance 提供
  target_ = Target(armor, t, 0.2, 4, {}, target_config_);
  // }

  return true;
}

/*
  在目标装甲板的基础上，使用卡尔曼滤波器来预测目标和更新目标的状态，返回是否找到了目标
*/
bool Tracker::update_target(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, double dt)
{
  // 卡尔曼滤波器找到目标
  target_.predict(t);

  // 收集与当前目标同兵种、同装甲板类型的观测，同一帧的多块装甲板合并成一次批量量测更新。
  // 只传指针：Armor 含 cv::Mat pattern 与两个 vector，按值收集等于每帧多拷一份检测结果。
  std::vector<const Armor *> matched;
  matched.reserve(armors.size());
  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;

    // 解算失败的装甲板位姿是全零，喂进 EKF 会把目标拉到原点，直接跳过
    if (solve_armors_ && !solver_.solve(armor)) continue;
    matched.push_back(&armor);
  }

  // 发现没有找到，返回 false，说明目标丢失了
  if (matched.empty()) return false;

  target_.update(matched, dt);

  return true;
}

}  // namespace auto_aim