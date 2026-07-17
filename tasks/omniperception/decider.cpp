#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace omniperception
{
Decider::Decider(const std::string & config_path) : count_(0)
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ =
    (yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red : auto_aim::Color::blue;
  mode_ = yaml["mode"].as<double>();
}

io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos,
  const std::vector<OmniCameraConfig> & omni_configs)
{
  if (omni_configs.empty()) return {false, false, 0, 0};

  int n = static_cast<int>(omni_configs.size());
  if (count_ < 0 || count_ >= n) count_ = 0;

  auto & cfg = omni_configs[count_];

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  cfg.camera->read(img, timestamp);

  if (img.empty()) return io::Command{false, false, 0, 0};

  auto armors = yolo.detect(img);
  auto empty = armor_filter(armors);

  if (!empty) {
    auto da = delta_angle(armors, cfg);

    tools::logger()->debug(
      "[omni camera {}] delta yaw:{:.2f}, target pitch:{:.2f}, armor number:{}, armor name:{}",
      count_, da[0], da[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    count_ = (count_ + 1) % n;

    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + da[0] / 57.3),
      tools::limit_rad(da[1] / 57.3)};
  }

  count_ = (count_ + 1) % n;
  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(const std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) return io::Command{false, false, 0, 0};

  for (const auto & dr : detection_queue) {
    if (dr.armors.empty()) continue;
    double yaw_deg = dr.delta_yaw * 57.3;
    if (std::abs(yaw_deg) > 360.0) {
      tools::logger()->warn("omni angle rejected: {:.1f}° out of range", yaw_deg);
      continue;
    }
    tools::logger()->info(
      "omniperceptron find {}, delta yaw {:.2f}°",
      auto_aim::ARMOR_NAMES[dr.armors.front().name],
      dr.delta_yaw * 57.3);
    return io::Command{true, false, dr.delta_yaw, dr.delta_pitch};
  }

  return io::Command{false, false, 0, 0};
}

void Decider::clear_angle_stack()
{
}

Eigen::Vector2d Decider::delta_angle(
  const std::list<auto_aim::Armor> & armors, const OmniCameraConfig & cfg)
{
  Eigen::Vector2d da;
  da[0] = cfg.mount_yaw + (cfg.fov_h / 2) - armors.front().center_norm.x * cfg.fov_h;
  da[1] = armors.front().center_norm.y * cfg.fov_v - cfg.fov_v / 2 + cfg.mount_pitch;
  return da;
}

bool Decider::armor_filter(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return true;
  // 过滤非敌方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 25赛季没有5号装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::five; });
  // 不打工程
  // armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::two; });
  // 不打前哨站
  armors.remove_if(
    [&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::outpost; });

  // 过滤掉刚复活无敌的装甲板
  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(invincible_armor_.begin(), invincible_armor_.end(), a.name) !=
           invincible_armor_.end();
  });

  return armors.empty();
}

void Decider::set_priority(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return;

  const PriorityMap & priority_map = (mode_ == MODE_ONE) ? mode1 : mode2;

  if (!armors.empty()) {
    for (auto & armor : armors) {
      armor.priority = priority_map.at(armor.name);
    }
  }
}

void Decider::sort(std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) return;

  // 对每个 DetectionResult 调用 armor_filter 和 set_priority
  for (auto & dr : detection_queue) {
    armor_filter(dr.armors);
    set_priority(dr.armors);

    // 对每个 DetectionResult 中的 armors 进行排序
    dr.armors.sort(
      [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });
  }

  // 根据优先级对 DetectionResult 进行排序
  std::sort(
    detection_queue.begin(), detection_queue.end(),
    [](const DetectionResult & a, const DetectionResult & b) {
      return a.armors.front().priority < b.armors.front().priority;
    });
}

Eigen::Vector4d Decider::get_target_info(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets)
{
  if (armors.empty() || targets.empty()) return Eigen::Vector4d::Zero();

  auto target = targets.front();

  for (const auto & armor : armors) {
    if (armor.name == target.name) {
      return Eigen::Vector4d{
        armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
        static_cast<double>(armor.name) + 1};  //避免歧义+1(详见通信协议)
    }
  }

  return Eigen::Vector4d::Zero();
}

void Decider::get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids)
{
  invincible_armor_.clear();

  if (invincible_enemy_ids.empty()) return;

  for (const auto & id : invincible_enemy_ids) {
    tools::logger()->info("invincible armor id: {}", id);
    invincible_armor_.push_back(auto_aim::ArmorName(id - 1));
  }
}

void Decider::get_auto_aim_target(
  std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target)
{
  if (auto_aim_target.empty()) return;

  std::vector<auto_aim::ArmorName> auto_aim_targets;

  for (const auto & target : auto_aim_target) {
    if (target <= 0 || static_cast<size_t>(target) > auto_aim::ARMOR_NAMES.size()) {
      tools::logger()->warn("Received invalid auto_aim target value: {}", int(target));
      continue;
    }
    auto_aim_targets.push_back(static_cast<auto_aim::ArmorName>(target - 1));
    tools::logger()->info("nav send auto_aim target is {}", auto_aim::ARMOR_NAMES[target - 1]);
  }

  if (auto_aim_targets.empty()) return;

  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(auto_aim_targets.begin(), auto_aim_targets.end(), a.name) ==
           auto_aim_targets.end();
  });
}

}  // namespace omniperception