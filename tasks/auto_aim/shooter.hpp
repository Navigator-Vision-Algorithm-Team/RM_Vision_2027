#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  // 单目标版本：target 为 nullptr 视为无目标（不更新内部历史指令）
  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer, const auto_aim::Target * target,
    const Eigen::Vector3d & gimbal_pos);

  // 兼容接口
  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP