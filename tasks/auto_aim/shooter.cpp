#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

bool Shooter::shoot(
  const io::Command & command,
  const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets,
  const Eigen::Vector3d & gimbal_pos
  ){
  if (targets.empty()) return false;
  return shoot(command, aimer, &targets.front(), gimbal_pos);
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer, const auto_aim::Target * target,
  const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || target == nullptr || !auto_fire_) return false;

  // 只取一次：ekf_x() 每次调用都会新建一个 VectorXd（堆分配）
  const Eigen::VectorXd x = target->ekf_x();
  if (x.size() < 3) return false;

  const auto tolerance = std::sqrt(tools::square(x[0]) + tools::square(x[2])) > judge_distance_
                           ? second_tolerance_
                           : first_tolerance_;

  const double command_jump = std::abs(last_command_.yaw - command.yaw);
  const double gimbal_error = std::abs(gimbal_pos[0] - last_command_.yaw);

  // 无论是否开火都要记录本次指令，作为下一帧的突变判据
  last_command_ = command;

  return command_jump < tolerance * 2 && gimbal_error < tolerance &&
         aimer.debug_aim_point.valid;
}

}  // namespace auto_aim