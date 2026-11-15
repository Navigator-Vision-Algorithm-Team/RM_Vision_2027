#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/command.hpp"
#include "io/mode.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);

  // 单目标版本：只读引用，内部按需拷贝一份用于预测迭代。
  // 主链路（Tracker 返回 const Target*）走这个重载，避免每帧先组一个 list<Target> 再深拷贝。
  io::Command aim(
    const Target & target, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    const Target & target, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

  // 指针重载：直接接 Tracker::track() 的返回值，nullptr 时返回空命令
  io::Command aim(
    const Target * target, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    const Target * target, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

  // 兼容接口：空列表返回空命令，否则转调单目标版本
  io::Command aim(
    const std::list<Target> & targets, std::chrono::steady_clock::time_point timestamp,
    double bullet_speed, bool to_now = true);

  io::Command aim(
    const std::list<Target> & targets, std::chrono::steady_clock::time_point timestamp,
    double bullet_speed, io::ShootMode shoot_mode, bool to_now = true);

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;

  AimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP