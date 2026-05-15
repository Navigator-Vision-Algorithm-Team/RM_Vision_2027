#ifndef RM_SERIAL_DRIVER__TOOLS__AIMER_HPP_
#define RM_SERIAL_DRIVER__TOOLS__AIMER_HPP_

#include <Eigen/Dense>
#ifdef ROS_DISTRO_HUMBLE
// Humble / C++14: std::optional not available, use sentinel-NaN fallback
#include <cmath>
struct OptionalDouble
{
  double value = NAN;
  bool has_value() const { return !std::isnan(value); }
};
#else
// Jazzy / C++17
#include <optional>
#endif
#include <vector>

namespace rm_serial_driver
{
namespace tools
{

struct AimerParams
{
  double yaw_offset = 0;           // rad
  double pitch_offset = 0;         // rad
  double coming_angle = 55.0 * M_PI / 180.0;   // rad
  double leaving_angle = 20.0 * M_PI / 180.0;  // rad
  double high_speed_delay_time = 0.15;   // s
  double low_speed_delay_time = 0.1;     // s
  double decision_speed = 2.0;           // rad/s threshold
  double fire_thresh = 0.02;            // rad
#ifdef ROS_DISTRO_HUMBLE
  OptionalDouble left_yaw_offset;
  OptionalDouble right_yaw_offset;
#else
  std::optional<double> left_yaw_offset;
  std::optional<double> right_yaw_offset;
#endif
};

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;  // x, y, z, angle
};

struct AimCommand
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
};

class Aimer
{
public:
  explicit Aimer(const AimerParams & params);

  AimCommand aim(
    const Eigen::VectorXd & ekf_x,
    const std::vector<Eigen::Vector4d> & armor_xyza_list,
    bool jumped, double bullet_speed,
    double dt = 0.005);

  AimCommand aim(
    const Eigen::VectorXd & ekf_x,
    const std::vector<Eigen::Vector4d> & armor_xyza_list,
    bool jumped, double bullet_speed,
    int shoot_mode, double dt = 0.005);

  AimPoint debug_aim_point;

private:
  AimerParams params_;
  double lock_id_ = -1;

  AimPoint choose_aim_point(
    const Eigen::VectorXd & ekf_x,
    const std::vector<Eigen::Vector4d> & armor_xyza_list,
    bool jumped, int armor_num);

  // Simple parabolic trajectory solver
  struct Trajectory
  {
    bool unsolvable;
    double fly_time;
    double pitch;
    Trajectory(double v0, double d, double h);
  };
};

}  // namespace tools
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__TOOLS__AIMER_HPP_
