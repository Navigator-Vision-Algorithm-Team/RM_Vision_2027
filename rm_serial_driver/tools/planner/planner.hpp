#ifndef RM_SERIAL_DRIVER__TOOLS__PLANNER_PLANNER_HPP_
#define RM_SERIAL_DRIVER__TOOLS__PLANNER_PLANNER_HPP_

#include <Eigen/Dense>
#include <vector>

#include "tinympc/tiny_api.hpp"

namespace rm_serial_driver
{
namespace tools
{

constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

struct PlannerParams
{
  double yaw_offset = 0;
  double pitch_offset = 0;
  double fire_thresh = 0.02;
  double low_speed_delay_time = 0.1;
  double high_speed_delay_time = 0.15;
  double decision_speed = 2.0;
  double max_yaw_acc = 700.0;
  double max_pitch_acc = 500.0;
  std::vector<double> Q_yaw = {4, 2};
  std::vector<double> R_yaw = {0.1};
  std::vector<double> Q_pitch = {20, 4};
  std::vector<double> R_pitch = {0.1};
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;

  explicit Planner(const PlannerParams & params);

  Plan plan(const Eigen::VectorXd & ekf_x,
            const std::vector<Eigen::Vector4d> & armor_xyza_list,
            double bullet_speed);

private:
  PlannerParams params_;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  void setup_yaw_solver();
  void setup_pitch_solver();

  Eigen::Matrix<double, 2, 1> aim_state(
    const Eigen::VectorXd & ekf_x,
    const std::vector<Eigen::Vector4d> & armor_xyza_list,
    double bullet_speed);  // non-const: updates debug_xyza

  Trajectory get_trajectory(
    Eigen::VectorXd ekf_x,
    const std::vector<Eigen::Vector4d> & armor_xyza_list,
    double yaw0, double bullet_speed);
};

}  // namespace tools
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__TOOLS__PLANNER_PLANNER_HPP_
