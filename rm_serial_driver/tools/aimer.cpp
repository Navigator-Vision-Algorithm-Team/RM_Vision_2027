#include "aimer.hpp"

#include <algorithm>
#include <cmath>

namespace rm_serial_driver
{
namespace tools
{

namespace
{
double limit_rad(double angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle <= -M_PI) angle += 2 * M_PI;
  return angle;
}
}  // namespace

// ── Simple parabolic trajectory solver (from sp_vision_25) ──

constexpr double G = 9.7833;

Aimer::Trajectory::Trajectory(double v0, double d, double h)
{
  double a = G * d * d / (2 * v0 * v0);
  double b = -d;
  double c = a + h;
  double delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  double tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  double tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  double pitch_1 = std::atan(tan_pitch_1);
  double pitch_2 = std::atan(tan_pitch_2);
  double t_1 = d / (v0 * std::cos(pitch_1));
  double t_2 = d / (v0 * std::cos(pitch_2));

  // Choose the shorter fly time (flatter trajectory)
  if (t_1 < t_2) {
    pitch = pitch_1;
    fly_time = t_1;
  } else {
    pitch = pitch_2;
    fly_time = t_2;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Aimer
// ═══════════════════════════════════════════════════════════════════════════

Aimer::Aimer(const AimerParams & params) : params_(params) {}

AimCommand Aimer::aim(
  const Eigen::VectorXd & ekf_x,
  const std::vector<Eigen::Vector4d> & armor_xyza_list,
  bool jumped, double bullet_speed, double dt)
{
  if (armor_xyza_list.empty()) return {false, false, 0, 0};

  int armor_num = static_cast<int>(armor_xyza_list.size());

  // Compute delay time based on target angular velocity
  double delay_time = std::abs(ekf_x[7]) > params_.decision_speed
                        ? params_.high_speed_delay_time
                        : params_.low_speed_delay_time;

  if (bullet_speed < 14) bullet_speed = 23;

  // Total prediction time
  double total_dt = dt + delay_time;

  // Simple linear prediction of target state at future time
  Eigen::VectorXd ekf_x_future = ekf_x;
  ekf_x_future[0] += ekf_x[1] * total_dt;   // xc += vxc * dt
  ekf_x_future[2] += ekf_x[3] * total_dt;   // yc += vyc * dt
  ekf_x_future[4] += ekf_x[5] * total_dt;   // zc += vzc * dt
  ekf_x_future[6] = limit_rad(ekf_x[6] + ekf_x[7] * total_dt); // yaw += vyaw * dt

  // Update armor positions with predicted state
  std::vector<Eigen::Vector4d> future_armor_xyza;
  for (int i = 0; i < armor_num; i++) {
    double r = (armor_num == 4 && (i == 1 || i == 3))
                 ? ekf_x_future[8] + ekf_x_future[9]
                 : ekf_x_future[8];
    double angle = limit_rad(ekf_x_future[6] + i * 2 * M_PI / armor_num);
    double ax = ekf_x_future[0] - r * std::cos(angle);
    double ay = ekf_x_future[2] - r * std::sin(angle);
    double az = (armor_num == 4 && (i == 1 || i == 3))
                  ? ekf_x_future[4] + ekf_x_future[10]
                  : ekf_x_future[4];
    future_armor_xyza.push_back({ax, ay, az, angle});
  }

  // Choose best aim point
  auto aim_point0 = choose_aim_point(ekf_x_future, future_armor_xyza, jumped, armor_num);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  double d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // Iterative fly-time compensation (max 10 iterations)
  double prev_fly_time = trajectory0.fly_time;
  Trajectory current_traj = trajectory0;

  for (int iter = 0; iter < 10; ++iter) {
    // Predict target at future + fly_time
    double iter_dt = total_dt + prev_fly_time;
    Eigen::VectorXd ekf_x_iter = ekf_x;
    ekf_x_iter[0] += ekf_x[1] * iter_dt;
    ekf_x_iter[2] += ekf_x[3] * iter_dt;
    ekf_x_iter[4] += ekf_x[5] * iter_dt;
    ekf_x_iter[6] = limit_rad(ekf_x[6] + ekf_x[7] * iter_dt);

    std::vector<Eigen::Vector4d> iter_armor_xyza;
    for (int i = 0; i < armor_num; i++) {
      double r = (armor_num == 4 && (i == 1 || i == 3))
                   ? ekf_x_iter[8] + ekf_x_iter[9]
                   : ekf_x_iter[8];
      double angle = limit_rad(ekf_x_iter[6] + i * 2 * M_PI / armor_num);
      double ax = ekf_x_iter[0] - r * std::cos(angle);
      double ay = ekf_x_iter[2] - r * std::sin(angle);
      double az = (armor_num == 4 && (i == 1 || i == 3))
                    ? ekf_x_iter[4] + ekf_x_iter[10]
                    : ekf_x_iter[4];
      iter_armor_xyza.push_back({ax, ay, az, angle});
    }

    auto aim_point = choose_aim_point(ekf_x_iter, iter_armor_xyza, jumped, armor_num);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = Trajectory(bullet_speed, d, xyz.z());

    if (current_traj.unsolvable) {
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + params_.yaw_offset;
  double pitch = -(current_traj.pitch + params_.pitch_offset);
  return {true, false, yaw, pitch};
}

AimCommand Aimer::aim(
  const Eigen::VectorXd & ekf_x,
  const std::vector<Eigen::Vector4d> & armor_xyza_list,
  bool jumped, double bullet_speed,
  int shoot_mode, double dt)
{
  double yaw_offset;
#ifdef ROS_DISTRO_HUMBLE
  if (shoot_mode == 1 && params_.left_yaw_offset.has_value()) {
    yaw_offset = params_.left_yaw_offset.value;       // member access
  } else if (shoot_mode == 2 && params_.right_yaw_offset.has_value()) {
    yaw_offset = params_.right_yaw_offset.value;
  }
#else
  if (shoot_mode == 1 && params_.left_yaw_offset.has_value()) {
    yaw_offset = params_.left_yaw_offset.value();     // std::optional API
  } else if (shoot_mode == 2 && params_.right_yaw_offset.has_value()) {
    yaw_offset = params_.right_yaw_offset.value();
  }
#endif
  else {
    yaw_offset = params_.yaw_offset;
  }

  auto command = aim(ekf_x, armor_xyza_list, jumped, bullet_speed, dt);
  command.yaw = command.yaw - params_.yaw_offset + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(
  const Eigen::VectorXd & ekf_x,
  const std::vector<Eigen::Vector4d> & armor_xyza_list,
  bool jumped, int armor_num)
{
  // If armor hasn't jumped, only the current armor position is known
  if (!jumped) return {true, armor_xyza_list[0]};

  // Center yaw in world frame
  double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // Delta angles for each armor plate
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    double delta_angle = limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.push_back(delta_angle);
  }

  // Non-spinning case: select armor within shootable range
  if (std::abs(ekf_x[7]) <= 2 && armor_num != 3) {  // not outpost
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60.0 / 180.0 * M_PI) continue;
      id_list.push_back(i);
    }

    if (id_list.empty()) {
      return {false, armor_xyza_list[0]};
    }

    // Lock mode: prevent oscillation between two visible armors
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // Not in lock mode: enter lock on the one with smaller delta_angle
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1]))
                     ? id0
                     : id1;

      return {true, armor_xyza_list[static_cast<int>(lock_id_)]};
    }

    // Only one in range: exit lock
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  // Spinning case: use coming/leaving angle logic
  double coming_angle = params_.coming_angle;
  double leaving_angle = params_.leaving_angle;

  if (armor_num == 3) {  // outpost
    coming_angle = 70.0 / 180.0 * M_PI;
    leaving_angle = 30.0 / 180.0 * M_PI;
  }

  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle)
      return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle)
      return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

}  // namespace tools
}  // namespace rm_serial_driver
