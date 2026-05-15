#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cfloat>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace rm_auto_aim
{

// ── anonymous helpers (from sp_vision_25 tools/math_tools) ──

namespace
{
double limit_rad(double angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle <= -M_PI) angle += 2 * M_PI;
  return angle;
}

// cartesian → spherical (yaw, pitch, distance)
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)
{
  double x = xyz[0], y = xyz[1], z = xyz[2];
  double yaw = std::atan2(y, x);
  double pitch = std::atan2(z, std::sqrt(x * x + y * y));
  double distance = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, distance};
}

// Jacobian of xyz → ypd
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz)
{
  double x = xyz[0], y = xyz[1], z = xyz[2];
  double x2y2 = x * x + y * y;
  double sqrt_x2y2 = std::sqrt(x2y2);
  double x2y2z2 = x2y2 + z * z;
  double sqrt_all = std::sqrt(x2y2z2);

  double dyaw_dx = -y / x2y2;
  double dyaw_dy = x / x2y2;

  double dpitch_dx = -(x * z) / ((z * z / x2y2 + 1) * std::pow(x2y2, 1.5));
  double dpitch_dy = -(y * z) / ((z * z / x2y2 + 1) * std::pow(x2y2, 1.5));
  double dpitch_dz = 1.0 / ((z * z / x2y2 + 1) * sqrt_x2y2);

  double dd_dx = x / sqrt_all;
  double dd_dy = y / sqrt_all;
  double dd_dz = z / sqrt_all;

  Eigen::MatrixXd J(3, 3);
  J << dyaw_dx, dyaw_dy, 0,
       dpitch_dx, dpitch_dy, dpitch_dz,
       dd_dx, dd_dy, dd_dz;
  return J;
}
}  // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════════
// Tracker
// ═══════════════════════════════════════════════════════════════════════════

Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST),
  tracked_id(std::string("")),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(11)),
  update_count(0),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff)
{
}

void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No armor detected!");
    return;
  }

  // Choose armor closest to image center
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  initEKF(tracked_armor);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;
  detect_count_ = 1;
  update_count = 0;
  jumped = false;
  last_id_ = 0;
  is_converged_ = false;
  switch_count_ = 0;

  updateArmorsNum(tracked_armor);
}

void Tracker::update(const Armors::SharedPtr & armors_msg, double dt)
{
  // ---- EKF predict ----
  predict(dt);

  bool matched = false;
  target_state = ekf.x;

  if (!armors_msg->armors.empty()) {
    // Find the closest armor with matching number
    Armor same_id_armor;
    int same_id_armors_count = 0;
    auto predicted_position = getArmorPositionFromState(ekf.x);
    double min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;

    for (const auto & armor : armors_msg->armors) {
      if (armor.number == tracked_id) {
        same_id_armor = armor;
        same_id_armors_count++;
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        if (position_diff < min_position_diff) {
          min_position_diff = position_diff;
          yaw_diff = std::abs(orientationToYaw(armor.pose.orientation) - ekf.x(6));
          tracked_armor = armor;
        }
      }
    }

    info_position_diff = min_position_diff;
    info_yaw_diff = yaw_diff;

    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
      matched = true;
      update_count++;

      // ---- EKF update using sp_vision_25 measurement model ----
      // Match armor to expected plate id
      int id = 0;
      double min_angle_error = 1e10;
      const auto xyza_list = armor_xyza_list();
      int armor_num = static_cast<int>(tracked_armors_num);

      // Sort by distance, pick best among 3 closest
      std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
      for (int i = 0; i < armor_num; i++) {
        xyza_i_list.push_back({xyza_list[i], i});
      }
      std::sort(
        xyza_i_list.begin(), xyza_i_list.end(),
        [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
          Eigen::Vector3d ypd1 = xyz2ypd(a.first.head(3));
          Eigen::Vector3d ypd2 = xyz2ypd(b.first.head(3));
          return ypd1[2] < ypd2[2];
        });

      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      for (int i = 0; i < std::min(3, armor_num); i++) {
        const auto & xyza = xyza_i_list[i].first;
        Eigen::Vector3d ypd = xyz2ypd(xyza.head(3));
        double angle_error = std::abs(limit_rad(measured_yaw - xyza[3])) +
                             std::abs(limit_rad(
                               std::atan2(tracked_armor.pose.position.y, tracked_armor.pose.position.x) -
                               ypd[0]));

        if (std::abs(angle_error) < std::abs(min_angle_error)) {
          id = xyza_i_list[i].second;
          min_angle_error = angle_error;
        }
      }

      if (id != 0) jumped = true;
      if (id != last_id_) {
        is_switch_ = true;
        switch_count_++;
      } else {
        is_switch_ = false;
      }
      last_id_ = id;

      update_ypda(tracked_armor, id);
      target_state = ekf.x;
      RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update (sp_vision_25 model)");

    } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
      handleArmorJump(same_id_armor);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found!");
    }
  }

  // Prevent radius from spreading (physical constraints)
  if (ekf.x(8) < 0.12) {
    ekf.x(8) = 0.12;
  } else if (ekf.x(8) > 0.4) {
    ekf.x(8) = 0.4;
  }
  target_state = ekf.x;

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
        RCLCPP_INFO(rclcpp::get_logger("armor_tracker"), "Target tracking confirmed!");
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_ = 1;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
        RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Target lost!");
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }
}

// ── predict (sp_vision_25 Target::predict) ──

void Tracker::predict(double dt)
{
  // State transition matrix F (11x11 constant velocity model)
  // clang-format off
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(11, 11);
  F(0, 1) = dt;   // xc += vxc * dt
  F(2, 3) = dt;   // yc += vyc * dt
  F(4, 5) = dt;   // zc += vzc * dt
  F(6, 7) = dt;   // yaw += vyaw * dt
  // clang-format on

  // Process noise Q (Piecewise White Noise Model)
  double v1, v2;
  if (tracked_id == "outpost") {
    v1 = 10;   // outpost accel variance
    v2 = 0.1;  // outpost angular accel variance
  } else {
    v1 = 100;  // accel variance
    v2 = 400;  // angular accel variance
  }
  double a = dt * dt * dt * dt / 4;
  double b = dt * dt * dt / 2;
  double c = dt * dt;

  // clang-format off
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
  Q(0, 0) = a * v1; Q(0, 1) = b * v1;
  Q(1, 0) = b * v1; Q(1, 1) = c * v1;
  Q(2, 2) = a * v1; Q(2, 3) = b * v1;
  Q(3, 2) = b * v1; Q(3, 3) = c * v1;
  Q(4, 4) = a * v1; Q(4, 5) = b * v1;
  Q(5, 4) = b * v1; Q(5, 5) = c * v1;
  Q(6, 6) = a * v2; Q(6, 7) = b * v2;
  Q(7, 6) = b * v2; Q(7, 7) = c * v2;
  // r1, r2_diff, z_diff have zero process noise (static parameters)
  // clang-format on

  // Outpost speed clamping
  if (is_converged() && tracked_id == "outpost" && std::abs(ekf.x[7]) > 2)
    ekf.x[7] = ekf.x[7] > 0 ? outpost_max_v_yaw : -outpost_max_v_yaw;

  ekf.predict(F, Q, [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = limit_rad(x_prior[6]);
    return x_prior;
  });
}

// ── update (sp_vision_25 Target::update_ypda) ──

void Tracker::update_ypda(const Armor & armor, int id)
{
  // Measurement Jacobian: d(ypd, angle) / d(state)
  Eigen::MatrixXd H = h_jacobian(ekf.x, id);

  // Adaptive measurement noise R
  double delta_angle = limit_rad(orientationToYaw(armor.pose.orientation) -
                                 std::atan2(ekf.x[2], ekf.x[0]));
  Eigen::Vector3d armor_ypd = xyz2ypd(Eigen::Vector3d(
    armor.pose.position.x, armor.pose.position.y, armor.pose.position.z));
  Eigen::Vector4d R_dig;
  R_dig << 4e-3, 4e-3,
           std::log(std::abs(delta_angle) + 1) + 1,
           std::log(std::abs(armor_ypd[2]) + 1) / 200 + 9e-2;
  Eigen::Matrix4d R = R_dig.asDiagonal();

  // Nonlinear observation function: state → [ypd_yaw, ypd_pitch, ypd_dist, angle]
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::Vector3d xyz = h_armor_xyz(x, id);
    Eigen::Vector3d ypd = xyz2ypd(xyz);
    double angle = limit_rad(x[6] + id * 2 * M_PI / static_cast<int>(tracked_armors_num));
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // Angular-aware subtraction
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = limit_rad(c[0]);  // yaw
    c[1] = limit_rad(c[1]);  // pitch
    c[3] = limit_rad(c[3]);  // angle
    return c;
  };

  // Measurement vector
  Eigen::Vector3d ypd = xyz2ypd(Eigen::Vector3d(
    armor.pose.position.x, armor.pose.position.y, armor.pose.position.z));
  double measured_yaw = orientationToYaw(armor.pose.orientation);
  Eigen::Vector4d z{{ypd[0], ypd[1], ypd[2], measured_yaw}};

  ekf.update(z, H, R, h, z_subtract);
}

// ── armor position from state ──

Eigen::Vector3d Tracker::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  int armor_num = static_cast<int>(tracked_armors_num);
  double angle = limit_rad(x[6] + id * 2 * M_PI / armor_num);
  bool use_l_h = (armor_num == 4) && (id == 1 || id == 3);

  double r = use_l_h ? x[8] + x[9] : x[8];
  double armor_x = x[0] - r * std::cos(angle);
  double armor_y = x[2] - r * std::sin(angle);
  double armor_z = use_l_h ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Tracker::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  int armor_num = static_cast<int>(tracked_armors_num);
  double angle = limit_rad(x[6] + id * 2 * M_PI / armor_num);
  bool use_l_h = (armor_num == 4) && (id == 1 || id == 3);

  double r = use_l_h ? x[8] + x[9] : x[8];
  double dx_da = r * std::sin(angle);
  double dy_da = -r * std::cos(angle);
  double dx_dr = -std::cos(angle);
  double dy_dr = -std::sin(angle);
  double dx_dl = use_l_h ? -std::cos(angle) : 0.0;
  double dy_dl = use_l_h ? -std::sin(angle) : 0.0;
  double dz_dh = use_l_h ? 1.0 : 0.0;

  // Jacobian of armor_xyz w.r.t state → (4x11)
  // clang-format off
  Eigen::MatrixXd H_xyza(4, 11);
  H_xyza << 1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0,
            0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0,
            0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh,
            0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0;
  // clang-format on

  Eigen::Vector3d armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_ypd = xyz2ypd_jacobian(armor_xyz);

  // Chain rule: d(ypd) / d(state) = d(ypd)/d(xyz) * d(xyz)/d(state)
  // clang-format off
  Eigen::MatrixXd H_full(4, 11);
  H_full << H_ypd(0, 0), H_ypd(0, 1), H_ypd(0, 2), 0,
            H_ypd(1, 0), H_ypd(1, 1), H_ypd(1, 2), 0,
            H_ypd(2, 0), H_ypd(2, 1), H_ypd(2, 2), 0,
                       0,            0,            0, 1;
  // clang-format on

  return H_full * H_xyza;
}

std::vector<Eigen::Vector4d> Tracker::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> result;
  int armor_num = static_cast<int>(tracked_armors_num);

  for (int i = 0; i < armor_num; i++) {
    double angle = limit_rad(ekf.x[6] + i * 2 * M_PI / armor_num);
    Eigen::Vector3d xyz = h_armor_xyz(ekf.x, i);
    result.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return result;
}

// ── convergence / divergence ──

bool Tracker::is_converged() const
{
  return is_converged_;
}

bool Tracker::is_diverged() const
{
  double r = ekf.x[8];
  bool r_ok = r > 0.05 && r < 0.5;

  if (static_cast<int>(tracked_armors_num) == 4) {
    double l = ekf.x[8] + ekf.x[9];
    bool l_ok = l > 0.05 && l < 0.5;
    return !(r_ok && l_ok);
  }

  return !r_ok;
}

// ── EKF init (sp_vision_25 style) ──

void Tracker::initEKF(const Armor & a)
{
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  double r = 0.26;
  dz = 0;
  another_r = r;

  if (a.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
    r = 0.2;
  } else if (tracked_id == "outpost") {
    r = 0.2765;
  }

  // 11D state: [xc, vxc, yc, vyc, zc, vzc, yaw, vyaw, r1, r2_diff, z_diff]
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  Eigen::VectorXd x0(11);
  x0 << xc, 0, yc, 0, za, 0, yaw, 0, r, 0, 0;

  // Initial covariance diagonal (from sp_vision_25 Target)
  Eigen::VectorXd P0_dig(11);
  P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;

  if (tracked_id == "outpost") {
    P0_dig << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
  }

  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // Angle-aware state addition
  auto x_add = [](const Eigen::VectorXd & a_vec, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a_vec + b;
    c[6] = limit_rad(c[6]);
    return c;
  };

  ekf = ExtendedKalmanFilter(x0, P0, x_add);
  target_state = ekf.x;
}

void Tracker::updateArmorsNum(const Armor & armor)
{
  if (armor.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
    tracked_armors_num = ArmorsNum::BALANCE_2;
  } else if (tracked_id == "outpost") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

void Tracker::handleArmorJump(const Armor & current_armor)
{
  double yaw = orientationToYaw(current_armor.pose.orientation);
  ekf.x(6) = yaw;
  updateArmorsNum(current_armor);

  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    dz = ekf.x(4) - current_armor.pose.position.z;
    ekf.x(4) = current_armor.pose.position.z;
    std::swap(ekf.x(8), another_r);
  }
  RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

  // If position diff is large, reset diverged state
  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(ekf.x);
  if ((current_p - infer_p).norm() > max_match_distance_) {
    double r_val = ekf.x(8);
    ekf.x(0) = p.x + r_val * cos(yaw);
    ekf.x(1) = 0;
    ekf.x(2) = p.y + r_val * sin(yaw);
    ekf.x(3) = 0;
    ekf.x(4) = p.z;
    ekf.x(5) = 0;
    ekf.x(9) = 0;
    ekf.x(10) = 0;
    RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Reset diverged state!");
  }

  target_state = ekf.x;
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
{
  double xc = x(0), yc = x(2), zc = x(4);
  double yaw = x(6), r = x(8);
  return Eigen::Vector3d(xc - r * cos(yaw), yc - r * sin(yaw), zc);
}

}  // namespace rm_auto_aim
