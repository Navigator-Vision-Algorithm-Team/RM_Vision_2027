#include "planner.hpp"

#include <cmath>
#include <stdexcept>

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

constexpr double G = 9.7833;

struct BulletTraj
{
  bool unsolvable = false;
  double fly_time = 0;
  double pitch = 0;
  BulletTraj() = default;
  BulletTraj(double v0, double d, double h)
  {
    double a = G * d * d / (2 * v0 * v0);
    double b = -d;
    double c = a + h;
    double delta = b * b - 4 * a * c;
    if (delta < 0) { unsolvable = true; return; }
    unsolvable = false;
    double tan_p1 = (-b + std::sqrt(delta)) / (2 * a);
    double tan_p2 = (-b - std::sqrt(delta)) / (2 * a);
    double p1 = std::atan(tan_p1), p2 = std::atan(tan_p2);
    double t1 = d / (v0 * std::cos(p1)), t2 = d / (v0 * std::cos(p2));
    if (t1 < t2) { pitch = p1; fly_time = t1; }
    else         { pitch = p2; fly_time = t2; }
  }
};
}  // namespace

Planner::Planner(const PlannerParams & params) : params_(params)
{
  setup_yaw_solver();
  setup_pitch_solver();
}

Plan Planner::plan(
  const Eigen::VectorXd & ekf_x,
  const std::vector<Eigen::Vector4d> & armor_xyza_list,
  double bullet_speed)
{
  if (armor_xyza_list.empty()) {
    Plan p; p.control = false; return p;
  }

  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 1. Find closest armor
  double min_dist = 1e10;
  Eigen::Vector3d xyz;
  for (const auto & xyza : armor_xyza_list) {
    double dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    Eigen::Matrix<double, 2, 1> yaw_pitch_result = aim_state(ekf_x, armor_xyza_list, bullet_speed);
    yaw0 = yaw_pitch_result(0);
    traj = get_trajectory(ekf_x, armor_xyza_list, yaw0, bullet_speed);
  } catch (const std::exception &) {
    Plan p; p.control = false; return p;
  }

  // 3. Solve yaw via TinyMPC
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch via TinyMPC
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);
  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  // Fire decision: tracking error at shoot_offset steps into the future
  const int shoot_offset = 2;
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset) -
        yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset),
      traj(2, HALF_HORIZON + shoot_offset) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset)) < params_.fire_thresh;

  return plan;
}

Eigen::Matrix<double, 2, 1> Planner::aim_state(
  const Eigen::VectorXd & ekf_x,
  const std::vector<Eigen::Vector4d> & armor_xyza_list,
  double bullet_speed)
{
  double min_dist = 1e10;
  Eigen::Vector3d xyz;
  double yaw = 0;

  for (const auto & xyza : armor_xyza_list) {
    double dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  double azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = BulletTraj(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  Eigen::Matrix<double, 2, 1> result;
  result << limit_rad(azim + params_.yaw_offset),
            -bullet_traj.pitch - params_.pitch_offset;
  return result;
}

Trajectory Planner::get_trajectory(
  Eigen::VectorXd ekf_x,
  const std::vector<Eigen::Vector4d> & armor_xyza_list,
  double yaw0, double bullet_speed)
{
  Trajectory traj;
  int armor_num = static_cast<int>(armor_xyza_list.size());

  // Helper: advance state and compute aim point
  auto predict_and_aim = [&](Eigen::VectorXd & ekf, double dt) -> Eigen::Matrix<double, 2, 1> {
    ekf[0] += ekf[1] * dt;
    ekf[2] += ekf[3] * dt;
    ekf[4] += ekf[5] * dt;
    ekf[6] = limit_rad(ekf[6] + ekf[7] * dt);

    std::vector<Eigen::Vector4d> future_armors;
    for (int i = 0; i < armor_num; i++) {
      bool use_l_h = (armor_num == 4 && (i == 1 || i == 3));
      double r = use_l_h ? ekf[8] + ekf[9] : ekf[8];
      double angle = limit_rad(ekf[6] + i * 2 * M_PI / armor_num);
      double ax = ekf[0] - r * std::cos(angle);
      double ay = ekf[2] - r * std::sin(angle);
      double az = use_l_h ? ekf[4] + ekf[10] : ekf[4];
      future_armors.push_back({ax, ay, az, angle});
    }

    return aim_state(ekf, future_armors, bullet_speed);
  };

  // Step back by HALF_HORIZON
  double back_dt = -DT * (HALF_HORIZON + 1);
  ekf_x[0] += ekf_x[1] * back_dt;
  ekf_x[2] += ekf_x[3] * back_dt;
  ekf_x[4] += ekf_x[5] * back_dt;
  ekf_x[6] = limit_rad(ekf_x[6] + ekf_x[7] * back_dt);

  auto yaw_pitch_last = predict_and_aim(ekf_x, 0);

  // Step forward to start
  ekf_x[0] += ekf_x[1] * DT;
  ekf_x[2] += ekf_x[3] * DT;
  ekf_x[4] += ekf_x[5] * DT;
  ekf_x[6] = limit_rad(ekf_x[6] + ekf_x[7] * DT);
  auto yaw_pitch = predict_and_aim(ekf_x, 0);

  for (int i = 0; i < HORIZON; i++) {
    ekf_x[0] += ekf_x[1] * DT;
    ekf_x[2] += ekf_x[3] * DT;
    ekf_x[4] += ekf_x[5] * DT;
    ekf_x[6] = limit_rad(ekf_x[6] + ekf_x[7] * DT);
    auto yaw_pitch_next = predict_and_aim(ekf_x, 0);

    double yaw_vel = limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    double pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

void Planner::setup_yaw_solver()
{
  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q_vec(params_.Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R_vec(params_.R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q_vec.asDiagonal(), R_vec.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -params_.max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, params_.max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver()
{
  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q_vec(params_.Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R_vec(params_.R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q_vec.asDiagonal(), R_vec.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -params_.max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, params_.max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;
}

}  // namespace tools
}  // namespace rm_serial_driver
