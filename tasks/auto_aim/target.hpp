#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority = ArmorPriority::fifth;
  bool jumped;
  int last_id;  // debug only

  Target() : armor_num_(0) {}
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);

  // 单块装甲板更新（内部转为批量更新）
  void update(const Armor & armor, double dt = -1.0);
  // 同一帧内的多块装甲板合并为一次批量更新，避免重复使用同一帧信息
  void update(const std::vector<Armor> & armors, double dt = -1.0);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  // IMM 的一致性指标：三个模型取最差值，任一模型持续失配即视为可疑
  double nis_failure_rate() const;
  size_t nis_window_samples() const;
  size_t min_nis_samples() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  // 3 模型 IMM：M1 常规 / M2 高机动 / M3 静止阻尼
  static constexpr int MODEL_NUM = 3;
  std::array<tools::ExtendedKalmanFilter, MODEL_NUM> ekf_;
  std::array<double, MODEL_NUM> mu_{{0.8, 0.15, 0.05}};
  Eigen::MatrixXd Pi_;      // 模型转移矩阵 Pi_(i,j) = P(i -> j)
  Eigen::VectorXd x_fused_;
  Eigen::MatrixXd P_fused_;
  std::chrono::steady_clock::time_point t_;

  void init_models(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0);
  void fuse();

  // 一次 update 中匹配到的一块装甲板
  struct MatchedArmor
  {
    const Armor * armor;
    int id;
    double error;
  };

  void update_ypda(const std::vector<MatchedArmor> & matched, double dt);  // yaw pitch distance angle

  // 观测噪声（yaw, pitch, distance, armor_yaw 的方差）
  Eigen::VectorXd measurement_noise(const Armor & armor) const;

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP