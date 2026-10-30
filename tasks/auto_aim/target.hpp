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

// 单个运动模型的参数
struct ModelConfig
{
  double sa2 = 10.0;     // 每帧加速度增量的方差 (m/s^2)^2
  double v2 = 150.0;     // 角加速度方差 (rad/s^2)^2
  bool ca = false;       // true: 加速度状态延续（匀加速）；false: 每帧清零（匀速）
  bool damping = false;  // true: 平移速度指数衰减（静止）
};

// 观测噪声 R 的在线自适应：用监控 NIS 反馈一个全局缩放因子
struct AdaptiveRConfig
{
  bool enable = false;        // 默认关闭，需要显式开启
  double tau_seconds = 2.0;   // 反馈时间常数，越小平滑越差但收敛越快
  double min_scale = 0.3;  // 与 configs/IMM.yaml 一致；0.05 会让近/中距离过冲
  double max_scale = 20.0;
  size_t start_samples = 30;  // 冷启动保护：前 N 次更新不调整
  double ratio_min = 0.25;    // 单帧 NIS/dof 限幅，防止一帧离群把 scale 拉飞
  double ratio_max = 4.0;
};

// IMM / EKF 的全部可调参数，默认值与 configs/IMM.yaml 一致
struct TargetConfig
{
  std::array<ModelConfig, 3> models{{
    {10.0, 150.0, false, false},  // M1 匀速
    {10.0, 400.0, true, false},   // M2 匀加速
    {0.1, 150.0, false, true},    // M3 静止
  }};

  double acc_damping = 0.90;  // M2 加速度衰减系数
  double damping_tau = 0.30;  // M3 平移速度衰减时间常数

  double pi_diag = 0.90;  // 模型保持概率
  double pi_off = 0.05;   // 模型跳转概率，需满足 pi_diag + 2*pi_off = 1
  std::array<double, 3> mu0{{0.8, 0.15, 0.05}};
  double mu_floor = 1e-3;

  bool huber_enable = true;
  double huber_k = -1.0;  // <=0 时按块维度自动取 chi2 分位点
  size_t huber_block = 4;

  double window_seconds = 1.0;
  size_t min_window_samples = 20;
  double nis_threshold = -1.0;
  double tracker_fail_rate = 0.4;

  // 观测噪声 R 的四个对角项：
  //   R_yaw = yaw_var, R_pitch = pitch_var
  //   R_dist = dist_gain * log(|Δangle| + 1) + dist_offset
  //   R_ayaw = ayaw_gain * log(distance + 1) + ayaw_offset
  double yaw_var = 4e-3;
  double pitch_var = 4e-3;
  double dist_gain = 1.0;
  double dist_offset = 1.0;
  double ayaw_gain = 0.005;
  double ayaw_offset = 0.09;

  AdaptiveRConfig adaptive_r;

  // 初始协方差对角（顺序 x vx y vy z vz a w r l h ax ay az）
  std::array<double, 14> p0{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1, 25, 25, 25}};

  Eigen::VectorXd p0_vector() const
  {
    return Eigen::Map<const Eigen::VectorXd>(p0.data(), static_cast<Eigen::Index>(p0.size()));
  }

  // 从 YAML 读取；文件不存在或缺键时保留默认值
  static TargetConfig load(const std::string & path);
};

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority = ArmorPriority::fifth;
  bool jumped;
  int last_id;  // debug only

  Target() : armor_num_(0) {}
  // P0_dig 为空时使用 config 里的 initial_covariance
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig = {}, const TargetConfig & config = TargetConfig{});
  Target(double x, double vyaw, double radius, double h, const TargetConfig & config = TargetConfig{});

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

  // 三个模型的当前概率（M1/M2/M3），供调试展示
  Eigen::Vector3d model_probabilities() const;
  // 主模型（M1）的 NIS/dof，理想值 1
  double nis_ratio() const;
  // R 的当前在线自适应缩放因子（未开启时恒为 1）
  double r_scale() const { return r_scale_; }

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
  TargetConfig config_;
  double r_scale_ = 1.0;  // 观测噪声 R 的在线自适应缩放因子
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