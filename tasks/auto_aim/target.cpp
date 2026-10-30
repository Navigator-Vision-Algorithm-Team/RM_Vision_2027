#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numeric>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{

TargetConfig TargetConfig::load(const std::string & path)
{
  TargetConfig cfg;

  try {
    auto yaml = YAML::LoadFile(path);

    if (yaml["models"] && yaml["models"].IsSequence()) {
      int i = 0;
      for (const auto & m : yaml["models"]) {
        if (i >= static_cast<int>(cfg.models.size())) break;
        auto & model = cfg.models[i];
        if (m["sa2"]) model.sa2 = m["sa2"].as<double>();
        if (m["v2"]) model.v2 = m["v2"].as<double>();
        if (m["ca"]) model.ca = m["ca"].as<bool>();
        if (m["damping"]) model.damping = m["damping"].as<bool>();
        i++;
      }
    }

    if (yaml["motion"]) {
      const auto & motion = yaml["motion"];
      if (motion["acc_damping"]) cfg.acc_damping = motion["acc_damping"].as<double>();
      if (motion["damping_tau"]) cfg.damping_tau = motion["damping_tau"].as<double>();
    }

    if (yaml["transition"]) {
      const auto & transition = yaml["transition"];
      if (transition["diag"]) cfg.pi_diag = transition["diag"].as<double>();
      if (transition["off_diag"]) cfg.pi_off = transition["off_diag"].as<double>();
    }

    if (yaml["probability"]) {
      const auto & probability = yaml["probability"];
      if (probability["initial"] && probability["initial"].IsSequence()) {
        int i = 0;
        for (const auto & v : probability["initial"]) {
          if (i >= static_cast<int>(cfg.mu0.size())) break;
          cfg.mu0[i] = v.as<double>();
          i++;
        }
      }
      if (probability["floor"]) cfg.mu_floor = probability["floor"].as<double>();
    }

    if (yaml["huber"]) {
      const auto & huber = yaml["huber"];
      if (huber["enable"]) cfg.huber_enable = huber["enable"].as<bool>();
      if (huber["k"]) cfg.huber_k = huber["k"].as<double>();
      if (huber["block_size"]) cfg.huber_block = huber["block_size"].as<size_t>();
    }

    if (yaml["consistency"]) {
      const auto & consistency = yaml["consistency"];
      if (consistency["window_seconds"])
        cfg.window_seconds = consistency["window_seconds"].as<double>();
      if (consistency["min_window_samples"])
        cfg.min_window_samples = consistency["min_window_samples"].as<size_t>();
      if (consistency["nis_threshold"])
        cfg.nis_threshold = consistency["nis_threshold"].as<double>();
      if (consistency["tracker_fail_rate"])
        cfg.tracker_fail_rate = consistency["tracker_fail_rate"].as<double>();
    }

    if (yaml["measurement"]) {
      const auto & m = yaml["measurement"];
      if (m["yaw_var"]) cfg.yaw_var = m["yaw_var"].as<double>();
      if (m["pitch_var"]) cfg.pitch_var = m["pitch_var"].as<double>();
      if (m["distance_gain"]) cfg.dist_gain = m["distance_gain"].as<double>();
      if (m["distance_offset"]) cfg.dist_offset = m["distance_offset"].as<double>();
      if (m["armor_yaw_gain"]) cfg.ayaw_gain = m["armor_yaw_gain"].as<double>();
      if (m["armor_yaw_offset"]) cfg.ayaw_offset = m["armor_yaw_offset"].as<double>();
    }

    if (yaml["initial_covariance"] && yaml["initial_covariance"].IsSequence()) {
      int i = 0;
      for (const auto & v : yaml["initial_covariance"]) {
        if (i >= static_cast<int>(cfg.p0.size())) break;
        cfg.p0[i] = v.as<double>();
        i++;
      }
    }
  } catch (const std::exception & e) {
    tools::logger()->warn("[TargetConfig] 读取 {} 失败，使用默认值：{}", path, e.what());
  }

  // 转移矩阵的列和必须为 1，否则按对角重新归一
  if (std::abs(cfg.pi_diag + 2.0 * cfg.pi_off - 1.0) > 1e-6) {
    tools::logger()->warn("[TargetConfig] transition 列和 != 1，已按 diag 自动修正");
    cfg.pi_off = (1.0 - cfg.pi_diag) / 2.0;
  }

  return cfg;
}

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig, const TargetConfig & config)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  config_(config),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h ax ay az
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  // 加速度（ax/ay/az）放在末尾，保证 0~10 的索引与旧版完全一致：
  // aimer / planner / shooter / commandgener 里的 ekf_x()[...] 全部无需改动
  /*
  | 下标 | 状态量  | 含义               |
| -: | ---- | ---------------- |
|  0 | `x`  | 目标旋转中心的世界坐标 x    |
|  1 | `vx` | 旋转中心在 x 方向的速度    |
|  2 | `y`  | 目标旋转中心的世界坐标 y    |
|  3 | `vy` | 旋转中心在 y 方向的速度    |
|  4 | `z`  | 目标旋转中心的世界坐标 z    |
|  5 | `vz` | 旋转中心在 z 方向的速度    |
|  6 | `a`  | 当前目标/装甲板的旋转角 yaw |
|  7 | `w`  | 角速度，yaw 每秒变化多少   |
|  8 | `r`  | 第一组装甲板到旋转中心的半径   |
|  9 | `l`  | 两组装甲板半径差，`r2-r1` |
| 10 | `h`  | 两组装甲板高度差，`z2-z1` |
| 11 | `ax` | 旋转中心在 x 方向的加速度   |
| 12 | `ay` | 旋转中心在 y 方向的加速度   |
| 13 | `az` | 旋转中心在 z 方向的加速度   |
  */
  // 索引：x vx y vy z vz a w r l h ax ay az
  Eigen::VectorXd x0{
    {center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0, 0, 0, 0}};  //初始化预测量

  // 未显式给 P0 时用配置文件里的 initial_covariance
  const Eigen::VectorXd p0_dig = P0_dig.size() == 14 ? P0_dig : config_.p0_vector();
  Eigen::MatrixXd P0 = p0_dig.asDiagonal();

  init_models(x0, P0);
}

Target::Target(double x, double vyaw, double radius, double h, const TargetConfig & config)
: armor_num_(4), config_(config)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h, 0, 0, 0}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  init_models(x0, P0);
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::init_models(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0)
{
  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  for (int j = 0; j < MODEL_NUM; j++) {
    ekf_[j] = tools::ExtendedKalmanFilter(x0, P0, x_add);

    // 量测是 4 维一组（yaw/pitch/distance/armor_yaw），逐板做 Huber 降权
    ekf_[j].enable_huber = config_.huber_enable;
    ekf_[j].huber_k = config_.huber_k;
    ekf_[j].huber_block_size = config_.huber_block;
    ekf_[j].window_seconds = config_.window_seconds;
    ekf_[j].min_window_samples = config_.min_window_samples;
    ekf_[j].nis_threshold = config_.nis_threshold;
  }

  // 模型转移矩阵（列为随机向量，列和为 1）
  Pi_ = Eigen::MatrixXd::Constant(MODEL_NUM, MODEL_NUM, config_.pi_off);
  Pi_.diagonal().setConstant(config_.pi_diag);

  mu_ = config_.mu0;
  x_fused_ = x0;
  P_fused_ = P0;
}

void Target::fuse()
{
  const int n = static_cast<int>(ekf_[0].x.rows());

  x_fused_ = Eigen::VectorXd::Zero(n);
  double sin_sum = 0.0, cos_sum = 0.0;
  for (int j = 0; j < MODEL_NUM; j++) {
    x_fused_ += mu_[j] * ekf_[j].x;
    sin_sum += mu_[j] * std::sin(ekf_[j].x[6]);
    cos_sum += mu_[j] * std::cos(ekf_[j].x[6]);
  }
  // 角度取圆均值，避免 ±pi 附近加权后落到错误分支
  x_fused_[6] = std::atan2(sin_sum, cos_sum);

  // 融合协方差要加上模型均值差项，否则会低估不确定性
  P_fused_ = Eigen::MatrixXd::Zero(n, n);
  for (int j = 0; j < MODEL_NUM; j++) {
    Eigen::VectorXd dx = ekf_[j].x - x_fused_;
    dx[6] = tools::limit_rad(dx[6]);
    P_fused_ += mu_[j] * (ekf_[j].P + dx * dx.transpose());
  }
}

void Target::predict(double dt)
{
  const int state_dim = static_cast<int>(ekf_[0].x.rows());

  // ---- 1) IMM 混合：把各模型状态按转移概率混合成每个模型的初值 ----
  std::array<double, MODEL_NUM> c{};      // 预测的模型概率
  std::array<double, MODEL_NUM> w[MODEL_NUM];  // w[j][i] = P(i -> j) * mu_i / c_j

  for (int j = 0; j < MODEL_NUM; j++) {
    c[j] = 0.0;
    for (int i = 0; i < MODEL_NUM; i++) c[j] += Pi_(i, j) * mu_[i];
    c[j] = std::max(c[j], 1e-9);
    for (int i = 0; i < MODEL_NUM; i++) w[j][i] = Pi_(i, j) * mu_[i] / c[j];
  }

  std::array<Eigen::VectorXd, MODEL_NUM> x_mix;
  std::array<Eigen::MatrixXd, MODEL_NUM> P_mix;

  for (int j = 0; j < MODEL_NUM; j++) {
    x_mix[j] = Eigen::VectorXd::Zero(state_dim);
    double sin_sum = 0.0, cos_sum = 0.0;
    for (int i = 0; i < MODEL_NUM; i++) {
      x_mix[j] += w[j][i] * ekf_[i].x;
      sin_sum += w[j][i] * std::sin(ekf_[i].x[6]);
      cos_sum += w[j][i] * std::cos(ekf_[i].x[6]);
    }
    x_mix[j][6] = std::atan2(sin_sum, cos_sum);

    P_mix[j] = Eigen::MatrixXd::Zero(state_dim, state_dim);
    for (int i = 0; i < MODEL_NUM; i++) {
      Eigen::VectorXd dx = ekf_[i].x - x_mix[j];
      dx[6] = tools::limit_rad(dx[6]);
      P_mix[j] += w[j][i] * (ekf_[i].P + dx * dx.transpose());
    }
  }

  // ---- 2) 各模型从混合初值出发独立预测 ----
  // 状态索引：0:x 1:vx 2:y 3:vy 4:z 5:vz 6:a 7:w 8:r 9:l 10:h 11:ax 12:ay 13:az
  // 加速度放在末尾，保持既有 0~10 的索引不变（aimer/planner/shooter 无需改动）
  constexpr int GROUPS[3][3] = {{0, 1, 11}, {2, 3, 12}, {4, 5, 13}};

  // 加速度增量 Δa ~ N(0, sa2) 通过一帧传播到位置/速度的系数
  const double dt2 = dt * dt;
  const double q_pp = dt2 * dt2 / 4.0;  // 位置-位置
  const double q_pv = dt2 * dt / 2.0;   // 位置-速度
  const double q_vv = dt2;              // 速度-速度
  const double q_pa = dt2 / 2.0;        // 位置-加速度
  const double q_va = dt;               // 速度-加速度

  // M3 静止模型的速度衰减时间常数（太小会拖累小陀螺的 w 估计）
  const double alpha = std::exp(-std::abs(dt) / config_.damping_tau);

  for (int j = 0; j < MODEL_NUM; j++) {
    ekf_[j].x = x_mix[j];
    ekf_[j].P = P_mix[j];

    const auto & model = config_.models[j];

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim, state_dim);
    F(6, 7) = dt;  // 角度：a' = a + w*dt

    for (int g = 0; g < 3; g++) {
      const int p = GROUPS[g][0], v = GROUPS[g][1], acc = GROUPS[g][2];

      if (model.damping) {
        // 静止：平移速度指数衰减、加速度清零；角速度不阻尼（目标仍可原地小陀螺）
        F(p, v) = alpha * dt;
        F(v, v) = alpha;
        F(acc, acc) = 0.0;
      } else if (model.ca) {
        // 匀加速：位置与速度都吃加速度项。
        // 加速度自身带 0.95 的轻微阻尼：纯随机游走会让加速度无界漂移，
        // 远距离弱观测时（far_noisy）会把位置和 yaw 一起带抖
        F(p, v) = dt;
        F(p, acc) = 0.5 * dt * dt;
        F(v, v) = 1.0;
        F(v, acc) = dt;
        F(acc, acc) = config_.acc_damping;
      } else {
        // 匀速：加速度每帧清零，不参与位置/速度递推
        F(p, v) = dt;
        F(v, v) = 1.0;
        F(acc, acc) = 0.0;
      }
    }

    const double sa2 = model.sa2;
    const double v2 = model.v2;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim, state_dim);

    for (int g = 0; g < 3; g++) {
      const int p = GROUPS[g][0], v = GROUPS[g][1], acc = GROUPS[g][2];
      Q(p, p) = q_pp * sa2;
      Q(p, v) = q_pv * sa2;
      Q(v, p) = q_pv * sa2;
      Q(v, v) = q_vv * sa2;
      Q(p, acc) = q_pa * sa2;
      Q(acc, p) = q_pa * sa2;
      Q(v, acc) = q_va * sa2;
      Q(acc, v) = q_va * sa2;
      Q(acc, acc) = sa2;
    }

    // 角度/角速度：沿用 PV 模型的 2x2 块
    Q(6, 6) = q_pp * v2;
    Q(6, 7) = q_pv * v2;
    Q(7, 6) = q_pv * v2;
    Q(7, 7) = q_vv * v2;

    auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
      Eigen::VectorXd x_prior = F * x;
      x_prior[6] = tools::limit_rad(x_prior[6]);
      return x_prior;
    };

    ekf_[j].predict(F, Q, f);
  }

  // ---- 3) 模型概率推进到先验 ----
  for (int j = 0; j < MODEL_NUM; j++) mu_[j] = c[j];

  fuse();
}

void Target::update(const Armor & armor, double dt)
{
  update(std::vector<Armor>{armor}, dt);
}

void Target::update(const std::vector<Armor> & armors, double dt)
{
  if (armors.empty()) return;

  const std::vector<Eigen::Vector4d> xyza_list = armor_xyza_list();

  // 预测量按距离升序，只在最近的 3 块装甲板里寻找匹配
  std::vector<int> id_by_distance(armor_num_);
  std::iota(id_by_distance.begin(), id_by_distance.end(), 0);
  std::sort(id_by_distance.begin(), id_by_distance.end(), [&](int a, int b) {
    return tools::xyz2ypd(xyza_list[a].head(3))[2] < tools::xyz2ypd(xyza_list[b].head(3))[2];
  });
  const int search_num = std::min<int>(3, armor_num_);

  // 装甲板匹配：位置方位角 + 装甲板朝向角联合误差最小
  std::vector<MatchedArmor> candidates;
  for (const auto & armor : armors) {
    int best_id = -1;
    double min_angle_error = 1e10;
    for (int k = 0; k < search_num; k++) {
      const int i = id_by_distance[k];
      const Eigen::Vector4d & xyza = xyza_list[i];
      Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
      auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                         std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

      if (angle_error < min_angle_error) {
        min_angle_error = angle_error;
        best_id = i;
      }
    }
    if (best_id >= 0) candidates.push_back({&armor, best_id, min_angle_error});
  }
  if (candidates.empty()) return;

  // 同一 id 只保留角度误差最小的一块：同一帧里重复检测到同一块板时，
  // 若逐块 update，等于把一帧的信息用了多次，会让协方差过度自信
  std::vector<MatchedArmor> matched;
  for (const auto & candidate : candidates) {
    auto it = std::find_if(
      matched.begin(), matched.end(),
      [&](const MatchedArmor & m) { return m.id == candidate.id; });
    if (it == matched.end()) {
      matched.push_back(candidate);
    } else if (candidate.error < it->error) {
      *it = candidate;
    }
  }

  // 主装甲板 = 角度误差最小的那块，jumped / is_switch_ 语义沿用单板实现
  std::sort(
    matched.begin(), matched.end(),
    [](const MatchedArmor & a, const MatchedArmor & b) { return a.error < b.error; });
  const int primary_id = matched.front().id;

  if (primary_id != 0) jumped = true;

  is_switch_ = (primary_id != last_id);
  if (is_switch_) switch_count_++;

  last_id = primary_id;
  update_count_++;

  update_ypda(matched, dt);
}

Eigen::VectorXd Target::measurement_noise(const Armor & armor) const
{
  // 观测噪声：目标越偏离视野中心，PnP 的距离越不可信；距离越远，装甲板 yaw 越不可信
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  return Eigen::VectorXd{
    {config_.yaw_var, config_.pitch_var,
     config_.dist_gain * std::log(std::abs(delta_angle) + 1) + config_.dist_offset,
     config_.ayaw_gain * std::log(std::abs(armor.ypd_in_world[2]) + 1) + config_.ayaw_offset}};
}

void Target::update_ypda(const std::vector<MatchedArmor> & matched, double dt)
{
  const int n = static_cast<int>(matched.size());
  const int m = 4 * n;
  const int state_dim = static_cast<int>(ekf_[0].x.rows());

  // 量测 z 与噪声 R 与模型无关，先统一构造
  Eigen::VectorXd z(m);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
  std::vector<int> ids(n);

  for (int k = 0; k < n; k++) {
    const Armor & armor = *matched[k].armor;
    ids[k] = matched[k].id;
    R.block(4 * k, 4 * k, 4, 4) = measurement_noise(armor).asDiagonal();
    z.segment<4>(4 * k) << armor.ypd_in_world[0], armor.ypd_in_world[1], armor.ypd_in_world[2],
      armor.ypr_in_world[0];
  }

  // 非线性转换函数 h: x -> z，按 4 维一组堆叠
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd z_pred(m);
    for (int k = 0; k < n; k++) {
      const int id = ids[k];
      Eigen::Vector3d xyz = h_armor_xyz(x, id);
      Eigen::Vector3d ypd = tools::xyz2ypd(xyz);
      auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
      z_pred.segment<4>(4 * k) << ypd[0], ypd[1], ypd[2], angle;
    }
    return z_pred;
  };

  // 防止夹角求差出现异常值：每 4 维中的 yaw / pitch / armor_yaw 分别回绕
  auto z_subtract = [n](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    for (int k = 0; k < n; k++) {
      c[4 * k] = tools::limit_rad(c[4 * k]);
      c[4 * k + 1] = tools::limit_rad(c[4 * k + 1]);
      c[4 * k + 3] = tools::limit_rad(c[4 * k + 3]);
    }
    return c;
  };

  // 每个模型用自己的先验状态线性化并独立更新，同时记录对数似然
  std::array<double, MODEL_NUM> log_lik{};
  std::array<double, MODEL_NUM> nis_arr{};
  std::array<double, MODEL_NUM> logdet_arr{};
  for (int j = 0; j < MODEL_NUM; j++) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, state_dim);
    for (int k = 0; k < n; k++) {
      H.block(4 * k, 0, 4, state_dim) = h_jacobian(ekf_[j].x, ids[k]);
    }

    ekf_[j].update(z, H, R, h, z_subtract, dt);

    const auto & data = ekf_[j].data;
    const double nis_j = data.count("nis") ? data.at("nis") : 0.0;
    const double log_det_j = data.count("log_det_s") ? data.at("log_det_s") : 0.0;
    nis_arr[j] = nis_j;
    logdet_arr[j] = log_det_j;
    // 忽略所有模型共用的 -m/2·log(2π) 常数项
    log_lik[j] = -0.5 * (nis_j + log_det_j);
  }

  // 模型概率更新：对数域做 log-sum-exp，防止 exp 下溢
  double max_log = -1e30;
  for (int j = 0; j < MODEL_NUM; j++) {
    max_log = std::max(max_log, log_lik[j] + std::log(std::max(mu_[j], 1e-9)));
  }

  double sum = 0.0;
  for (int j = 0; j < MODEL_NUM; j++) {
    mu_[j] = std::exp(log_lik[j] + std::log(std::max(mu_[j], 1e-9)) - max_log);
    sum += mu_[j];
  }

  // 概率下限 1e-3：避免某个模型被永久锁死后再也回不来
  sum = std::max(sum, 1e-9);
  for (int j = 0; j < MODEL_NUM; j++) mu_[j] = std::max(mu_[j] / sum, config_.mu_floor);

  double normalize = 0.0;
  for (int j = 0; j < MODEL_NUM; j++) normalize += mu_[j];
  for (int j = 0; j < MODEL_NUM; j++) mu_[j] /= normalize;

  fuse();

  static const bool imm_debug = std::getenv("IMM_DEBUG") != nullptr;
  if (imm_debug) {
    tools::logger()->debug(
      "[IMM] mu={:.2f},{:.2f},{:.2f} w={:.2f} nis={:.1f},{:.1f},{:.1f} "
      "ldS={:.1f},{:.1f},{:.1f}",
      mu_[0], mu_[1], mu_[2], x_fused_[7], nis_arr[0], nis_arr[1], nis_arr[2], logdet_arr[0],
      logdet_arr[1], logdet_arr[2]);
  }
}

Eigen::VectorXd Target::ekf_x() const { return x_fused_; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_[0]; }

double Target::nis_failure_rate() const
{
  // 任一模型持续失配即视为可疑，取最差值
  double worst = 0.0;
  for (int j = 0; j < MODEL_NUM; j++) worst = std::max(worst, ekf_[j].nis_failure_rate());
  return worst;
}

size_t Target::nis_window_samples() const
{
  size_t best = 0;
  for (int j = 0; j < MODEL_NUM; j++) best = std::max(best, ekf_[j].nis_window_samples());
  return best;
}

size_t Target::min_nis_samples() const
{
  size_t best = 0;
  for (int j = 0; j < MODEL_NUM; j++) best = std::max(best, ekf_[j].min_window_samples);
  return best;
}

Eigen::Vector3d Target::model_probabilities() const
{
  return Eigen::Vector3d(mu_[0], mu_[1], mu_[2]);
}

double Target::nis_ratio() const
{
  const auto & data = ekf_[0].data;
  return data.count("nis_ratio") ? data.at("nis_ratio") : 0.0;
}

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  if (x_fused_.size() < 14) return _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(x_fused_[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(x_fused_, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  if (x_fused_.size() < 14) return true;

  auto r_ok = x_fused_[8] > 0.05 && x_fused_[8] < 0.5;
  auto l_ok = x_fused_[8] + x_fused_[9] > 0.05 && x_fused_[8] + x_fused_[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", x_fused_[8], x_fused_[9]);
  return true;
}

bool Target::convergened()
{
  // 3V3 无前哨站：所有目标统一为更新 3 次后即认为收敛
  if (update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  // 前哨站特殊判断（3V3 无前哨站，已停用）
  // if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
  //   is_converged_ = true;
  // }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // 列索引：0:x 1:vx 2:y 3:vy 4:z 5:vz 6:a 7:w 8:r 9:l 10:h 11:ax 12:ay 13:az
  // 加速度三列对观测量没有直接偏导（位置只依赖 x/y/z 与几何量），保持为 0
  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh, 0, 0, 0},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0, 0, 0, 0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
