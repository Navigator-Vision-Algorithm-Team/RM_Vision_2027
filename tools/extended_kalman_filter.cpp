#include "extended_kalman_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace tools
{
namespace
{
// 分位点查表（自由度 1~20），比近似公式更贴近真值
constexpr double CHI2_95[] = {
  3.8415, 5.9915, 7.8147,  9.4877,  11.0705, 12.5916, 14.0671, 15.5073, 16.9190, 18.3070,
  19.6751, 21.0261, 22.3620, 23.6848, 24.9958, 26.2962, 27.5871, 28.8693, 30.1435, 31.4104};

constexpr double CHI2_99[] = {
  6.6349, 9.2103, 11.3449, 13.2767, 15.0863, 16.8119, 18.4753, 20.0902, 21.6660, 23.2093,
  24.7250, 26.2170, 27.6882, 29.1412, 30.5779, 32.0002, 33.4087, 34.8053, 36.1909, 37.5662};

double chi2_quantile(int dof, const double * table, double z_p)
{
  if (dof <= 0) return 0.0;
  if (dof <= 20) return table[dof - 1];

  // Wilson-Hilferty 近似，dof > 20 时相对误差 < 0.5%
  const double k = static_cast<double>(dof);
  const double t = 1.0 - 2.0 / (9.0 * k) + z_p * std::sqrt(2.0 / (9.0 * k));
  return k * t * t * t;
}
}  // namespace

double chi2_quantile_95(int dof)
{
  return chi2_quantile(dof, CHI2_95, 1.6448536269514722);
}

double chi2_quantile_99(int dof)
{
  return chi2_quantile(dof, CHI2_99, 2.3263478740408408);
}

ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["residual_norm"] = 0.0;
  data["nis"] = 0.0;
  data["nis_threshold"] = 0.0;
  data["nis_fail"] = 0.0;          // 每次 update 都会复位
  data["nis_failure_rate"] = 0.0;
  data["nis_mean"] = 0.0;
  data["nis_ratio"] = 0.0;         // nis_mean / 观测维度，理想值 1
  data["nis_dof"] = 0.0;
  data["huber_weight"] = 1.0;  // 本帧平均 Huber 权重，1 表示未降权
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  P = 0.5 * (P + P.transpose());  // 抑制浮点误差导致的非对称
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract,
  double dt)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract, dt);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, double dt)
{
  return update(
    z, H, R, [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; }, dt);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract,
  double dt)
{
  if (I.rows() != x.rows()) I = Eigen::MatrixXd::Identity(x.rows(), x.rows());

  // 先验状态与先验协方差：NIS 必须基于先验计算，否则后验残差被 (I-KH) 压扁
  const Eigen::VectorXd x_prior = x;
  const Eigen::MatrixXd P_prior = P;

  // 新息（innovation）
  const Eigen::VectorXd innovation = z_subtract(z, h(x_prior));

  // 监控用的新息协方差：始终基于原始 R，保证 NIS 与 Tracker 的失配判据口径不变
  const Eigen::MatrixXd S_monitor = H * P_prior * H.transpose() + R;
  Eigen::LDLT<Eigen::MatrixXd> ldlt_monitor(S_monitor);
  const bool s_ok =
    ldlt_monitor.info() == Eigen::Success && ldlt_monitor.vectorD().minCoeff() > 0.0;

  Eigen::MatrixXd S_monitor_inv;
  if (s_ok) {
    S_monitor_inv = ldlt_monitor.solve(Eigen::MatrixXd::Identity(S_monitor.rows(), S_monitor.cols()));
  } else {
    // S 数值奇异（理论上不会发生，R 正定）：退化为单位阵，避免 NaN 污染状态
    S_monitor_inv = Eigen::MatrixXd::Identity(S_monitor.rows(), S_monitor.cols());
  }

  /// 卡方检验：NIS = r^T S^-1 r ~ chi2(dof)，dof = 观测维度
  const double nis = innovation.dot(S_monitor_inv * innovation);
  const double threshold =
    nis_threshold > 0.0 ? nis_threshold : chi2_quantile_95(static_cast<int>(z.size()));
  const bool nis_fail = nis > threshold;

  // ---- Huber 软降权 ----
  // 超阈值的量测不丢弃，而是把对应块的 R 放大（等价于降低该组观测权重）。
  // 逐块计算：一块离群板不会把同帧其它正确板一起降权。
  // 注意：块内 NIS 只取 S 的对角块，忽略了块间相关，属于工程近似。
  Eigen::MatrixXd R_eff = R;
  double weight_sum = 1.0;
  int weight_n = 1;
  if (enable_huber && huber_block_size > 0 && s_ok) {
    const int dof = static_cast<int>(z.size());
    const int block = static_cast<int>(huber_block_size);
    const double k = huber_k > 0.0 ? huber_k : std::sqrt(chi2_quantile_95(block));

    weight_sum = 0.0;
    weight_n = 0;
    for (int offset = 0; offset + block <= dof; offset += block) {
      const Eigen::MatrixXd S_kk = S_monitor.block(offset, offset, block, block);
      const Eigen::VectorXd r_k = innovation.segment(offset, block);

      Eigen::LDLT<Eigen::MatrixXd> ldlt_k(S_kk);
      double nis_k = 0.0;
      if (ldlt_k.info() == Eigen::Success && ldlt_k.vectorD().minCoeff() > 0.0) {
        nis_k = r_k.dot(ldlt_k.solve(r_k));
      }

      const double w = nis_k > k * k ? k / std::sqrt(nis_k) : 1.0;
      R_eff.block(offset, offset, block, block) = R.block(offset, offset, block, block) / w;
      weight_sum += w;
      weight_n++;
    }
  }
  const double huber_weight = weight_n > 0 ? weight_sum / weight_n : 1.0;

  // 用加权后的 R 重新求 S 与增益；Huber 关闭时 R_eff == R，退化为标准 EKF
  const Eigen::MatrixXd S = H * P_prior * H.transpose() + R_eff;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
  Eigen::MatrixXd S_inv;
  if (ldlt.info() == Eigen::Success && ldlt.vectorD().minCoeff() > 0.0) {
    S_inv = ldlt.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
  } else {
    S_inv = S_monitor_inv;
  }

  const Eigen::MatrixXd K = P_prior * H.transpose() * S_inv;

  // Stable Compution of the Posterior Covariance (Joseph form)
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  const Eigen::MatrixXd I_KH = I - K * H;
  P = I_KH * P_prior * I_KH.transpose() + K * R_eff * K.transpose();
  P = 0.5 * (P + P.transpose());

  x = x_add(x_prior, K * innovation);

  total_count_++;
  if (nis_fail) nis_fail_count_++;

  // 按时间窗统计，避免帧率变化时判据的时间尺度漂移
  const double step = dt > 0.0 ? dt : last_dt_;
  last_dt_ = step;
  time_ += step;

  nis_window_.push_back({time_, nis, nis_fail ? 1 : 0});
  while (nis_window_.size() > 1 && time_ - nis_window_.front().time > window_seconds) {
    nis_window_.pop_front();
  }
  // 防御：即使时间戳异常也不让窗口无限增长
  constexpr size_t max_samples = 2000;
  while (nis_window_.size() > max_samples) nis_window_.pop_front();

  data["residual_norm"] = innovation.norm();
  if (innovation.size() >= 4) {
    data["residual_yaw"] = innovation[0];
    data["residual_pitch"] = innovation[1];
    data["residual_distance"] = innovation[2];
    data["residual_angle"] = innovation[3];
  }
  // log|S|：IMM 的模型似然需要它（L ∝ exp(-0.5·NIS) / sqrt(|S|)）
  double log_det_s = 0.0;
  if (s_ok) {
    for (int i = 0; i < S_monitor.rows(); i++) {
      log_det_s += std::log(std::abs(ldlt_monitor.vectorD()[i]));
    }
  }

  data["nis"] = nis;
  data["log_det_s"] = log_det_s;
  data["nis_threshold"] = threshold;
  data["nis_fail"] = nis_fail ? 1.0 : 0.0;  // 每帧复位，不再“一红到底”
  data["nis_failure_rate"] = nis_failure_rate();
  data["nis_mean"] = nis_mean();
  data["nis_ratio"] = z.size() > 0 ? nis_mean() / static_cast<double>(z.size()) : 0.0;
  data["nis_dof"] = static_cast<double>(z.size());
  data["huber_weight"] = huber_weight;  // 本帧平均权重：1 表示未降权

  return x;
}

double ExtendedKalmanFilter::nis_failure_rate() const
{
  if (nis_window_.empty()) return 0.0;
  const int failures =
    std::accumulate(nis_window_.begin(), nis_window_.end(), 0, [](int sum, const NisRecord & r) {
      return sum + r.fail;
    });
  return static_cast<double>(failures) / static_cast<double>(nis_window_.size());
}

double ExtendedKalmanFilter::nis_mean() const
{
  if (nis_window_.empty()) return 0.0;
  const double sum =
    std::accumulate(nis_window_.begin(), nis_window_.end(), 0.0, [](double sum, const NisRecord & r) {
      return sum + r.nis;
    });
  return sum / static_cast<double>(nis_window_.size());
}

}  // namespace tools
