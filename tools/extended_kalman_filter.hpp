#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace tools
{

/// 卡方分布 95% 分位点（上侧），自由度 dof
double chi2_quantile_95(int dof);

/// 卡方分布 99% 分位点（上侧），自由度 dof
double chi2_quantile_99(int dof);

class ExtendedKalmanFilter
{
public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);

  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

  /// dt 为本次更新对应的时间间隔（秒），仅用于 NIS 的按时间窗口统计；
  /// 传入 <=0 时沿用上一次的时间间隔
  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; },
    double dt = -1.0);

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; },
    double dt = -1.0);

  // 便捷重载：线性观测 + 只指定时间间隔
  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, double dt);

  std::map<std::string, double> data;  //卡方检验数据

  // ---- NIS 一致性监控（按时间窗统计，与帧率解耦）----
  double window_seconds = 1.0;      // 统计窗口长度（秒）
  size_t min_window_samples = 20;   // 判据生效前窗口内的最小样本数
  double nis_threshold = -1.0;      // <=0 时按观测维度自动取 95% 分位点

  // ---- 单帧鲁棒更新：Huber 软降权 ----
  // 超出阈值的量测不丢弃，而是把对应块的 R 放大（等价于降低该组观测的权重），
  // 避免硬门控在“连续拒绝 -> 强制放行”之间的抖动
  bool enable_huber = true;
  double huber_k = -1.0;        // <=0 时按块维度取 sqrt(chi2_quantile_95(block))
  size_t huber_block_size = 0;  // 0 = 整批一个权重；4 = 按装甲板逐块降权

  double nis_failure_rate() const;  // 窗口内 NIS 超限比例
  double nis_mean() const;          // 窗口内 NIS 均值（理想值 = 观测维度）
  size_t nis_window_samples() const { return nis_window_.size(); }
  int total_count() const { return total_count_; }
  int nis_fail_count() const { return nis_fail_count_; }

private:
  struct NisRecord
  {
    double time;
    double nis;
    int fail;
  };

  Eigen::MatrixXd I;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;

  std::deque<NisRecord> nis_window_;
  double time_ = 0.0;
  double last_dt_ = 0.01;

  int nis_fail_count_ = 0;
  int total_count_ = 0;
};

}  // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP
