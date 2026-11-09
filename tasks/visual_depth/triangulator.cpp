#include "triangulator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/SVD>

namespace visual_depth
{
namespace
{

// 相机系 -> 像素（针孔；观测已去畸变，所以这里不需要畸变模型）
inline Eigen::Vector2d project(const CameraIntrinsics & K, const Eigen::Vector3d & p)
{
  return {K.fx * p.x() / p.z() + K.cx, K.fy * p.y() / p.z() + K.cy};
}

// 一对观测的残差与雅可比：r = pi(K(Rc X + tc)) - uv
//   J = d pi / d p * Rc   （2×3）
inline bool residual_jacobian(
  const CameraIntrinsics & K, const Eigen::Vector2d & uv, const Eigen::Matrix3d & Rc,
  const Eigen::Vector3d & tc, const Eigen::Vector3d & X, Eigen::Vector2d & r, Eigen::Matrix<double, 2, 3> & J)
{
  const Eigen::Vector3d p = Rc * X + tc;
  if (p.z() <= 1e-3) return false;  // 在相机背后

  const double iz = 1.0 / p.z();
  const double iz2 = iz * iz;
  r = Eigen::Vector2d(
        K.fx * p.x() * iz + K.cx - uv.x(), K.fy * p.y() * iz + K.cy - uv.y());

  Eigen::Matrix<double, 2, 3> Jp;
  Jp << K.fx * iz, 0.0, -K.fx * p.x() * iz2, 0.0, K.fy * iz, -K.fy * p.y() * iz2;
  J = Jp * Rc;
  return true;
}

}  // namespace

Triangulator::Triangulator(const CameraIntrinsics & K, const TriConfig & tri, const BaConfig & ba)
: K_(K), tri_(tri), ba_(ba)
{
}

TriResult Triangulator::triangulate(
  const std::vector<cv::Point2f> & px, const std::vector<PoseSample> & poses) const
{
  TriResult result;
  const size_t n = px.size();
  result.observations = static_cast<int>(n);
  if (n < 2 || poses.size() != n) {
    result.reject = "obs";
    return result;
  }

  // ---- 1) 基线检查：纯旋转时三角化必然退化（§4.6） ----
  double max_baseline = 0.0;
  for (size_t i = 0; i < n; i++) {
    for (size_t j = i + 1; j < n; j++) {
      max_baseline = std::max(max_baseline, (poses[i].t - poses[j].t).norm());
    }
  }
  if (max_baseline < tri_.min_baseline) {
    result.reject = "baseline";
    return result;
  }

  // ---- 2) 归一化坐标与「世界 -> 相机」投影矩阵 ----
  // 观测已去畸变 => 纯针孔；xn = K^-1 [u v 1]，DLT 直接在归一化坐标上做。
  std::vector<Eigen::Vector3d> xn(n);
  std::vector<Eigen::Matrix3d> Rc(n);
  std::vector<Eigen::Vector3d> tc(n);
  std::vector<Eigen::Vector2d> uv(n);
  for (size_t i = 0; i < n; i++) {
    uv[i] = Eigen::Vector2d(px[i].x, px[i].y);
    xn[i] = Eigen::Vector3d(
      (px[i].x - K_.cx) / K_.fx, (px[i].y - K_.cy) / K_.fy, 1.0);
    Rc[i] = poses[i].R.transpose();
    tc[i] = -Rc[i] * poses[i].t;
  }

  // ---- 3) DLT 求初值 ----
  Eigen::MatrixXd A(static_cast<Eigen::Index>(2 * n), 4);
  for (size_t i = 0; i < n; i++) {
    Eigen::Matrix<double, 3, 4> P;
    P.leftCols<3>() = Rc[i];
    P.col(3) = tc[i];
    const Eigen::RowVector4d r0 = P.row(0);
    const Eigen::RowVector4d r1 = P.row(1);
    const Eigen::RowVector4d r2 = P.row(2);
    A.row(static_cast<Eigen::Index>(2 * i)) = xn[i].z() * r0 - xn[i].x() * r2;
    A.row(static_cast<Eigen::Index>(2 * i + 1)) = xn[i].z() * r1 - xn[i].y() * r2;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d Xh = svd.matrixV().col(3);
  if (std::abs(Xh[3]) < 1e-9) {
    result.reject = "dlt";
    return result;
  }
  Eigen::Vector3d X = Xh.head<3>() / Xh[3];
  if (!X.allFinite()) {
    result.reject = "dlt";
    return result;
  }

  // ---- 4) 固定位姿的 LM 精化（M-A：每点独立 3×3 系统） ----
  const bool use_robust = ba_.enable && static_cast<int>(n) >= tri_.min_observations;
  const int max_iter = use_robust ? std::max(1, ba_.max_iterations)
                                  : std::max(1, tri_.refine_iterations);
  const double huber = use_robust ? std::max(1e-3, ba_.huber_threshold_px) : 0.0;

  auto evaluate = [&](const Eigen::Vector3d & Xv, Eigen::Matrix3d * H, Eigen::Vector3d * b,
                      double * sse) {
    double sum_sse = 0.0;
    if (H) H->setZero();
    if (b) b->setZero();
    for (size_t i = 0; i < n; i++) {
      Eigen::Vector2d r;
      Eigen::Matrix<double, 2, 3> J;
      if (!residual_jacobian(K_, uv[i], Rc[i], tc[i], Xv, r, J)) {
        sum_sse += 1e6;  // 落在相机背后：直接判为不可行
        continue;
      }
      sum_sse += r.squaredNorm();
      if (!H || !b) continue;

      const double rn = r.norm();
      const double w = (huber > 0.0 && rn > huber) ? huber / rn : 1.0;
      *H += w * J.transpose() * J;
      *b += w * J.transpose() * r;
    }
    if (sse) *sse = sum_sse;
  };

  double sse = 0.0;
  Eigen::Matrix3d H;
  Eigen::Vector3d b;
  evaluate(X, &H, &b, &sse);

  double lambda = 1e-4;
  for (int iter = 0; iter < max_iter; iter++) {
    Eigen::Matrix3d Hd = H + lambda * Eigen::Matrix3d(H.diagonal().asDiagonal());
    const Eigen::Vector3d dX = Hd.ldlt().solve(-b);
    if (!dX.allFinite()) break;

    const Eigen::Vector3d Xn = X + dX;
    double sse_n = 0.0;
    evaluate(Xn, nullptr, nullptr, &sse_n);

    if (sse_n < sse) {
      const double gain = sse - sse_n;
      X = Xn;
      lambda = std::max(lambda * 0.3, 1e-8);
      evaluate(X, &H, &b, &sse);
      if (gain < 1e-6 || sse < 1e-9) break;
    } else {
      lambda *= 5.0;
      if (lambda > 1e6) break;
    }
  }

  // ---- 5) 质量评估 ----
  // 5.1 重投影 RMS（所有观测）
  double sse_all = 0.0;
  int ok = 0;
  for (size_t i = 0; i < n; i++) {
    Eigen::Vector2d r;
    Eigen::Matrix<double, 2, 3> J;
    if (!residual_jacobian(K_, uv[i], Rc[i], tc[i], X, r, J)) continue;
    sse_all += r.squaredNorm();
    ok++;
  }
  if (ok < 2) {
    result.reject = "behind";
    return result;
  }
  result.reproj_px = std::sqrt(sse_all / static_cast<double>(ok));

  // 5.2 视差角：视线两两夹角的最大值（n <= 15，直接暴力枚举）
  std::vector<Eigen::Vector3d> rays(n, Eigen::Vector3d::UnitZ());
  for (size_t i = 0; i < n; i++) {
    const Eigen::Vector3d d = X - poses[i].t;
    const double norm = d.norm();
    if (norm > 1e-6) rays[i] = d / norm;
  }
  double max_angle = 0.0;
  for (size_t i = 0; i < n; i++) {
    for (size_t j = i + 1; j < n; j++) {
      const double c = std::clamp(rays[i].dot(rays[j]), -1.0, 1.0);
      max_angle = std::max(max_angle, std::acos(c));
    }
  }
  result.parallax_deg = max_angle * 180.0 / M_PI;

  // 5.3 深度：以最新一次观测的相机系为准
  const Eigen::Vector3d p_last = Rc[n - 1] * X + tc[n - 1];
  result.depth_m = p_last.z();

  // 5.4 法方程条件数（H 已经是最终解处线性化得到的）
  Eigen::JacobiSVD<Eigen::Matrix3d> svd_h(H);
  const auto & sv = svd_h.singularValues();
  const double cond = sv[2] > 1e-12 ? sv[0] / sv[2] : 1e12;

  // ---- 6) 判据（§4.3 Step 4） ----
  if (result.parallax_deg < tri_.min_parallax_deg) {
    result.reject = "parallax";
    return result;
  }
  if (result.depth_m < tri_.min_depth || result.depth_m > tri_.max_depth) {
    result.reject = "depth";
    return result;
  }
  if (result.reproj_px > tri_.max_reproj_px) {
    result.reject = "reproj";
    return result;
  }
  if (cond > tri_.max_condition) {
    result.reject = "cond";
    return result;
  }

  result.p_world = X;
  result.valid = true;
  return result;
}

// -----------------------------------------------------------------------------

DynamicFilter::DynamicFilter(const DynamicFilterConfig & cfg) : cfg_(cfg) {}

void DynamicFilter::clear()
{
  states_.clear();
  rejected_ids_.clear();
  order_.clear();
  rejected_ = 0;
}

bool DynamicFilter::update(int track_id, const Eigen::Vector3d & p_world, double * spread_out)
{
  if (!cfg_.enable) return false;
  if (rejected_ids_.count(track_id)) return true;

  auto & state = states_[track_id];
  if (state.hist.empty()) order_.push_back(track_id);

  state.hist.push_back(p_world);
  while (static_cast<int>(state.hist.size()) > std::max(2, cfg_.consistency_window)) {
    state.hist.pop_front();
  }

  // 窗口内的最大两两距离
  double spread = 0.0;
  for (size_t i = 0; i < state.hist.size(); i++) {
    for (size_t j = i + 1; j < state.hist.size(); j++) {
      spread = std::max(spread, (state.hist[i] - state.hist[j]).norm());
    }
  }
  if (spread_out) *spread_out = spread;

  if (spread > cfg_.max_spread) {
    state.streak++;
  } else {
    state.streak = 0;
  }

  if (state.streak >= std::max(1, cfg_.reject_streak)) {
    rejected_ids_.insert(track_id);
    rejected_++;
    state.hist.clear();
    state.streak = 0;
    return true;
  }

  // 轨迹 id 单调递增、不会复用，FIFO 淘汰最老的即可（防止 map 无界增长）
  while (states_.size() > kMaxStates && !order_.empty()) {
    const int old = order_.front();
    order_.pop_front();
    if (old != track_id) states_.erase(old);
  }

  return false;
}

}  // namespace visual_depth
