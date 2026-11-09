#ifndef VISUAL_DEPTH__TRIANGULATOR_HPP
#define VISUAL_DEPTH__TRIANGULATOR_HPP

// =============================================================================
//  triangulator.hpp —— 固定位姿三角化 + 局部 BA（M-A 模式，方案 §4.3 / §4.4 / V6）
//
//  位姿来自导航（Point-LIO，cm 级），所以这里做的是「已知位姿的结构恢复」：
//
//      对同一条轨迹的 N 次观测 (u_i, v_i) 与位姿 (R_i, t_i)：
//        1) 归一化坐标 x_i = K^-1 [u v 1]（观测已去畸变，纯针孔）
//        2) DLT：堆一个 2N×4 矩阵，SVD 取最小奇异向量 -> X0（齐次，除以 w）
//        3) GN/LM：最小化重投影误差，3~5 次迭代即可收敛
//        4) 质量判据：视差 / 深度 / 重投影 / 条件数
//
//  【M-A = 只优化点，不碰位姿】这是本模块唯一实现的模式（方案 §4.4）：
//    · 每个 3D 点用自己在多帧上的全部观测做一次 3×3 LM，点与点之间没有耦合，
//      所以不需要组装大矩阵，也不存在「优化后的位姿和导航位姿不自洽」的问题；
//    · 位姿保持导航给的值，输出点天然与导航地图同一套坐标；
//    · M-B（位姿联合优化 + Schur 补）**本期不实现**，`mode` 只接受 points_only。
//
//  【为什么不用 Ceres/g2o】aarch64 上 Ceres 要拉 abseil/glog/suitesparse，
//    交叉编译成本远超收益；M-A 的 3×3 系统手写 LM 就够，且只依赖 Eigen。
//
//  本文件不依赖 ROS，可单独做离线单测。
// =============================================================================

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <deque>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace visual_depth
{

struct CameraIntrinsics
{
  double fx = 1.0;
  double fy = 1.0;
  double cx = 0.0;
  double cy = 0.0;
};

// 某个观测时刻的相机位姿：p_world = R * p_cam + t（R = R_world<-cam）
struct PoseSample
{
  double stamp = 0.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

struct TriConfig
{
  double min_baseline = 0.15;         // m，窗口内最大基线；低于它判为纯旋转退化
  double min_parallax_deg = 0.5;      // 视差角下限
  double min_depth = 0.4;             // m
  double max_depth = 12.0;            // m（§10 R1：单目 8 m 外深度不可信）
  double max_reproj_px = 3.0;         // 最大重投影误差（RMS）
  double max_condition = 1e5;         // 法方程条件数上限
  int min_observations = 3;           // 进入 BA 的最少观测数
  int refine_iterations = 4;          // 不进 BA 时的 GN 迭代次数
};

struct BaConfig
{
  bool enable = true;
  int window_size = 8;              // 参与一条轨迹优化的最大观测数（帧）
  int max_iterations = 8;
  double huber_threshold_px = 3.0;
};

struct TriResult
{
  Eigen::Vector3d p_world = Eigen::Vector3d::Zero();
  int observations = 0;
  double parallax_deg = 0.0;
  double reproj_px = 0.0;  // 优化后的重投影 RMS
  double depth_m = 0.0;    // 最新一次观测下的深度
  bool valid = false;
  const char * reject = "";  // 拒绝原因（静态字符串，热路径上不做字符串分配）
};

class Triangulator
{
public:
  Triangulator(const CameraIntrinsics & K, const TriConfig & tri, const BaConfig & ba);

  // px: 已去畸变的像素坐标；poses: 与 px 一一对应的世界位姿
  TriResult triangulate(
    const std::vector<cv::Point2f> & px, const std::vector<PoseSample> & poses) const;

  const TriConfig & tri_config() const { return tri_; }
  const BaConfig & ba_config() const { return ba_; }

private:
  CameraIntrinsics K_;
  TriConfig tri_;
  BaConfig ba_;
};

// -----------------------------------------------------------------------------
//  多帧一致性（动态点过滤，方案 §4.7）
//
//  同一个 3D 点在连续 K 个窗口里的估计位置差超过 max_spread => 判为动态
//  （落在敌人/队友身上）；连续 reject_streak 次判为动态则永久丢弃这条轨迹。
//
//  为什么必须做：PCD 里混进动态点，small_gicp 重定位会被拉偏；costmap 里
//  出现假障碍，规划会被直接堵死（风险 R4）。
// -----------------------------------------------------------------------------
struct DynamicFilterConfig
{
  bool enable = true;
  int consistency_window = 3;
  double max_spread = 0.2;  // m
  int reject_streak = 3;
};

class DynamicFilter
{
public:
  explicit DynamicFilter(const DynamicFilterConfig & cfg);

  // 返回 true 表示该点是动态点，应丢弃。
  // spread：可选输出，窗口内最大两两距离（m），供 C4 的 curvature 一致性字段用。
  bool update(int track_id, const Eigen::Vector3d & p_world, double * spread = nullptr);

  size_t rejected() const { return rejected_; }
  void clear();

private:
  struct State
  {
    std::deque<Eigen::Vector3d> hist;
    int streak = 0;
  };

  DynamicFilterConfig cfg_;
  std::unordered_map<int, State> states_;
  std::unordered_set<int> rejected_ids_;
  std::deque<int> order_;  // FIFO，限制 map 规模（轨迹 id 单调递增，不会复用）
  size_t rejected_ = 0;
  static constexpr size_t kMaxStates = 4096;
};

}  // namespace visual_depth

#endif  // VISUAL_DEPTH__TRIANGULATOR_HPP
