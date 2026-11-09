#ifndef VISUAL_DEPTH__FEATURE_TRACKER_HPP
#define VISUAL_DEPTH__FEATURE_TRACKER_HPP

// =============================================================================
//  feature_tracker.hpp —— 单相机跨帧特征跟踪（方案 §6 V5，L1 阶段）
//
//  只跑一台相机（左后 omni_cam_3）：
//    · 小yaw 的遮挡出现在左后方，只需要覆盖该方向；
//    · 光流 + 角点检测约 1 核，4 路全开会把自瞄的算力抢掉（风险 R9）。
//
//  实现是标准的「goodFeaturesToTrack + calcOpticalFlowPyrLK + 前后向校验」：
//    1) 前一帧的每个轨迹点在当前帧做前向 LK；
//    2) 再反向 LK 回前一帧，回不去的（误差 > fb_error_px）判为漂移，丢弃；
//    3) 存活的轨迹追加一次观测；
//    4) 轨迹数不足 max_corners 时在 ROI 里补新角点（ROI 外的点用 mask 屏蔽）。
//
//  ⚠ ROI 只影响「新特征在哪里检测」，不改图像几何：观测的像素坐标始终是
//    全画幅坐标，内参 K 不需要任何裁剪修正（裁剪才需要改 cx/cy，容易错）。
//
//  ⚠ 观测在入库时就已去畸变（cv::undistortPoints，P=K）：三角化那边只处理
//    纯针孔模型，不必再考虑畸变，误差模型也统一。
//
//  本文件不依赖 ROS，可单独做离线单测。
// =============================================================================

#include <cstddef>
#include <opencv2/core.hpp>
#include <vector>

namespace visual_depth
{

// 一次特征观测
struct FeatureObs
{
  int track_id = -1;
  int camera_id = -1;
  double stamp = 0.0;        // 图像采集时刻（秒，调用方统一时间基准）
  cv::Point2f px{0.f, 0.f};  // 去畸变后的全画幅像素坐标
};

struct FeatureTrackerConfig
{
  int max_corners = 300;           // 每帧参与跟踪的特征点上限
  double quality = 0.01;           // goodFeaturesToTrack 质量阈值
  double min_distance = 8.0;       // 角点之间 / 与已有轨迹之间的最小像素距离
  int win_size = 21;               // LK 窗口
  int max_level = 3;               // LK 金字塔层数
  double fb_error_px = 1.0;        // 前后向一致性阈值（像素）
  double roi_bottom_ratio = 0.5;   // 只用图像下半部检测新特征（地平线以下）
  int max_obs_per_track = 15;      // 单条轨迹保留的观测上限（滑动）
  double max_age_s = 3.0;          // 超过该时长没有新观测的轨迹直接丢弃
};

// 一条轨迹：跨帧的同一物理点
struct FeatureTrack
{
  int id = -1;
  // 当前帧在「原始（带畸变）图像」上的位置，仅 LK 内部使用；
  // 下游只应该读 obs（已去畸变）。
  cv::Point2f px{0.f, 0.f};
  std::vector<FeatureObs> obs;  // 按时间升序
};

class FeatureTracker
{
public:
  // camera_matrix: 3x3 CV_64F（与图像工作分辨率对应）
  // distort_coeffs: 1x5 / 5x1，空 Mat => 无畸变
  FeatureTracker(
    const FeatureTrackerConfig & cfg, const cv::Mat & camera_matrix,
    const cv::Mat & distort_coeffs);

  // 推入一帧灰度图（全画幅）。同一 tracker 只能喂同一台相机。
  void push(const cv::Mat & gray, int camera_id, double stamp);

  const std::vector<FeatureTrack> & tracks() const { return tracks_; }

  size_t track_count() const { return tracks_.size(); }
  size_t observation_count() const;

  // 图像尺寸变化（换分辨率 / 重新打开相机）时清空状态
  void reset();

private:
  void track_existing(const cv::Mat & gray, int camera_id, double stamp);
  void detect_new(const cv::Mat & gray, int camera_id, double stamp);
  std::vector<cv::Point2f> undistort(const std::vector<cv::Point2f> & pts) const;

  FeatureTrackerConfig cfg_;
  cv::Mat K_;
  cv::Mat D_;
  bool has_distortion_ = false;

  cv::Mat prev_gray_;
  std::vector<FeatureTrack> tracks_;
  int next_id_ = 0;
};

}  // namespace visual_depth

#endif  // VISUAL_DEPTH__FEATURE_TRACKER_HPP
