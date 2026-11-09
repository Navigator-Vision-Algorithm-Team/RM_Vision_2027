#include "feature_tracker.hpp"

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <utility>

namespace visual_depth
{

FeatureTracker::FeatureTracker(
  const FeatureTrackerConfig & cfg, const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs)
: cfg_(cfg), K_(camera_matrix), D_(distort_coeffs)
{
  has_distortion_ = !D_.empty() && cv::countNonZero(D_.reshape(1, 1)) > 0;
}

void FeatureTracker::reset()
{
  prev_gray_.release();
  tracks_.clear();
  next_id_ = 0;
}

size_t FeatureTracker::observation_count() const
{
  size_t n = 0;
  for (const auto & t : tracks_) n += t.obs.size();
  return n;
}

std::vector<cv::Point2f> FeatureTracker::undistort(const std::vector<cv::Point2f> & pts) const
{
  if (pts.empty()) return {};
  if (!has_distortion_ || K_.empty()) return pts;

  std::vector<cv::Point2f> out;
  // 用 P = K：输出仍然是「去畸变后的像素坐标」，与观测的像素口径一致
  cv::undistortPoints(pts, out, K_, D_, cv::noArray(), K_);
  return out;
}

void FeatureTracker::push(const cv::Mat & gray, int camera_id, double stamp)
{
  if (gray.empty()) return;

  // 分辨率变了（换相机 / 重开）或首帧：状态全部作废，重新开始
  if (!prev_gray_.empty() && prev_gray_.size() != gray.size()) reset();

  if (!prev_gray_.empty() && !tracks_.empty()) track_existing(gray, camera_id, stamp);
  detect_new(gray, camera_id, stamp);

  // 深拷贝：调用方（推理线程）下一帧可能复用/释放这块缓冲，
  // 光流必须拿到确定属于「上一帧」的像素。
  gray.copyTo(prev_gray_);
}

void FeatureTracker::track_existing(const cv::Mat & gray, int camera_id, double stamp)
{
  std::vector<cv::Point2f> prev_pts;
  prev_pts.reserve(tracks_.size());
  for (const auto & t : tracks_) prev_pts.push_back(t.px);

  std::vector<cv::Point2f> next_pts;
  std::vector<uchar> fwd_status;
  std::vector<float> fwd_err;
  cv::calcOpticalFlowPyrLK(
    prev_gray_, gray, prev_pts, next_pts, fwd_status, fwd_err,
    cv::Size(cfg_.win_size, cfg_.win_size), cfg_.max_level);

  // 反向校验：current -> previous，回不到原位的点是漂移点
  std::vector<cv::Point2f> back_pts;
  std::vector<uchar> bwd_status;
  std::vector<float> bwd_err;
  cv::calcOpticalFlowPyrLK(
    gray, prev_gray_, next_pts, back_pts, bwd_status, bwd_err,
    cv::Size(cfg_.win_size, cfg_.win_size), cfg_.max_level);

  // 先筛出存活点，再一次性去畸变（避免逐点调用 undistortPoints）
  std::vector<int> keep_index;
  std::vector<cv::Point2f> keep_px;
  keep_index.reserve(tracks_.size());
  keep_px.reserve(tracks_.size());
  for (size_t i = 0; i < tracks_.size(); i++) {
    if (!fwd_status[i] || !bwd_status[i]) continue;
    if (cv::norm(back_pts[i] - prev_pts[i]) > static_cast<float>(cfg_.fb_error_px)) continue;
    const auto & p = next_pts[i];
    if (p.x < 0.f || p.y < 0.f || p.x > gray.cols - 1.f || p.y > gray.rows - 1.f) continue;
    keep_index.push_back(static_cast<int>(i));
    keep_px.push_back(p);
  }

  const auto keep_undist = undistort(keep_px);

  std::vector<FeatureTrack> kept;
  kept.reserve(keep_index.size());
  for (size_t k = 0; k < keep_index.size(); k++) {
    auto track = std::move(tracks_[keep_index[k]]);
    track.px = keep_px[k];

    FeatureObs obs;
    obs.track_id = track.id;
    obs.camera_id = camera_id;
    obs.stamp = stamp;
    obs.px = keep_undist[k];

    track.obs.push_back(obs);
    if (static_cast<int>(track.obs.size()) > cfg_.max_obs_per_track) {
      track.obs.erase(track.obs.begin());
    }
    // 太久没更新的历史直接截断（角速度大时轨迹窗口不需要拉太长）
    if (cfg_.max_age_s > 0.0 && track.obs.size() > 1 &&
        stamp - track.obs.front().stamp > cfg_.max_age_s) {
      track.obs.erase(track.obs.begin());
    }

    kept.push_back(std::move(track));
  }

  tracks_ = std::move(kept);
}

void FeatureTracker::detect_new(const cv::Mat & gray, int camera_id, double stamp)
{
  const int need = cfg_.max_corners - static_cast<int>(tracks_.size());
  if (need <= 0) return;

  // ROI：图像下半部（地平线以下的障碍 / 地面），省 60% 特征点。
  // 注意像素坐标仍按全画幅记录，只是 roi.y 是检测窗口的原点。
  const double ratio = std::clamp(cfg_.roi_bottom_ratio, 0.05, 1.0);
  const int y0 = std::clamp(
    static_cast<int>(gray.rows * (1.0 - ratio)), 0, std::max(0, gray.rows - 1));
  const cv::Rect roi(0, y0, gray.cols, gray.rows - y0);
  if (roi.width < 8 || roi.height < 8) return;

  cv::Mat view = gray(roi);
  cv::Mat mask(roi.size(), CV_8U, cv::Scalar(255));

  // 已经在跟踪的点周围屏蔽，避免同一物理点被重复检测成两条轨迹
  const int gap = std::max(1, static_cast<int>(cfg_.min_distance));
  for (const auto & t : tracks_) {
    if (t.px.y < static_cast<float>(roi.y)) continue;
    cv::circle(mask, cv::Point2f(t.px.x, t.px.y - roi.y), gap, cv::Scalar(0), -1);
  }

  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(view, corners, need, cfg_.quality, cfg_.min_distance, mask, 3, false, 0.04);
  if (corners.empty()) return;

  // 加回 ROI 偏移，恢复全画幅坐标；再批量去畸变
  for (auto & p : corners) p.y += static_cast<float>(roi.y);
  const auto undist = undistort(corners);

  for (size_t i = 0; i < corners.size(); i++) {
    FeatureTrack track;
    track.id = next_id_++;
    track.px = corners[i];

    FeatureObs obs;
    obs.track_id = track.id;
    obs.camera_id = camera_id;
    obs.stamp = stamp;
    obs.px = undist[i];
    track.obs.push_back(obs);

    tracks_.push_back(std::move(track));
  }
}

}  // namespace visual_depth
