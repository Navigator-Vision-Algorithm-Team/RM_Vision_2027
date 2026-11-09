#ifndef VISUAL_DEPTH__GROUND_PROJECTION_HPP
#define VISUAL_DEPTH__GROUND_PROJECTION_HPP

// =============================================================================
//  ground_projection.hpp —— 地面平面反投影（纯旋转兜底，方案 §4.5 / L3）
//
//  机器人原地小陀螺时基线 ≈ 0，三角化必然失效。此时用「单帧 + 位姿」求地面点：
//
//      像素射线 d = K^-1 [u v 1]（相机系）-> 世界系方向 Dw = R * d
//      与平面 z = ground_z 求交：  lambda = (ground_z - t.z) / Dw.z
//      交点                        P = t + lambda * Dw
//
//  只有 Dw.z < 0（射线朝下）时才有解；射线朝上（看天空）直接丢弃。
//
//  【适用】只对贴地物体有效：地面、场地边线、矮挡板（§4.5）。
//  【价值】纯旋转时唯一能产出深度的方式，且不需要基线（风险 R6）。
//
//  ⚠ ground_z 是「地面在 camera_init 下的 z」。Point-LIO 的原点在 IMU 上电位置，
//    所以地面一般是负值（约等于 -IMU 离地高度），必须按实车标定填，不能默认 0。
//
//  本文件不依赖 ROS，可单独做离线单测。
// =============================================================================

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <vector>

#include "triangulator.hpp"  // CameraIntrinsics

namespace visual_depth
{

struct GroundProjectionConfig
{
  bool enable = true;
  double ground_z = 0.0;   // 地面在 camera_init 下的 z（m），实车通常为负
  double max_range = 6.0;  // m，超过该距离的地面点不再使用
};

class GroundProjector
{
public:
  explicit GroundProjector(const GroundProjectionConfig & cfg);

  bool enabled() const { return cfg_.enable; }

  // px: 已去畸变的全画幅像素；R/t: 该帧的世界位姿（T_world<-cam）
  void project(
    const std::vector<cv::Point2f> & px, const CameraIntrinsics & K,
    const Eigen::Matrix3d & R_world_cam, const Eigen::Vector3d & t_world_cam,
    std::vector<Eigen::Vector3d> & out) const;

private:
  GroundProjectionConfig cfg_;
};

}  // namespace visual_depth

#endif  // VISUAL_DEPTH__GROUND_PROJECTION_HPP
