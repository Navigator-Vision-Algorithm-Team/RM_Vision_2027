#include "ground_projection.hpp"

#include <cmath>

namespace visual_depth
{

GroundProjector::GroundProjector(const GroundProjectionConfig & cfg) : cfg_(cfg) {}

void GroundProjector::project(
  const std::vector<cv::Point2f> & px, const CameraIntrinsics & K, const Eigen::Matrix3d & R_world_cam,
  const Eigen::Vector3d & t_world_cam, std::vector<Eigen::Vector3d> & out) const
{
  if (!cfg_.enable || px.empty()) return;

  for (const auto & p : px) {
    // 相机系归一化射线（观测已去畸变，纯针孔）
    const Eigen::Vector3d d(
      (p.x - K.cx) / K.fx, (p.y - K.cy) / K.fy, 1.0);

    const Eigen::Vector3d dw = R_world_cam * d;
    if (dw.z() > -1e-3) continue;  // 射线朝上 / 平行地面：与地面无交点

    const double lambda = (cfg_.ground_z - t_world_cam.z()) / dw.z();
    if (!std::isfinite(lambda) || lambda <= 0.0) continue;

    const Eigen::Vector3d world = t_world_cam + lambda * dw;
    // 量程按「到当前相机的距离」算：camera_init 原点是上电位置，
    // 车跑起来之后 world.norm() 已经不是「离我多远」了。
    if ((world - t_world_cam).norm() > cfg_.max_range) continue;

    out.push_back(world);
  }
}

}  // namespace visual_depth
