#ifndef AUTO_AIM__PNP_HPP
#define AUTO_AIM__PNP_HPP

// =============================================================================
//  pnp.hpp —— 装甲板 PnP 解算的「公共内核」
//
//  背景 / 为什么要有这个文件：
//    原先这套 PnP（solvePnP + SOLVEPNP_IPPE）只写在 tasks/auto_aim/solver.cpp 里，
//    是主相机专用的（内参、畸变、R_camera2gimbal 都直接来自 yaml 的全局字段）。
//    现在全向相机也要用同一套解算，所以把「与相机无关」的部分抽出来：
//      · 装甲板 3D 模型点（尺寸常量 + 4 个角点）
//      · solvePnP 的调用与结果（相机系下的平移 / 姿态 / 重投影误差）
//    主相机 Solver 和全向相机 OmniPnp 都调用这里，保证两边算法完全一致，
//    以后改尺寸、改 solvePnP flag 只需要改这一处。
// =============================================================================

#include <Eigen/Dense>  // 必须在 opencv2/core/eigen.hpp 上面
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "armor.hpp"

namespace auto_aim
{

// ----- 装甲板物理尺寸（单位：m）-----
// 赛季若改变大/小装甲板宽度或灯条长度，只改这里即可（主相机、全向相机共用）。
extern const double LIGHTBAR_LENGTH;
extern const double BIG_ARMOR_WIDTH;
extern const double SMALL_ARMOR_WIDTH;

// 返回给定类型装甲板的 4 个 3D 模型点。
// 点的顺序必须与 detector/YOLO 输出的 armor.points 顺序一致：
//   [右上, 左上, 左下, 右下]（以灯条竖直、面向相机为基准）
const std::vector<cv::Point3f> & armor_object_points(ArmorType type);

// PnP 的完整结果。字段含义与 OpenCV solvePnP 输出一一对应。
struct PnPResult
{
  bool success = false;  // solvePnP 是否成功且几何上有效

  // 装甲板中心在「相机坐标系」下的坐标（m），即 OpenCV solvePnP 的 tvec。
  Eigen::Vector3d xyz_in_camera = Eigen::Vector3d::Zero();

  // 装甲板坐标系 -> 相机坐标系的旋转，即 solvePnP 的 rvec 转成的旋转矩阵。
  Eigen::Matrix3d R_armor2camera = Eigen::Matrix3d::Identity();

  // 4 个模型点在图像上的平均重投影误差（px），可用于剔除离谱的远距离误检。
  double reprojection_error = 0.0;
};

// =============================================================================
//  主相机与全向相机共用的 PnP 内核。
//
//  参数：
//    armor           —— 至少需要 armor.type 与 armor.points（4 个像素点）
//    camera_matrix   —— 3x3 内参（CV_64F）
//    distort_coeffs  —— 1xN 畸变系数（CV_64F，可为空 => 认为无畸变）
//
//  说明：
//    · 使用 SOLVEPNP_IPPE，与原先主相机的用法完全一致（平面 4 点专用解法）；
//    · 平面 4 点存在「镜像二义性」（两个位姿都能重投影出几乎相同的 4 个点）。
//      solvePnP + IPPE 返回其中重投影误差较小的一个。全向相机视野斜、距离远时
//      该二义性更容易显现，后续如有需要可改用 solvePnPGeneric 取两解再按
//      「目标必须在相机前方 + 距离在合理范围」筛选（见 decider.cpp 中的 TODO）。
// =============================================================================
PnPResult solve_pnp_armor(
  const Armor & armor, const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs);

}  // namespace auto_aim

#endif  // AUTO_AIM__PNP_HPP
